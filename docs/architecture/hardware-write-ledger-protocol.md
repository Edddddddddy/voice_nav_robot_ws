# Hardware-write ledger protocol

This document is the normative Interface contract for the default-off,
test-only hardware-write proof channel used by VN-0011A and VN-0011B. It does
not add a ROS Interface, product node, or safety certification claim.

## Module and seam

The Hardware-Write Ledger is one deep Module. Its shared-memory ABI is the seam
between the test Parent and the Gazebo Writer process. POSIX creation/attach
and the Gazebo hardware wrapper are Adapters; neither owns ledger policy.

The Writer Interface is deliberately small and both operations are `noexcept`:

```text
begin_write(sim_stamp) -> write ticket
upstream.write(...)
finish_write(ticket, delegated result, wheel observation)
```

`begin_write` consumes eligible control, assigns the next global sequence, and
returns an opaque ticket. `finish_write` consumes that exact ticket once. A
wheel observation is `VALID`, `MISSING_ENTITY`, `MISSING_COMPONENT`,
`EMPTY_COMPONENT`, or Adapter-local `INSPECTION_FAILURE`; only `VALID` carries
left/right IEEE-754 bits. `INSPECTION_FAILURE` is deliberately outside the
recordable ABI range and therefore latches `OBSERVATION`. The Adapter must
report every outcome and preserve the upstream return value.

If the delegated upstream write throws, the Adapter still finishes the exact
outstanding ticket once with the named out-of-range
`kHardwareWriteDelegatedException` sentinel and then rethrows the original
exception. Writer latches `PROTOCOL` (and any independent observation fault),
consumes the sequence, and never fabricates a normal upstream `ERROR` return.

One Writer permits exactly one outstanding ticket because the pinned hardware
Interface is synchronous. A nested or concurrent `begin_write`, sequence
exhaustion, a zero/wrapped sequence, or a duplicate, stale, foreign, or
mismatched `finish_write` latches a global protocol/sequence fault and returns
or consumes an invalid ticket without throwing. It never reuses or fabricates
a sequence. A successful `begin_write` that is not followed by its matching
finish leaves `last_completed_write_seq` unchanged; a later begin detects the
outstanding ticket, and Writer death leaves any ACTIVE interval invalid. A
valid finish records or faults the observation before release-publishing its
sequence as completed.

One lock-free lifecycle word serializes `IDLE -> BEGINNING -> OUTSTANDING ->
FINISHING -> IDLE`. Admission and finish each use one compare/exchange; a
conflict fails immediately, latches `PROTOCOL`, and never reads or mutates the
other caller's ordinary ticket fields. The synchronous hardware Interface owns
the only legitimate finish call. A mismatched finish restores OUTSTANDING only
to preserve the evidence state; any foreign or concurrent finish is a terminal
protocol violation and the harness must restart rather than depend on a retry.

These methods perform no allocation, filesystem I/O, logging, ROS call,
transport publication, explicit lock, retry loop, or blocking wait. Conflict
paths are fail-fast and bounded. This lock-free-atomic contract is not a formal
claim that the whole method is wait-free or immune to an OS page fault.
Attachment and mapping are outside the write seam: its constructor may throw
`std::invalid_argument`, `std::system_error`, or allocation failure before
returning a usable Writer. The POSIX Attached Adapter owns mapping RAII and
delegates ledger policy to the Writer implementation. Its claimed-PID
inspection is a test diagnostic, not part of the Gazebo Writer Interface.
The default Gazebo Adapter reads an exact `journal_name` / `journal_nonce` pair
from `HardwareInfo` during `initSim`, attaches once before the first journaled
write, and fails initialization for a partial, invalid, unclaimable, or changed
identity. Product Xacro and launch files contain no such test seam.

The Parent Interface posts ARM/SEAL, polls the matching receipt, copies every
bounded page of one sealed interval, and ACKs an exact sealed identity. Only
one request may be outstanding.
The monotonically increasing request ticket is its idempotency identity; a
duplicate with identical checksum returns the original receipt, while reuse
with different payload, a gap, or wrap latches a protocol fault.

Every rejected request publishes `INVALID` with the canonical response
identity `{bank_index=INVALID_BANK_INDEX, bank_epoch=0}` and the
`last_completed_write_seq` observed when control was consumed. It never echoes
an unselected request identity. A bad checksum or invalid SEAL field latches
the global `PROTOCOL` fault but does not mutate an ACTIVE or terminal bank;
the Parent may issue a new correctly checksummed request only with the next
ticket. Tests cover every SEAL field, request checksum, and byte-for-byte
immutability of both `SEALED_OK` and `SEALED_FAULT` evidence.

One owned request envelope prevents either role from reading ordinary fields
while the other role may write them:

```text
IDLE -> WRITING -> READY -> READING -> IDLE
```

Parent claims `IDLE -> WRITING`, fills the complete request, then
release-publishes READY. Writer claims `READY -> READING`, copies the request
to a local snapshot, and all later processing uses only that snapshot. An
immediate request releases IDLE before its response ticket. A deferred SEAL
retains READING across non-qualifying writes and the qualifying finish, then
releases IDLE before release-publishing its response ticket. Thus observing a
response also observes a reusable envelope, while a pending request cannot be
republished or replaced. An exact replay is idempotent only after the original
receipt exists. The response stores the consumed request checksum, so its CRC
never depends on a mutable request word.

## Linearization and interval semantics

Every enabled upstream `write()` call receives one global, contiguous,
non-wrapping `write_seq`, including unarmed calls and calls whose later
observation is invalid. The Writer owns this sequence; the Adapter cannot
provide or skip it.

ARM is consumed at the next write entry before sequence assignment. If the
next sequence is `s`, the new ACTIVE bank records `arm_fence=s-1`, and `s` is
the first included invocation. The ARM receipt is published only after the
bank is ACTIVE.

SEAL carries an exact ACTIVE-bank identity and `not_before_sim_stamp_ns`. It
may also require an exact trigger stamp. After the Writer consumes the request:

1. writes below the threshold remain part of the ACTIVE interval;
2. the first qualifying write is delegated upstream;
3. its result and wheel observation are captured and appended;
4. the accumulator is finalized and all sticky faults are fixed;
5. the bank checksum is written, then state is release-published as
   `SEALED_OK` or `SEALED_FAULT`;
6. only then is the SEAL receipt release-published.

Consequently the trigger belongs to `(arm_fence, seal_fence]` and
`seal_fence` equals its sequence. Posting, polling, WorldControl ACK, and World
Statistics receipt times are not ledger fences. With exact-stamp mode, a
trigger later than the requested stamp seals with a stamp fault. No qualifying
write produces no SEALED state; the bounded coordinator times out and selects
`RESTART_REQUIRED`.

For VN-0011B, Parent posts ARM before the bounded paused probe. After a zero
controller update is independently proven at stamp `S`, it posts exact SEAL
for `S + fixed_step` and sends the one final
`{pause: true, multi_step: 1}`. Paused repeats at `S` stay inside the interval;
the post-update write at `S + fixed_step` is included and seals it. World
Statistics must independently prove exactly `N -> N+1` and re-paused state.

Any fault in the qualifying invocation still consumes its sequence and wins
the first terminal transition. A successful bank is non-empty, has
`invocation_count == seal_fence - arm_fence`, has contiguous segment ranges,
and satisfies the armed predicate for the whole interval.

Bank attempt metadata and stored tuple segments have distinct meanings.
`first_write_seq`, `last_write_seq`, and `invocation_count` cover every
contiguous included invocation, including observation, semantic, invocation
budget, and segment-capacity failures. A segment is created only for an
otherwise recordable tuple: the observation is `VALID`, the delegated result
fits the ABI, both command values are finite, the armed predicate passes, and
the invocation and segment budgets permit storage. Consequently a faulted
bank may contain no segments or strictly ordered, non-overlapping segment
ranges with gaps; a successful bank must have exact segment coverage whose
summed counts equal `invocation_count`. Identical tuples on opposite sides of
a gap never fold together.

Budgets are derived from the actual hardware `write()` cadence. In Gazebo that
cadence can be the 1 ms simulation step even when controller updates run at
100 Hz; sizing from the controller rate alone can create a false `CAPACITY`
terminal fault before SEAL.

A simulation-stamp regression is relational: its otherwise recordable tuple
has already been captured when comparison with the prior stored tuple detects
the fault. ABI v1 retains that offending tuple as a distinct, non-folded
segment and latches `SIM_STAMP`, so the bank can only become `SEALED_FAULT`.
Readers must not infer that segments in a faulted bank are monotonic or valid
proof; they are bounded forensic evidence governed by the sticky fault mask.

## Banks and ACK

There are exactly two banks with this state machine:

```text
FREE -> ACTIVE -> SEALED_OK | SEALED_FAULT -> FREE
```

At most one bank is ACTIVE. ARM chooses a FREE bank and increments its
non-wrapping `bank_epoch`. A terminal bank, its segments, fences, faults, and
checksum never change. Parent reads only after acquire-observing SEALED, copies
exactly the bounded segment count into a complete immutable page chain, then
rechecks the terminal state, full bank bytes, copied segments, epoch, and root
checksum before accepting it.

Exactly one Parent owns ACK. Inside that Parent, the complete immutable
snapshot is registered once and must be claimed before validation; an
in-flight claim excludes a duplicate ACK or another read of the same bank.
ACK then performs a release compare/exchange from the exact SEALED state to
FREE. Before that transition, Parent must have validated every page,
acquire-rechecked the unchanged terminal state and identity, and stopped all
reads. Its identity is `{generation, interval_id, bank_index, bank_epoch,
seal_fence, bank_checksum}`. Writer never mutates a SEALED bank, so those fields
remain stable until the successful state transition. Claim failure, validation
failure, or a failed single CAS cannot be retried with that snapshot; a new
complete read is required while the exact bank remains terminal. Reuse with a
newer epoch makes every old snapshot stale. Writer acquire-observes FREE before
resetting the bank. Two retained banks make a new ARM return `NO_FREE_BANK`.

## Fixed ABI v1

All structures contain only naturally aligned `uint64_t` words. Sizes are part
of the Interface and are checked by a C translation unit.

| Structure | Bytes | Purpose |
| --- | ---: | --- |
| region header | 192 | static layout/identity plus init, Writer claim, global sequence/fault state |
| control mailbox | 192 | one owned request envelope plus its bounded response |
| bank header | 128 | lifecycle, identity, fences, budgets, counts, predicate, faults, root checksum |
| segment | 64 | recordable tuple evidence: generation, sequence range/count, stamp, observation/result, exact command bits |
| snapshot page header | 192 | sealed identity, page/range/count chain and checksums |

The region layout is:

```text
header
control
bank[0] header + C segments
bank[1] header + C segments
```

Therefore `bank_stride = 128 + C * 64` and
`region_bytes = 384 + 2 * bank_stride`. `C` is the fixed segment capacity per
bank. The configured page segment limit is at most `C`; pages are immutable
copies outside the mapping and contain at most that limit.

The header's static words are magic, ABI version, endian tag, the five
structure sizes, region bytes, bank count, capacity, page limit, owner UID,
generation, a non-zero 128-bit nonce, and feature flags. Mutable init state,
Writer PID, last completed global sequence, and global faults follow. Reserved
words are zero and the final word is the header checksum. That checksum covers
all static words and zero reserved words, but excludes mutable publication
words and itself.

The control mailbox contains operation, flags, interval, exact bank identity
for SEAL, segment/invocation budgets, trigger stamp, request checksum, ticket,
and ownership state. Its response contains a bounded result, selected bank
identity, fence, consumed-request checksum, response checksum, and a final
release-published response ticket. Six reserved words remain zero. Each
checksum binds the header identity and ticket as well as its explicit fields;
no CRC is calculated over raw struct padding.

`INVALID_BANK_INDEX` is the public all-ones `uint64_t` sentinel used only when
no bank was selected. A canonical invalid response always pairs it with epoch
zero; valid bank indexes remain in `[0, BANK_COUNT)`.

The bank checksum excludes the atomic state word and itself, and covers every
other fixed bank word plus exactly the finalized segments. A page checksum
covers its header except the checksum word plus its copied segments; it binds
the bank checksum and previous-page checksum. CRC64-ECMA-182 consumes every
word least-significant byte first.

Shared publication words are plain aligned ABI fields accessed exclusively by
the Module's GCC/Clang `__atomic` shim. The build statically requires lock-free
eight-byte atomics. This is an explicit Linux/little-endian project ABI, not a
claim of portable ISO C++ interprocess atomics.

## Ownership and failure rules

The Parent creates one strict `/voice_nav_hardware_<16-lower-hex>` `0600` POSIX
object with `O_EXCL`, sizes and clears it, writes a non-zero 32-lower-hex nonce
plus static identity/layout, then release-publishes READY. Writer validates the
out-of-band name and nonce first, opens with `O_RDWR | O_CLOEXEC` but never
creates or unlinks, independently derives bounded capacity from the file size,
and checks exact UID/mode/link-count/inode/size before and after mapping. Only
after acquire-observing READY does it validate ABI geometry, identity, nonce,
reserved zeros, checksum, and two pristine FREE banks, then uniquely claims
its PID. Parent unlinks the name only after observing that claim; existing
mappings remain valid. READY freezes the region size and immutable header for
the mapping lifetime.

Writer death with ACTIVE or pending control makes that interval unusable.
Only a fully checksummed release-published sealed state is readable. Parent
death does not clear or ACK evidence. Tests terminate only their directly
owned child process and never scan or signal unrelated ROS/Gazebo processes.

The heap-backed `HardwareWriteLedger` remains a pure reference implementation
for focused folding, validation, pagination, and checksum tests. Runtime crash
evidence instead uses `AttachedHardwareWriteLedger` and
`HardwareWriteLedgerWriter` directly over the fixed-width shared mapping; no
heap-backed ledger object is placed in or copied around that mapping.
