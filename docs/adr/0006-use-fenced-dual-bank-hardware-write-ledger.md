---
status: accepted
---

# Use a fenced dual-bank hardware-write ledger

**Decision status:** Accepted

**Implementation status:** In progress in VN-0011A; VN-0011B consumes the
same protocol but is not delivered by this ADR.

## Context

The crash and Managed Safe Pause proofs need every actual
`GazeboSimSystemInterface::write()` invocation, not a sampled ROS topic. The
test parent and the Gazebo hardware Adapter are separate processes. The Writer
path must be preallocated and non-blocking, while sealed evidence must survive
until the parent validates and acknowledges it.

Managed Safe Pause adds a decisive ordering constraint. While Gazebo is
paused, the final permitted `{pause: true, multi_step: 1}` must both write the
post-controller-update zeros and finish the proof interval. A SEAL operation
that closes at the next write entry would exclude that required write; sending
another step only to close the ledger would violate the resume contract.

## Decision

Use one deep Hardware-Write Ledger Module with two roles at one POSIX
shared-memory seam:

- the Writer role owns global sequence assignment and exposes only
  `begin_write(sim_stamp)` and
  `finish_write(ticket, delegated_result, wheel_observation)` to the Gazebo
  Adapter;
- the Parent role posts one checksummed ARM or SEAL request at a time, polls
  its receipt, reads sealed snapshot pages, and acknowledges an exact sealed
  identity.

The Adapter supplies observations but never supplies generation, interval,
bank, or sequence values. Every enabled upstream write obtains exactly one
non-wrapping sequence, even when the ledger is unarmed or the wheel observation
is missing. Such failures become explicit terminal evidence instead of a
silent return.

ARM linearizes at a write entry before that call receives its sequence. SEAL
is deferred and inclusive: the first call whose simulation stamp reaches the
request's threshold is delegated and observed, then appended and finalized,
and only then is the bank release-published as SEALED. The successful interval
is `(arm_fence, seal_fence]`; the triggering write is included. VN-0011B uses
the exact-stamp flag, so jumping over the expected final-step stamp produces a
faulted sealed interval.

The mapping has two fixed banks. At most one is ACTIVE. `SEALED_OK` and
`SEALED_FAULT` banks are immutable until the Parent finishes all reads and
release-ACKs the exact generation, interval, bank epoch, fence, and checksum.
The Writer may reuse only a FREE bank observed with acquire semantics. If no
bank is free, ARM returns `NO_FREE_BANK`; evidence is never overwritten.

The version-1 ABI uses naturally aligned, fixed-width `uint64_t` words on the
project's Linux little-endian GCC/Clang target. Shared synchronization fields
remain plain ABI words and are accessed only through one `__atomic` shim after
proving lock-free eight-byte atomics. `_Atomic`, `std::atomic`, pointers,
containers, and packed structs never appear in the mapping. CRC64-ECMA-182
binds static identity, control messages, sealed banks, and chained pages. C
size/alignment/offset tests and an independent Python reader are part of the
Interface contract.

The fixed ABI and linearization details are normative in
[Hardware-write ledger protocol](../architecture/hardware-write-ledger-protocol.md).

## Considered options

- **SEAL at the next write entry:** rejected because it excludes the one
  post-update write that VN-0011B must prove.
- **Send an extra paused step after SEAL:** rejected because resume permits one
  final proof step, not a bookkeeping step.
- **Use DDS or an overwrite ring:** rejected because neither proves complete,
  retained, gap-free evidence.
- **Wrap the heap-backed pure ledger and copy it at SEAL:** rejected because
  the process boundary, retention state, and crash-visible publication remain
  outside the Module and the Adapter would still own protocol rules.
- **Put C++ atomics or objects in `mmap`:** rejected because their language ABI
  and object-lifetime rules are not the cross-language storage contract.

## Consequences

- Control, sequence, folding, terminal fault, retention, checksum, and ACK
  rules have one owner and one test surface.
- The POSIX attachment Adapter validates the exact object before constructing
  the Writer view; normal product composition does not enable this seam.
- A Writer death with ACTIVE or pending state invalidates that interval. A
  release-published sealed bank remains readable; Parent death never implies
  ACK.
- ABI changes after v1 require a new version/layout, not reinterpretation of an
  existing mapping.

Implementation and acceptance are tracked by
[VN-0011A](../work-items/0011a-process-death-crash-stop.md).
