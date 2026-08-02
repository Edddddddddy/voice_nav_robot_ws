---
status: accepted
---

# Give each hardware-ledger request exclusive publication ownership

The first fixed control mailbox release-published only a ticket while its
ordinary request words remained reusable. A Parent retry could therefore
mutate those words while Writer was copying them, and storing the same ticket
value again did not create a safe new publication. Before ABI v1 release, the
control area will instead use one owned request envelope:
`IDLE -> WRITING -> READY -> READING -> IDLE`. Parent claims `WRITING`, fills
the fixed request, and release-publishes `READY`; Writer claims `READING`,
copies one local snapshot, and release-publishes `IDLE` before processing only
that snapshot. The release-published response ticket remains the receipt
linearization point, and the response stores the checksum of the consumed
snapshot rather than reading a mutable request word.

A ring was rejected because FIFO queueing conflicts with the protocol's one
outstanding request and fail-fast admission. Two request slots were safe but
added unused selection and lifecycle state. The single envelope keeps the
Parent and Writer Interfaces unchanged, permits an exact same-ticket retry as
a new ownership handoff, and grows the pre-v1 control area from 128 to 192
bytes. Parent still may not publish a different ticket before the matching
response; the envelope state protects memory ownership, not a hidden queue.

