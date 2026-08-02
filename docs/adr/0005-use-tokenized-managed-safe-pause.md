---
status: accepted
---

# Use a tokenized Managed Safe Pause policy

**Decision status:** Accepted

**Implementation status:** Planned in VN-0011B; this ADR does not claim that
the coordinator or runtime evidence already exists.

## Context

Jazzy `diff_drive_controller` computes command age from controller/simulation
time. Gazebo pause stops that clock, so pausing first can freeze a still-valid
non-zero command indefinitely. A positive Gazebo pause response confirms only
that a request was accepted; it does not prove zero motion or a committed
paused world.

The project also cannot prevent a trusted local operator from invoking Gazebo
Transport directly. Managed Safe Pause therefore needs a project-owned policy
boundary without pretending to be a transport security boundary.

## Decision

`voice_nav_bringup` will own an in-process `SafePauseCoordinator` with an
injected private Gazebo/control port. It will not add a public pause ROS
Interface, a new package, or a fifth resident process. VN-0011B delivers this
package-private coordinator with a test Adapter; the Adapter is the caller and
receives its result. It does not expose a user-facing product pause function.
A future lifecycle supervisor may host the product integration.

VN-0011B mints and resumes a token only while the same
`diff_drive_controller` generation remains ACTIVE. If zero proof needs
deactivation, or the controller becomes inactive/replaced, the transaction
returns `RESTART_REQUIRED`; activation/reconstruction is future supervisor
work rather than a second resume path.

The transaction is ordered as follows:

1. while simulation advances, inhibit MotionGate and prove Gate output,
   controller output, both wheel commands, wheel states, and odometry are zero
   for the configured continuous window;
2. request pause and confirm it from World Statistics plus a stable
   iteration/simulation-time observation, not from the request ACK alone;
3. only then mint an opaque, in-process, single-use token bound to partition,
   world, exact Gazebo process identity, paused iteration/time, controller
   generation/state/update stamp, fixed simulation step and controller period,
   Gate instance/control sequence/final-publisher identity, lossless-oracle
   generation/sealed zero-proof fence, and the zero-proof stamp/sequence;
4. managed resume validates and consumes that token without enabling
   continuous run. It performs bounded exact
   `{pause: true, multi_step: 1}` transactions, each followed by exact
   iteration/re-pause confirmation, until a new
   controller update is observed no later than
   `ceil(control_period / step_size) + 1` steps. That update's controller output
   and both command interfaces must be zero. It then performs one additional
   single step and requires the lossless hardware-write invocation to write
   those post-update zeros; only then may it send `pause:false`;
5. a missing, stale, replayed, or mismatched token returns
   `RESTART_REQUIRED`, never unpauses in place, and selects structured
   simulation/control generation shutdown.

At resume, the original Gate may either still be present, inhibited, and match
the bound instance/control/final-publisher identity, or it may have exited
after token creation with exact process-death evidence while the final command
topic has zero publishers. Any replacement Gate, changed instance or control
sequence, or new/different final publisher invalidates the token and returns
`RESTART_REQUIRED`.

The BEST_EFFORT asynchronous ros2_control introspection topic provides
mandatory sampled pre-pause command/state corroboration, but cannot prove that
an intermediate write was absent or that the very first resumed write was
zero. VN-0011B therefore uses a default-off test-only oracle at the actual
hardware-write seam. The oracle losslessly accounts for each `write()`
invocation, Gazebo iteration, generation, and both resulting
`JointVelocityCmd` values. Every invocation advances non-wrapping `write_seq`.
Only consecutive calls with identical generation, iteration/stamp, and exact
command bits may atomically extend one active accumulator; its first/last
sequence and invocation count must agree. A tuple change or `SEAL` finalizes
it, after which the segment is immutable; snapshots expose only finalized,
sealed data through immutable pages. Atomic
ARM/SEAL fences, a pre-armed bounded
iteration/transition segment budget, latched overflow/overwrite/unaccounted-
write or zero-window-nonzero failure, retained sealed intervals, and immutable
paged checksum/continuity validation make completeness explicit without
allocating one slot for every valid same-iteration repeat.
Without sending `pause:false` for continuous run, the coordinator repeatedly
requests `{pause: true, multi_step: 1}`. The explicit true value is mandatory:
Gazebo applies the pause field before processing the step, and protobuf's
default false would otherwise enter continuous run. A positive response is
only queued intent. Gazebo advances one iteration and re-pauses after each
request; World Statistics must confirm exactly `+1` and paused state every
time. Omitted/false pause or duplicate requests fail closed. The fixed step is
strictly smaller than the 100 Hz controller period. A new same-stamp
`/cmd_vel_out` plus complete introspection sample is
the fail-closed controller-update observation; missing, lossy, late, or
non-zero evidence selects `RESTART_REQUIRED`. The bound above guarantees that
one update must be encountered without assuming its phase.

`gz_ros2_control` writes command interfaces in every `PreUpdate`, then reads
and updates the controller in `PostUpdate` only when its period is due. The
update-step write therefore still reflects the pre-update zero. After the new
controller update is proven zero, one final exact
`{pause: true, multi_step: 1}` is required: its lossless invocation must be the
exact next iteration and write two zeros. A
non-zero update fails before that final write; retained pre-pause data, wrong
ordering/stamp, a gap, extra iteration, or a record beginning later also fails
closed. Paused runner loops may create multiple invocations at one iteration;
`write_seq` ranges are contiguous, identical repeats are count-preserving
segments, iteration is nondecreasing, each request causes one exact transition,
and every armed invocation through SEAL must be zero. Continuous
`pause:false` is allowed only after the post-update zero write. State and
odometry remain separate physical-response evidence. The
ordering is defined by the pinned
[Gazebo Sim 8 runner](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/SimulationRunner.cc)
and
[gz_ros2_control Jazzy plugin](https://github.com/ros-controls/gz_ros2_control/blob/jazzy/gz_ros2_control/src/gz_ros2_control_plugin.cpp).

Automatic relaunch belongs to a future supervisor. In VN-0011B,
`RESTART_REQUIRED` means the old generation is not resumed and is shut down
through the owned lifecycle; it does not claim that a replacement generation
has already been launched.

## Considered options

- **Pause first and rely on controller timeout:** rejected because controller
  time no longer advances.
- **Treat the pause service ACK as commit:** rejected because it does not prove
  World Statistics state or a zero command.
- **Persist or publish the token:** rejected because it expands the authority
  surface and permits stale cross-generation reuse.
- **Add a resident pause watchdog:** rejected because the transaction can stay
  behind an in-process deep module and the product topology remains smaller.
- **Resume any observed pause in place:** rejected because an Unmanaged Pause
  has no pre-pause zero proof and may retain a stale command.

## Consequences

- Managed Safe Pause is a two-phase, fail-closed operational transaction.
- Zero proof, pause commit, and resume commit have separate observation points.
- Token validation is deterministic and unit-testable with fake ports and a
  manual clock/generation source.
- Direct GUI/Transport unpause remains possible for a local operator; it is
  explicitly unsupported by the managed policy rather than falsely blocked.
- A failed proof requires old-generation shutdown and a future generation
  restart instead of risking command replay; this ADR does not claim automatic
  relaunch.

Implementation and acceptance are tracked by
[VN-0011](../work-items/0011-crash-stop-and-safe-pause.md).
