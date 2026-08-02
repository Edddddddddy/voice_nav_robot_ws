# VoiceNav Robot glossary

This is the canonical product language for v1.0.

## Language

**Voice Turn**:
One Wake Word-to-response interaction containing one final utterance and its
resulting decision or reply.
_Avoid_: Conversation, request

**Mission**:
A validated, ordered sequence of one to three semantic robot actions with one
terminal outcome.
_Avoid_: Script, job, raw command list

**Mission Step**:
One semantic action: move a distance, rotate an angle, navigate to a Named
Place, or save a map.
_Avoid_: Twist, controller command

**Named Place**:
A stable human-facing identifier resolved to a map pose only inside the trusted
navigation Implementation.
_Avoid_: Target string, raw pose

**Mapping Mode**:
The separately launched mode in which slam_toolbox owns `map → odom`, builds a
map, and may save it; global localization navigation is unavailable.
_Avoid_: SLAM session, online switch state

**Navigation Mode**:
The separately launched mode in which a saved map is loaded, AMCL owns
`map → odom`, and Nav2 accepts Named Place Missions; online map construction is
unavailable.
_Avoid_: AMCL mode, online switch state

**Motion Gate**:
The independent final-velocity authority implemented by `motion_gate_node`; it
enforces the Runtime-bound 250 ms authority lease, current candidate data-plane
binding, candidate freshness, limits, inhibition, and zero output. Raw velocity
samples never renew authority.
_Avoid_: Mission scheduler, LLM controller, Gazebo controller

**Operational Stop (运行停止)**:
A high-priority, idempotent request that makes the Motion Gate lock and publish
zero before ordinary Mission cleanup completes. Its ROS type is exactly
`StopMission.srv`. It is an operational simulation stop, not a
functional-safety emergency stop or a certified safety function.
_Avoid_: Emergency stop, e-stop, Safety Stop, pause, `OperationalStop.srv`

**Managed Safe Pause (托管安全暂停)**:
A package-private two-phase policy that proves Gate, controller, wheel, and
odometry zero while simulation advances, confirms the paused world, and only
then creates a Safe-Pause Token. VN-0011B initially proves it through a test
Adapter; it is not yet a user-facing product pause function.
_Avoid_: Managed pause, safe-pause as a generic state, Operational Stop

**Unmanaged Pause (非托管暂停)**:
A Gazebo pause not committed by the Managed Safe Pause transaction. It has no
Safe-Pause Token and cannot be resumed through the managed Adapter; the Adapter
returns `RESTART_REQUIRED` and selects old-generation shutdown.
_Avoid_: Unsafe pause, external stop, emergency pause

**Safe-Pause Token**:
An opaque, in-process, single-use proof bound to one paused
simulation/control/Gate generation and its zero evidence. It is not public ROS
IDL, a credential, or persistent storage.
_Avoid_: Pause ID, resume password, lease

**RESTART_REQUIRED**:
The package-private Managed Safe Pause outcome meaning the old generation must
not resume and must be structurally shut down. It does not mean that a
replacement generation has already been launched.
_Avoid_: Restarted, retry, auto-recovered

**Wake Word**:
The local phrase “小智” that opens an ordinary Voice Turn.
_Avoid_: Activation command

**Source Instance**:
One lifetime of an Agent or stop producer, identified so source sequences
cannot be replayed across restarts.
_Avoid_: Session, user

**Runtime Instance**:
One lifetime of `mission_runtime_node`, copied from `/mission/state` into a Goal
to fence work created for an earlier process.
_Avoid_: Node name, Mission ID

**Admission Epoch**:
A changing Runtime ownership generation copied from `/mission/state`; a Goal
built for an earlier generation is stale.
_Avoid_: Timestamp, Action goal UUID
