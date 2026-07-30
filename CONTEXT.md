# VoiceNav Robot

VoiceNav Robot turns a local Mandarin voice request into a constrained,
observable action performed by a simulated mobile robot.

## Language

**Voice Turn**:
One wake-to-response interaction containing a user utterance and its resulting
decision or reply.
_Avoid_: Conversation, request

**Mission**:
A validated, ordered sequence of one to three semantic robot actions that has
one terminal outcome.
_Avoid_: Script, job, raw command list

**Mission Step**:
One semantic action inside a Mission, such as moving a distance, rotating an
angle, navigating to a Named Place, or saving a map.
_Avoid_: Twist, controller command

**Named Place**:
A stable human-facing location identifier resolved to a map pose only inside
the trusted navigation implementation.
_Avoid_: Target string, raw pose

**Mapping Mode**:
The operating mode in which the robot builds and may save a map; global
localization navigation is unavailable.
_Avoid_: SLAM session

**Navigation Mode**:
The operating mode in which the robot loads a saved map, localizes, and accepts
navigation Missions; online map construction is unavailable.
_Avoid_: AMCL mode

**Motion Gate**:
The sole trusted authority that admits motion, applies limits and timeouts, and
can force the final velocity output to zero.
_Avoid_: LLM controller, Gazebo controller

**Safety Stop**:
A high-priority, idempotent request that closes the Motion Gate and requests
zero velocity before ordinary Mission cancellation finishes.
_Avoid_: Certified emergency stop, pause

**Wake Word**:
The local phrase “小智” that opens a Voice Turn.
_Avoid_: Activation command
