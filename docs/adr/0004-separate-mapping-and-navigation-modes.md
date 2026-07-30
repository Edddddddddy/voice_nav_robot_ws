---
status: accepted
---

# Launch Mapping and Navigation as separate modes

VoiceNav Robot launches Mapping Mode with slam_toolbox and Navigation Mode with
map server, AMCL, and Nav2. It does not switch online in v1.0, because separate
process compositions make `map → odom` ownership, lifecycle, map handoff,
failure recovery, and acceptance evidence deterministic.

## Considered options

- run mapping and localization together and switch owners dynamically;
- keep one launch with runtime lifecycle transitions;
- stop one bounded mode and start the other with an explicit saved map.

## Consequences

slam_toolbox and AMCL never own `map → odom` simultaneously. Mode transition
requires Operational Stop, process shutdown, TF-owner disappearance, saved-map
selection, and a new Runtime instance/epoch snapshot. Online mapping while
navigating is outside v1.0.
