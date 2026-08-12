---
status: accepted
---

# 将 Mapping 与 Navigation 作为独立模式启动

VoiceNav Robot 在 Mapping Mode 中启动 `slam_toolbox`，在 Navigation Mode 中启动 map server、AMCL 和
Nav2。v1.0 不做在线切换，因为独立 process composition 使 `map → odom` ownership、lifecycle、
map handoff、failure recovery 和 acceptance evidence 都具备确定性。

## 考虑过的方案

- 同时运行 mapping 与 localization，并动态切换 owner；
- 保留一个 launch，通过 runtime lifecycle transition 切换；
- 停止一个有界模式，用显式保存的地图启动另一个模式。

## 后果

`slam_toolbox` 与 AMCL 永远不会同时拥有 `map → odom`。模式转换要求 Operational Stop、process
shutdown、TF owner 消失、saved-map 选择，以及一个新的 Runtime instance/epoch snapshot。导航期间在线
建图不属于 v1.0。
