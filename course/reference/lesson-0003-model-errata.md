# Lesson 0003 验收实现勘误

**性质：** 历史课程勘误，不是新的作业契约。

Lesson 0003 的原 HTML 作业要求方箱底盘、`0.075 m` 轮半径以及
`left_wheel_link` / `right_wheel_link`。学习者最终通过验收、随后进入
Gazebo 课程并保留在当前 `main` 的实现不同：

| 项目 | 原始作业 | 已验收实现 |
| --- | --- | --- |
| 底盘 | `0.40 × 0.28 × 0.12 m` 方箱 | 半径 `0.20 m`、高 `0.18 m` 的圆柱 |
| 轮子 | 半径 `0.075 m`、宽 `0.04 m` | 半径 `0.035 m`、宽 `0.025 m` |
| 轮心 y | `±0.16 m` | `±0.20 m` |
| `base_link` 高度 | `0.075 m` | `0.035 m` |
| wheel link | `left_wheel_link` / `right_wheel_link` | `left_wheel` / `right_wheel` |
| `base_footprint → laser_link` | 原作业未记录数值验收 | `[0.100, 0.000, 0.195]` |

验收记录、RViz 截图和 `check_urdf` 输出都对应右列。因此：

- [Lesson 0003](../lessons/0003-build-static-robot-model.md) 保留原始作业，
  不把后来的实现伪装成最初要求；
- [学习记录](../records/0003-building-static-robot-tf.md) 与当前源码保留
  实际验收事实；
- 当前几何与 TF 速查见 [TF Frame Contract](tf-frame-contract.md)；
- 原 HTML 和原始 Git 对象仍可从 VN-0007 记录的外部 bundle 恢复。

这份勘误使“当时要求什么”和“最后验收了什么”同时可审计。
