# Lesson 0003 学习记录：构建静态机器人与 TF 树

状态：Completed

学习者手写了差速机器人 Xacro、显示 launch 和 package 安装规则，在 RViz 中检查模型与 TF，并理解 joint origin 相对父 frame，而不是相对地面。

## 验收证据

- `check_urdf`：根 link 为 `base_footprint`，子树包含 `base_link`、`left_wheel`、`right_wheel`、`caster_link`、`laser_link`。
- `tf2_echo base_footprint laser_link`：平移 `[0.100, 0.000, 0.195]`，旋转为单位四元数。
- RViz 截图显示 RobotModel 与 TF 正常。
- `voice_nav_sim` 的 12 个测试通过。

后续课程可以假设学习者已掌握 URDF link/joint、TF、Xacro、launch substitution 与运行资产安装。
