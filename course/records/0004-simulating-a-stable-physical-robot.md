# Lesson 0004 学习记录：创建并诊断稳定的 Gazebo 物理模型

状态：Completed

学习者为 Xacro 加入 collision、正质量、与形状匹配的惯量、关节限制和 dynamics，并保持 `base_footprint` 为纯 frame。`gz sdf -p` 捕获了 `check_urdf` 未覆盖的 invalid inertia 问题。

## 验收证据

- Xacro、URDF、SDF、依赖与测试检查全部完成。
- `gz model -m voice_nav_robot -p` 显示机器人落地后姿态接近零并保持稳定。
- Gazebo 截图显示完整机器人与地面，无持续漂移、翻倒、穿透或无效姿态。
- 能分别解释 visual、collision、inertial，以及质心略向后布置的支撑几何原因。

后续课程可以假设学习者理解基本惯量有效性、支撑区域、URDF-to-SDF 转换和稳定生成流程。
