# URDF 物理属性速查

VoiceNav Robot reference

## 一个 link 的三种描述

| 元素 | 回答的问题 | 常见错误 |
| --- | --- | --- |
| visual | 人看见什么？ | 把复杂显示网格同时拿去做碰撞。 |
| collision | 物体在哪里接触和阻挡？ | 漏写后模型会穿过地面或其他物体。 |
| inertial | 质量、质心和转动惯量是多少？ | 质量或惯量为零、数量级不合理、质心落在支撑面外。 |

三者的 `origin` 都相对当前 link frame，但可以不同。例如，圆柱底盘的外形保持居中，内部电池却可让质心稍微后移。

## 本项目用到的惯量公式

下式都以物体质心为原点，`m` 为质量，`r` 为半径，`h` 为 z 轴圆柱长度，`w` 为 y 轴轮子的宽度。

| 形状与轴向 | 主惯量 |
| --- | --- |
| 实心圆柱，轴沿 z（底盘、雷达） | `Ixx = Iyy = m(3r²+h²)/12`，`Izz = mr²/2` |
| 实心圆柱，轴沿 y（左右轮） | `Iyy = mr²/2`，`Ixx = Izz = m(3r²+w²)/12` |
| 实心球（万向支撑轮） | `Ixx = Iyy = Izz = 2mr²/5` |

## 当前模型基线

- 底盘：z 轴圆柱，半径 `0.20 m`、高度 `0.18 m`。
- 左右轮：y 轴圆柱，半径 `0.035 m`、宽 `0.025 m`，轮心 y 为 `±0.20 m`。
- 后置支撑球：半径 `0.045 m`，关节 x 为 `-0.13 m`。
- LiDAR：z 轴圆柱，半径 `0.04 m`、高度 `0.05 m`。

## 稳定性检查

- 所有质量和对角惯量必须大于零，且不能用极小值糊弄解析器。
- 主惯量必须满足三角不等式：`Ixx + Iyy ≥ Izz`、`Ixx + Izz ≥ Iyy`、`Iyy + Izz ≥ Ixx`。
- 对称基本形状的 `ixy`、`ixz`、`iyz` 可取 0。
- 整车质心在地面的投影应位于轮子接触点围成的支撑多边形内。
- 本车驱动轮轴位于 x=0、caster 位于 x<0，因此主车身质心略偏向 x<0。
- `base_footprint` 只是坐标 frame，不是实体，不为它伪造质量或碰撞体。

启动 Gazebo 前可用下面的命令做快速物理解析检查：

```bash
gz sdf -p /tmp/voice_nav_robot/model.urdf \
  > /tmp/voice_nav_robot/model.sdf
```

命令应安静退出并返回 0；`check_urdf` 通过不代表惯量一定物理有效。

资料：[ROS 2：Adding physical and collision properties to a URDF model](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Adding-Physical-and-Collision-Properties-to-a-URDF-Model.html)。
