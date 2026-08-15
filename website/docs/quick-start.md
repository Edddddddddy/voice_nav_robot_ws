# 运行项目

VoiceNav 的支持基线不是通用 Linux 桌面环境，而是一条明确、可复现的组合：Windows 11 主机、WSL2 Ubuntu 24.04、
ROS 2 Jazzy 与 Gazebo Harmonic。项目当前只面向仿真，不需要真实底盘、摄像头或云端账号。

<span class="vn-badge vn-badge--verified">当前可运行</span>
<span class="vn-badge vn-badge--boundary">仅支持 WSL2 仿真</span>

## 环境基线

| 层级 | 支持值 | 说明 |
| --- | --- | --- |
| Host | Windows 11 | 使用 WSL2 承载 ROS 环境 |
| Linux | Ubuntu 24.04 | 与 Jazzy 的支持周期一致 |
| ROS | ROS 2 Jazzy | 项目锁定的发行版 |
| Simulator | Gazebo Harmonic | 通过 `gz_ros2_control` 驱动模型 |
| RMW | `rmw_fastrtps_cpp` | MotionGate 的受支持实现边界 |

!!! warning "先确认项目边界"
    当前仓库不支持真实机器人硬件，也不提供功能安全急停。运行中的 “Stop” 是仿真 Operational Stop。

## 获取与构建

在已经安装 ROS 2 Jazzy 和 Gazebo Harmonic 的 WSL2 终端中执行：

```bash
git clone https://github.com/Edddddddddy/voice_nav_robot_ws.git
cd voice_nav_robot_ws

source /opt/ros/jazzy/setup.bash
rosdep check --from-paths src --ignore-src
colcon build --symlink-install --event-handlers console_direct+
source install/setup.bash
```

`package.xml` 是 ROS 依赖的权威来源。`rosdep check` 报缺依赖时，先按 Jazzy 的官方安装流程补齐，
不要通过删依赖、跳过 package 或更换 ROS 发行版来“让构建通过”。

## 启动当前产品组合

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch voice_nav_bringup product_sim.launch.py headless:=false
```

`product_sim.launch.py` 会组合：

1. Gazebo 仿真、机器人模型、LiDAR、controller 与 ROS bridge；
2. `motion_conditioning_container`；
3. 独立的 `motion_gate_node`；
4. `mission_runtime_node`。

默认 `headless:=true`，适合测试；希望看到 Gazebo 界面时显式设为 `false`。canonical launch 会设置
`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`，因为 MotionGate 的 writer 绑定证明只在这一支持边界内成立。

## 先观察，再发任务

新终端中 source workspace 后，可以先检查 Runtime 最新状态：

```bash
ros2 topic echo /mission/state --once
ros2 action info /mission/execute
ros2 service type /mission/stop
```

`/mission/state` 是 `RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1)` 的只读快照。只有当 Runtime、Gate 和依赖都健康时，
availability 才会进入 `AVAILABLE`；Gate 在没有活动步骤时应保持 inhibited。

!!! tip "建议的学习顺序"
    先读[系统总览](architecture/overview.md)，再看 [Mission Runtime](architecture/mission-runtime.md) 和
    [运动安全链](architecture/motion-safety.md)。公共 field 与结果码集中在[公共 ROS 接口](reference/interfaces.md)。

## 停止与清理

每次 motion test 都必须在正常路径和清理路径请求零速度，并观察 odometry 进入静止。不要只关闭 Gazebo 窗口来代替
Operational Stop；也不要把 controller timeout 当成“实体已经停止”的证明。

页面事实依据：[`README.md`](https://github.com/Edddddddddy/voice_nav_robot_ws/blob/main/README.md)、
[`product_sim.launch.py`](https://github.com/Edddddddddy/voice_nav_robot_ws/blob/main/src/voice_nav_bringup/launch/product_sim.launch.py)、
[`scripts/verify.sh`](https://github.com/Edddddddddy/voice_nav_robot_ws/blob/main/scripts/verify.sh)。
