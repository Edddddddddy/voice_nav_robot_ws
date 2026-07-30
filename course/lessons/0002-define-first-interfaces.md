# 定义第一个 Mission Interface

Lesson 0002 · 目标时间 30–45 分钟

**本课唯一成果：** 手写 `MissionStep.msg` 和 `ExecuteMission.action`，完成代码生成，并能用 ROS CLI 查看生成后的 Interface。

## 设计限制

这次只写 Interface，不写 Action server、client、Guard 或业务节点。

- LLM 只能表达语义动作，不能指定速度、加速度或控制器话题。
- STOP 不属于普通 MissionStep；它以后走独立的 operational stop（`StopMission`），本项目不宣称这是经过功能安全认证的急停。
- Result 的程序逻辑读取稳定错误码，不解析人类可读字符串。

## 1. MissionStep.msg

在 `voice_nav_interfaces/msg/MissionStep.msg` 中定义以下内容，具体 ROS IDL 语法由你手写：

| 类别 | 要求 |
| --- | --- |
| uint8 常量 | MOVE_DISTANCE=1、ROTATE_ANGLE=2、NAVIGATE_TO=3、SAVE_MAP=4 |
| 动作类型 | uint8 kind |
| 相对移动 | float32 distance_m |
| 相对旋转 | float32 angle_rad |
| 逻辑目标 | string target_id |

## 2. ExecuteMission.action

在 `voice_nav_interfaces/action/ExecuteMission.action` 中定义 Goal、Result、Feedback 三段。

### Goal

- `string source_instance_id`
- `uint64 source_seq`
- `string session_id`
- `string turn_id`
- 同 package 的可变长 `MissionStep` 数组 `steps`

### Result

定义以下 `uint16` 常量：

```
SUCCEEDED=0
INVALID_PLAN=10
BUSY=11
MODE_MISMATCH=12
UNKNOWN_TARGET=13
DEPENDENCY_UNAVAILABLE=20
EXECUTION_FAILED=21
TIMEOUT=22
CANCELED=30
STOPPED=31
INTERNAL_ERROR=99
```

再定义：

- `uint16 code`
- `int32 failed_step`，其中 `-1` 表示没有具体失败步骤
- `string detail`，只用于诊断，调用方不得解析

### Feedback

定义 `uint8` 常量 `VALIDATING=1`、`EXECUTING=2`、`SAFE_STOPPING=3`，再定义：

- `uint8 phase`
- `uint32 step_index`
- `float32 progress`

## 3. 接入 rosidl

自行修改 `CMakeLists.txt`，完成：

- 查找 `rosidl_default_generators`。
- 调用 `rosidl_generate_interfaces` 注册两个文件。
- 导出 `rosidl_default_runtime`。

自行修改 `package.xml`，加入：

- `rosidl_default_generators` 的 build dependency。
- `rosidl_default_runtime` 的 exec dependency。
- `rosidl_interface_packages` group membership。

## 4. 构建与检查

```
cd /mnt/c/Users/lcy/code/ros2/voice_nav_robot_ws
source /opt/ros/jazzy/setup.bash

colcon build --packages-select voice_nav_interfaces \
  --symlink-install --event-handlers console_direct+

source install/setup.bash

ros2 interface show voice_nav_interfaces/msg/MissionStep
ros2 interface show voice_nav_interfaces/action/ExecuteMission

rosdep check --from-paths src/voice_nav_interfaces --ignore-src

colcon test --packages-select voice_nav_interfaces
colcon test-result --verbose
```

**验收：** 两个 `ros2 interface show` 命令成功，`rosdep check` 报告所有系统依赖均已满足，测试没有失败；生成的 Action 必须包含 Goal、Result 和 Feedback 三部分。

## 提交给教师

1. 两个 Interface 文件的完整内容。
1. `CMakeLists.txt` 和 `package.xml` 的完整内容。
1. 两个 `ros2 interface show` 的输出。
1. `rosdep check` 输出。
1. `colcon test-result --verbose` 输出。
1. 回答：为什么速度和超时不应该由 LLM 写入 MissionStep？

## 主要资料

先阅读 [ROS 2 Jazzy：Interfaces](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Interfaces.html) 中的 Messages 和 Actions。需要复习选型时查看 [ROS Interface 选型速查](../reference/ros-interface-selection.md)。遇到任何不理解的 rosidl 语法，先停下来问教师。
