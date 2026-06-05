# ROS2 Multi-UAV Trajectory Execution & Analysis System

## 基于 ROS 2 与 PX4 SITL 的多无人机轨迹控制与仿真验证系统

本项目是一个基于 **Ubuntu 24.04 + ROS 2 Jazzy + PX4 SITL + Gazebo** 的多无人机编队控制与轨迹验证系统。系统采用 ROS 2 原生通信机制与 PX4 `px4_msgs` 接口，结合 Micro XRCE-DDS 运行环境，实现多架无人机在独立 namespace 下的 Offboard 位置控制、编队模式切换、轨迹平滑与仿真数据分析。

项目主要用于验证高层轨迹规划算法在 PX4 SITL 物理仿真环境中的执行效果，为后续接入学习型轨迹预测模型、速度/加速度前馈控制或更复杂的分布式编队控制算法提供工程测试基础。

## Core Features

### 1. ROS 2 多机 namespace 隔离

项目通过 launch 文件批量启动多架无人机控制节点，每架无人机运行在独立 namespace 下，例如 `/px4_1`、`/px4_2`、`/px4_3`。
相同的控制节点代码可以复用于不同无人机实例，避免多机系统中的 topic 串扰问题。

### 2. PX4 Offboard 控制链路

控制节点周期性发布 `OffboardControlMode` 和 `TrajectorySetpoint`，并通过 `VehicleCommand` 或 vehicle command service 完成解锁与 Offboard 模式切换。
系统当前采用位置 setpoint 控制，底层位置环、姿态环和电机控制由 PX4 内部控制器完成。

### 3. 多机编队模式切换

系统支持多种基础编队模式，包括：

* Hover 悬停
* Circle 环形轨迹
* V-shape V 字形编队
* Line 一字形编队
* AI trajectory interface 上层轨迹接口预留

Python 指挥节点通过全局 `/swarm_command` 话题向所有无人机广播模式编号，各无人机根据自身 ID 计算对应的编队目标点。

### 4. 一阶低通滤波轨迹平滑

针对阵型切换时目标 setpoint 突变导致的瞬态冲击问题，C++ 控制节点在目标点发布前加入一阶低通滤波：

```cpp
smooth_target += alpha * (target - smooth_target);
```

该方法可以降低编队切换过程中的目标突变，使轨迹过渡更加平滑。当前版本采用固定平滑系数，后续可进一步扩展为基于飞行状态或误差大小的自适应系数。

### 5. 仿真数据采集与轨迹分析

系统可结合 PX4 odometry 数据、rosbag2 或日志文件，对期望轨迹与实际轨迹进行离线分析。
当前实验已完成 5 机集群起飞、悬停、环形轨迹与 V 字形编队切换验证，并基于仿真数据完成三维轨迹可视化与误差统计。

## Evaluation

在 2 m/s 环形轨迹测试中，系统完成了多机轨迹跟踪与编队切换实验。根据离线轨迹分析，得到以下指标：

* Max transient error: 3.461 m
* Steady-state RMSE: 1.536 m

误差主要来自纯位置 setpoint 跟踪下的响应滞后、阵型切换阶段的目标突变以及 PX4 底层控制器的动态响应延迟。该结果为后续引入速度/加速度前馈、MPC 或学习型轨迹预测模型提供了 baseline。

## Tech Stack

* OS: Ubuntu 24.04
* Middleware: ROS 2 Jazzy, Micro XRCE-DDS Agent
* Flight Controller: PX4 Autopilot SITL
* Simulation: Gazebo
* Language: C++, Python
* Data Analysis: rosbag2, pandas, matplotlib, numpy

## Quick Start

```bash
# 1. Build ROS 2 workspace
cd ~/ros2_ws
colcon build --packages-select custom_msgs px4_swarm_controller
source install/local_setup.bash

# 2. Start Micro XRCE-DDS Agent
MicroXRCEAgent udp4 -p 8888

# 3. Start PX4 multi-UAV SITL
cd ~/PX4-Autopilot
./start_swarm.sh

# 4. Launch ROS 2 control nodes
ros2 launch px4_swarm_controller launch_simulation.py

# 5. Send formation command
python3 swarm_commander.py
```

## Current Limitations

* 当前版本主要采用位置 setpoint 控制，尚未直接控制姿态、角速度或电机执行器。
* AI trajectory interface 当前以目标点输入为主，后续可扩展为带时间戳的轨迹序列或 B-spline 控制点。
* 低通滤波方法简单稳定，但会引入一定相位滞后，后续可引入速度/加速度前馈或 MPC 优化。
* 多机编队目前以几何队形切换为主，尚未加入完整的在线避障和碰撞约束。
