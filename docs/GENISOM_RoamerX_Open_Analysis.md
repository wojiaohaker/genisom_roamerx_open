# GENISOM RoamerX Open 项目深入分析

> 分析日期：2026-07-22 | 源码版本：当前 main 分支

---

## 目录

1. [项目概览](#1-项目概览)
2. [技术栈](#2-技术栈)
3. [目录结构](#3-目录结构)
4. [模块一：接口层（interface）](#4-模块一接口层interface)
5. [模块二：导航栈（navigation）](#5-模块二导航栈navigation)
6. [模块三：SLAM（slam）](#6-模块三slam)
7. [模块四：定位（localization）](#7-模块四localization)
8. [构建系统](#8-构建系统)
9. [启动脚本与运维工具](#9-启动脚本与运维工具)
10. [通信架构](#10-通信架构)
11. [ROS 2 话题与服务总览](#11-ros-2-话题与服务总览)
12. [与 MATRiX 仿真器的集成](#12-与-matrix-仿真器的集成)
13. [配置参数详解](#13-配置参数详解)
14. [数据流与导航管线](#14-数据流与导航管线)
15. [代码统计](#15-代码统计)
16. [关键设计模式与架构特点](#16-关键设计模式与架构特点)
17. [已知限制与待完善项](#17-已知限制与待完善项)
18. [总结](#18-总结)

---

## 1. 项目概览

**GENISOM RoamerX Open** 是 GENISOM-AI 机器人的开源导航栈，基于 ROS 2 Nav2 框架深度定制，提供从 SLAM 建图、定位到自主路径规划和避障的完整导航能力。

**核心定位：**
- 作为 MATRiX 仿真器（UE5 + MuJoCo）的上层导航决策模块
- 支持 Gazebo 仿真和真实硬件（NX_XG3588、XG3588）部署
- 提供社区免费版（Open），鼓励外部贡献

**关键能力：**
- 基于行为树的导航任务编排
- MPPI（Model Predictive Path Integral）控制器实现动态路径跟踪
- NavFn 全局路径规划
- 2D costmap 障碍物检测与膨胀
- 多航点巡逻导航
- LiDAR+IMU 融合 SLAM 建图
- 基于 NDT_OMP / Fast GICP 的点云配准定位

---

## 2. 技术栈

| 层次 | 技术 |
|------|------|
| **操作系统** | Ubuntu 22.04 LTS |
| **中间件** | ROS 2 Humble Hawksbill |
| **DDS 实现** | rmw_zenoh_cpp（Zenoh DDS，非默认 FastDDS） |
| **导航框架** | Nav2（深度定制，包名以 `navigo_` 前缀重命名） |
| **路径规划** | NavFn（A* / Dijkstra） |
| **路径跟踪** | MPPI Controller（模型预测路径积分） |
| **行为树** | BehaviorTree.CPP v3 |
| **SLAM** | 基于 FAST-LIO2 的 LiDAR-Inertial 里程计 + 建图 |
| **定位** | NDT_OMP / Fast GICP 点云配准 + UKF 滤波 |
| **通信协议** | UDP（直连低延迟）/ LCM（分布式多进程） |
| **点云处理** | PCL 1.12.1 |
| **数学库** | Eigen 3.4.0 |
| **运动规划** | OMPL |
| **可视化** | RViz2 |
| **构建工具** | colcon + CMake |
| **语言** | C++17（核心）+ Python 3（脚本/Launch） |
| **仿真集成** | MATRiX（UE5 + MuJoCo）/ Gazebo |

---

## 3. 目录结构

```
genisom_roamerx_open/
├── build.sh                    # 构建入口脚本
├── format.sh                   # 代码格式化（clang-format）
├── LICENSE
├── README.md                   # 主文档
├── map/
│   ├── map.yaml                # 导航地图配置（pgm 格式）
│   └── map.pgm                 # 2D 占用栅格地图
├── script/
│   ├── bash/
│   │   ├── start_navigation.sh # 导航启动脚本（nav/rviz/all/stop/print）
│   │   ├── stop_navigation.sh  # 导航停止脚本（清理所有子进程）
│   │   └── cleanup_backend.sh  # Docker 环境清理脚本
│   └── dep/
│       ├── install_all.sh      # 依赖安装入口
│       ├── ros2_dep.sh         # ROS 2 依赖（controller, PCL, OMPL, BT, Nav2）
│       ├── gazebo_dep.sh       # Gazebo 仿真依赖
│       └── gamepad_dep.sh      # 手柄依赖
├── src/
│   ├── interface/
│   │   └── robots_dog_msgs/    # 消息/服务/动作定义包
│   │       ├── msg/            # 50+ 消息类型
│   │       ├── srv/            # 14 个服务类型
│   │       └── action/         # 12 个动作类型
│   ├── navigation/src/
│   │   ├── navigo_bt_navigator/       # 行为树导航器
│   │   ├── navigo_mppi_controller/    # MPPI 控制器
│   │   ├── navigo_costmap_2d/         # 2D 代价地图
│   │   ├── navigo_path_planner/       # 全局路径规划器
│   │   ├── navigo_navfn_planner/      # NavFn 规划算法
│   │   ├── navigo_path_controller/    # 路径跟踪控制器
│   │   ├── navigo_behaviors/          # 导航行为（spin, backup, wait）
│   │   ├── navigo_behavior_tree/      # 行为树节点库
│   │   ├── navigo_collision_monitor/  # 碰撞监测
│   │   ├── navigo_map_server/         # 地图服务
│   │   ├── navigo_waypoint_follower/  # 多航点跟随
│   │   ├── navigo_velocity_optimizer/ # 速度平滑优化
│   │   ├── navigo_core/               # 导航核心接口
│   │   ├── navigo_util/               # 工具库
│   │   └── robot_navigo/              # 集成启动包（launch, params, 节点）
│   ├── slam/src/
│   │   ├── config/             # SLAM 配置
│   │   ├── include/            # 头文件（FAST-LIO2, ikd-tree, MTK/IEKF）
│   │   ├── launch/             # Launch 文件
│   │   ├── src/                # 源码（common, ikd_tree, process）
│   │   └── test/               # 测试
│   └── localization/
│       ├── fast_gicp/          # Fast GICP 点云配准
│       ├── ndt_omp/            # NDT-OMP 点云配准
│       └── localization/       # 定位融合节点
└── demo_gif/                   # 演示 GIF 图片
```

---

## 4. 模块一：接口层（interface）

### 4.1 robots_dog_msgs

**路径：** `src/interface/robots_dog_msgs/`

这是整个导航栈的消息定义基础包，使用 `rosidl` 接口定义语言，定义了机器人导航所需的全部消息、服务和动作类型。

#### 消息类型（msg/）— 50+ 个

| 类别 | 消息名 | 用途 |
|------|--------|------|
| **导航状态** | `NavigationState`, `NavigationError`, `NavigationErrorReport`, `NavigationErrorSet` | 导航任务状态与错误报告 |
| **高层指令** | `HighCmd`, `HighState`, `HighLevelCmd`, `HighLevelRobotState` | 机器人高层控制指令与状态 |
| **底层控制** | `LowCmd`, `LowState`, `LowLevelCmd`, `LowLevelRobotState` | 电机级控制与反馈 |
| **传感器数据** | `IMU`, `Localization`, `BmsState`, `BmsCmd` | IMU、定位、电池管理 |
| **电机** | `MotorCmd`, `MotorState` | 单个电机指令/状态 |
| **腿控** | `LegControlData`, `Cartesian` | 腿机器人专用控制 |
| **地图/障碍物** | `ElectronicMap`, `ElectronicObstacle`, `ObstacleInstance2D`, `ObstacleScan` | 环境感知 |
| **路径/轨迹** | `Trajectory`, `TrajectoryPoint`, `PredictedPath`, `PredictedPathArray`, `PredictedPose` | 路径规划结果 |
| **SLAM** | `SlamState`, `Frontier` | 建图状态与探索前沿 |
| **系统监控** | `CpuInfo`, `DiskInfo`, `NetInfo`, `ProcessInfo`, `MonitorInfo`, `FaultInfo` | 系统健康监控 |
| **传感器包** | `RslidarPacket`, `CustomMsg`, `CustomPoint` | 原始传感器数据 |
| **其他** | `Pose6D`, `RectangleArray`, `RectangleWithCentre`, `StartNavigation`, `UniBestNav`, `UniHeading`, `UniRtkPvh`, `UwbPointStamped`, `Nmea`, `LED`, `LinktrackAoaNode0`, `PlannerDebugInfo`, `CmdVelWithTrajectory`, `Relocalization` | 各类辅助消息 |

#### 服务类型（srv/）— 14 个

| 服务名 | 用途 |
|--------|------|
| `MapState` | SLAM 建图状态控制（开始/保存） |
| `LoadMap` | 加载 PCD/YAML 地图 |
| `LocalizationState` | 定位状态查询 |
| `SetLocalizationType` | 切换定位算法 |
| `MappingType` | 切换建图模式 |
| `UseMapType` | 地图类型切换 |
| `GetOptimizedPath` | 获取优化路径 |
| `GetFrontierCentroids` | 获取探索前沿中心 |
| `IsPathValid` | 路径有效性检查 |
| `IsTargetInFov` | 目标是否在视野内 |
| `IsExplorationGoalsValid` | 探索目标有效性 |
| `IsExplorationGoalValid` | 单个探索目标有效性 |
| `RecordKeyPose` | 记录关键位姿 |
| `RecordPath` | 记录路径 |

#### 动作类型（action/）— 12 个

| 动作名 | 用途 |
|--------|------|
| `NavigateToPose` | 导航到单个目标位姿 |
| `NavigateThroughPoses` | 穿越多个目标位姿 |
| `FollowPath` | 跟随给定路径 |
| `FollowWaypoints` | 跟随航点 |
| `ComputePathToPose` | 计算到目标的路径 |
| `ComputePathsToPoses` | 计算到多目标的路径 |
| `ComputePathThroughPoses` | 计算穿越路径 |
| `ExploreThroughPoses` | 探索式穿越导航 |
| `BackUp` | 后退行为 |
| `ProcessInfo` | 进程信息查询 |

---

## 5. 模块二：导航栈（navigation）

**路径：** `src/navigation/src/`

这是项目最核心的模块，基于 Nav2 架构深度定制，所有包名使用 `navigo_` 前缀以区分上游 Nav2。

### 5.1 架构层次

```
robot_navigo（集成启动层）
    │
    ├── navigation_bringup.launch.py  → 入口 Launch
    │       │
    │       ├── bringup_launch.py     → 容器 + 生命周期管理
    │       │       │
    │       │       └── navigation_launch.py → 启动所有导航节点
    │       │
    │       ├── vel_cmd_udp_pub       → UDP 速度指令发布器
    │       └── mode_status_pub       → 模式状态发布器
    │
    └── 导航节点集群（7 个生命周期节点）
```

### 5.2 核心组件详解

#### 5.2.1 navigo_bt_navigator — 行为树导航器

**职责：** 使用行为树（Behavior Tree）编排整个导航流程。

- 支持两种导航模式：`navigate_to_pose`（单目标）和 `navigate_through_poses`（多目标穿越）
- 内置 40+ 个 BT 节点（插件形式），包括路径计算、路径跟踪、恢复行为、条件检查等
- BT 循环周期：10ms
- 服务超时：20s / 等待服务超时：1000ms

**关键 BT 节点插件：**
- `navigo_compute_path_to_pose_action_bt_node` — 路径规划触发
- `navigo_follow_path_action_bt_node` — 路径跟踪
- `navigo_spin_action_bt_node` / `navigo_back_up_action_bt_node` — 恢复行为
- `navigo_recovery_node_bt_node` — 故障恢复
- `navigo_goal_reached_condition_bt_node` — 目标到达判定
- `navigo_is_stuck_condition_bt_node` — 卡住检测

#### 5.2.2 navigo_mppi_controller — MPPI 控制器

**职责：** 基于模型预测路径积分（MPPI）的局部路径跟踪控制器。

**核心参数：**
| 参数 | 值 | 说明 |
|------|-----|------|
| `time_steps` | 56 | 预测步数 |
| `model_dt` | 0.05s | 预测步长 |
| `batch_size` | 2000 | 采样批次 |
| `vx_max` | 1.5 m/s | 最大前向速度 |
| `vy_max` | 0.5 m/s | 最大横向速度 |
| `wz_max` | 1.9 rad/s | 最大角速度 |
| `motion_model` | DiffDrive | 差速驱动模型 |
| `temperature` | 0.3 | 采样温度 |

**评价函数（Critics）：**
- `ConstraintCritic` — 运动学约束
- `CostCritic` — 代价地图碰撞代价
- `GoalCritic` — 目标趋近
- `PathAlignCritic` — 路径对齐
- `PathFollowCritic` — 路径跟随
- `PathAngleCritic` — 路径角度
- `GoalAngleCritic` — 目标角度
- `PreferForwardCritic` — 前进偏好

#### 5.2.3 navigo_navfn_planner — 全局路径规划器

**职责：** 基于 NavFn 算法（A* / Dijkstra）的全局路径规划。

- 规划频率：20 Hz
- 容差：0.5m
- 允许未知区域探索：`allow_unknown: true`

#### 5.2.4 navigo_costmap_2d — 2D 代价地图

**本地代价地图：**
- 更新频率：5 Hz，发布频率：2 Hz
- 滚动窗口：8m × 8m，分辨率：0.05m
- 机器人 footprint：`[0.30, 0.15]` ~ `[-0.30, -0.15]`（60cm × 30cm 矩形）
- 插件：`obstacle_layer`（障碍物层）+ `inflation_layer`（膨胀层）
- 障碍物数据源：`/laser_scan`（LaserScan 类型）
- 膨胀半径：0.25m，代价缩放因子：3.0

**全局代价地图：**
- 更新/发布频率：10 Hz
- 插件：`static_layer`（静态地图层）+ `inflation_layer`
- 膨胀半径：0.2m

#### 5.2.5 navigo_behaviors — 导航行为

提供 5 种恢复/辅助行为：
- `Spin` — 原地旋转
- `BackUp` — 后退
- `DriveOnHeading` — 向目标方向直行
- `Wait` — 原地等待
- `AssistedTeleop` — 辅助遥控

#### 5.2.6 navigo_waypoint_follower — 多航点跟随

- 循环频率：20 Hz
- 到达航点后暂停：200ms
- 失败后是否停止：`false`（继续下一航点）

#### 5.2.7 navigo_velocity_optimizer — 速度优化器

- 平滑频率：20 Hz
- 最大速度：[3.0, 2.0, 3.0]（vx, vy, wz）
- 最大加速度：[1.5, 1.0, 1.5]
- 反馈模式：`OPEN_LOOP`

### 5.3 robot_navigo — 集成启动包

**路径：** `src/navigation/src/robot_navigo/`

这是导航栈的顶层集成包，包含 Launch 文件、参数配置和自定义节点。

#### 自定义 C++ 节点

| 节点 | 文件 | 功能 |
|------|------|------|
| `vel_cmd_udp_pub` | `vel_cmd_udp_publisher.cpp` | 将 `cmd_vel` 转为 UDP 指令发送给 MC |
| `vel_cmd_lcm_pub` | `vel_cmd_lcm_publisher.cpp` | 将 `cmd_vel` 转为 LCM 指令 |
| `mode_status_pub` | `mode_status_publisher.cpp` | 发布导航模式状态 |
| `odom_communication_node` | `odom_communication_node.cpp` | 里程计通信（LCM） |
| `odom_communication_node_udp` | `odom_communication_node_udp.cpp` | 里程计通信（UDP） |
| `custom_odom_baselink` | `custom_odom_baselink.cpp` | 自定义 odom→base_link TF |
| `tf_publisher` | `tf_publisher.cpp` | TF 坐标变换发布 |
| `costmap_listener` | `costmap_listener.cpp` | 代价地图监听 |
| `obstacle_detector` | `obstacle_detector.cpp` | 障碍物检测 |
| `static_map_publisher` | `static_map_publisher.cpp` | 静态地图发布 |
| `mutiple_goal_nav` | `mutiple_goal_nav.cpp` | 多目标导航 |
| `vel_with_mc_trajectory_cmd_udp` | `vel_with_mc_trajectory_cmd_udp.cpp` | MC 轨迹指令 UDP |

#### Launch 启动流程

```
navigation_bringup.launch.py
├── 参数声明：platform, mc_controller_type, communication_type, map, tf_type...
├── 条件判断：
│   ├── communication_type == 'LCM'  → 启动 vel_cmd_lcm_pub
│   └── communication_type == 'UDP' + mc_controller_type == 'RL_TRACK_VELOCITY'
│       → 启动 vel_cmd_udp_pub
├── mode_status_pub（始终启动）
└── bringup_launch.py
    ├── component_container_isolated（组合式容器，use_composition=True 时）
    └── navigation_launch.py
        ├── map_server
        ├── controller_server（MPPI）
        ├── planner_server（NavFn）
        ├── behavior_server
        ├── velocity_optimizer
        ├── bt_navigator
        ├── waypoint_follower
        └── lifecycle_manager_navigation（管理以上 7 个节点的生命周期）
```

---

## 6. 模块三：SLAM

**路径：** `src/slam/src/`

### 6.1 概述

基于 FAST-LIO2 架构的 LiDAR-Inertial 融合 SLAM 系统，支持同时建图和定位。

### 6.2 核心算法

| 组件 | 算法 | 说明 |
|------|------|------|
| **状态估计** | iEKF（迭代扩展卡尔曼滤波） | 基于 MTK（流形理论）的误差状态卡尔曼滤波 |
| **点云配准** | ikd-Tree | 增量式 k-d 树，高效近邻搜索 |
| **地图输出** | 3D PCD + 2D PGM | 同时生成点云地图和占用栅格地图 |

### 6.3 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `lid_topic` | `/livox/lidar` | LiDAR 点云话题 |
| `imu_topic` | `/livox/imu` | IMU 数据话题 |
| `lidar_type` | 1（Livox Mid-360） | 雷达类型 |
| `scan_line` | 4 | 线束数 |
| `filter_size_surf` | 0.2m | 点云降采样体素边长 |
| `filter_size_map` | 0.2m | 地图降采样体素边长 |
| `map_resolution` | 0.05m | 2D 地图分辨率 |
| `fov_degree` | 360° | 视场角 |

### 6.4 输出话题

| 话题 | QoS | 说明 |
|------|-----|------|
| `/world_points` | Best Effort | map 坐标系下的点云 |
| `/body_points` | Reliable | body 坐标系下的点云 |
| `/path` | Reliable | 建图轨迹 |
| `/slam_odom` | Reliable | SLAM 里程计（含 TF） |

### 6.5 使用流程

1. 启动：`ros2 launch robot_slam slam.launch.py`
2. 开始建图：`ros2 service call /slam_state_service robots_dog_msgs/srv/MapState "{data: 3}"`
3. 保存地图：`ros2 service call /slam_state_service robots_dog_msgs/srv/MapState "{data: 5}"`
4. 地图保存至 `~/.jszr/map/`（含 PCD、PGM、轨迹 TXT）

---

## 7. 模块四：定位（localization）

**路径：** `src/localization/`

### 7.1 概述

多传感器融合定位模块，基于 UKF（无迹卡尔曼滤波）融合 IMU 和 LiDAR 数据，输出高精度位姿估计。

### 7.2 子模块

| 包名 | 算法 | 说明 |
|------|------|------|
| `fast_gicp` | Fast GICP | 快速全局点云配准，支持 CUDA 加速 |
| `ndt_omp` | NDT-OMP | 正态分布变换（OpenMP 并行化） |
| `localization` | UKF + 配准 | 融合定位主节点 |

### 7.3 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `reg_method` | `NDT_OMP` | 配准方法（可选 Fast GICP） |
| `ndt_resolution` | 0.5 | NDT 分辨率 |
| `globalmap_voxel_size` | 0.3 | 全局地图体素大小 |
| `imu_init_time` | 3.0s | IMU 初始化时间 |
| `use_global_localization_init` | true | 全局定位初始化 |

### 7.4 输出

| 话题 | 说明 |
|------|------|
| `/odom/localization_odom` | 定位里程计 |
| `/aligned_points` | 对齐后的点云 |
| `/localization_info` | 定位信息（位置、方向、速度） |
| `/global_map_points` | 全局地图点云 |

---

## 8. 构建系统

### 8.1 构建入口

**文件：** `build.sh`

```bash
./build.sh all             # 编译全部 20 个包
./build.sh all debug       # Debug 模式编译
./build.sh clean all       # 清理后重新编译
```

### 8.2 编译包列表（20 个）

构建使用 `colcon build` 指定 20 个包，并行度 8 线程：

| 序号 | 包名 | 类型 |
|------|------|------|
| 1 | `robots_dog_msgs` | 消息定义 |
| 2 | `robot_slam` | SLAM |
| 3 | `navigo_behavior_tree` | BT 节点库 |
| 4 | `navigo_behaviors` | 导航行为 |
| 5 | `navigo_bt_navigator` | BT 导航器 |
| 6 | `navigo_collision_monitor` | 碰撞监测 |
| 7 | `navigo_core` | 导航核心 |
| 8 | `navigo_costmap_2d` | 代价地图 |
| 9 | `navigo_map_server` | 地图服务 |
| 10 | `navigo_mppi_controller` | MPPI 控制器 |
| 11 | `navigo_path_controller` | 路径控制器 |
| 12 | `navigo_navfn_planner` | NavFn 规划器 |
| 13 | `navigo_path_planner` | 路径规划服务 |
| 14 | `navigo_util` | 工具库 |
| 15 | `navigo_velocity_optimizer` | 速度优化 |
| 16 | `navigo_waypoint_follower` | 航点跟随 |
| 17 | `fast_gicp` | GICP 配准 |
| 18 | `ndt_omp` | NDT 配准 |
| 19 | `localization` | 定位融合 |
| 20 | `robot_navigo` | 集成启动 |

### 8.3 依赖安装

```bash
./script/dep/install_all.sh
```

依次安装：
1. **gazebo_dep.sh** — Gazebo 仿真相关包（17 个 ros-humble-gazebo-* 包）
2. **gamepad_dep.sh** — 手柄驱动（SDL2）
3. **ros2_dep.sh** — ROS 2 核心依赖：
   - ros-humble-controller-*（控制器接口）
   - ros-humble-pcl-*（点云库）
   - ros-humble-slam-toolbox
   - ros-humble-behaviortree-cpp
   - ros-humble-navigation2
   - libompl-dev
   - ros-humble-rviz2

---

## 9. 启动脚本与运维工具

### 9.1 start_navigation.sh

**路径：** `script/bash/start_navigation.sh`

支持 6 种模式：

| 命令 | 说明 |
|------|------|
| `start_navigation.sh` | toggle 模式（切换运行/停止） |
| `start_navigation.sh nav` | 前台启动导航栈 |
| `start_navigation.sh rviz` | 前台启动 RViz2 |
| `start_navigation.sh all` | 自动开两个终端（nav + rviz） |
| `start_navigation.sh stop` | 停止所有导航进程 |
| `start_navigation.sh print` | 打印推荐的三终端启动命令 |

**启动参数（UE 仿真模式）：**
```bash
ros2 launch robot_navigo navigation_bringup.launch.py \
    platform:=UE \
    mc_controller_type:=RL_TRACK_VELOCITY \
    communication_type:=UDP \
    map:="${MAP_PATH}"
```

### 9.2 stop_navigation.sh

精确清理所有导航相关进程，包括：
- ROS 2 Launch 包装进程
- 7 个导航组件进程（map_server, controller_server, planner_server 等）
- 生命周期管理器
- 速度发布节点
- 组合容器进程

---

## 10. 通信架构

### 10.1 通信协议选择

| 协议 | 使用场景 | 实现 |
|------|----------|------|
| **UDP** | UE 仿真 / 低延迟 | `vel_cmd_udp_publisher.cpp` |
| **LCM** | Gazebo 仿真 / 分布式 | `vel_cmd_lcm_publisher.cpp` |

通过 `communication_type` Launch 参数选择，Launch 文件中使用 `PythonExpression` 条件判断动态加载对应节点。

### 10.2 DDS 中间件

使用 **Zenoh DDS**（`rmw_zenoh_cpp`）替代默认 FastDDS：
- `RMW_IMPLEMENTATION=rmw_zenoh_cpp`
- `ROS_DOMAIN_ID=89`
- 需要运行 `ros2 run rmw_zenoh_cpp rmw_zenohd` 作为中枢路由守护进程

### 10.3 与 MATRiX 的通信链路

```
MATRiX (UE5 + MuJoCo)
    │
    ├── UDP 25001 ──→ 状态数据（关节角度、IMU、里程计）
    │
    ├── UDP 25002 ←── 控制指令（关节力矩/速度）
    │
    ├── ROS 2 /odom/mujoco_odom ──→ 导航里程计
    │
    ├── ROS 2 /livox/lidar ──→ LiDAR 点云
    │
    ├── ROS 2 /laser_scan ──→ 2D 激光扫描（costmap 用）
    │
    └── ROS 2 /image_raw/compressed ──→ 相机图像

robot_forward（TF 桥接，闭源 .deb 包）
    │
    └── 订阅 /odom/mujoco_odom → 发布 odom→base_link TF
```

---

## 11. ROS 2 话题与服务总览

### 11.1 关键输入话题

| 话题 | 类型 | 来源 | 用途 |
|------|------|------|------|
| `/odom/mujoco_odom` | `nav_msgs/Odometry` | MATRiX / robot_forward | 导航里程计 |
| `/laser_scan` | `sensor_msgs/LaserScan` | MATRiX LiDAR | 障碍物检测 |
| `/livox/lidar` | `sensor_msgs/PointCloud2` | MATRiX LiDAR | SLAM / 定位 |
| `/livox/imu` | `sensor_msgs/Imu` | MATRiX IMU | SLAM / 定位 |
| `/tf` | `tf2_msgs/TFMessage` | robot_forward | 坐标变换 |

### 11.2 关键输出话题

| 话题 | 类型 | 用途 |
|------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 速度指令（导航→MC） |
| `/map` | `nav_msgs/OccupancyGrid` | 全局地图 |
| `/local_costmap/costmap_raw` | `nav_msgs/OccupancyGrid` | 本地代价地图 |
| `/plan` | `nav_msgs/Path` | 全局规划路径 |
| `/trajectory` | 自定义 | MPPI 候选轨迹（可视化） |

### 11.3 关键服务

| 服务 | 类型 | 用途 |
|------|------|------|
| `/map_server/load_map` | `nav2_msgs/LoadMap` | 加载导航地图 |
| `/slam_state_service` | `robots_dog_msgs/MapState` | SLAM 建图控制 |
| `/load_map_service` | `robots_dog_msgs/LoadMap` | 加载 PCD 定位地图 |

### 11.4 关键 Action

| Action | 用途 |
|--------|------|
| `/navigate_to_pose` | 导航到单个目标 |
| `/navigate_through_poses` | 穿越多目标 |

---

## 12. 与 MATRiX 仿真器的集成

### 12.1 目录布局要求

```
<workspace>/
├── matrix/                    # MATRiX 仿真器
└── genisom_roamerx_open/      # 本导航栈
```

### 12.2 启动顺序

```
终端 0: MATRiX 仿真器（sim_launcher）
    ↓ 提供 /odom/mujoco_odom、/livox/lidar、/laser_scan
终端 1: robot_forward（TF 桥接，需 root）
    ↓ 发布 odom→base_link TF
终端 2: 导航栈（start_navigation.sh nav）
    ↓ 消费传感器数据，产出 /cmd_vel
终端 3: RViz2（start_navigation.sh rviz）
    ↓ 可视化 + 发送导航目标
```

### 12.3 环境变量要求

所有终端必须一致：
```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=89
export SDK_CLIENT_IP=127.0.0.1
```

### 12.4 平台参数配置

| 场景 | platform | communication_type | mc_controller_type |
|------|----------|-------------------|-------------------|
| UE 仿真 | `UE` | `UDP` | `RL_TRACK_VELOCITY` |
| Gazebo 仿真 | `GAZEBO` | `LCM` | `RL_TRACK_VELOCITY` |
| 硬件部署 | `NX_XG3588` / `XG3588` | `UDP` | `RL_TRACK_VELOCITY` |

---

## 13. 配置参数详解

### 13.1 主参数文件

**路径：** `src/navigation/src/robot_navigo/params/navigo_params.yaml`（342 行）

### 13.2 关键配置分组

#### 里程计话题

所有导航组件统一使用 `/odom/mujoco_odom` 作为里程计源：
- `bt_navigator.odom_topic`
- `planner_server.odom_topic`
- `controller_server.odom_topic`
- `waypoint_follower.odom_topic`
- `velocity_optimizer.odom_topic`

#### 机器人尺寸

```yaml
footprint: "[[0.30,0.15],[0.30,-0.15],[-0.30,-0.15],[-0.30,0.15]]"
```
60cm × 30cm 矩形 footprint。

#### MPPI 控制器调优

- 预测时域：56 × 0.05s = 2.8s
- 采样量：2000 条轨迹
- 运动模型：差速驱动（DiffDrive）
- 8 个评价函数协同工作，权重从 2.0（PathAngle）到 14.0（PathAlign）不等

---

## 14. 数据流与导航管线

### 14.1 完整导航数据流

```
┌─────────────────────────────────────────────────────────────┐
│                      MATRiX (UE5 + MuJoCo)                   │
│  LiDAR → /livox/lidar, /laser_scan                          │
│  IMU   → /livox/imu                                         │
│  Odom  → /odom/mujoco_odom                                  │
└───────────────┬─────────────────────────────────────────────┘
                │
    ┌───────────▼───────────┐
    │    robot_forward       │  (TF 桥接)
    │  /odom/mujoco_odom    │
    │    → odom→base_link TF │
    └───────────┬───────────┘
                │
    ┌───────────▼───────────────────────────────────────────┐
    │              Navigation Stack                          │
    │                                                        │
    │  /laser_scan → obstacle_layer → local_costmap          │
    │  /map        → static_layer  → global_costmap          │
    │                                                        │
    │  NavFn Planner: global_costmap → /plan (路径)          │
    │                                                        │
    │  MPPI Controller: /plan + local_costmap + /odom        │
    │    → /cmd_vel (速度指令)                                │
    │                                                        │
    │  BT Navigator: 编排 Planner → Controller → Recovery     │
    │                                                        │
    │  Velocity Optimizer: /cmd_vel → /cmd_vel_smoothed      │
    └───────────┬───────────────────────────────────────────┘
                │
    ┌───────────▼───────────┐
    │  vel_cmd_udp_pub       │  (UDP 通信)
    │  /cmd_vel → UDP 包     │
    └───────────┬───────────┘
                │ UDP
    ┌───────────▼───────────┐
    │  MC (运动控制器)       │
    │  速度指令 → 关节力矩   │
    │  → MATRiX UDP 25002   │
    └───────────────────────┘
```

### 14.2 生命周期管理

`lifecycle_manager_navigation` 按顺序管理 7 个节点的状态转换：
```
map_server → controller_server → planner_server → behavior_server
→ velocity_optimizer → bt_navigator → waypoint_follower
```
每个节点经历：Unconfigured → Inactive → Active

---

## 15. 代码统计

| 指标 | 数值 |
|------|------|
| **源文件总数** | 238 个（.cpp + .h + .py） |
| **CMakeLists.txt** | 25 个 |
| **总代码行数** | ~40,962 行 |
| **消息定义** | 50+ msg, 14 srv, 12 action |
| **导航组件包** | 16 个 navigo_* 包 |
| **Launch 文件** | 3 层嵌套（navigation_bringup → bringup → navigation_launch） |

---

## 16. 关键设计模式与架构特点

### 16.1 Nav2 深度定制

项目并非简单使用 Nav2，而是将所有核心组件以 `navigo_` 前缀重新实现：
- 包名从 `nav2_*` 改为 `navigo_*`
- 插件类名从 `nav2_*` 改为 `navigo_*`
- 这使得可以独立修改核心逻辑而不受上游更新影响

### 16.2 组合式部署（Composition）

支持两种部署模式：
- **组合模式**（`use_composition=True`）：所有节点加载到同一进程（`component_container_isolated`），减少进程间通信开销
- **独立模式**（`use_composition=False`）：每个节点独立进程，便于调试

### 16.3 多平台抽象

通过 Launch 参数实现平台无关：
- `platform` 参数决定 `use_sim_time`、通信方式等
- `communication_type` 参数动态选择 UDP/LCM
- `mc_controller_type` 参数适配不同控制器

### 16.4 消息层解耦

`robots_dog_msgs` 包定义了完整的消息接口，使得：
- 导航栈与硬件/仿真器完全解耦
- 不同平台只需实现相同的消息接口
- 支持多种机器人形态（xgb, go2, go2w, xgw, custom）

### 16.5 Zenoh DDS

选用 Zenoh 而非默认 FastDDS 的原因：
- 更低的网络延迟和带宽占用
- 支持零拷贝传输
- 更适合嵌入式/机器人场景
- 需要额外运行 `rmw_zenohd` 守护进程作为中枢路由

---

## 17. 已知限制与待完善项

| 项目 | 状态 | 说明 |
|------|------|------|
| **硬件部署文档** | TODO | `README.md` 中硬件部署部分为空 |
| **robot_forward** | 闭源 | TF 桥接工具以 `.deb` 包形式提供，无源码 |
| **pub_tf 包** | 可选缺失 | `genisom_roamerx_open` 可能不包含 `pub_tf` 包，由 `robot_forward` 替代 |
| **SLAM 话题配置** | 需手动修改 | 需手动修改 `src/slam/src/config/config.yaml` 中的 `lid_topic` 和 `imu_topic` |
| **多机器人** | 支持但未文档化 | Launch 文件支持 namespace，但缺少多机器人导航文档 |
| **路径跟踪控制器** | 部分注释 | `vel_with_mc_trajectory_cmd_udp` 节点在 Launch 中被注释掉 |
| **Gazebo 集成** | 可用但分离 | Gazebo 依赖可安装，但主要仿真推荐使用 MATRiX |

---

## 18. 总结

GENISOM RoamerX Open 是一个**架构清晰、模块化程度高**的机器人导航栈，具有以下核心优势：

1. **完整的导航能力**：从 SLAM 建图、定位到路径规划、避障、多航点巡逻，覆盖自主导航全链路
2. **深度定制的 Nav2**：所有核心组件以 `navigo_` 前缀重新实现，可独立演进
3. **多平台支持**：通过 Launch 参数统一 UE 仿真、Gazebo 仿真和硬件部署
4. **高性能控制器**：MPPI 控制器（2000 批次采样、2.8s 预测时域）提供平滑的路径跟踪
5. **灵活的通信架构**：UDP/LCM 双协议支持，Zenoh DDS 中间件
6. **丰富的消息接口**：50+ 消息、14 服务、12 动作类型，覆盖机器人全部功能域

项目与 MATRiX 仿真器形成**仿真-导航闭环**：MATRiX 提供物理仿真和传感器数据，RoamerX Open 提供导航决策，通过 ROS 2 + UDP 实现数据交换。

---

<div align="center">
分析文档 · GENISOM RoamerX Open · 2026-07-22
</div>