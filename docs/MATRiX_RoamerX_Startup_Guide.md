

# MATRiX + RoamerX 完整启动流程

> 最后更新：2026-07-23 | 基于实际调试验证

---

## 前置条件

### 1. 目录布局

确保 `matrix/` 和 `genisom_roamerx_open/` 在同一层级：

```
/home/qiyuan/Softwares/
├── Matrix/
└── genisom_roamerx_open/
```

### 2. 已安装依赖

| 依赖 | 安装方式 | 验证命令 |
|------|----------|----------|
| ROS 2 Humble | `sudo apt install ros-humble-desktop` | `source /opt/ros/humble/setup.bash` |
| rmw_zenoh_cpp | `sudo apt install ros-humble-rmw-zenoh-cpp` | `ls /opt/ros/humble/lib/rmw_zenoh_cpp/rmw_zenohd` |
| robot-forward | `sudo apt install -y /path/to/robot-forward_0.2.6_amd64.deb` | `dpkg -l robot-forward` |
| genisom_roamerx_open 已构建 | `cd genisom_roamerx_open && ./build.sh all` | `ls install/setup.bash` |

### 3. 配置文件修改

**`sdk_config.yaml`**（路径：`Matrix/src/robot_mc/build/export/config/sdk_config.yaml`）

```yaml
target_ip: "127.0.0.1"
target_port: 43988
```

> 默认值是 `25003`，必须改为 `43988` 才能与 RoamerX 通信。

---

## 启动流程（5 个终端）

### 终端 0 — 启动 Zenoh DDS 路由守护进程

**必须先启动**，否则所有 ROS 2 节点无法互相发现。

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=89
/opt/ros/humble/lib/rmw_zenoh_cpp/rmw_zenohd
```

> 此进程需**保持运行**，不要关闭终端。

---

### 终端 1 — 启动 MATRiX 仿真（UE5 + MuJoCo + MC）

```bash
cd /home/qiyuan/Softwares/Matrix
./bin/sim_launcher
```

在 Launcher 界面中：
1. 选择**机器人类型**（如 xgb）
2. 选择**地图场景**（如 Yard / Town10）
3. **勾选 MuJoCo 物理引擎**（必须开启，否则无传感器数据）

`sim_launcher` 会自动设置环境变量并启动三个子进程：
- `robot_mujoco` — MuJoCo 物理引擎（UDP 25001/25002）
- `zsibot_mujoco_ue` — UE5 渲染
- `mc_ctrl` — 运动控制器

---

### 终端 2 — 启动 robot_forward（TF 桥接，需 root）

```bash
sudo -i
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=89
export SDK_CLIENT_IP=127.0.0.1
source /opt/robot/robot-forward/install/setup.bash
/opt/robot/robot-forward/install/robot_forward/lib/robot_forward/robot_forward
```

此进程负责：
- 订阅 `/odom/mujoco_odom`（来自 MATRiX）
- 发布 `odom → base_link` TF 变换
- 发布 `map → odom` 静态 TF（identity 变换）

---

### 终端 3 — 启动导航栈

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=89
export SDK_CLIENT_IP=127.0.0.1

cd /home/qiyuan/Softwares/genisom_roamerx_open
bash script/bash/start_navigation.sh nav
```

启动的导航组件（7 个生命周期节点）：
- `map_server` — 加载 2D 占用栅格地图
- `controller_server` — MPPI 路径跟踪控制器
- `planner_server` — NavFn 全局路径规划
- `behavior_server` — 恢复行为（spin, backup, wait）
- `velocity_optimizer` — 速度平滑
- `bt_navigator` — 行为树导航编排
- `waypoint_follower` — 多航点跟随

外加：
- `vel_cmd_udp_pub` — 将 `cmd_vel` 转为 UDP 指令发送给 MC
- `mode_status_pub` — 导航模式状态发布

---

### 终端 4 — 启动 RViz2 可视化

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=89
export SDK_CLIENT_IP=127.0.0.1

cd /home/qiyuan/Softwares/genisom_roamerx_open
bash script/bash/start_navigation.sh rviz
```

---

## 启动后验证

### 检查 ROS 2 话题

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=89
ros2 topic list
```

应能看到以下关键话题：

| 话题 | 方向 | 说明 |
|------|------|------|
| `/odom/mujoco_odom` | MATRiX → 导航 | 仿真里程计 |
| `/tf` | robot_forward → 导航 | odom→base_link 动态变换 |
| `/tf_static` | robot_forward → 导航 | map→odom 静态变换 |
| `/laser_scan` | MATRiX → 导航 | 2D 激光扫描（costmap 用） |
| `/livox/lidar` | MATRiX → 导航 | 3D LiDAR 点云 |
| `/livox/imu` | MATRiX → 导航 | IMU 数据 |
| `/map` | map_server → 导航 | 2D 占用栅格地图 |
| `/cmd_vel` | 导航 → MC | 速度指令 |
| `/plan` | planner → 可视化 | 全局规划路径 |

### 检查 TF 树

```bash
ros2 run tf2_tools view_frames
```

应看到：`map → odom → base_link`

### 检查 RViz2

- **Global Status** 应为绿色 OK
- **Fixed Frame** 设为 `map`
- 应能看到机械狗模型和地图

---

## 发送导航目标

在 RViz2 顶部工具栏点击 **"Nav2 Goal"**（或 "2D Goal Pose"），然后在地图上点击目标位置并拖动设置方向。

机器人将：
1. NavFn 规划全局路径
2. MPPI 控制器跟踪路径
3. 实时避障（静态 + 动态障碍物）
4. 到达目标后停止

---

## 停止导航

```bash
cd /home/qiyuan/Softwares/genisom_roamerx_open
bash script/bash/stop_navigation.sh
```

停止 MATRiX 仿真：直接关闭 `sim_launcher` 终端或按 `Ctrl+C`。

---

## 常见问题排查

### Q1: RViz2 显示 "No tf data"

**原因：** `rmw_zenohd` 未运行，节点无法互相发现。

**解决：**
```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=89
/opt/ros/humble/lib/rmw_zenoh_cpp/rmw_zenohd
```

### Q2: ROS 2 话题列表为空

**原因：** 同上，`rmw_zenohd` 未启动。

### Q3: `COLCON_TRACE: 未绑定的变量`

**原因：** `start_navigation.sh` 的 `set -u` 与 colcon 生成的 setup 脚本冲突。

**解决：** 已将脚本中 `set -euo pipefail` 改为 `set -eo pipefail`。

### Q4: 机械狗不动 / 导航无响应

检查清单：
1. MuJoCo 物理是否开启（sim_launcher 中勾选）
2. `sdk_config.yaml` 的 `target_port` 是否为 `43988`
3. `robot_forward` 是否以 root 权限运行
4. `rmw_zenohd` 是否在运行
5. 所有终端的环境变量是否一致

### Q5: robot-forward .deb 文件找不到

文件位于 MATRiX 项目的 `deps/` 目录：
```bash
sudo apt install -y /home/qiyuan/Softwares/Matrix/deps/robot-forward_0.2.6_amd64.deb
```

---

## 启动顺序总结

```
终端 0: rmw_zenohd          （Zenoh DDS 路由，必须最先启动）
    ↓ 节点发现就绪
终端 1: sim_launcher         （UE5 + MuJoCo + MC，提供传感器数据和里程计）
    ↓ /odom/mujoco_odom, /livox/lidar, /laser_scan 就绪
终端 2: robot_forward (root)  （TF 桥接，map→odom→base_link）
    ↓ TF 树完整
终端 3: start_navigation.sh nav  （导航栈，消费传感器数据，产出 cmd_vel）
    ↓ 导航就绪
终端 4: start_navigation.sh rviz  （RViz2 可视化 + 发送导航目标）
```

---

<div align="center">
MATRiX + RoamerX 启动指南 · 2026-07-23
</div>




Gazebo+RoamerX

单点导航

一、启动导航堆栈

```
cd /home/qiyuan/Softwares/genisom_roamerx_open
source install/setup.bash
```

```
ros2 launch robot_navigo navigation_bringup.launch.py \
    platform:=GAZEBO \
    mc_controller_type:=RL_TRACK_VELOCITY \
    communication_type:=LCM \
    map:=/home/qiyuan/Softwares/genisom_roamerx_open/map/map.yaml
```





```
ros2 launch pub_tf pub_tf.launch.py tf_type:=gazebo_tf
```



二、启动Gazebo

```
# 先 source Gazebo 工作空间
source /home/qiyuan/Softwares/Gazebo/install/setup.bash

# 启动 Gazebo + diff_drive 世界（含桥接和TF）
ros2 launch ros_gz_sim_demos diff_drive.launch.py
```

