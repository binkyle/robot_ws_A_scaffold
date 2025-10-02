---

# qos_rt_lab

一个用于 ROS 2 QoS（可靠性、耐久性与队列深度）快速实验与对照验证的最小实验包。
包含一个发布者节点 `qos_rt_pub` 和一个订阅者节点 `qos_rt_sub`，默认在话题 `/qos_rt_lab/header` 上传输 `std_msgs/msg/Header`，并提供 `std_srvs/srv/Trigger` 的 `/reset` 服务用于恢复起始参数。

> 适合用它来做：不同 QoS 配置、发送频率、队列深度、不同 RMW 实现（FastDDS/CycloneDDS）下的丢包与实时性对比。

---

## 环境需求

* Ubuntu 22.04 (jammy) / WSL 发行版皆可
* ROS 2 Humble（建议 `ros-humble-ros-core` 或 `ros-humble-desktop`）
* 已创建并构建的 ROS 2 工作区（示例路径）：

  ```
  /mnt/e/work/projects/robot_ws_A_scaffold
  ├── src
  │   └── qos_rt_lab
  ├── build
  ├── install
  └── log
  ```

> WSL 小贴士：纯 CLI 实验完全 OK；若需 RViz/rqt 等 GUI，请使用 WSLg 或配置 X server。

---

## 快速开始（最短路径）

每开一个新终端都先叠环境（先底座，再工作区）：

```bash
# 1) 叠底座 ROS
source /opt/ros/humble/setup.bash

# 2) 进入工作区根并叠 overlay
cd /mnt/e/work/projects/robot_ws_A_scaffold
source install/setup.bash     # 或 source install/local_setup.bash
```

构建/重建（只构建本包）：

```bash
colcon build --symlink-install --packages-select qos_rt_lab
source install/setup.bash
```

运行发布者与订阅者（两个终端分别执行）：

```bash
# 终端 A - 发布者
ros2 run qos_rt_lab qos_rt_pub

# 终端 B - 订阅者
ros2 run qos_rt_lab qos_rt_sub
```

观测工具（任选）：

```bash
ros2 topic list
ros2 topic info /qos_rt_lab/header -v
ros2 topic hz  /qos_rt_lab/header
ros2 topic bw  /qos_rt_lab/header
ros2 topic echo /qos_rt_lab/header
```

调用 reset 服务（恢复为启动参数）：

```bash
ros2 service call /reset std_srvs/srv/Trigger {}
```

---

## 节点与接口

### 话题

* 话题名：`/qos_rt_lab/header`
* 类型：`std_msgs/msg/Header`
* 含有 `stamp` 与 `frame_id`，用于观测时序。

### 服务

* 服务名：`/reset`
* 类型：`std_srvs/srv/Trigger`
* 作用：把发布者节点参数恢复为“启动时”的 `depth` 与 `rate_hz`，并自动重建 Publisher 与定时器。

### 运行参数（发布者）

* `depth`（int，默认 10）：队列深度（`KeepLast(depth)`）
* `rate_hz`（double，默认 10.0）：发布频率（Hz）

示例：

```bash
# 高频+小队列，易丢包
ros2 run qos_rt_lab qos_rt_pub --ros-args -p depth:=1 -p rate_hz:=50.0

# 低频+大队列，稳定
ros2 run qos_rt_lab qos_rt_pub --ros-args -p depth:=100 -p rate_hz:=5.0
```

> 也支持通过 `--params-file` 加载 YAML。

---

## 默认 QoS 配置

代码中发布端与订阅端均使用：

* History：`KeepLast(depth)`
* Reliability：`RELIABLE`
* Durability：`VOLATILE`

你可以在订阅端将 Reliability 改为 `BEST_EFFORT` 以做对照（仅改一行），然后重建：

```cpp
// src/qos_rt_lab/src/qos_rt_sub.cpp
rclcpp::QoS qos{ rclcpp::KeepLast(static_cast<size_t>(10)) };
qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT); // 改这里
qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
```

查看 “Offered/Requested QoS” 是否匹配：

```bash
ros2 topic info /qos_rt_lab/header -v
```

---

## 一键起两个节点（可选）

若有 `launch/qos_rt_lab.launch.py`（示例文件如下），可用：

```bash
ros2 launch qos_rt_lab qos_rt_lab.launch.py depth:=20 rate_hz:=10.0
```

**示例 launch 文件**（放在 `src/qos_rt_lab/launch/qos_rt_lab.launch.py`）：

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    depth = LaunchConfiguration('depth', default='10')
    rate  = LaunchConfiguration('rate_hz', default='10.0')

    return LaunchDescription([
        DeclareLaunchArgument('depth', default_value='10'),
        DeclareLaunchArgument('rate_hz', default_value='10.0'),

        Node(
            package='qos_rt_lab',
            executable='qos_rt_pub',
            name='qos_rt_pub',
            parameters=[{'depth': depth, 'rate_hz': rate}],
            output='screen'
        ),
        Node(
            package='qos_rt_lab',
            executable='qos_rt_sub',
            name='qos_rt_sub',
            output='screen'
        ),
    ])
```

> 记得在 `CMakeLists.txt` 里安装 launch 目录：
>
> ```cmake
> install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
> ```

---

## 典型对照实验清单

1. **队列深度 vs 频率**

   * 组合 A：`depth=1, rate_hz=50`
   * 组合 B：`depth=100, rate_hz=5`
     观察 `ros2 topic hz`、订阅端日志与丢包情况（BestEffort 更易丢）。

2. **RELIABLE vs BEST_EFFORT**

   * 发布端固定 `RELIABLE`；订阅端分别用 `RELIABLE` 与 `BEST_EFFORT`。
   * 在高频/低带宽/突发重负载下对比丢包与延迟。

3. **VOLATILE vs TRANSIENT_LOCAL**（可自行扩展）

   * 将发布端耐久性改为 `TRANSIENT_LOCAL`，在订阅者晚加入的场景观测是否补历史。

4. **RMW 实现切换**

   * 安装 CycloneDDS 并切换：

     ```bash
     sudo apt-get install -y ros-humble-rmw-cyclonedds-cpp
     export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
     ros2 run qos_rt_lab qos_rt_pub
     ```
   * 与默认 FastDDS（`rmw_fastrtps_cpp`）对比 `hz/bw`、稳定性与资源开销。

5. **录包与回放**

   ```bash
   ros2 bag record /qos_rt_lab/header -o /tmp/qos_bag
   # 停掉发布端后
   ros2 bag play /tmp/qos_bag
   ```

---

## 可选扩展：E2E 延迟计算

当前消息包含 `Header.stamp`，可在订阅端计算 `now - msg.stamp` 并打印/统计（建议使用 `rclcpp::Clock(RCL_ROS_TIME)`）。
这能把“丢包感知”升级为“延迟直观量化”。

---

## 构建与安装（注意事项）

* **避免 conda 干扰**：构建/运行前建议 `conda deactivate`，确保 `python3` 指向系统 `/usr/bin/python3`。
* **不要用 sudo 运行 ros2**：`sudo` 会丢环境变量。
* **每个新终端都要叠环境**：

  ```bash
  source /opt/ros/humble/setup.bash
  source /mnt/e/work/projects/robot_ws_A_scaffold/install/setup.bash
  ```

  想省事可写入 `~/.bashrc`。

### merged vs isolated 安装布局

* 如果用 **默认/isolated**（每个包都有独立前缀）：

  * 索引文件位置：`install/qos_rt_lab/share/ament_index/resource_index/packages/qos_rt_lab`
* 如果更偏好 **merged**（单一 install 前缀）：

  ```bash
  rm -rf build/ install/ log/
  source /opt/ros/humble/setup.bash
  colcon build --symlink-install --merge-install --packages-select qos_rt_lab
  source install/setup.bash
  # 索引文件在：
  ls install/share/ament_index/resource_index/packages/qos_rt_lab
  ```

---

## 常见问题（FAQ）

**Q1：`Package 'qos_rt_lab' not found`**

* 解决：

  1. 已构建并安装？

     ```bash
     colcon build --symlink-install --packages-select qos_rt_lab
     ```
  2. 环境已叠？

     ```bash
     source /opt/ros/humble/setup.bash
     source install/setup.bash
     ```
  3. 检查索引文件（根据布局选择路径）：

     ```bash
     ls install/qos_rt_lab/share/ament_index/resource_index/packages/qos_rt_lab
     # 或 merged:
     ls install/share/ament_index/resource_index/packages/qos_rt_lab
     ```
  4. 开一个“干净 shell”再叠：

     ```bash
     env -i bash --noprofile --norc
     source /opt/ros/humble/setup.bash
     cd /mnt/e/work/projects/robot_ws_A_scaffold
     source install/setup.bash
     ros2 run qos_rt_lab qos_rt_pub
     ```

**Q2：构建时出现 `catkin_pkg`/`pytest` 等 Python 依赖报错？**

* 这是系统 Python/conda 混用的常见现象。优先 `conda deactivate`，并确保使用系统 `/usr/bin/python3`。
* 尽量用 `ros-humble-ros-core`/`ros-humble-desktop` 自带工具链。

**Q3：`colcon` 提示 `CMAKE_PREFIX_PATH doesn't exist` 警告？**

* 这通常是你清理了 `install/` 后，环境变量还残留旧路径引起的构建前警告。构建成功后不影响使用。

---

## 包结构

```
qos_rt_lab/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── qos_rt_lab.launch.py           # 可选
├── scripts/                           # 预留脚本目录（可选）
├── src/
│   ├── qos_rt_pub.cpp                 # 发布者节点
│   └── qos_rt_sub.cpp                 # 订阅者节点
└── README.md                          # 本文档
```

---

## 许可

本包遵循仓库根目录的 `LICENSE` 文件（示例为 Apache-2.0）。

---
