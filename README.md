# robot_ws_A_scaffold — ROS 2 QoS + Diagnostics + Degrade/Restore (Cut‑in A)

最小可运行工作区：QoS 参数化、诊断、自动降级/恢复 + 参数矩阵脚本。

## 快速开始
```bash
# 安装 ROS 2（22.04→Humble；24.04→Jazzy）
sudo sh tools/install/ros2_auto_install_adapt.sh

# 构建 & 运行
colcon build --symlink-install --packages-select qos_rt_lab
source install/setup.bash
ros2 launch qos_rt_lab lab.launch.py
```
