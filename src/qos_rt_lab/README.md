
# qos_rt_lab

- `qos_rt_pub`：可动态降级（服务）、可注入故障（丢包/延迟）。
- `qos_rt_sub`：统计延迟/抖动/丢包，发布 diagnostics，并输出 JSON。
- `qos_supervisor`：基于统计阈值自动调用降级/恢复。

运行：
```bash
colcon build --symlink_install --packages-select qos_rt_lab
source install/setup.bash
ros2 launch qos_rt_lab lab.launch.py
```
