

**目标**：用“中间件/QoS + 诊断 + 失效降级”构建一个可投递的系统级 Demo。  
包含：发布/订阅（1kHz）、QoS 参数切换、诊断健康度、阈值越界自动降级（best_effort/降频）、恢复时再回到高性能模式。

## 快速开始
```bash
# 1) 安装 ROS 2（建议 Humble/Iron），并 source
source /opt/ros/humble/setup.bash

# 2) 构建
cd robot_ws_A
colcon build --symlink-install --packages-select qos_rt_lab
source install/setup.bash

# 3) 运行（默认可靠+keep_last=10，1kHz）
ros2 launch qos_rt_lab lab.launch.py

# 可调参：
# reliability:=reliable|best_effort
# history:=keep_last|keep_all depth:=10
# deadline_ms:=0 latency_budget_ms:=0 rate_hz:=1000
# 阈值：lat_p95_warn_ms lat_p95_error_ms jitter_warn_us jitter_error_us loss_warn loss_error
```

## 组件
- `qos_rt_pub`：1kHz Header；支持 QoS 参数；支持**故障注入**（`fault_drop_n`、`fault_delay_us`）。
- `qos_rt_sub`：统计**延迟/抖动/丢包**；输出到日志与 JSON；**diagnostic_updater** 发布健康度。
- `qos_supervisor`：订阅统计，超过阈值自动 `degrade`（best_effort + 降频），恢复后 `restore`。
- `scripts/run_matrix.py`：批量跑参数矩阵，产出 CSV/JSON。

