# rm_pid_tuner - LLM-based PID Auto-Tuning System for ROS

[![ROS Version](https://img.shields.io/badge/ROS-Noetic-blue.svg)](http://wiki.ros.org/noetic)
[![License](https://img.shields.io/badge/License-BSD-green.svg)](LICENSE)

A ROS package that provides automatic PID parameter tuning using Large Language Models (LLM). This system integrates seamlessly with the `rm_controllers` framework used in RoboMaster robots.

## Features

- 🤖 **LLM-Powered Analysis**: Uses MiniMax M2.5 or other LLM APIs to analyze control system data and suggest optimal PID parameters
- 🔄 **Hybrid Fallback**: Automatic fallback to rule-based tuning when LLM is unavailable
- 🎯 **Multi-Controller Support**: Supports gimbal and chassis controllers
- 📊 **Dual Visualization**: Both Web Dashboard (React) and rqt plugin
- ⚙️ **Dynamic Reconfigure**: Real-time parameter updates without restarting
- 🔧 **Conservative Mode**: Reduces overshoot with PI control and reduced gains
- 🔒 **Security**: HTTPS encrypted API communication, environment variable support for API keys
- 🛡️ **Safety Limits**: Parameter change rate limits and boundary validation

## Installation

### Prerequisites

- ROS Noetic
- Python 3 with `requests`, `numpy`
- nlohmann-json (for C++ JSON parsing)
- Node.js (for Web Dashboard, optional)

### Build

```bash
# Navigate to your catkin workspace
cd ~/ros_ws/dev_ws

# Install system dependencies
sudo apt-get install libnlohmann-json-dev

# Build
catkin build rm_pid_tuner

# Source
source devel/setup.bash
```

### Install Python Dependencies

```bash
# Python dependencies
pip3 install requests numpy
```

## Quick Start

### 1. Configure API Key (推荐方式)

**方式 A: 使用环境变量 (推荐，更安全)**

```bash
# 在 ~/.bashrc 中添加:
export MINIMAX_API_KEY="your-api-key-here"

# 或在启动前设置:
export MINIMAX_API_KEY="your-api-key-here"
roslaunch rm_pid_tuner pid_tuner.launch
```

**方式 B: 通过 launch 参数传递**

```bash
roslaunch rm_pid_tuner pid_tuner.launch api_key:="your-api-key-here"
```

**方式 C: 修改配置文件 (不推荐)**

编辑 `config/pid_tuner_config.yaml`:

```yaml
llm:
  api_url: "https://api.minimax.chat/v1"
  api_key: "your-api-key-here"  # 请勿将真实 key 提交到版本控制
```

### 2. Launch the Node

```bash
# 基本启动
roslaunch rm_pid_tuner pid_tuner.launch

# 禁用 LLM，只使用本地规则
roslaunch rm_pid_tuner pid_tuner.launch use_llm:=false

# 使用自定义 API URL
roslaunch rm_pid_tuner pid_tuner.launch api_url:="https://api.minimaxi.com/v1"

# 启动 Web Dashboard
roslaunch rm_pid_tuner pid_tuner.launch web:=true

# 启动 RViz
roslaunch rm_pid_tuner pid_tuner.launch rviz:=true
```

### 3. Start Tuning

**命令行方式:**

```bash
# 调参云台 yaw 轴
rosservice call /pid_tuner/start_tuning \
  "controller_name: 'gimbal_controller'
   joint_names: ['yaw']
   target_setpoint: 0.0
   max_rounds: 30
   conservative_mode: true"

# 调参底盘 follow
rosservice call /pid_tuner/start_tuning \
  "controller_name: 'chassis_controller'
   joint_names: ['follow']
   target_setpoint: 0.0
   max_rounds: 20"
```

**暂停/恢复调参:**

```bash
# 暂停
rosservice call /pid_tuner/pause_tuning

# 恢复
rosservice call /pid_tuner/resume_tuning

# 停止并恢复原始参数
rosservice call /pid_tuner/stop_tuning "save_params: false, restore_params: true"
```

**Web Dashboard:**
1. Open http://localhost:8080
2. Select controller and joint
3. Click "Start Tuning"

**rqt 插件:**
```bash
rqt --standalone rqt_pid_tuner
```

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    rm_pid_tuner                              │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌──────────────────┐    │
│  │ LLM Client  │  │ Data Buffer │  │ Rule Engine      │    │
│  │ (MiniMax)   │  │ (Sliding)   │  │ (Fallback)       │    │
│  └─────────────┘  └─────────────┘  └──────────────────┘    │
│                                                              │
│  Services:                                                   │
│    - /start_tuning, /stop_tuning, /set_params               │
│    - /pause_tuning, /resume_tuning                          │
│    - /llm_analyze (Python LLM interface)                    │
│                                                              │
│  Topics:                                                     │
│    - /tuning_status, /tuning_log, /params_update            │
└─────────────────────────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────┐
│                    rm_controllers                             │
│  GimbalController  │  ChassisController  │  ...              │
└─────────────────────────────────────────────────────────────┘
```

## Services

| Service | Type | Description |
|---------|------|-------------|
| `/pid_tuner/start_tuning` | `StartTuning` | 开始调参过程 |
| `/pid_tuner/stop_tuning` | `StopTuning` | 停止调参，可选保存参数 |
| `/pid_tuner/pause_tuning` | `StopTuning` | 暂停调参 |
| `/pid_tuner/resume_tuning` | `StartTuning` | 恢复调参 |
| `/pid_tuner/set_params` | `SetPidParams` | 手动设置 PID 参数 |
| `/pid_tuner/llm_analyze` | `LLMAnalyze` | LLM 分析服务 (内部) |

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/pid_tuner/tuning_status` | `TuningStatus` | 当前调参状态和指标 |
| `/pid_tuner/tuning_log` | `TuningLog` | 调参历史日志 |
| `/pid_tuner/params_update` | `PidParams` | PID 参数更新通知 |

## Configuration

配置文件: `config/pid_tuner_config.yaml`

### LLM 配置

```yaml
llm:
  api_url: "https://api.minimax.chat/v1"  # API 端点 (使用 HTTPS)
  api_key: "your-api-key-here"             # 建议使用环境变量
  model: "MiniMax-M2.5"                    # 模型名称
  temperature: 0.3                          # 温度参数 (0-1)
  max_tokens: 500                           # 最大输出 token
  timeout: 30.0                             # 超时时间 (秒)
```

### 调参配置

```yaml
tuning:
  buffer_size: 25           # 数据缓冲大小 (5-100)
  max_rounds: 30            # 最大调参轮数 (1-100)
  min_error_threshold: 0.3  # 完成阈值
  conservative_mode: true   # 保守模式 (减少超调)
  z_n_gain_factor: 0.5      # Z-N 增益折扣因子 (0-1)
  max_p_change: 0.5         # P 参数每轮最大变化 (50%)
  max_i_change: 0.5         # I 参数每轮最大变化 (50%)
  max_d_change: 0.5         # D 参数每轮最大变化 (50%)
  p_max: 100.0              # P 参数绝对上限
  i_max: 10.0               # I 参数绝对上限
  d_max: 10.0               # D 参数绝对上限
```

## Troubleshooting

### 常见问题

**Q: LLM 服务不可用**

```
[WARN] LLM service not available, using local fallback logic
```

A: 确保 LLM 接口节点已启动:
```bash
roslaunch rm_pid_tuner pid_tuner.launch use_llm:=true
```

**Q: API Key 未配置**

```
[WARN] API Key not configured! Will use fallback logic.
```

A: 设置环境变量或在配置文件中配置 API Key:
```bash
export MINIMAX_API_KEY="your-key-here"
```

**Q: 参数读取失败**

```
[WARN] Failed to read PID params for yaw, using defaults
```

A: 确保控制器节点已启动且 PID 参数已加载到参数服务器。

**Q: 调参过程中出现错误**

```
[ERROR] Error occurred: ...
```

A: 可以尝试恢复调参:
```bash
rosservice call /pid_tuner/resume_tuning
```

或停止并恢复原始参数:
```bash
rosservice call /pid_tuner/stop_tuning "save_params: false, restore_params: true"
```

### 调试模式

启用详细日志:
```bash
roslaunch rm_pid_tuner pid_tuner.launch --screen
```

查看 LLM 服务状态:
```bash
rosservice info /pid_tuner/llm_analyze
```

## Safety Features

1. **参数变化限制**: 每轮参数变化不超过 50%
2. **绝对值限制**: P/I/D 参数有绝对上限
3. **原始参数备份**: 可随时恢复调参前的参数
4. **保守模式**: 使用 PI 控制器 + 降低增益，避免超调
5. **状态机保护**: ERROR 状态下需要手动恢复

## Security Notes

- ✅ API 通信使用 HTTPS 加密
- ✅ API Key 支持环境变量配置
- ✅ 输入参数有白名单验证
- ⚠️ 请勿将 API Key 提交到版本控制
- ⚠️ 建议使用环境变量而非配置文件存储密钥

## License

BSD 3-Clause License. See [LICENSE](LICENSE) for details.

## Author

- Original llm-pid-tuner: KINGSTON-115
- ROS adaptation and security improvements

## Acknowledgments

- rm_controllers framework
- MiniMax API
- ROS community
