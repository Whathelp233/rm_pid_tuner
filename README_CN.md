# rm_pid_tuner - 基于 LLM 的 ROS PID 自动调参系统

[![ROS Version](https://img.shields.io/badge/ROS-Noetic-blue.svg)](http://wiki.ros.org/noetic)
[![License](https://img.shields.io/badge/License-BSD-green.svg)](LICENSE)

一个基于大语言模型（LLM）的 ROS PID 参数自动调参系统，与 RoboMaster 机器人使用的 `rm_controllers` 框架无缝集成。

## 功能特性

- 🤖 **LLM 智能分析**：使用 MiniMax M2.5 等 LLM API 分析控制系统数据，给出最优 PID 参数建议
- 🔄 **混合后备机制**：LLM 不可用时自动切换到基于规则的调参
- 🎯 **多控制器支持**：支持云台和底盘控制器
- 📊 **双重可视化**：Web Dashboard (React) 和 rqt 插件
- ⚙️ **动态重配置**：无需重启即可实时更新参数
- 🔧 **保守模式**：使用 PI 控制和降低增益来减少超调
- 🔒 **安全性**：HTTPS 加密 API 通信，支持环境变量配置 API Key
- 🛡️ **安全限制**：参数变化率限制和边界验证

## 安装

### 系统要求

- ROS Noetic
- Python 3（需要 `requests`、`numpy`）
- nlohmann-json（用于 C++ JSON 解析）
- Node.js（用于 Web Dashboard，可选）

### 编译

```bash
# 进入 catkin 工作空间
cd ~/ros_ws/dev_ws

# 安装系统依赖
sudo apt-get install libnlohmann-json-dev

# 编译
catkin build rm_pid_tuner

# 加载环境
source devel/setup.bash
```

### 安装 Python 依赖

```bash
pip3 install requests numpy
```

## 快速开始

### 1. 配置 API Key（推荐方式）

**方式 A：使用环境变量（推荐，更安全）**

```bash
# 在 ~/.bashrc 中添加：
export MINIMAX_API_KEY="your-api-key-here"

# 或在启动前设置：
export MINIMAX_API_KEY="your-api-key-here"
roslaunch rm_pid_tuner pid_tuner.launch
```

**方式 B：通过 launch 参数传递**

```bash
roslaunch rm_pid_tuner pid_tuner.launch api_key:="your-api-key-here"
```

**方式 C：修改配置文件（不推荐）**

编辑 `config/pid_tuner_config.yaml`：

```yaml
llm:
  api_url: "https://api.minimax.chat/v1"
  api_key: "your-api-key-here"  # 请勿将真实 key 提交到版本控制
```

### 2. 启动节点

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

### 3. 开始调参

**命令行方式：**

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

**暂停/恢复调参：**

```bash
# 暂停
rosservice call /pid_tuner/pause_tuning

# 恢复
rosservice call /pid_tuner/resume_tuning

# 停止并恢复原始参数
rosservice call /pid_tuner/stop_tuning "save_params: false, restore_params: true"
```

**Web Dashboard：**
1. 打开 http://localhost:8080
2. 选择控制器和关节
3. 点击"开始调参"

**rqt 插件：**
```bash
rqt --standalone rqt_pid_tuner
```

## 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                    rm_pid_tuner                              │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌──────────────────┐    │
│  │ LLM 客户端  │  │ 数据缓冲器  │  │ 规则引擎         │    │
│  │ (MiniMax)   │  │ (滑动窗口)  │  │ (后备方案)       │    │
│  └─────────────┘  └─────────────┘  └──────────────────┘    │
│                                                              │
│  服务：                                                      │
│    - /start_tuning, /stop_tuning, /set_params               │
│    - /pause_tuning, /resume_tuning                          │
│    - /llm_analyze (Python LLM 接口)                         │
│                                                              │
│  话题：                                                      │
│    - /tuning_status, /tuning_log, /params_update            │
└─────────────────────────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────┐
│                    rm_controllers                             │
│  GimbalController  │  ChassisController  │  ...              │
└─────────────────────────────────────────────────────────────┘
```

## 服务列表

| 服务 | 类型 | 说明 |
|---------|------|-------------|
| `/pid_tuner/start_tuning` | `StartTuning` | 开始调参过程 |
| `/pid_tuner/stop_tuning` | `StopTuning` | 停止调参，可选保存参数 |
| `/pid_tuner/pause_tuning` | `StopTuning` | 暂停调参 |
| `/pid_tuner/resume_tuning` | `StartTuning` | 恢复调参 |
| `/pid_tuner/set_params` | `SetPidParams` | 手动设置 PID 参数 |
| `/pid_tuner/llm_analyze` | `LLMAnalyze` | LLM 分析服务（内部） |

## 话题列表

| 话题 | 类型 | 说明 |
|-------|------|-------------|
| `/pid_tuner/tuning_status` | `TuningStatus` | 当前调参状态和指标 |
| `/pid_tuner/tuning_log` | `TuningLog` | 调参历史日志 |
| `/pid_tuner/params_update` | `PidParams` | PID 参数更新通知 |

## 配置说明

配置文件：`config/pid_tuner_config.yaml`

### LLM 配置

```yaml
llm:
  api_url: "https://api.minimax.chat/v1"  # API 端点（使用 HTTPS）
  api_key: "your-api-key-here"             # 建议使用环境变量
  model: "MiniMax-M2.5"                    # 模型名称
  temperature: 0.3                          # 温度参数（0-1）
  max_tokens: 500                           # 最大输出 token
  timeout: 30.0                             # 超时时间（秒）
```

### 调参配置

```yaml
tuning:
  buffer_size: 25           # 数据缓冲大小（5-100）
  max_rounds: 30            # 最大调参轮数（1-100）
  min_error_threshold: 0.3  # 完成阈值
  conservative_mode: true   # 保守模式（减少超调）
  z_n_gain_factor: 0.5      # Z-N 增益折扣因子（0-1）
  max_p_change: 0.5         # P 参数每轮最大变化（50%）
  max_i_change: 0.5         # I 参数每轮最大变化（50%）
  max_d_change: 0.5         # D 参数每轮最大变化（50%）
  p_max: 100.0              # P 参数绝对上限
  i_max: 10.0               # I 参数绝对上限
  d_max: 10.0               # D 参数绝对上限
```

## 调参参数说明

| 参数 | 类型 | 范围 | 默认值 | 说明 |
|------|------|------|--------|------|
| `controller_name` | string | - | 必填 | 控制器名称，如 `gimbal_controller` |
| `joint_names` | string[] | - | 必填 | 关节名称列表，如 `['yaw', 'pitch']` |
| `target_setpoint` | float | - | 0.0 | 目标设定值 |
| `tolerance` | float | > 0 | 0.3 | 允许误差阈值 |
| `max_rounds` | int | 1-100 | 30 | 最大调参轮数 |
| `buffer_size` | int | 5-100 | 25 | 数据缓冲大小 |
| `conservative_mode` | bool | - | true | 保守模式，减少超调 |
| `z_n_gain_factor` | float | 0-1 | 0.5 | Z-N 增益折扣因子 |
| `auto_save` | bool | - | false | 是否自动保存参数 |

## 故障排除

### 常见问题

**Q: LLM 服务不可用**

```
[WARN] LLM service not available, using local fallback logic
```

A: 确保 LLM 接口节点已启动：
```bash
roslaunch rm_pid_tuner pid_tuner.launch use_llm:=true
```

**Q: API Key 未配置**

```
[WARN] API Key not configured! Will use fallback logic.
```

A: 设置环境变量或在配置文件中配置 API Key：
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

A: 可以尝试恢复调参：
```bash
rosservice call /pid_tuner/resume_tuning
```

或停止并恢复原始参数：
```bash
rosservice call /pid_tuner/stop_tuning "save_params: false, restore_params: true"
```

### 调试模式

启用详细日志：
```bash
roslaunch rm_pid_tuner pid_tuner.launch --screen
```

查看 LLM 服务状态：
```bash
rosservice info /pid_tuner/llm_analyze
```

查看调参状态：
```bash
rostopic echo /pid_tuner/tuning_status
```

## 安全特性

1. **参数变化限制**：每轮参数变化不超过 50%
2. **绝对值限制**：P/I/D 参数有绝对上限
3. **原始参数备份**：可随时恢复调参前的参数
4. **保守模式**：使用 PI 控制器 + 降低增益，避免超调
5. **状态机保护**：ERROR 状态下需要手动恢复

## 安全注意事项

- ✅ API 通信使用 HTTPS 加密
- ✅ API Key 支持环境变量配置
- ✅ 输入参数有白名单验证
- ⚠️ 请勿将 API Key 提交到版本控制
- ⚠️ 建议使用环境变量而非配置文件存储密钥

## 调参工作流程

1. **启动系统**：启动机器人控制器和 PID 调参器
2. **发起调参**：通过服务调用指定要调参的控制器和关节
3. **数据采集**：系统自动采集控制误差数据
4. **LLM 分析**：数据缓冲区满后，调用 LLM 分析并给出参数建议
5. **参数应用**：系统应用新的 PID 参数（带安全限制）
6. **循环迭代**：重复步骤 3-5，直到达到目标或最大轮数
7. **完成/保存**：调参完成后可选择保存参数或恢复原始参数

## 许可证

BSD 3-Clause License。详见 [LICENSE](LICENSE)。

## 作者

- 原始 llm-pid-tuner: KINGSTON-115
- ROS 适配及安全改进

## 致谢

- rm_controllers 框架
- MiniMax API
- ROS 社区
