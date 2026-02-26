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

## 详细工作流程与逻辑

### 整体工作流程图

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           PID 自动调参完整流程                                │
└─────────────────────────────────────────────────────────────────────────────┘

用户请求                 系统处理                      LLM/规则引擎
    │                        │                            │
    ▼                        ▼                            │
┌─────────┐           ┌─────────────┐                     │
│ 调用    │──────────▶│ IDLE ──────▶│                     │
│ start_  │           │   状态      │                     │
│ tuning  │           └──────┬──────┘                     │
└─────────┘                  │                            │
                             ▼                            │
                    ┌─────────────────┐                   │
                    │ 初始化调参参数   │                   │
                    │ - 备份原始 PID   │                   │
                    │ - 清空数据缓冲   │                   │
                    │ - 设置目标值     │                   │
                    └────────┬────────┘                   │
                             │                            │
                             ▼                            │
              ┌──────────────────────────────┐            │
              │        TUNING 状态           │            │
              │  ┌────────────────────────┐  │            │
              │  │     数据采集循环       │  │            │
              │  │  ┌──────────────────┐  │  │            │
              │  │  │ 1. 订阅控制器状态 │  │  │            │
              │  │  │ 2. 计算误差 error │  │  │            │
              │  │  │ 3. 添加到缓冲区   │  │  │            │
              │  │  └────────┬─────────┘  │  │            │
              │  └───────────┼────────────┘  │            │
              │              │               │            │
              │              ▼               │            │
              │    ┌─────────────────┐       │            │
              │    │ 缓冲区满？      │       │            │
              │    │ (buffer_size)   │       │            │
              │    └────────┬────────┘       │            │
              │             │                │            │
              │    ┌────────┴────────┐       │            │
              │    │ 否              │ 是    │            │
              │    ▼                 ▼       │            │
              │  继续           ┌─────────┐  │            │
              │  采集           │ 调用    │──┼───────────▶│
              │                 │ LLM     │  │            │
              │                 │ 分析    │  │            │
              │                 └────┬────┘  │            │
              │                      │       │            │
              │                      ▼       │            │
              │              ┌─────────────┐ │            │
              │              │ LLM 返回    │◀┤            │
              │              │ 建议 PID    │ │            │
              │              └──────┬──────┘ │            │
              │                     │        │            │
              │                     ▼        │            │
              │           ┌────────────────┐ │            │
              │           │ 安全限制检查   │ │            │
              │           │ - 变化率限制   │ │            │
              │           │ - 绝对值限制   │ │            │
              │           └───────┬────────┘ │            │
              │                   │          │            │
              │                   ▼          │            │
              │           ┌────────────────┐ │            │
              │           │ 应用新参数     │ │            │
              │           │ 更新控制器 PID │ │            │
              │           └───────┬────────┘ │            │
              │                   │          │            │
              │                   ▼          │            │
              │           ┌────────────────┐ │            │
              │           │ 检查收敛条件   │ │            │
              │           │ error < 阈值？ │ │            │
              │           │ rounds > 最大？│ │            │
              │           └───────┬────────┘ │            │
              │         是        │        否│            │
              │          ┌────────┴────────┐ │            │
              │          ▼                 ▼ │            │
              │    ┌───────────┐    清空缓冲区│            │
              │    │ 完成调参   │    继续采集 │            │
              │    └───────────┘             │            │
              └──────────────────────────────┘            │
                             │                            │
                             ▼                            │
                    ┌─────────────────┐                   │
                    │   COMPLETED     │                   │
                    │   或 STOPPED    │                   │
                    └─────────────────┘                   │
```

### 核心组件工作逻辑

#### 1. PidTuner 主控制器 (pid_tuner.cpp)

```cpp
// 状态机定义
enum class State {
  IDLE,        // 空闲状态，等待调参请求
  TUNING,      // 调参中，正在采集数据
  PAUSED,      // 暂停状态
  ERROR,       // 错误状态
  COMPLETED    // 调参完成
};

// 调参循环核心逻辑
void PidTuner::tuningLoop() {
  while (ros::ok() && state_ == State::TUNING) {
    // 1. 采集数据
    collectData();

    // 2. 检查缓冲区是否已满
    if (data_buffer_->isFull()) {
      // 3. 调用 LLM 分析
      auto suggestion = callLLMAnalyze();

      // 4. 应用安全限制
      applySafetyLimits(suggestion);

      // 5. 更新 PID 参数
      updatePidParams(suggestion);

      // 6. 检查收敛条件
      if (checkConvergence()) {
        state_ = State::COMPLETED;
        break;
      }

      // 7. 清空缓冲区，开始下一轮
      data_buffer_->clear();
      current_round_++;
    }

    rate_.sleep();
  }
}
```

#### 2. DataBuffer 数据缓冲器 (data_buffer.cpp)

```cpp
// 线程安全的数据缓冲
class DataBuffer {
private:
  std::deque<DataPoint> buffer_;    // 环形缓冲区
  std::mutex mutex_;                 // 线程安全锁
  size_t max_size_;                  // 最大容量

  // 增量统计缓存（避免重复计算）
  mutable bool cache_valid_ = false;
  mutable double cached_sum_error_ = 0.0;
  mutable double cached_sum_abs_error_ = 0.0;
  mutable double cached_max_error_ = 0.0;

public:
  // 添加数据点（自动更新缓存）
  void add(const DataPoint& point) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (buffer_.size() >= max_size_) {
      // 移除最旧的数据点，更新缓存
      removeFromCache(buffer_.front());
      buffer_.pop_front();
    }
    buffer_.push_back(point);
    addToCache(point);
  }

  // 计算性能指标
  Metrics calculateMetrics() {
    std::lock_guard<std::mutex> lock(mutex_);
    return {
      .mean_error = cached_sum_error_ / buffer_.size(),
      .mae = cached_sum_abs_error_ / buffer_.size(),
      .max_error = cached_max_error_,
      .rmse = std::sqrt(cached_sum_sq_error_ / buffer_.size()),
      .itae = calculateITAE(),  // 积分时间加权绝对误差
      .ise = calculateISE(),    // 积分平方误差
      .iae = calculateIAE()     // 积分绝对误差
    };
  }
};
```

#### 3. LLM 接口服务 (llm_interface.py)

```python
class LLMInterface:
    def handle_analyze_request(self, req):
        """处理 LLM 分析请求"""

        # 1. 构建提示词
        prompt = self._build_prompt(
            controller_name=req.controller_name,
            current_params=(req.current_p, req.current_i, req.current_d),
            data_text=req.data_text,
            conservative_mode=req.conservative_mode
        )

        # 2. 调用 LLM API（带重试机制）
        for attempt in range(MAX_RETRIES):
            try:
                response = self._call_api(prompt)
                if response.status_code == 200:
                    return self._parse_response(response)
            except Timeout:
                if attempt < MAX_RETRIES - 1:
                    time.sleep(2 ** attempt)  # 指数退避
                    continue

        # 3. LLM 失败时使用规则引擎后备
        return self._fallback_analysis(req)

    def _build_prompt(self, ...):
        """构建发送给 LLM 的提示词"""
        return f"""
你是一个 PID 控制专家。根据以下控制数据，分析系统性能并给出 PID 参数调整建议。

控制器类型: {controller_name}
当前参数: P={p}, I={i}, D={d}
保守模式: {conservative_mode}

控制数据（时间, 设定值, 实际值, 误差）:
{data_text}

当前性能指标:
- 平均误差: {mean_error}
- 最大误差: {max_error}
- RMSE: {rmse}

请以 JSON 格式返回:
{{
  "analysis": "性能分析",
  "suggested_p": 建议P值,
  "suggested_i": 建议I值,
  "suggested_d": 建议D值
}}
"""

    def _fallback_analysis(self, req):
        """规则引擎后备方案"""
        # 基于误差特征的规则调参
        if mean_error > threshold:
            # 稳态误差大，增加 I
            new_i = current_i * 1.2
        if overshoot > threshold:
            # 超调大，减少 P，增加 D
            new_p = current_p * 0.8
            new_d = current_d * 1.2
        # ...
```

#### 4. ControllerFactory 控制器工厂 (controller_factory.h)

```cpp
// 控制器配置工厂 - 支持不同控制器类型
class ControllerConfigFactory {
public:
  static std::unique_ptr<IControllerConfig> create(const std::string& type) {
    if (type == "gimbal_controller") {
      return std::make_unique<GimbalControllerConfig>();
    } else if (type == "chassis_controller") {
      return std::make_unique<ChassisControllerConfig>();
    } else if (type == "shooter_controller") {
      return std::make_unique<ShooterControllerConfig>();
    }
    // 支持自定义控制器
    return std::make_unique<CustomControllerConfig>(type);
  }
};

// 云台控制器配置
class GimbalControllerConfig : public IControllerConfig {
public:
  std::string getParamNamespace() const override {
    return "/gimbal_controller";
  }
  std::vector<std::string> getJointNames() const override {
    return {"yaw", "pitch"};
  }
  PidLimits getLimits() const override {
    return {.p_max = 50.0, .i_max = 5.0, .d_max = 5.0};
  }
};
```

### 数据流转图

```
┌─────────────────────────────────────────────────────────────────────┐
│                           数据流转                                   │
└─────────────────────────────────────────────────────────────────────┘

  rm_controllers                    rm_pid_tuner                    外部
       │                                 │                           │
       │ /joint_states                   │                           │
       │────────────────────────────────▶│                           │
       │                                 │                           │
       │  读取 PID 参数                   │                           │
       │◀────────────────────────────────│                           │
       │  (ros::param)                   │                           │
       │                                 │                           │
       │                                 │  HTTPS POST                │
       │                                 │──────────────────────────▶│ LLM API
       │                                 │                           │ (MiniMax)
       │                                 │  JSON Response            │
       │                                 │◀──────────────────────────│
       │                                 │                           │
       │  更新 PID 参数                   │                           │
       │◀────────────────────────────────│                           │
       │  (ros::param::set)              │                           │
       │                                 │                           │
       │                                 │ /tuning_status             │
       │                                 │──────────────────────────▶│ Dashboard
       │                                 │                           │ Web/rqt
       │                                 │                           │
```

### 调参算法逻辑

#### LLM 分析流程

```
输入: 当前 PID 参数 + 误差数据缓冲区
输出: 建议的 PID 参数

Step 1: 数据预处理
  - 将缓冲区数据转换为文本格式
  - 计算统计指标 (mean, max, RMSE, ITAE, ISE, IAE)

Step 2: 构建 Prompt
  - 包含控制器类型、当前参数、误差数据、性能指标
  - 指定输出格式为 JSON

Step 3: 调用 LLM API
  - 发送 HTTPS POST 请求
  - 超时/失败时进行指数退避重试（最多3次）

Step 4: 解析响应
  - 使用 nlohmann_json 解析 JSON 响应
  - 提取 suggested_p, suggested_i, suggested_d

Step 5: 安全限制
  - 每轮变化率限制: max_change = 50%
  - 绝对值限制: P < 100, I < 10, D < 10
  - 保守模式: 降低增益，可能禁用 D
```

#### 规则引擎后备逻辑

```cpp
// 当 LLM 不可用时的后备调参策略
PidParams fallbackAdjustment(const Metrics& metrics, const PidParams& current) {
  PidParams suggested = current;

  // 1. 稳态误差分析
  if (metrics.mean_error > threshold) {
    // 存在稳态误差，增加积分增益
    suggested.i *= 1.2;
  }

  // 2. 超调分析
  if (metrics.max_error > overshoot_threshold) {
    // 超调过大，减少比例，增加微分
    suggested.p *= 0.8;
    suggested.d *= 1.2;
  }

  // 3. 响应速度分析
  if (metrics.rise_time > slow_threshold) {
    // 响应太慢，增加比例
    suggested.p *= 1.3;
  }

  // 4. 震荡分析
  if (metrics.oscillation_count > 3) {
    // 震荡过多，减少比例，增加微分
    suggested.p *= 0.7;
    suggested.d *= 1.5;
  }

  return suggested;
}
```

### 状态机转换图

```
                    ┌─────────────────────────────────────┐
                    │                                     │
                    ▼                                     │
              ┌─────────┐   start_tuning    ┌─────────┐   │
              │  IDLE   │──────────────────▶│ TUNING  │   │
              └─────────┘                   └────┬────┘   │
                    ▲                            │        │
                    │                            │        │
                    │           ┌────────────────┼─────── │
                    │           │                │        │
                    │     pause │          stop  │  error │
                    │           │                │        │
                    │           ▼                ▼        ▼
                    │     ┌─────────┐      ┌─────────┐ ┌───────┐
                    │     │ PAUSED  │      │STOPPED/ │ │ ERROR │
                    │     └────┬────┘      │COMPLETED│ └───┬───┘
                    │          │           └─────────┘     │
                    │  resume  │                           │
                    └──────────┼───────────────────────────┘
                               │            resume
                               ▼
                         ┌─────────┐
                         │ TUNING  │
                         └─────────┘
```

### 性能指标说明

| 指标 | 公式 | 说明 |
|------|------|------|
| MAE | $\frac{1}{n}\sum\|e_i\|$ | 平均绝对误差 |
| RMSE | $\sqrt{\frac{1}{n}\sum e_i^2}$ | 均方根误差 |
| ITAE | $\sum t_i \cdot \|e_i\|$ | 积分时间加权绝对误差（惩罚长时间误差） |
| ISE | $\sum e_i^2$ | 积分平方误差（惩罚大误差） |
| IAE | $\sum \|e_i\|$ | 积分绝对误差 |

### 简化工作流程

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
