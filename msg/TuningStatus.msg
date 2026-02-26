# 调参状态消息
# 用于报告当前的 PID 调参状态

std_msgs/Header header

string controller_name        # 控制器名称
string joint_name             # 关节名称

string status                 # 状态: IDLE, TUNING, DONE, ERROR

uint32 round                  # 当前调参轮数
uint32 max_rounds             # 最大轮数

float64 target_setpoint       # 目标设定值
float64 current_value         # 当前值

float64 avg_error             # 平均误差
float64 max_error             # 最大误差
float64 min_error             # 最小误差

float64[] error_history       # 误差历史 (最近 N 个数据点)
float64[] setpoint_history    # 设定值历史
float64[] value_history       # 实际值历史

PidParams current_params      # 当前 PID 参数
PidParams suggested_params    # LLM 建议的参数

string analysis               # LLM 分析结果
string llm_status             # LLM 调用状态: SUCCESS, FAILED, TIMEOUT

float64 elapsed_time          # 已用时间 (秒)
float64 estimated_remaining   # 预计剩余时间 (秒)
