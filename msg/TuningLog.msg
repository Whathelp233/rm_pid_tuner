# 调参日志消息
# 记录每一轮调参的详细信息

std_msgs/Header header

uint32 round                  # 调参轮数

string controller_name        # 控制器名称
string joint_name             # 关节名称

PidParams old_params          # 调整前的 PID 参数
PidParams new_params          # 调整后的 PID 参数

string analysis               # LLM 分析结论
string action                 # 执行的操作: INCREASE_KP, DECREASE_KP, etc.

float64 avg_error_before      # 调整前的平均误差
float64 avg_error_after       # 调整后的平均误差
float64 max_error             # 最大误差

float64 timestamp             # 时间戳
float64 duration              # 本轮耗时 (秒)
