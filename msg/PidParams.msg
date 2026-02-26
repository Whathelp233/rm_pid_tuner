# PID 参数消息
# 用于传输 PID 控制器的参数

std_msgs/Header header

string controller_name        # 控制器名称 (e.g., "gimbal_controller")
string joint_name             # 关节名称 (e.g., "yaw_joint", "pitch_joint")

float64 p                     # 比例系数 (Kp)
float64 i                     # 积分系数 (Ki)
float64 d                     # 微分系数 (Kd)

float64 i_clamp_max           # 积分上限
float64 i_clamp_min           # 积分下限
bool antiwindup               # 是否启用抗饱和

float64 feedforward           # 前馈系数 (可选)
