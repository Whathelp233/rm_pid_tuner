/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2024
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#pragma once

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <rm_pid_tuner/PidParams.h>
#include <rm_pid_tuner/TuningStatus.h>
#include <rm_pid_tuner/TuningLog.h>
#include <rm_pid_tuner/StartTuning.h>
#include <rm_pid_tuner/StopTuning.h>
#include <rm_pid_tuner/SetPidParams.h>
#include <rm_pid_tuner/LLMAnalyze.h>

#include <rm_msgs/GimbalPosState.h>
#include <rm_msgs/GimbalDesError.h>
#include <nav_msgs/Odometry.h>

#include <rm_pid_tuner/data_buffer.h>
#include <rm_pid_tuner/PidTunerConfig.h>

#include <dynamic_reconfigure/server.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <string>
#include <vector>
#include <memory>
#include <unordered_map>
#include <functional>
#include <fstream>

#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

namespace rm_pid_tuner
{
/**
 * @brief 控制器类型枚举
 */
enum class ControllerType
{
  GIMBAL,
  CHASSIS,
  SHOOTER
};

/**
 * @brief 调参状态枚举
 */
enum class TuningState
{
  IDLE,
  TUNING,
  DONE,
  ERROR,
  PAUSED
};

/**
 * @brief 单个关节的调参配置
 */
struct JointConfig
{
  std::string name;                     // 关节名称
  std::string topic_pos_state;          // 位置状态话题
  std::string topic_error;              // 误差话题
  std::string param_prefix;             // ROS 参数前缀

  double target_setpoint;               // 目标设定值
  double tolerance;                     // 允许误差

  // 当前 PID 参数
  double p, i, d;
  double i_clamp_max, i_clamp_min;
  bool antiwindup;

  // 原始 PID 参数 (用于恢复)
  double original_p, original_i, original_d;
};

/**
 * @brief 调参配置
 */
struct TuningConfig
{
  std::string controller_name;          // 控制器名称
  ControllerType controller_type;       // 控制器类型
  std::vector<JointConfig> joints;      // 关节配置列表

  int max_rounds;                       // 最大调参轮数
  int buffer_size;                      // 数据缓冲大小
  double min_error_threshold;           // 最小误差阈值

  bool conservative_mode;               // 保守模式
  double z_n_gain_factor;               // Z-N 增益折扣因子

  bool auto_save;                       // 自动保存
};

/**
 * @brief PID 调参器主类
 */
class PidTuner
{
public:
  PidTuner(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
  ~PidTuner() = default;

  /**
   * @brief 初始化调参器
   */
  bool init();

  /**
   * @brief 更新循环 (在 ROS spin 中调用)
   */
  void update(const ros::Time& time, const ros::Duration& period);

private:
  // ==================== 回调函数 ====================

  /**
   * @brief 云台位置状态回调
   */
  void gimbalPosStateCallback(const rm_msgs::GimbalPosStateConstPtr& msg);

  /**
   * @brief 云台误差回调
   */
  void gimbalErrorCallback(const rm_msgs::GimbalDesErrorConstPtr& msg);

  /**
   * @brief 底盘里程计回调
   */
  void chassisOdomCallback(const nav_msgs::OdometryConstPtr& msg);

  /**
   * @brief dynamic_reconfigure 回调
   */
  void reconfigCB(PidTunerConfig& config, uint32_t level);

  // ==================== 服务回调 ====================

  /**
   * @brief 开始调参服务回调
   */
  bool startTuningCB(StartTuning::Request& req, StartTuning::Response& res);

  /**
   * @brief 停止调参服务回调
   */
  bool stopTuningCB(StopTuning::Request& req, StopTuning::Response& res);

  /**
   * @brief 设置 PID 参数服务回调
   */
  bool setPidParamsCB(SetPidParams::Request& req, SetPidParams::Response& res);

  /**
   * @brief 暂停调参服务回调
   */
  bool pauseTuningCB(StopTuning::Request& req, StopTuning::Response& res);

  /**
   * @brief 恢复调参服务回调
   */
  bool resumeTuningCB(StartTuning::Request& req, StartTuning::Response& res);

  // ==================== 调参逻辑 ====================

  /**
   * @brief 开始调参
   */
  bool startTuning(const TuningConfig& config);

  /**
   * @brief 停止调参
   */
  void stopTuning(bool save_params = false, bool restore_params = false);

  /**
   * @brief 暂停调参
   */
  bool pauseTuning();

  /**
   * @brief 恢复调参
   */
  bool resumeTuning();

  /**
   * @brief 处理错误状态
   */
  void handleError(const std::string& error_msg);

  /**
   * @brief 执行一轮调参
   */
  void performTuningRound();

  /**
   * @brief 调用 LLM 分析数据
   */
  bool callLLM(const std::string& joint_name, const DataBuffer& buffer);

  /**
   * @brief 应用新的 PID 参数
   */
  void applyPidParams(const std::string& joint_name, double p, double i, double d);

  /**
   * @brief 从参数服务器读取 PID 参数
   */
  bool readPidParams(const std::string& param_prefix, JointConfig& config);

  /**
   * @brief 保存 PID 参数到参数服务器
   */
  bool savePidParams(const std::string& param_prefix, const JointConfig& config);

  /**
   * @brief 保存 PID 参数到 YAML 文件
   * @param filepath 目标文件路径
   * @return 是否成功
   */
  bool savePidParamsToFile(const std::string& filepath);

  // ==================== 辅助函数 ====================

  /**
   * @brief 发布调参状态
   */
  void publishStatus();

  /**
   * @brief 发布调参日志
   */
  void publishLog(const std::string& joint_name,
                  const PidParams& old_params,
                  const PidParams& new_params,
                  const std::string& analysis);

  /**
   * @brief 生成 LLM 分析的数据文本
   */
  std::string generateDataText(const std::string& joint_name, const DataBuffer& buffer);

  /**
   * @brief 解析 LLM 返回的 JSON
   */
  bool parseLLMResponse(const std::string& response,
                        std::string& analysis,
                        double& p, double& i, double& d,
                        std::string& status);

  /**
   * @brief 限制参数变化幅度
   */
  double limitChange(double old_val, double new_val, double max_change_ratio);

  // ==================== 成员变量 ====================

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // 状态
  TuningState state_{TuningState::IDLE};
  int current_round_{0};
  ros::Time start_time_;
  TuningConfig config_;
  std::string last_error_msg_;  // 最后的错误信息

  // 数据缓冲器 (每个关节一个)
  std::unordered_map<std::string, std::unique_ptr<DataBuffer>> data_buffers_;

  // LLM 配置
  std::string llm_api_url_;
  std::string llm_api_key_;
  std::string llm_model_;
  double llm_temperature_;
  double llm_timeout_;

  // ROS 接口 - 订阅者
  ros::Subscriber sub_gimbal_pos_state_;
  ros::Subscriber sub_gimbal_error_;
  ros::Subscriber sub_chassis_odom_;

  // ROS 接口 - 发布者
  using StatusPublisher = realtime_tools::RealtimePublisher<TuningStatus>;
  using LogPublisher = realtime_tools::RealtimePublisher<TuningLog>;
  using ParamsPublisher = realtime_tools::RealtimePublisher<PidParams>;

  std::shared_ptr<StatusPublisher> status_pub_;
  std::shared_ptr<LogPublisher> log_pub_;
  std::shared_ptr<ParamsPublisher> params_pub_;

  // ROS 接口 - 服务
  ros::ServiceServer srv_start_tuning_;
  ros::ServiceServer srv_stop_tuning_;
  ros::ServiceServer srv_set_params_;
  ros::ServiceServer srv_pause_tuning_;
  ros::ServiceServer srv_resume_tuning_;

  // ROS 接口 - 服务客户端 (LLM)
  ros::ServiceClient llm_client_;

  // dynamic_reconfigure
  std::unique_ptr<dynamic_reconfigure::Server<PidTunerConfig>> dynamic_reconfigure_server_;
  PidTunerConfig dr_config_;
  realtime_tools::RealtimeBuffer<PidTunerConfig> config_rt_buffer_;

  // 发布速率
  double publish_rate_;
  ros::Time last_publish_time_;
};

}  // namespace rm_pid_tuner
