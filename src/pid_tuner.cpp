/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2024
 * All rights reserved.
 *******************************************************************************/

#include "rm_pid_tuner/pid_tuner.h"

#include <pluginlib/class_list_macros.hpp>
#include <ros/package.h>
#include <fstream>
#include <sstream>

namespace rm_pid_tuner
{
PidTuner::PidTuner(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
  : nh_(nh), private_nh_(private_nh)
{
}

bool PidTuner::init()
{
  ROS_INFO("[PidTuner] Initializing...");

  // 加载 LLM 配置
  private_nh_.param("llm/api_url", llm_api_url_, std::string("https://api.minimax.chat/v1"));
  private_nh_.param("llm/api_key", llm_api_key_, std::string(""));
  private_nh_.param("llm/model", llm_model_, std::string("MiniMax-M2.5"));
  private_nh_.param("llm/temperature", llm_temperature_, 0.3);
  private_nh_.param("llm/timeout", llm_timeout_, 30.0);

  // 检查 API Key
  if (llm_api_key_.empty() || llm_api_key_ == "your-api-key-here")
  {
    ROS_WARN("[PidTuner] LLM API Key not configured! Set ~llm/api_key parameter.");
  }

  // 加载其他配置
  private_nh_.param("publish_rate", publish_rate_, 10.0);

  // 初始化发布者
  status_pub_ = std::make_shared<StatusPublisher>(private_nh_, "tuning_status", 10);
  log_pub_ = std::make_shared<LogPublisher>(private_nh_, "tuning_log", 100);
  params_pub_ = std::make_shared<ParamsPublisher>(private_nh_, "params_update", 10);

  // 初始化服务
  srv_start_tuning_ = private_nh_.advertiseService("start_tuning", &PidTuner::startTuningCB, this);
  srv_stop_tuning_ = private_nh_.advertiseService("stop_tuning", &PidTuner::stopTuningCB, this);
  srv_set_params_ = private_nh_.advertiseService("set_params", &PidTuner::setPidParamsCB, this);
  srv_pause_tuning_ = private_nh_.advertiseService("pause_tuning", &PidTuner::pauseTuningCB, this);
  srv_resume_tuning_ = private_nh_.advertiseService("resume_tuning", &PidTuner::resumeTuningCB, this);

  // 初始化 LLM 服务客户端
  // 注意：需要先启动 llm_interface.py 节点
  llm_client_ = private_nh_.serviceClient<rm_pid_tuner::LLMAnalyze>("llm_analyze");

  // 初始化 dynamic_reconfigure
  dynamic_reconfigure_server_ =
      std::make_unique<dynamic_reconfigure::Server<PidTunerConfig>>(private_nh_);
  dynamic_reconfigure_server_->setCallback(boost::bind(&PidTuner::reconfigCB, this, _1, _2));

  ROS_INFO("[PidTuner] Initialized successfully");
  ROS_INFO("[PidTuner] Services available:");
  ROS_INFO("  - ~start_tuning");
  ROS_INFO("  - ~stop_tuning");
  ROS_INFO("  - ~set_params");

  return true;
}

void PidTuner::update(const ros::Time& time, const ros::Duration& period)
{
  // 更新配置
  PidTunerConfig config = *config_rt_buffer_.readFromRT();

  // 根据状态执行不同逻辑
  switch (state_)
  {
    case TuningState::TUNING:
    {
      // 检查每个关节的数据缓冲器是否已满
      bool all_full = true;
      for (auto& pair : data_buffers_)
      {
        if (!pair.second->isFull())
        {
          all_full = false;
          break;
        }
      }

      if (all_full)
      {
        try
        {
          performTuningRound();
        }
        catch (const std::exception& e)
        {
          handleError(std::string("Exception in tuning round: ") + e.what());
        }
      }
      break;
    }

    case TuningState::PAUSED:
      // 暂停状态，不执行任何操作
      break;

    case TuningState::ERROR:
      // 错误状态，等待外部恢复或停止
      break;

    case TuningState::IDLE:
    case TuningState::DONE:
    default:
      // 空闲或完成状态
      break;
  }

  // 发布状态
  if (time - last_publish_time_ > ros::Duration(1.0 / publish_rate_))
  {
    publishStatus();
    last_publish_time_ = time;
  }
}

void PidTuner::gimbalPosStateCallback(const rm_msgs::GimbalPosStateConstPtr& msg)
{
  if (state_ != TuningState::TUNING)
    return;

  // 根据控制器名称找到对应的关节
  for (auto& joint : config_.joints)
  {
    auto it = data_buffers_.find(joint.name);
    if (it == data_buffers_.end())
      continue;

    DataPoint point;
    point.timestamp = msg->header.stamp.toSec();
    point.setpoint = msg->set_point;  // 根据 rm_msgs/GimbalPosState 消息结构
    point.value = msg->process_value;
    point.error = msg->error;
    point.control = msg->command;
    point.p = joint.p;
    point.i = joint.i;
    point.d = joint.d;

    it->second->add(point);
  }
}

void PidTuner::gimbalErrorCallback(const rm_msgs::GimbalDesErrorConstPtr& msg)
{
  if (state_ != TuningState::TUNING)
    return;

  // 可以用误差话题来补充数据
}

void PidTuner::chassisOdomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  if (state_ != TuningState::TUNING)
    return;

  // 底盘里程计数据处理
  // 根据需要提取速度、位置等信息
}

void PidTuner::reconfigCB(PidTunerConfig& config, uint32_t level)
{
  ROS_INFO("[PidTuner] Dynamic reconfigure callback");
  config_rt_buffer_.writeFromNonRT(config);
}

bool PidTuner::startTuningCB(StartTuning::Request& req, StartTuning::Response& res)
{
  ROS_INFO("[PidTuner] Start tuning request received for: %s", req.controller_name.c_str());

  // 状态检查
  if (state_ == TuningState::TUNING)
  {
    res.success = false;
    res.message = "Already tuning. Stop current tuning first.";
    return true;
  }

  if (state_ == TuningState::ERROR)
  {
    // 允许从错误状态重新开始
    ROS_WARN("[PidTuner] Restarting from ERROR state");
    last_error_msg_.clear();
  }

  // 输入验证
  if (req.controller_name.empty())
  {
    res.success = false;
    res.message = "Controller name cannot be empty";
    return true;
  }

  if (req.joint_names.empty())
  {
    res.success = false;
    res.message = "Joint names cannot be empty";
    return true;
  }

  // 验证控制器名称格式（防止注入）
  // 只允许字母、数字和下划线
  auto isValidName = [](const std::string& name) -> bool
  {
    if (name.empty() || name.length() > 64)
      return false;
    // 不允许以数字开头
    if (std::isdigit(name[0]))
      return false;
    for (char c : name)
    {
      if (!std::isalnum(c) && c != '_')
        return false;
    }
    return true;
  };

  // 验证路径参数（允许 / 但不允许 .. 或特殊字符）
  auto isValidPathParam = [](const std::string& path) -> bool
  {
    if (path.empty() || path.length() > 256)
      return false;
    // 不允许路径遍历
    if (path.find("..") != std::string::npos)
      return false;
    for (char c : path)
    {
      if (!std::isalnum(c) && c != '_' && c != '/' && c != '.' && c != '-')
        return false;
    }
    return true;
  };

  if (!isValidName(req.controller_name))
  {
    res.success = false;
    res.message = "Invalid controller name format";
    return true;
  }

  for (const auto& joint_name : req.joint_names)
  {
    if (!isValidName(joint_name))
    {
      res.success = false;
      res.message = "Invalid joint name format: " + joint_name;
      return true;
    }
  }

  // 参数范围验证
  if (req.max_rounds > 100 || req.max_rounds < 1)
  {
    res.success = false;
    res.message = "max_rounds must be between 1 and 100";
    return true;
  }

  if (req.buffer_size > 100 || req.buffer_size < 5)
  {
    res.success = false;
    res.message = "buffer_size must be between 5 and 100";
    return true;
  }

  if (req.z_n_gain_factor <= 0 || req.z_n_gain_factor > 1)
  {
    res.success = false;
    res.message = "z_n_gain_factor must be between 0 and 1";
    return true;
  }

  // 构建调参配置
  TuningConfig config;
  config.controller_name = req.controller_name;

  // 根据控制器名称确定类型
  if (req.controller_name.find("gimbal") != std::string::npos)
  {
    config.controller_type = ControllerType::GIMBAL;
  }
  else if (req.controller_name.find("chassis") != std::string::npos)
  {
    config.controller_type = ControllerType::CHASSIS;
  }
  else
  {
    res.success = false;
    res.message = "Unknown controller type. Must contain 'gimbal' or 'chassis'";
    return true;
  }

  // 配置关节
  for (const auto& joint_name : req.joint_names)
  {
    JointConfig joint;
    joint.name = joint_name;
    joint.target_setpoint = req.target_setpoint;
    joint.tolerance = req.tolerance > 0 ? req.tolerance : 0.3;

    // 构建参数前缀 - 从配置读取或使用默认值
    if (config.controller_type == ControllerType::GIMBAL)
    {
      // 云台控制器参数路径格式
      std::string param_format = private_nh_.param<std::string>(
          "controllers/gimbal/param_format", "/controllers/${joint}/pid_pos");
      std::string topic_format = private_nh_.param<std::string>(
          "controllers/gimbal/topic_format", "/${joint}/pos_state");

      // 替换占位符
      joint.param_prefix = "/" + req.controller_name + param_format;
      joint.param_prefix.replace(joint.param_prefix.find("${joint}"), 8, joint_name);
      joint.topic_pos_state = "/" + req.controller_name + "/pos_state";
      joint.topic_error = "/" + req.controller_name + "/error";
    }
    else if (config.controller_type == ControllerType::CHASSIS)
    {
      // 底盘控制器参数路径 - 可配置
      std::string pid_suffix = private_nh_.param<std::string>(
          "controllers/chassis/pid_suffix", "pid_follow");

      joint.param_prefix = "/" + req.controller_name + "/" + pid_suffix;
      joint.topic_pos_state = "/" + req.controller_name + "/odom";
    }

    // 读取当前 PID 参数
    if (!readPidParams(joint.param_prefix, joint))
    {
      ROS_WARN("[PidTuner] Failed to read PID params for %s, using defaults", joint_name.c_str());
      joint.p = 1.0;
      joint.i = 0.1;
      joint.d = 0.05;
    }

    // 保存原始参数
    joint.original_p = joint.p;
    joint.original_i = joint.i;
    joint.original_d = joint.d;

    config.joints.push_back(joint);
  }

  config.max_rounds = req.max_rounds > 0 ? req.max_rounds : 30;
  config.buffer_size = req.buffer_size > 0 ? req.buffer_size : 25;
  config.min_error_threshold = req.tolerance > 0 ? req.tolerance : 0.3;
  config.conservative_mode = req.conservative_mode;
  config.z_n_gain_factor = req.z_n_gain_factor > 0 ? req.z_n_gain_factor : 0.5;
  config.auto_save = req.auto_save;

  // 开始调参
  if (startTuning(config))
  {
    res.success = true;
    res.message = "Tuning started successfully";
    res.session_id = static_cast<uint32_t>(ros::Time::now().toSec());
  }
  else
  {
    res.success = false;
    res.message = "Failed to start tuning";
  }

  return true;
}

bool PidTuner::stopTuningCB(StopTuning::Request& req, StopTuning::Response& res)
{
  ROS_INFO("[PidTuner] Stop tuning request received");

  if (state_ != TuningState::TUNING)
  {
    res.success = false;
    res.message = "Not currently tuning";
    return true;
  }

  stopTuning(req.save_params, req.restore_params);

  res.success = true;
  res.message = "Tuning stopped";
  res.total_rounds = current_round_;

  if (!config_.joints.empty())
  {
    // 填充最终参数
    for (const auto& joint : config_.joints)
    {
      PidParams params;
      params.controller_name = config_.controller_name;
      params.joint_name = joint.name;
      params.p = joint.p;
      params.i = joint.i;
      params.d = joint.d;
      res.final_params.push_back(params);

      if (req.restore_params)
      {
        PidParams orig;
        orig.controller_name = config_.controller_name;
        orig.joint_name = joint.name;
        orig.p = joint.original_p;
        orig.i = joint.original_i;
        orig.d = joint.original_d;
        res.original_params.push_back(orig);
      }
    }
  }

  return true;
}

bool PidTuner::setPidParamsCB(SetPidParams::Request& req, SetPidParams::Response& res)
{
  ROS_INFO("[PidTuner] Set PID params request received");

  for (const auto& params : req.params)
  {
    // 更新参数
    applyPidParams(params.joint_name, params.p, params.i, params.d);

    if (req.save_to_file)
    {
      // 保存到文件
      std::string filepath = req.filepath;
      if (filepath.empty())
      {
        // 使用默认路径
        filepath = ros::package::getPath("rm_pid_tuner") + "/config/tuned_pid_params.yaml";
      }

      if (savePidParamsToFile(filepath))
      {
        ROS_INFO("[PidTuner] Saved PID params to file: %s", filepath.c_str());
      }
      else
      {
        ROS_WARN("[PidTuner] Failed to save PID params to file: %s", filepath.c_str());
      }
    }

    res.applied_params.push_back(params);
  }

  res.success = true;
  res.message = "Parameters updated successfully";

  return true;
}

bool PidTuner::startTuning(const TuningConfig& config)
{
  config_ = config;

  // 初始化数据缓冲器
  data_buffers_.clear();
  for (const auto& joint : config.joints)
  {
    data_buffers_[joint.name] = std::make_unique<DataBuffer>(config.buffer_size);
  }

  // 订阅相关话题
  if (config.controller_type == ControllerType::GIMBAL)
  {
    if (!config.joints.empty())
    {
      sub_gimbal_pos_state_ = nh_.subscribe(
          config.joints[0].topic_pos_state, 100,
          &PidTuner::gimbalPosStateCallback, this);
      sub_gimbal_error_ = nh_.subscribe(
          config.joints[0].topic_error, 100,
          &PidTuner::gimbalErrorCallback, this);
    }
  }
  else if (config.controller_type == ControllerType::CHASSIS)
  {
    if (!config.joints.empty())
    {
      sub_chassis_odom_ = nh_.subscribe(
          config.joints[0].topic_pos_state, 100,
          &PidTuner::chassisOdomCallback, this);
    }
  }

  state_ = TuningState::TUNING;
  current_round_ = 0;
  start_time_ = ros::Time::now();

  ROS_INFO("[PidTuner] Tuning started for %zu joints", config.joints.size());

  return true;
}

void PidTuner::stopTuning(bool save_params, bool restore_params)
{
  ROS_INFO("[PidTuner] Stopping tuning...");

  if (restore_params)
  {
    // 恢复原始参数
    for (auto& joint : config_.joints)
    {
      applyPidParams(joint.name, joint.original_p, joint.original_i, joint.original_d);
    }
  }
  else if (save_params)
  {
    // 保存当前参数
    for (const auto& joint : config_.joints)
    {
      savePidParams(joint.param_prefix, joint);
    }
  }

  // 取消订阅
  sub_gimbal_pos_state_.shutdown();
  sub_gimbal_error_.shutdown();
  sub_chassis_odom_.shutdown();

  state_ = TuningState::IDLE;

  ROS_INFO("[PidTuner] Tuning stopped after %d rounds", current_round_);
}

void PidTuner::performTuningRound()
{
  current_round_++;
  ROS_INFO("[PidTuner] === Round %d / %d ===", current_round_, config_.max_rounds);

  // 对每个关节进行调参
  for (auto& joint : config_.joints)
  {
    auto it = data_buffers_.find(joint.name);
    if (it == data_buffers_.end() || it->second->isEmpty())
    {
      ROS_WARN("[PidTuner] No data for joint %s", joint.name.c_str());
      continue;
    }

    // 计算指标
    auto metrics = it->second->calculateMetrics();
    ROS_INFO("[PidTuner] Joint %s: avg_error=%.2f, max_error=%.2f",
             joint.name.c_str(), metrics.avg_error, metrics.max_error);

    // 检查是否完成
    if (metrics.avg_error < config_.min_error_threshold)
    {
      ROS_INFO("[PidTuner] Joint %s tuning complete! Error below threshold.", joint.name.c_str());
      continue;
    }

    // 调用 LLM 分析
    if (callLLM(joint.name, *it->second))
    {
      // LLM 调用成功，参数已在 callLLM 中更新
    }
    else
    {
      ROS_WARN("[PidTuner] LLM call failed for joint %s", joint.name.c_str());
    }

    // 清空缓冲器，开始下一轮
    it->second->clear();
  }

  // 检查是否达到最大轮数
  if (current_round_ >= config_.max_rounds)
  {
    ROS_INFO("[PidTuner] Reached maximum rounds (%d)", config_.max_rounds);
    stopTuning(false, false);
  }
}

bool PidTuner::callLLM(const std::string& joint_name, const DataBuffer& buffer)
{
  // 找到对应的关节配置
  JointConfig* joint = nullptr;
  for (auto& j : config_.joints)
  {
    if (j.name == joint_name)
    {
      joint = &j;
      break;
    }
  }

  if (!joint)
  {
    ROS_ERROR("[PidTuner] Joint not found: %s", joint_name.c_str());
    return false;
  }

  // 生成数据文本
  std::string data_text = generateDataText(joint_name, buffer);

  // 准备服务请求
  rm_pid_tuner::LLMAnalyze srv;
  srv.request.controller_name = config_.controller_name;
  srv.request.joint_name = joint_name;
  srv.request.current_p = joint->p;
  srv.request.current_i = joint->i;
  srv.request.current_d = joint->d;
  srv.request.data_text = data_text;
  srv.request.conservative_mode = config_.conservative_mode;
  srv.request.round_number = current_round_;

  double new_p = joint->p;
  double new_i = joint->i;
  double new_d = joint->d;
  std::string analysis;

  // 尝试调用 LLM 服务
  bool use_fallback = true;

  if (llm_client_.exists() && llm_client_.call(srv))
  {
    if (srv.response.success)
    {
      use_fallback = srv.response.used_fallback;
      new_p = srv.response.suggested_p;
      new_i = srv.response.suggested_i;
      new_d = srv.response.suggested_d;
      analysis = srv.response.analysis;

      ROS_INFO("[PidTuner] LLM response received: %s (fallback=%s, time=%.2fs)",
               srv.response.status.c_str(),
               use_fallback ? "yes" : "no",
               srv.response.response_time);

      // 检查是否完成
      if (srv.response.status == "DONE")
      {
        ROS_INFO("[PidTuner] Joint %s tuning marked as DONE by LLM", joint_name.c_str());
      }
    }
    else
    {
      ROS_WARN("[PidTuner] LLM service returned failure: %s", srv.response.message.c_str());
    }
  }
  else
  {
    // LLM 服务不可用，使用本地后备逻辑
    ROS_WARN("[PidTuner] LLM service not available, using local fallback logic");

    auto metrics = buffer.calculateMetrics();

    if (config_.conservative_mode && current_round_ <= 2 && metrics.avg_error > 50)
    {
      new_p *= config_.z_n_gain_factor;
      new_i *= config_.z_n_gain_factor;
      new_d = 0;
      analysis = "[本地后备] 保守模式: PI, 降低增益";
      ROS_INFO("[PidTuner] Local fallback: Conservative mode: PI, %.1fx gain", config_.z_n_gain_factor);
    }
    else
    {
      if (metrics.error_stddev > 1.0)
      {
        new_p *= 0.9;
        new_d *= 1.2;
        analysis = "[本地后备] 检测到震荡，减小Kp，增大Kd";
      }
      else if (metrics.avg_error > 10)
      {
        new_p *= 1.2;
        analysis = "[本地后备] 响应较慢，增大Kp";
      }
      else if (metrics.avg_error > 2)
      {
        new_i *= 1.3;
        analysis = "[本地后备] 存在稳态误差，增大Ki";
      }
      else
      {
        analysis = "[本地后备] 性能良好，保持当前参数";
      }
    }
  }

  // 限制参数变化幅度
  PidTunerConfig dr_config = *config_rt_buffer_.readFromRT();
  new_p = limitChange(joint->p, new_p, dr_config.max_p_change);
  new_i = limitChange(joint->i, new_i, dr_config.max_i_change);
  new_d = limitChange(joint->d, new_d, dr_config.max_d_change);

  // 应用绝对值限制
  new_p = std::max(0.0, std::min(new_p, dr_config.p_max));
  new_i = std::max(0.0, std::min(new_i, dr_config.i_max));
  new_d = std::max(0.0, std::min(new_d, dr_config.d_max));

  // 应用参数
  PidParams old_params, new_params;
  old_params.p = joint->p;
  old_params.i = joint->i;
  old_params.d = joint->d;

  joint->p = new_p;
  joint->i = new_i;
  joint->d = new_d;

  applyPidParams(joint_name, new_p, new_i, new_d);

  new_params.p = new_p;
  new_params.i = new_i;
  new_params.d = new_d;

  // 发布日志
  publishLog(joint_name, old_params, new_params, analysis);

  return true;
}

void PidTuner::applyPidParams(const std::string& joint_name, double p, double i, double d)
{
  // 找到参数前缀
  std::string param_prefix;
  for (const auto& joint : config_.joints)
  {
    if (joint.name == joint_name)
    {
      param_prefix = joint.param_prefix;
      break;
    }
  }

  if (param_prefix.empty())
  {
    ROS_WARN("[PidTuner] Unknown joint: %s", joint_name.c_str());
    return;
  }

  // 设置参数到参数服务器
  nh_.setParam(param_prefix + "/p", p);
  nh_.setParam(param_prefix + "/i", i);
  nh_.setParam(param_prefix + "/d", d);

  ROS_INFO("[PidTuner] Applied PID params for %s: P=%.4f, I=%.4f, D=%.4f",
           joint_name.c_str(), p, i, d);
}

bool PidTuner::readPidParams(const std::string& param_prefix, JointConfig& config)
{
  bool success = true;

  if (!nh_.getParam(param_prefix + "/p", config.p))
  {
    ROS_WARN("[PidTuner] Failed to read %s/p", param_prefix.c_str());
    success = false;
  }

  if (!nh_.getParam(param_prefix + "/i", config.i))
  {
    ROS_WARN("[PidTuner] Failed to read %s/i", param_prefix.c_str());
    success = false;
  }

  if (!nh_.getParam(param_prefix + "/d", config.d))
  {
    ROS_WARN("[PidTuner] Failed to read %s/d", param_prefix.c_str());
    success = false;
  }

  // 可选参数
  nh_.getParam(param_prefix + "/i_clamp_max", config.i_clamp_max);
  nh_.getParam(param_prefix + "/i_clamp_min", config.i_clamp_min);
  nh_.getParam(param_prefix + "/antiwindup", config.antiwindup);

  return success;
}

bool PidTuner::savePidParams(const std::string& param_prefix, const JointConfig& config)
{
  nh_.setParam(param_prefix + "/p", config.p);
  nh_.setParam(param_prefix + "/i", config.i);
  nh_.setParam(param_prefix + "/d", config.d);

  ROS_INFO("[PidTuner] Saved PID params to %s", param_prefix.c_str());
  return true;
}

bool PidTuner::savePidParamsToFile(const std::string& filepath)
{
  try
  {
    // 创建 YAML 文件内容
    YAML::Emitter out;
    out << YAML::BeginMap;

    out << YAML::Key << "controller_name" << YAML::Value << config_.controller_name;
    out << YAML::Key << "tuning_timestamp" << YAML::Value << ros::Time::now().toSec();
    out << YAML::Key << "total_rounds" << YAML::Value << current_round_;

    out << YAML::Key << "joints" << YAML::Value << YAML::BeginMap;
    for (const auto& joint : config_.joints)
    {
      out << YAML::Key << joint.name << YAML::Value << YAML::BeginMap;
      out << YAML::Key << "p" << YAML::Value << joint.p;
      out << YAML::Key << "i" << YAML::Value << joint.i;
      out << YAML::Key << "d" << YAML::Value << joint.d;
      out << YAML::Key << "original_p" << YAML::Value << joint.original_p;
      out << YAML::Key << "original_i" << YAML::Value << joint.original_i;
      out << YAML::Key << "original_d" << YAML::Value << joint.original_d;
      out << YAML::EndMap;
    }
    out << YAML::EndMap;

    out << YAML::EndMap;

    // 确保目录存在
    std::string dir = filepath.substr(0, filepath.find_last_of('/'));
    if (!dir.empty())
    {
      std::string mkdir_cmd = "mkdir -p " + dir;
      int ret = system(mkdir_cmd.c_str());
      (void)ret;  // 忽略返回值
    }

    // 写入文件
    std::ofstream fout(filepath);
    if (!fout.is_open())
    {
      ROS_ERROR("[PidTuner] Failed to open file for writing: %s", filepath.c_str());
      return false;
    }

    fout << "# Auto-generated PID parameters by rm_pid_tuner\n";
    fout << "# Controller: " << config_.controller_name << "\n";
    fout << "# Rounds: " << current_round_ << "\n\n";
    fout << out.c_str();
    fout.close();

    ROS_INFO("[PidTuner] Successfully saved PID params to: %s", filepath.c_str());
    return true;
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("[PidTuner] Exception while saving PID params: %s", e.what());
    return false;
  }
}

void PidTuner::publishStatus()
{
  if (status_pub_ && status_pub_->trylock())
  {
    auto& msg = status_pub_->msg_;

    msg.header.stamp = ros::Time::now();
    msg.controller_name = config_.controller_name;

    switch (state_)
    {
      case TuningState::IDLE:
        msg.status = "IDLE";
        msg.analysis = "";
        break;
      case TuningState::TUNING:
        msg.status = "TUNING";
        msg.analysis = "";
        break;
      case TuningState::DONE:
        msg.status = "DONE";
        msg.analysis = "Tuning completed successfully";
        break;
      case TuningState::ERROR:
        msg.status = "ERROR";
        msg.analysis = last_error_msg_;
        break;
      case TuningState::PAUSED:
        msg.status = "PAUSED";
        msg.analysis = "Tuning paused by user";
        break;
    }

    msg.round = current_round_;
    msg.max_rounds = config_.max_rounds;
    msg.elapsed_time = (ros::Time::now() - start_time_).toSec();

    // 填充关节状态
    if (!config_.joints.empty() && !data_buffers_.empty())
    {
      const auto& joint = config_.joints[0];
      auto it = data_buffers_.find(joint.name);
      if (it != data_buffers_.end() && !it->second->isEmpty())
      {
        auto metrics = it->second->calculateMetrics();
        msg.avg_error = metrics.avg_error;
        msg.max_error = metrics.max_error;
        msg.current_value = metrics.latest_value;
        msg.target_setpoint = joint.target_setpoint;

        msg.current_params.p = joint.p;
        msg.current_params.i = joint.i;
        msg.current_params.d = joint.d;
      }
    }

    status_pub_->unlockAndPublish();
  }
}

void PidTuner::publishLog(const std::string& joint_name,
                          const PidParams& old_params,
                          const PidParams& new_params,
                          const std::string& analysis)
{
  if (log_pub_ && log_pub_->trylock())
  {
    auto& msg = log_pub_->msg_;

    msg.header.stamp = ros::Time::now();
    msg.round = current_round_;
    msg.controller_name = config_.controller_name;
    msg.joint_name = joint_name;

    msg.old_params = old_params;
    msg.new_params = new_params;
    msg.analysis = analysis;

    auto it = data_buffers_.find(joint_name);
    if (it != data_buffers_.end())
    {
      auto metrics = it->second->calculateMetrics();
      msg.avg_error_before = metrics.avg_error;
      msg.max_error = metrics.max_error;
    }

    msg.timestamp = ros::Time::now().toSec();

    log_pub_->unlockAndPublish();
  }
}

std::string PidTuner::generateDataText(const std::string& joint_name, const DataBuffer& buffer)
{
  std::ostringstream oss;

  // 找到关节配置
  const JointConfig* joint = nullptr;
  for (const auto& j : config_.joints)
  {
    if (j.name == joint_name)
    {
      joint = &j;
      break;
    }
  }

  if (!joint)
    return "";

  auto metrics = buffer.calculateMetrics();

  oss << "当前 PID 参数: P=" << joint->p << ", I=" << joint->i << ", D=" << joint->d << "\n";
  oss << "目标值: " << joint->target_setpoint << "\n";
  oss << "当前值: " << metrics.latest_value << "\n";
  oss << "平均误差: " << metrics.avg_error << "\n";
  oss << "最大误差: " << metrics.max_error << "\n";
  oss << "误差标准差: " << metrics.error_stddev << "\n\n";
  oss << "最近数据 (时间, 设定值, 实际值, 误差):\n";

  auto recent = buffer.getRecent(20);
  for (const auto& point : recent)
  {
    oss << point.timestamp << ", " << point.setpoint << ", "
        << point.value << ", " << point.error << "\n";
  }

  return oss.str();
}

bool PidTuner::parseLLMResponse(const std::string& response,
                                std::string& analysis,
                                double& p, double& i, double& d,
                                std::string& status)
{
  using json = nlohmann::json;

  try
  {
    // 预处理：去掉 Markdown 代码块标记
    std::string clean_response = response;
    // 移除 ```json 和 ```
    size_t pos;
    while ((pos = clean_response.find("```json")) != std::string::npos)
    {
      clean_response.erase(pos, 7);
    }
    while ((pos = clean_response.find("```")) != std::string::npos)
    {
      clean_response.erase(pos, 3);
    }

    // 去除首尾空白
    clean_response.erase(0, clean_response.find_first_not_of(" \t\n\r"));
    clean_response.erase(clean_response.find_last_not_of(" \t\n\r") + 1);

    // 尝试直接解析
    json j;
    try
    {
      j = json::parse(clean_response);
    }
    catch (const json::parse_error& e)
    {
      // 尝试提取 JSON 块
      size_t start = clean_response.find('{');
      size_t end = clean_response.rfind('}');
      if (start != std::string::npos && end != std::string::npos && end > start)
      {
        std::string json_str = clean_response.substr(start, end - start + 1);
        try
        {
          j = json::parse(json_str);
        }
        catch (const json::parse_error& e2)
        {
          ROS_ERROR("[PidTuner] JSON parse error: %s", e2.what());
          return false;
        }
      }
      else
      {
        ROS_ERROR("[PidTuner] No valid JSON found in response");
        return false;
      }
    }

    // 提取字段
    if (j.contains("analysis") && j["analysis"].is_string())
    {
      analysis = j["analysis"].get<std::string>();
    }
    else
    {
      analysis = "解析成功";
    }

    if (j.contains("p") && j["p"].is_number())
    {
      p = j["p"].get<double>();
    }
    else
    {
      ROS_WARN("[PidTuner] Missing or invalid 'p' in response");
      return false;
    }

    if (j.contains("i") && j["i"].is_number())
    {
      i = j["i"].get<double>();
    }
    else
    {
      ROS_WARN("[PidTuner] Missing or invalid 'i' in response");
      return false;
    }

    if (j.contains("d") && j["d"].is_number())
    {
      d = j["d"].get<double>();
    }
    else
    {
      ROS_WARN("[PidTuner] Missing or invalid 'd' in response");
      return false;
    }

    if (j.contains("status") && j["status"].is_string())
    {
      status = j["status"].get<std::string>();
    }
    else
    {
      status = "TUNING";
    }

    return true;
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("[PidTuner] Exception during JSON parsing: %s", e.what());
    return false;
  }
}

double PidTuner::limitChange(double old_val, double new_val, double max_change_ratio)
{
  // 处理零值情况
  if (old_val == 0.0)
  {
    // 如果旧值为0，限制新值不超过一个合理的初始值
    return std::min(std::abs(new_val), 1.0) * (new_val >= 0 ? 1.0 : -1.0);
  }
  if (new_val == 0.0)
  {
    // 如果新值为0，至少保留旧值的一定比例
    return old_val * (1.0 - max_change_ratio);
  }

  double ratio = new_val / old_val;
  if (ratio > 1 + max_change_ratio)
    return old_val * (1 + max_change_ratio);
  else if (ratio < 1 - max_change_ratio)
    return old_val * (1 - max_change_ratio);

  return new_val;
}

bool PidTuner::pauseTuning()
{
  if (state_ != TuningState::TUNING)
  {
    ROS_WARN("[PidTuner] Cannot pause: not in TUNING state (current: %d)", static_cast<int>(state_));
    return false;
  }

  state_ = TuningState::PAUSED;
  ROS_INFO("[PidTuner] Tuning paused at round %d", current_round_);
  return true;
}

bool PidTuner::resumeTuning()
{
  if (state_ != TuningState::PAUSED)
  {
    ROS_WARN("[PidTuner] Cannot resume: not in PAUSED state (current: %d)", static_cast<int>(state_));
    return false;
  }

  state_ = TuningState::TUNING;
  ROS_INFO("[PidTuner] Tuning resumed from round %d", current_round_);
  return true;
}

void PidTuner::handleError(const std::string& error_msg)
{
  last_error_msg_ = error_msg;
  state_ = TuningState::ERROR;
  ROS_ERROR("[PidTuner] Error occurred: %s", error_msg.c_str());

  // 发布错误状态
  publishStatus();
}

bool PidTuner::pauseTuningCB(StopTuning::Request& req, StopTuning::Response& res)
{
  ROS_INFO("[PidTuner] Pause tuning request received");

  if (state_ == TuningState::PAUSED)
  {
    res.success = false;
    res.message = "Already paused";
    return true;
  }

  if (pauseTuning())
  {
    res.success = true;
    res.message = "Tuning paused successfully";
    res.total_rounds = current_round_;
  }
  else
  {
    res.success = false;
    res.message = "Failed to pause tuning";
  }

  return true;
}

bool PidTuner::resumeTuningCB(StartTuning::Request& req, StartTuning::Response& res)
{
  ROS_INFO("[PidTuner] Resume tuning request received");

  if (state_ == TuningState::ERROR)
  {
    // 从错误状态恢复
    last_error_msg_.clear();
  }

  if (resumeTuning())
  {
    res.success = true;
    res.message = "Tuning resumed successfully";
    res.session_id = static_cast<uint32_t>(ros::Time::now().toSec());
  }
  else
  {
    res.success = false;
    res.message = "Failed to resume tuning";
  }

  return true;
}

}  // namespace rm_pid_tuner
