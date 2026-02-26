/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2024
 * All rights reserved.
 *******************************************************************************/

#pragma once

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>

#include <rm_pid_tuner/data_buffer.h>
#include <rm_pid_tuner/PidParams.h>
#include <rm_pid_tuner/TuningStatus.h>

#include <string>
#include <vector>

namespace rm_pid_tuner
{
/**
 * @brief PID 调参可视化工具
 *
 * 提供在 RViz 中可视化误差曲线和参数变化的功能
 */
class Visualizer
{
public:
  Visualizer(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
  ~Visualizer() = default;

  /**
   * @brief 初始化可视化器
   */
  void init();

  /**
   * @brief 更新可视化
   */
  void update(const TuningStatus& status, const DataBuffer& buffer);

  /**
   * @brief 清除所有可视化标记
   */
  void clear();

private:
  /**
   * @brief 创建误差曲线标记
   */
  visualization_msgs::Marker createErrorCurve(const DataBuffer& buffer,
                                               const std::string& frame_id);

  /**
   * @brief 创建设定值曲线标记
   */
  visualization_msgs::Marker createSetpointCurve(const DataBuffer& buffer,
                                                  const std::string& frame_id);

  /**
   * @brief 创建参数变化条形图
   */
  visualization_msgs::MarkerArray createParamBars(const PidParams& current,
                                                   const PidParams& suggested);

  /**
   * @brief 获取状态对应的颜色
   */
  std_msgs::ColorRGBA getStatusColor(const std::string& status);

  /**
   * @brief 获取误差对应的颜色 (红->黄->绿)
   */
  std_msgs::ColorRGBA getErrorColor(double error, double max_error);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // 发布者
  ros::Publisher marker_pub_;
  ros::Publisher marker_array_pub_;

  // 配置
  std::string frame_id_;
  double scale_;
  bool show_setpoint_;
  bool show_error_;
  bool show_params_;
};

}  // namespace rm_pid_tuner
