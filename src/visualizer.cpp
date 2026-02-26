/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2024
 * All rights reserved.
 *******************************************************************************/

#include "rm_pid_tuner/visualizer.h"

namespace rm_pid_tuner
{
Visualizer::Visualizer(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
  : nh_(nh), private_nh_(private_nh)
{
}

void Visualizer::init()
{
  // 加载配置
  private_nh_.param("frame_id", frame_id_, std::string("base_link"));
  private_nh_.param("scale", scale_, 0.1);
  private_nh_.param("show_setpoint", show_setpoint_, true);
  private_nh_.param("show_error", show_error_, true);
  private_nh_.param("show_params", show_params_, true);

  // 初始化发布者
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
}

void Visualizer::update(const TuningStatus& status, const DataBuffer& buffer)
{
  if (buffer.isEmpty())
    return;

  visualization_msgs::MarkerArray marker_array;

  // 创建误差曲线
  if (show_error_)
  {
    auto error_marker = createErrorCurve(buffer, frame_id_);
    marker_array.markers.push_back(error_marker);
  }

  // 创建设定值曲线
  if (show_setpoint_)
  {
    auto setpoint_marker = createSetpointCurve(buffer, frame_id_);
    marker_array.markers.push_back(setpoint_marker);
  }

  // 创建参数条形图
  if (show_params_)
  {
    auto param_markers = createParamBars(status.current_params, status.suggested_params);
    for (const auto& marker : param_markers.markers)
    {
      marker_array.markers.push_back(marker);
    }
  }

  marker_array_pub_.publish(marker_array);
}

void Visualizer::clear()
{
  visualization_msgs::Marker delete_marker;
  delete_marker.action = visualization_msgs::Marker::DELETEALL;
  marker_pub_.publish(delete_marker);
}

visualization_msgs::Marker Visualizer::createErrorCurve(const DataBuffer& buffer,
                                                         const std::string& frame_id)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = "pid_tuner_error";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = scale_ * 0.5;  // Line width

  auto data = buffer.getAll();
  double max_error = buffer.calculateMaxError();

  for (size_t i = 0; i < data.size(); ++i)
  {
    geometry_msgs::Point point;
    point.x = i * scale_;  // 时间轴
    point.y = data[i].error;  // 误差值
    point.z = 0;

    marker.points.push_back(point);

    // 根据误差大小设置颜色
    std_msgs::ColorRGBA color = getErrorColor(std::abs(data[i].error), max_error);
    marker.colors.push_back(color);
  }

  return marker;
}

visualization_msgs::Marker Visualizer::createSetpointCurve(const DataBuffer& buffer,
                                                            const std::string& frame_id)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = "pid_tuner_setpoint";
  marker.id = 1;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = scale_ * 0.3;  // Line width

  // 设定值为绿色
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 0.7;

  auto data = buffer.getAll();

  for (size_t i = 0; i < data.size(); ++i)
  {
    geometry_msgs::Point point;
    point.x = i * scale_;
    point.y = data[i].setpoint;
    point.z = 0.1;  // 稍微偏移以区分误差曲线

    marker.points.push_back(point);
  }

  return marker;
}

visualization_msgs::MarkerArray Visualizer::createParamBars(const PidParams& current,
                                                              const PidParams& suggested)
{
  visualization_msgs::MarkerArray marker_array;

  // 创建 P, I, D 三个条形图
  double bar_width = scale_ * 2;
  double bar_spacing = scale_ * 3;
  double base_x = 0;

  std::vector<std::pair<std::string, std::pair<double, double>>> params = {
      {"P", {current.p, suggested.p}},
      {"I", {current.i, suggested.i}},
      {"D", {current.d, suggested.d}}};

  for (size_t i = 0; i < params.size(); ++i)
  {
    const std::string& name = params[i].first;
    const std::pair<double, double>& values = params[i].second;

    // 当前值条形图
    visualization_msgs::Marker current_bar;
    current_bar.header.frame_id = frame_id_;
    current_bar.header.stamp = ros::Time::now();
    current_bar.ns = "pid_tuner_current_" + name;
    current_bar.id = i * 2;
    current_bar.type = visualization_msgs::Marker::CYLINDER;
    current_bar.action = visualization_msgs::Marker::ADD;

    current_bar.pose.position.x = base_x + i * bar_spacing;
    current_bar.pose.position.y = 0;
    current_bar.pose.position.z = values.first / 2;
    current_bar.pose.orientation.w = 1.0;

    current_bar.scale.x = bar_width;
    current_bar.scale.y = bar_width;
    current_bar.scale.z = values.first;

    current_bar.color.r = 0.2;
    current_bar.color.g = 0.6;
    current_bar.color.b = 1.0;
    current_bar.color.a = 0.8;

    marker_array.markers.push_back(current_bar);

    // 建议值条形图 (半透明)
    visualization_msgs::Marker suggested_bar;
    suggested_bar.header = current_bar.header;
    suggested_bar.ns = "pid_tuner_suggested_" + name;
    suggested_bar.id = i * 2 + 1;
    suggested_bar.type = visualization_msgs::Marker::CYLINDER;
    suggested_bar.action = visualization_msgs::Marker::ADD;

    suggested_bar.pose.position.x = base_x + i * bar_spacing + bar_width;
    suggested_bar.pose.position.y = 0;
    suggested_bar.pose.position.z = values.second / 2;
    suggested_bar.pose.orientation.w = 1.0;

    suggested_bar.scale.x = bar_width;
    suggested_bar.scale.y = bar_width;
    suggested_bar.scale.z = values.second;

    suggested_bar.color.r = 1.0;
    suggested_bar.color.g = 0.6;
    suggested_bar.color.b = 0.2;
    suggested_bar.color.a = 0.6;

    marker_array.markers.push_back(suggested_bar);
  }

  return marker_array;
}

std_msgs::ColorRGBA Visualizer::getStatusColor(const std::string& status)
{
  std_msgs::ColorRGBA color;

  if (status == "IDLE")
  {
    color.r = 0.5;
    color.g = 0.5;
    color.b = 0.5;
    color.a = 1.0;
  }
  else if (status == "TUNING")
  {
    color.r = 0.0;
    color.g = 0.8;
    color.b = 1.0;
    color.a = 1.0;
  }
  else if (status == "DONE")
  {
    color.r = 0.0;
    color.g = 1.0;
    color.b = 0.0;
    color.a = 1.0;
  }
  else if (status == "ERROR")
  {
    color.r = 1.0;
    color.g = 0.0;
    color.b = 0.0;
    color.a = 1.0;
  }
  else
  {
    color.r = 1.0;
    color.g = 1.0;
    color.b = 1.0;
    color.a = 1.0;
  }

  return color;
}

std_msgs::ColorRGBA Visualizer::getErrorColor(double error, double max_error)
{
  std_msgs::ColorRGBA color;

  // 误差从 0 到 max_error，颜色从绿色渐变到红色
  double ratio = max_error > 0 ? std::min(error / max_error, 1.0) : 0.0;

  color.r = ratio;
  color.g = 1.0 - ratio;
  color.b = 0.0;
  color.a = 1.0;

  return color;
}

}  // namespace rm_pid_tuner
