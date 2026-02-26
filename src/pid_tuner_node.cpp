/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2024
 * All rights reserved.
 *
 * LLM-based PID Auto-Tuning System for ROS
 *
 * This node provides automatic PID parameter tuning using Large Language Models.
 * It subscribes to controller state topics and publishes tuning commands.
 *******************************************************************************/

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include "rm_pid_tuner/pid_tuner.h"
#include "rm_pid_tuner/visualizer.h"

#include <memory>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pid_tuner_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  ROS_INFO("===========================================");
  ROS_INFO("  LLM PID Auto-Tuning System");
  ROS_INFO("===========================================");

  // 创建调参器
  auto tuner = std::make_unique<rm_pid_tuner::PidTuner>(nh, private_nh);

  // 初始化
  if (!tuner->init())
  {
    ROS_ERROR("Failed to initialize PID tuner");
    return 1;
  }

  // 创建可视化器
  auto visualizer = std::make_unique<rm_pid_tuner::Visualizer>(nh, private_nh);
  visualizer->init();

  // 主循环
  ros::Rate rate(50);  // 50 Hz

  while (ros::ok())
  {
    ros::Time now = ros::Time::now();
    ros::Duration period(0.02);  // 20ms

    tuner->update(now, period);

    ros::spinOnce();
    rate.sleep();
  }

  ROS_INFO("PID Tuner node shutting down...");

  return 0;
}
