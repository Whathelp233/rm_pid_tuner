/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2024
 * All rights reserved.
 *******************************************************************************/

#pragma once

#include <string>
#include <vector>
#include <deque>
#include <mutex>
#include <fstream>
#include <chrono>

#include <yaml-cpp/yaml.h>

namespace rm_pid_tuner
{
/**
 * @brief 单次调参记录
 */
struct TuningRecord
{
  std::string controller_name;
  std::string joint_name;
  int round_number;
  double timestamp;

  // 参数变化
  double old_p, old_i, old_d;
  double new_p, new_i, new_d;

  // 性能指标
  double avg_error_before;
  double avg_error_after;
  double max_error;
  double itae;  // 积分时间绝对误差
  double ise;   // 积分平方误差

  // 分析结果
  std::string analysis;
  bool used_llm;
  double response_time_ms;

  // 序列化到 YAML
  YAML::Node toYaml() const;
  static TuningRecord fromYaml(const YAML::Node& node);
};

/**
 * @brief 调参会话记录
 */
struct TuningSession
{
  std::string session_id;
  std::string controller_name;
  std::vector<std::string> joint_names;
  double start_time;
  double end_time;
  int total_rounds;
  bool completed;
  std::string status;  // "COMPLETED", "STOPPED", "ERROR"

  // 初始和最终参数
  std::map<std::string, std::tuple<double, double, double>> initial_params;  // joint -> (p, i, d)
  std::map<std::string, std::tuple<double, double, double>> final_params;

  // 调参记录
  std::vector<TuningRecord> records;

  YAML::Node toYaml() const;
  static TuningSession fromYaml(const YAML::Node& node);
};

/**
 * @brief 调参历史管理器
 *
 * 负责记录、存储和查询调参历史
 */
class TuningHistory
{
public:
  explicit TuningHistory(size_t max_records = 1000, const std::string& storage_path = "");

  /**
   * @brief 开始新的调参会话
   */
  std::string startSession(const std::string& controller_name,
                           const std::vector<std::string>& joint_names);

  /**
   * @brief 添加调参记录
   */
  void addRecord(const TuningRecord& record);

  /**
   * @brief 结束当前会话
   */
  void endSession(bool completed, const std::string& status = "COMPLETED");

  /**
   * @brief 获取当前会话 ID
   */
  std::string getCurrentSessionId() const;

  /**
   * @brief 获取最近的调参记录
   */
  std::vector<TuningRecord> getRecentRecords(size_t n = 10) const;

  /**
   * @brief 获取指定会话的所有记录
   */
  std::vector<TuningRecord> getSessionRecords(const std::string& session_id) const;

  /**
   * @brief 获取所有会话列表
   */
  std::vector<TuningSession> getAllSessions() const;

  /**
   * @brief 保存历史到文件
   */
  bool saveToFile(const std::string& filepath = "");

  /**
   * @brief 从文件加载历史
   */
  bool loadFromFile(const std::string& filepath = "");

  /**
   * @brief 清空历史
   */
  void clear();

  /**
   * @brief 获取记录数量
   */
  size_t size() const;

private:
  mutable std::mutex mutex_;
  std::deque<TuningSession> sessions_;
  std::string current_session_id_;
  size_t max_records_;
  std::string storage_path_;

  std::string generateSessionId() const;
};

}  // namespace rm_pid_tuner
