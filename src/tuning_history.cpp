/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2024
 * All rights reserved.
 *******************************************************************************/

#include "rm_pid_tuner/tuning_history.h"

#include <sstream>
#include <iomanip>
#include <ctime>
#include <algorithm>

namespace rm_pid_tuner
{
// ==================== TuningRecord ====================

YAML::Node TuningRecord::toYaml() const
{
  YAML::Node node;
  node["controller_name"] = controller_name;
  node["joint_name"] = joint_name;
  node["round_number"] = round_number;
  node["timestamp"] = timestamp;
  node["old_p"] = old_p;
  node["old_i"] = old_i;
  node["old_d"] = old_d;
  node["new_p"] = new_p;
  node["new_i"] = new_i;
  node["new_d"] = new_d;
  node["avg_error_before"] = avg_error_before;
  node["avg_error_after"] = avg_error_after;
  node["max_error"] = max_error;
  node["itae"] = itae;
  node["ise"] = ise;
  node["analysis"] = analysis;
  node["used_llm"] = used_llm;
  node["response_time_ms"] = response_time_ms;
  return node;
}

TuningRecord TuningRecord::fromYaml(const YAML::Node& node)
{
  TuningRecord record;
  record.controller_name = node["controller_name"].as<std::string>("");
  record.joint_name = node["joint_name"].as<std::string>("");
  record.round_number = node["round_number"].as<int>(0);
  record.timestamp = node["timestamp"].as<double>(0.0);
  record.old_p = node["old_p"].as<double>(0.0);
  record.old_i = node["old_i"].as<double>(0.0);
  record.old_d = node["old_d"].as<double>(0.0);
  record.new_p = node["new_p"].as<double>(0.0);
  record.new_i = node["new_i"].as<double>(0.0);
  record.new_d = node["new_d"].as<double>(0.0);
  record.avg_error_before = node["avg_error_before"].as<double>(0.0);
  record.avg_error_after = node["avg_error_after"].as<double>(0.0);
  record.max_error = node["max_error"].as<double>(0.0);
  record.itae = node["itae"].as<double>(0.0);
  record.ise = node["ise"].as<double>(0.0);
  record.analysis = node["analysis"].as<std::string>("");
  record.used_llm = node["used_llm"].as<bool>(false);
  record.response_time_ms = node["response_time_ms"].as<double>(0.0);
  return record;
}

// ==================== TuningSession ====================

YAML::Node TuningSession::toYaml() const
{
  YAML::Node node;
  node["session_id"] = session_id;
  node["controller_name"] = controller_name;
  node["joint_names"] = joint_names;
  node["start_time"] = start_time;
  node["end_time"] = end_time;
  node["total_rounds"] = total_rounds;
  node["completed"] = completed;
  node["status"] = status;

  // 初始参数
  YAML::Node initial_params_node;
  for (const auto& [joint, params] : initial_params)
  {
    YAML::Node params_node;
    params_node["p"] = std::get<0>(params);
    params_node["i"] = std::get<1>(params);
    params_node["d"] = std::get<2>(params);
    initial_params_node[joint] = params_node;
  }
  node["initial_params"] = initial_params_node;

  // 最终参数
  YAML::Node final_params_node;
  for (const auto& [joint, params] : final_params)
  {
    YAML::Node params_node;
    params_node["p"] = std::get<0>(params);
    params_node["i"] = std::get<1>(params);
    params_node["d"] = std::get<2>(params);
    final_params_node[joint] = params_node;
  }
  node["final_params"] = final_params_node;

  // 调参记录
  YAML::Node records_node;
  for (const auto& record : records)
  {
    records_node.push_back(record.toYaml());
  }
  node["records"] = records_node;

  return node;
}

TuningSession TuningSession::fromYaml(const YAML::Node& node)
{
  TuningSession session;
  session.session_id = node["session_id"].as<std::string>("");
  session.controller_name = node["controller_name"].as<std::string>("");
  session.start_time = node["start_time"].as<double>(0.0);
  session.end_time = node["end_time"].as<double>(0.0);
  session.total_rounds = node["total_rounds"].as<int>(0);
  session.completed = node["completed"].as<bool>(false);
  session.status = node["status"].as<std::string>("UNKNOWN");

  if (node["joint_names"])
  {
    for (const auto& jn : node["joint_names"])
    {
      session.joint_names.push_back(jn.as<std::string>());
    }
  }

  if (node["initial_params"])
  {
    for (const auto& it : node["initial_params"])
    {
      std::string joint = it.first.as<std::string>();
      double p = it.second["p"].as<double>(0.0);
      double i = it.second["i"].as<double>(0.0);
      double d = it.second["d"].as<double>(0.0);
      session.initial_params[joint] = std::make_tuple(p, i, d);
    }
  }

  if (node["final_params"])
  {
    for (const auto& it : node["final_params"])
    {
      std::string joint = it.first.as<std::string>();
      double p = it.second["p"].as<double>(0.0);
      double i = it.second["i"].as<double>(0.0);
      double d = it.second["d"].as<double>(0.0);
      session.final_params[joint] = std::make_tuple(p, i, d);
    }
  }

  if (node["records"])
  {
    for (const auto& r : node["records"])
    {
      session.records.push_back(TuningRecord::fromYaml(r));
    }
  }

  return session;
}

// ==================== TuningHistory ====================

TuningHistory::TuningHistory(size_t max_records, const std::string& storage_path)
  : max_records_(max_records), storage_path_(storage_path)
{
  // 如果指定了存储路径，尝试加载历史
  if (!storage_path_.empty())
  {
    loadFromFile(storage_path_);
  }
}

std::string TuningHistory::generateSessionId() const
{
  auto now = std::chrono::system_clock::now();
  auto now_time = std::chrono::system_clock::to_time_t(now);
  auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      now.time_since_epoch()) % 1000;

  std::ostringstream oss;
  oss << "session_" << std::put_time(std::localtime(&now_time), "%Y%m%d_%H%M%S")
      << "_" << std::setfill('0') << std::setw(3) << now_ms.count();
  return oss.str();
}

std::string TuningHistory::startSession(const std::string& controller_name,
                                        const std::vector<std::string>& joint_names)
{
  std::lock_guard<std::mutex> lock(mutex_);

  TuningSession session;
  session.session_id = generateSessionId();
  session.controller_name = controller_name;
  session.joint_names = joint_names;
  session.start_time = std::chrono::duration<double>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  session.total_rounds = 0;
  session.completed = false;
  session.status = "IN_PROGRESS";

  sessions_.push_back(session);
  current_session_id_ = session.session_id;

  // 如果超过最大记录数，删除最旧的会话
  while (sessions_.size() > max_records_)
  {
    sessions_.pop_front();
  }

  return current_session_id_;
}

void TuningHistory::addRecord(const TuningRecord& record)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (current_session_id_.empty())
  {
    return;
  }

  // 找到当前会话
  for (auto& session : sessions_)
  {
    if (session.session_id == current_session_id_)
    {
      session.records.push_back(record);
      session.total_rounds++;

      // 更新初始参数（如果是第一条记录）
      if (session.records.size() == 1)
      {
        session.initial_params[record.joint_name] =
            std::make_tuple(record.old_p, record.old_i, record.old_d);
      }

      // 更新最终参数
      session.final_params[record.joint_name] =
          std::make_tuple(record.new_p, record.new_i, record.new_d);

      break;
    }
  }
}

void TuningHistory::endSession(bool completed, const std::string& status)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (current_session_id_.empty())
  {
    return;
  }

  for (auto& session : sessions_)
  {
    if (session.session_id == current_session_id_)
    {
      session.end_time = std::chrono::duration<double>(
          std::chrono::system_clock::now().time_since_epoch()).count();
      session.completed = completed;
      session.status = status;
      break;
    }
  }

  current_session_id_.clear();

  // 自动保存
  if (!storage_path_.empty())
  {
    saveToFile(storage_path_);
  }
}

std::string TuningHistory::getCurrentSessionId() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return current_session_id_;
}

std::vector<TuningRecord> TuningHistory::getRecentRecords(size_t n) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<TuningRecord> result;

  // 从最近的会话开始收集记录
  for (auto it = sessions_.rbegin(); it != sessions_.rend() && result.size() < n; ++it)
  {
    for (auto rit = it->records.rbegin(); rit != it->records.rend() && result.size() < n; ++rit)
    {
      result.push_back(*rit);
    }
  }

  return result;
}

std::vector<TuningRecord> TuningHistory::getSessionRecords(const std::string& session_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  for (const auto& session : sessions_)
  {
    if (session.session_id == session_id)
    {
      return session.records;
    }
  }

  return {};
}

std::vector<TuningSession> TuningHistory::getAllSessions() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return std::vector<TuningSession>(sessions_.begin(), sessions_.end());
}

bool TuningHistory::saveToFile(const std::string& filepath)
{
  std::lock_guard<std::mutex> lock(mutex_);

  std::string path = filepath.empty() ? storage_path_ : filepath;
  if (path.empty())
  {
    return false;
  }

  try
  {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "version" << YAML::Value << "1.0";
    out << YAML::Key << "total_sessions" << YAML::Value << sessions_.size();
    out << YAML::Key << "last_updated" << YAML::Value <<
        std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();

    out << YAML::Key << "sessions" << YAML::Value << YAML::BeginSeq;
    for (const auto& session : sessions_)
    {
      out << session.toYaml();
    }
    out << YAML::EndSeq;
    out << YAML::EndMap;

    std::ofstream fout(path);
    if (!fout.is_open())
    {
      return false;
    }
    fout << out.c_str();
    fout.close();

    return true;
  }
  catch (const std::exception& e)
  {
    return false;
  }
}

bool TuningHistory::loadFromFile(const std::string& filepath)
{
  std::lock_guard<std::mutex> lock(mutex_);

  std::string path = filepath.empty() ? storage_path_ : filepath;
  if (path.empty())
  {
    return false;
  }

  try
  {
    YAML::Node root = YAML::LoadFile(path);

    if (!root["sessions"])
    {
      return false;
    }

    sessions_.clear();
    for (const auto& session_node : root["sessions"])
    {
      sessions_.push_back(TuningSession::fromYaml(session_node));
    }

    return true;
  }
  catch (const std::exception& e)
  {
    return false;
  }
}

void TuningHistory::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  sessions_.clear();
  current_session_id_.clear();
}

size_t TuningHistory::size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return sessions_.size();
}

}  // namespace rm_pid_tuner
