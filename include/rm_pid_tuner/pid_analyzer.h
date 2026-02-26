/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2024
 * All rights reserved.
 *******************************************************************************/

#pragma once

#include <string>
#include <vector>
#include <map>
#include <memory>

#include "rm_pid_tuner/data_buffer.h"

namespace rm_pid_tuner
{
/**
 * @brief 分析结果结构体
 */
struct AnalysisResult
{
  std::string analysis;       // 分析说明
  double suggested_p;         // 建议的 P 参数
  double suggested_i;         // 建议的 I 参数
  double suggested_d;         // 建议的 D 参数
  std::string status;         // 状态: TUNING, DONE, ERROR

  // 性能评估
  double performance_score;   // 性能评分 (0-100)
  bool needs_tuning;          // 是否需要继续调参
  std::string recommendation; // 建议
};

/**
 * @brief PID 分析器接口
 */
class IPidAnalyzer
{
public:
  virtual ~IPidAnalyzer() = default;

  /**
   * @brief 分析数据并给出调参建议
   */
  virtual AnalysisResult analyze(const DataBuffer::Metrics& metrics,
                                 double current_p, double current_i, double current_d,
                                 bool conservative_mode = false) = 0;

  /**
   * @brief 获取分析器名称
   */
  virtual std::string getName() const = 0;
};

/**
 * @brief 基于规则的分析器
 *
 * 使用经典控制理论规则进行 PID 参数分析
 */
class RuleBasedAnalyzer : public IPidAnalyzer
{
public:
  explicit RuleBasedAnalyzer(double oscillation_threshold = 1.0,
                              double slow_response_threshold = 10.0,
                              double steady_error_threshold = 2.0);

  AnalysisResult analyze(const DataBuffer::Metrics& metrics,
                        double current_p, double current_i, double current_d,
                        bool conservative_mode = false) override;

  std::string getName() const override { return "RuleBasedAnalyzer"; }

private:
  double oscillation_threshold_;
  double slow_response_threshold_;
  double steady_error_threshold_;

  // 辅助分析方法
  double calculatePerformanceScore(const DataBuffer::Metrics& metrics) const;
  std::string generateRecommendation(const DataBuffer::Metrics& metrics) const;
};

/**
 * @brief Ziegler-Nichols 分析器
 *
 * 基于 Ziegler-Nichols 调参方法的分析器
 */
class ZieglerNicholsAnalyzer : public IPidAnalyzer
{
public:
  explicit ZieglerNicholsAnalyzer(double gain_factor = 0.6);

  AnalysisResult analyze(const DataBuffer::Metrics& metrics,
                        double current_p, double current_i, double current_d,
                        bool conservative_mode = false) override;

  std::string getName() const override { return "ZieglerNicholsAnalyzer"; }

private:
  double gain_factor_;  // 增益折扣因子

  // Z-N 参数计算
  double calculateKu(const DataBuffer::Metrics& metrics, double current_p) const;
  double calculatePu(const DataBuffer::Metrics& metrics) const;
};

/**
 * @brief 混合分析器
 *
 * 结合多种分析方法，选择最佳建议
 */
class HybridAnalyzer : public IPidAnalyzer
{
public:
  HybridAnalyzer();

  void addAnalyzer(std::unique_ptr<IPidAnalyzer> analyzer);
  void setWeight(const std::string& analyzer_name, double weight);

  AnalysisResult analyze(const DataBuffer::Metrics& metrics,
                        double current_p, double current_i, double current_d,
                        bool conservative_mode = false) override;

  std::string getName() const override { return "HybridAnalyzer"; }

private:
  std::vector<std::unique_ptr<IPidAnalyzer>> analyzers_;
  std::map<std::string, double> weights_;

  AnalysisResult combineResults(const std::vector<AnalysisResult>& results) const;
};

/**
 * @brief 分析器工厂
 */
class AnalyzerFactory
{
public:
  enum class AnalyzerType
  {
    RULE_BASED,
    ZIEGLER_NICHOLS,
    HYBRID
  };

  static std::unique_ptr<IPidAnalyzer> create(AnalyzerType type);

  static std::unique_ptr<HybridAnalyzer> createHybrid();
};

}  // namespace rm_pid_tuner
