/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2024
 * All rights reserved.
 *******************************************************************************/

#include "rm_pid_tuner/pid_analyzer.h"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace rm_pid_tuner
{
// ==================== RuleBasedAnalyzer ====================

RuleBasedAnalyzer::RuleBasedAnalyzer(double oscillation_threshold,
                                     double slow_response_threshold,
                                     double steady_error_threshold)
  : oscillation_threshold_(oscillation_threshold)
  , slow_response_threshold_(slow_response_threshold)
  , steady_error_threshold_(steady_error_threshold)
{
}

AnalysisResult RuleBasedAnalyzer::analyze(const DataBuffer::Metrics& metrics,
                                          double current_p, double current_i, double current_d,
                                          bool conservative_mode)
{
  AnalysisResult result;
  result.suggested_p = current_p;
  result.suggested_i = current_i;
  result.suggested_d = current_d;
  result.status = "TUNING";

  // 计算性能评分
  result.performance_score = calculatePerformanceScore(metrics);
  result.needs_tuning = result.performance_score < 80;
  result.recommendation = generateRecommendation(metrics);

  // 检查是否完成
  if (metrics.avg_error < 0.3 && metrics.error_stddev < 0.5)
  {
    result.status = "DONE";
    result.analysis = "调参完成：误差已达到目标范围";
    result.needs_tuning = false;
    return result;
  }

  // 分析并给出建议
  std::ostringstream analysis_ss;

  // 1. 振荡检测
  if (metrics.error_stddev > oscillation_threshold_)
  {
    // 存在振荡，减小 Kp，增大 Kd
    double factor = conservative_mode ? 0.85 : 0.9;
    result.suggested_p *= factor;
    result.suggested_d *= conservative_mode ? 1.3 : 1.2;
    analysis_ss << "[振荡] 标准差=" << metrics.error_stddev << " > " << oscillation_threshold_
                << "，减小Kp(" << factor << "x)，增大Kd\n";
  }

  // 2. 响应速度检测
  if (metrics.avg_error > slow_response_threshold_)
  {
    // 响应太慢，增大 Kp
    double factor = conservative_mode ? 1.2 : 1.3;
    result.suggested_p *= factor;
    analysis_ss << "[响应慢] 平均误差=" << metrics.avg_error << " > " << slow_response_threshold_
                << "，增大Kp(" << factor << "x)\n";
  }
  else if (metrics.avg_error > steady_error_threshold_)
  {
    // 存在稳态误差，增大 Ki
    double factor = conservative_mode ? 1.3 : 1.4;
    result.suggested_i *= factor;
    analysis_ss << "[稳态误差] 平均误差=" << metrics.avg_error << " > " << steady_error_threshold_
                << "，增大Ki(" << factor << "x)\n";
  }

  // 3. 超调检测 (通过最大误差判断)
  if (metrics.max_error > 2 * metrics.avg_error && metrics.max_error > 5.0)
  {
    // 存在超调，减小 Kp，增大 Kd
    double factor = conservative_mode ? 0.7 : 0.8;
    result.suggested_p *= factor;
    result.suggested_d *= conservative_mode ? 1.5 : 1.3;
    analysis_ss << "[超调] 最大误差=" << metrics.max_error
                << "，减小Kp(" << factor << "x)，增大Kd\n";
  }

  // 4. 保守模式额外处理
  if (conservative_mode)
  {
    // 保守模式下限制参数变化
    result.suggested_p = current_p + (result.suggested_p - current_p) * 0.5;
    result.suggested_i = current_i + (result.suggested_i - current_i) * 0.5;
    result.suggested_d = current_d + (result.suggested_d - current_d) * 0.5;
  }

  // 5. 参数下限保护
  result.suggested_p = std::max(result.suggested_p, 0.01);
  result.suggested_i = std::max(result.suggested_i, 0.001);
  result.suggested_d = std::max(result.suggested_d, 0.0);

  result.analysis = analysis_ss.str();
  if (result.analysis.empty())
  {
    result.analysis = "性能良好，保持当前参数";
    result.needs_tuning = false;
  }

  return result;
}

double RuleBasedAnalyzer::calculatePerformanceScore(const DataBuffer::Metrics& metrics) const
{
  // 评分公式：100 - (avg_error * 10 + stddev * 5 + max_error * 2)
  double score = 100.0;

  // 平均误差扣分
  score -= std::min(metrics.avg_error * 10, 30.0);

  // 标准差扣分（振荡）
  score -= std::min(metrics.error_stddev * 5, 30.0);

  // 最大误差扣分
  score -= std::min(metrics.max_error * 2, 20.0);

  // ITAE 扣分（如果有）
  if (metrics.itae > 0)
  {
    score -= std::min(metrics.itae * 0.01, 20.0);
  }

  return std::max(0.0, std::min(100.0, score));
}

std::string RuleBasedAnalyzer::generateRecommendation(const DataBuffer::Metrics& metrics) const
{
  if (metrics.avg_error < 0.3)
  {
    return "Excellent: 误差已达到目标范围";
  }
  else if (metrics.avg_error < 1.0)
  {
    return "Good: 接近目标，可微调";
  }
  else if (metrics.error_stddev > oscillation_threshold_)
  {
    return "Warning: 存在振荡，建议减小Kp";
  }
  else if (metrics.avg_error > slow_response_threshold_)
  {
    return "Warning: 响应较慢，建议增大Kp";
  }
  else if (metrics.avg_error > steady_error_threshold_)
  {
    return "Info: 存在稳态误差，建议增大Ki";
  }
  else
  {
    return "Normal: 继续调参";
  }
}

// ==================== ZieglerNicholsAnalyzer ====================

ZieglerNicholsAnalyzer::ZieglerNicholsAnalyzer(double gain_factor)
  : gain_factor_(gain_factor)
{
}

AnalysisResult ZieglerNicholsAnalyzer::analyze(const DataBuffer::Metrics& metrics,
                                               double current_p, double current_i, double current_d,
                                               bool conservative_mode)
{
  AnalysisResult result;

  // 计算 Z-N 参数
  double Ku = calculateKu(metrics, current_p);
  double Pu = calculatePu(metrics);

  // 应用 Z-N 公式（带折扣因子）
  double factor = conservative_mode ? gain_factor_ * 0.8 : gain_factor_;

  result.suggested_p = Ku * factor;
  result.suggested_i = Ku * Pu * factor * 0.5 / 1.2;  // Ti = Pu/1.2
  result.suggested_d = Ku * Pu * factor * 0.125;     // Td = Pu/8

  result.status = metrics.avg_error < 0.5 ? "DONE" : "TUNING";
  result.performance_score = 100 - metrics.avg_error * 10;
  result.needs_tuning = result.performance_score < 80;

  std::ostringstream oss;
  oss << "[Z-N方法] Ku=" << Ku << ", Pu=" << Pu
      << " → P=" << result.suggested_p
      << ", I=" << result.suggested_i
      << ", D=" << result.suggested_d;
  result.analysis = oss.str();
  result.recommendation = "基于 Ziegler-Nichols 方法计算";

  return result;
}

double ZieglerNicholsAnalyzer::calculateKu(const DataBuffer::Metrics& metrics,
                                           double current_p) const
{
  // 估计临界增益
  // 基于当前增益和振荡程度
  double oscillation_factor = 1.0 + metrics.error_stddev * 0.1;
  return current_p * oscillation_factor;
}

double ZieglerNicholsAnalyzer::calculatePu(const DataBuffer::Metrics& metrics) const
{
  // 估计临界周期
  // 基于误差变化率
  return std::max(1.0, 2.0 + metrics.error_stddev * 0.5);
}

// ==================== HybridAnalyzer ====================

HybridAnalyzer::HybridAnalyzer()
{
}

void HybridAnalyzer::addAnalyzer(std::unique_ptr<IPidAnalyzer> analyzer)
{
  std::string name = analyzer->getName();
  analyzers_.push_back(std::move(analyzer));
  if (weights_.find(name) == weights_.end())
  {
    weights_[name] = 1.0;  // 默认权重
  }
}

void HybridAnalyzer::setWeight(const std::string& analyzer_name, double weight)
{
  weights_[analyzer_name] = weight;
}

AnalysisResult HybridAnalyzer::analyze(const DataBuffer::Metrics& metrics,
                                       double current_p, double current_i, double current_d,
                                       bool conservative_mode)
{
  std::vector<AnalysisResult> results;

  for (const auto& analyzer : analyzers_)
  {
    results.push_back(analyzer->analyze(metrics, current_p, current_i, current_d, conservative_mode));
  }

  if (results.empty())
  {
    AnalysisResult fallback;
    fallback.suggested_p = current_p;
    fallback.suggested_i = current_i;
    fallback.suggested_d = current_d;
    fallback.status = "ERROR";
    fallback.analysis = "No analyzers available";
    return fallback;
  }

  return combineResults(results);
}

AnalysisResult HybridAnalyzer::combineResults(const std::vector<AnalysisResult>& results) const
{
  AnalysisResult combined;
  combined.suggested_p = 0;
  combined.suggested_i = 0;
  combined.suggested_d = 0;
  combined.performance_score = 0;
  double total_weight = 0;

  for (size_t i = 0; i < results.size() && i < analyzers_.size(); ++i)
  {
    std::string name = analyzers_[i]->getName();
    double weight = weights_.count(name) ? weights_.at(name) : 1.0;

    combined.suggested_p += results[i].suggested_p * weight;
    combined.suggested_i += results[i].suggested_i * weight;
    combined.suggested_d += results[i].suggested_d * weight;
    combined.performance_score += results[i].performance_score * weight;
    total_weight += weight;
  }

  if (total_weight > 0)
  {
    combined.suggested_p /= total_weight;
    combined.suggested_i /= total_weight;
    combined.suggested_d /= total_weight;
    combined.performance_score /= total_weight;
  }

  // 使用最佳结果的 status 和 recommendation
  combined.status = "TUNING";
  combined.needs_tuning = true;
  for (const auto& r : results)
  {
    if (r.status == "DONE")
    {
      combined.status = "DONE";
      combined.needs_tuning = false;
      break;
    }
  }

  combined.recommendation = "混合分析结果";
  std::ostringstream oss;
  oss << "[混合分析] 综合了 " << results.size() << " 个分析器的结果";
  combined.analysis = oss.str();

  return combined;
}

// ==================== AnalyzerFactory ====================

std::unique_ptr<IPidAnalyzer> AnalyzerFactory::create(AnalyzerType type)
{
  switch (type)
  {
    case AnalyzerType::RULE_BASED:
      return std::make_unique<RuleBasedAnalyzer>();
    case AnalyzerType::ZIEGLER_NICHOLS:
      return std::make_unique<ZieglerNicholsAnalyzer>();
    case AnalyzerType::HYBRID:
      return createHybrid();
    default:
      return std::make_unique<RuleBasedAnalyzer>();
  }
}

std::unique_ptr<HybridAnalyzer> AnalyzerFactory::createHybrid()
{
  auto hybrid = std::make_unique<HybridAnalyzer>();
  hybrid->addAnalyzer(std::make_unique<RuleBasedAnalyzer>());
  hybrid->addAnalyzer(std::make_unique<ZieglerNicholsAnalyzer>());
  hybrid->setWeight("RuleBasedAnalyzer", 0.6);
  hybrid->setWeight("ZieglerNicholsAnalyzer", 0.4);
  return hybrid;
}

}  // namespace rm_pid_tuner
