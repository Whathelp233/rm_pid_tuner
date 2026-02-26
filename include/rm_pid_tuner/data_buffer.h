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

#include <deque>
#include <vector>
#include <cmath>
#include <algorithm>
#include <mutex>
#include <memory>

namespace rm_pid_tuner
{
/**
 * @brief 数据点结构体，用于存储单个采样点的数据
 */
struct DataPoint
{
  double timestamp;   // 时间戳 (秒)
  double setpoint;    // 设定值
  double value;       // 实际值
  double error;       // 误差
  double control;     // 控制输出 (如 PWM)
  double p, i, d;     // 当前 PID 参数
};

/**
 * @brief 数据缓冲器，使用滑动窗口存储最近的 N 条数据
 * 线程安全，支持增量指标计算
 */
class DataBuffer
{
public:
  explicit DataBuffer(size_t max_size = 100);

  /**
   * @brief 添加数据点到缓冲器
   */
  void add(const DataPoint& data);

  /**
   * @brief 清空缓冲器
   */
  void clear();

  /**
   * @brief 检查缓冲器是否已满
   */
  bool isFull() const;

  /**
   * @brief 获取缓冲器中的数据点数量
   */
  size_t size() const;

  /**
   * @brief 检查缓冲器是否为空
   */
  bool isEmpty() const;

  /**
   * @brief 获取最近 N 条数据（线程安全）
   * @param n 要获取的数据点数量
   * @return 数据点 vector
   */
  std::vector<DataPoint> getRecent(size_t n) const;

  /**
   * @brief 获取所有数据（线程安全）
   */
  std::vector<DataPoint> getAll() const;

  /**
   * @brief 获取最新的数据点
   */
  DataPoint getLatest() const;

  /**
   * @brief 计算指标汇总（使用缓存优化）
   */
  struct Metrics
  {
    double avg_error;
    double max_error;
    double min_error;
    double error_variance;
    double error_stddev;
    double latest_value;
    double latest_setpoint;
    double latest_p, latest_i, latest_d;
    // 额外的性能指标
    double itae;    // 积分时间绝对误差
    double ise;     // 积分平方误差
    double iae;     // 积分绝对误差
  };

  Metrics calculateMetrics() const;

  // 保留旧接口兼容性
  double calculateAverageError() const;
  double calculateMaxError() const;
  double calculateMinError() const;
  double calculateErrorVariance() const;
  double calculateErrorStdDev() const;

private:
  // 增量计算辅助函数
  void updateIncrementalStats(const DataPoint& new_point);
  void removeFromIncrementalStats(const DataPoint& old_point);
  void invalidateCache();

  mutable std::mutex mutex_;
  std::deque<DataPoint> buffer_;
  size_t max_size_;

  // 增量统计缓存
  mutable bool cache_valid_{false};
  mutable double cached_sum_abs_error_{0.0};
  mutable double cached_max_abs_error_{0.0};
  mutable double cached_min_abs_error_{std::numeric_limits<double>::max()};
  mutable double cached_sum_sq_error_{0.0};
  mutable size_t cached_count_{0};
};

}  // namespace rm_pid_tuner
