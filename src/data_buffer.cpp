/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2024
 * All rights reserved.
 *******************************************************************************/

#include "rm_pid_tuner/data_buffer.h"

#include <numeric>
#include <stdexcept>
#include <mutex>

namespace rm_pid_tuner
{
DataBuffer::DataBuffer(size_t max_size)
  : max_size_(max_size)
  , cached_min_abs_error_(std::numeric_limits<double>::max())
{
}

void DataBuffer::add(const DataPoint& data)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // 如果缓冲器已满，先移除最老的数据点
  if (buffer_.size() >= max_size_)
  {
    const DataPoint& old_point = buffer_.front();
    removeFromIncrementalStats(old_point);
    buffer_.pop_front();
  }

  // 添加新数据点并更新增量统计
  buffer_.push_back(data);
  updateIncrementalStats(data);
}

void DataBuffer::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  buffer_.clear();
  invalidateCache();
}

bool DataBuffer::isFull() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return buffer_.size() >= max_size_;
}

size_t DataBuffer::size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return buffer_.size();
}

std::vector<DataPoint> DataBuffer::getRecent(size_t n) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  size_t start = buffer_.size() > n ? buffer_.size() - n : 0;
  return std::vector<DataPoint>(buffer_.begin() + start, buffer_.end());
}

std::vector<DataPoint> DataBuffer::getAll() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return std::vector<DataPoint>(buffer_.begin(), buffer_.end());
}

void DataBuffer::updateIncrementalStats(const DataPoint& new_point)
{
  // 注意：调用此函数前必须已持有锁
  double abs_error = std::abs(new_point.error);

  cached_sum_abs_error_ += abs_error;
  cached_sum_sq_error_ += abs_error * abs_error;
  cached_max_abs_error_ = std::max(cached_max_abs_error_, abs_error);
  cached_min_abs_error_ = std::min(cached_min_abs_error_, abs_error);
  cached_count_++;
  cache_valid_ = true;
}

void DataBuffer::removeFromIncrementalStats(const DataPoint& old_point)
{
  // 注意：调用此函数前必须已持有锁
  // 这个函数是近似处理，因为移除旧数据后需要重新计算最大/最小值
  // 为简化实现，标记缓存为无效，下次计算时重新计算
  double abs_error = std::abs(old_point.error);

  cached_sum_abs_error_ -= abs_error;
  cached_sum_sq_error_ -= abs_error * abs_error;
  cached_count_--;

  // 检查是否移除的是最大或最小值，如果是则需要重新扫描
  if (abs_error >= cached_max_abs_error_ - 1e-10 ||
      abs_error <= cached_min_abs_error_ + 1e-10)
  {
    cache_valid_ = false;
  }
}

void DataBuffer::invalidateCache()
{
  cached_sum_abs_error_ = 0.0;
  cached_max_abs_error_ = 0.0;
  cached_min_abs_error_ = std::numeric_limits<double>::max();
  cached_sum_sq_error_ = 0.0;
  cached_count_ = 0;
  cache_valid_ = false;
}

double DataBuffer::calculateAverageError() const
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (buffer_.empty())
    return 0.0;

  if (cache_valid_ && cached_count_ > 0)
  {
    return cached_sum_abs_error_ / cached_count_;
  }

  // 缓存无效，重新计算
  double sum = 0.0;
  for (const auto& point : buffer_)
  {
    sum += std::abs(point.error);
  }
  return sum / buffer_.size();
}

double DataBuffer::calculateMaxError() const
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (buffer_.empty())
    return 0.0;

  if (cache_valid_)
  {
    return cached_max_abs_error_;
  }

  // 缓存无效，重新计算
  double max_err = 0.0;
  for (const auto& point : buffer_)
  {
    max_err = std::max(max_err, std::abs(point.error));
  }
  return max_err;
}

double DataBuffer::calculateMinError() const
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (buffer_.empty())
    return 0.0;

  if (cache_valid_)
  {
    return cached_min_abs_error_;
  }

  // 缓存无效，重新计算
  double min_err = std::numeric_limits<double>::max();
  for (const auto& point : buffer_)
  {
    min_err = std::min(min_err, std::abs(point.error));
  }
  return min_err == std::numeric_limits<double>::max() ? 0.0 : min_err;
}

double DataBuffer::calculateErrorVariance() const
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (buffer_.size() < 2)
    return 0.0;

  double mean = calculateAverageError();

  if (cache_valid_ && cached_count_ > 1)
  {
    // 使用缓存的平方和计算方差
    double variance = (cached_sum_sq_error_ / cached_count_) - (mean * mean);
    return variance > 0 ? variance : 0.0;
  }

  double sum_sq = 0.0;
  for (const auto& point : buffer_)
  {
    double diff = std::abs(point.error) - mean;
    sum_sq += diff * diff;
  }
  return sum_sq / buffer_.size();
}

double DataBuffer::calculateErrorStdDev() const
{
  return std::sqrt(calculateErrorVariance());
}

DataPoint DataBuffer::getLatest() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (buffer_.empty())
  {
    throw std::runtime_error("Buffer is empty");
  }
  return buffer_.back();
}

bool DataBuffer::isEmpty() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return buffer_.empty();
}

DataBuffer::Metrics DataBuffer::calculateMetrics() const
{
  std::lock_guard<std::mutex> lock(mutex_);

  Metrics metrics;

  if (buffer_.empty())
  {
    metrics.avg_error = 0.0;
    metrics.max_error = 0.0;
    metrics.min_error = 0.0;
    metrics.error_variance = 0.0;
    metrics.error_stddev = 0.0;
    metrics.latest_value = 0.0;
    metrics.latest_setpoint = 0.0;
    metrics.latest_p = 0.0;
    metrics.latest_i = 0.0;
    metrics.latest_d = 0.0;
    metrics.itae = 0.0;
    metrics.ise = 0.0;
    metrics.iae = 0.0;
    return metrics;
  }

  // 使用增量统计计算基本指标
  if (cache_valid_ && cached_count_ > 0)
  {
    metrics.avg_error = cached_sum_abs_error_ / cached_count_;
    metrics.max_error = cached_max_abs_error_;
    metrics.min_error = cached_min_abs_error_;

    double mean = metrics.avg_error;
    double variance = (cached_sum_sq_error_ / cached_count_) - (mean * mean);
    metrics.error_variance = variance > 0 ? variance : 0.0;
    metrics.error_stddev = std::sqrt(metrics.error_variance);
  }
  else
  {
    // 重新计算
    double sum = 0.0, sum_sq = 0.0;
    double max_err = 0.0, min_err = std::numeric_limits<double>::max();

    for (const auto& point : buffer_)
    {
      double abs_err = std::abs(point.error);
      sum += abs_err;
      sum_sq += abs_err * abs_err;
      max_err = std::max(max_err, abs_err);
      min_err = std::min(min_err, abs_err);
    }

    metrics.avg_error = sum / buffer_.size();
    metrics.max_error = max_err;
    metrics.min_error = min_err;
    metrics.error_variance = (sum_sq / buffer_.size()) - (metrics.avg_error * metrics.avg_error);
    metrics.error_stddev = std::sqrt(std::max(0.0, metrics.error_variance));
  }

  // 计算高级性能指标
  metrics.itae = 0.0;
  metrics.ise = 0.0;
  metrics.iae = 0.0;

  double t0 = buffer_.front().timestamp;
  for (const auto& point : buffer_)
  {
    double t = point.timestamp - t0;  // 相对时间
    double abs_err = std::abs(point.error);
    metrics.itae += t * abs_err;           // 时间加权绝对误差
    metrics.ise += point.error * point.error;  // 平方误差
    metrics.iae += abs_err;                 // 绝对误差积分
  }

  // 获取最新数据点信息
  const auto& latest = buffer_.back();
  metrics.latest_value = latest.value;
  metrics.latest_setpoint = latest.setpoint;
  metrics.latest_p = latest.p;
  metrics.latest_i = latest.i;
  metrics.latest_d = latest.d;

  return metrics;
}

}  // namespace rm_pid_tuner
