#include "perception_core/latency_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <vector>

namespace perception_core
{

LatencyTracker::LatencyTracker(std::size_t window_size)
: window_size_(window_size)
{
  if (window_size_ == 0) {
    throw std::invalid_argument("Latency window size must be positive");
  }
}

void LatencyTracker::observe(double latency_ms)
{
  if (!std::isfinite(latency_ms) || latency_ms < 0.0) {
    throw std::invalid_argument("Latency sample must be finite and non-negative");
  }
  samples_.push_back(latency_ms);
  while (samples_.size() > window_size_) {
    samples_.pop_front();
  }
}

void LatencyTracker::clear() noexcept
{
  samples_.clear();
}

LatencyStatistics LatencyTracker::statistics() const
{
  LatencyStatistics result;
  result.sample_count = samples_.size();
  if (samples_.empty()) {
    return result;
  }

  const auto [minimum, maximum] = std::minmax_element(samples_.begin(), samples_.end());
  result.minimum_ms = *minimum;
  result.maximum_ms = *maximum;
  result.mean_ms = std::accumulate(samples_.begin(), samples_.end(), 0.0) /
    static_cast<double>(samples_.size());

  std::vector<double> sorted(samples_.begin(), samples_.end());
  std::sort(sorted.begin(), sorted.end());
  const auto percentile = [&sorted](double value) {
      const auto index = static_cast<std::size_t>(
        std::ceil(value * static_cast<double>(sorted.size()))) - 1U;
      return sorted[index];
    };
  result.percentile_50_ms = percentile(0.50);
  result.percentile_95_ms = percentile(0.95);
  return result;
}

std::size_t LatencyTracker::windowSize() const noexcept
{
  return window_size_;
}

}  // namespace perception_core
