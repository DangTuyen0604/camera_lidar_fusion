#ifndef PERCEPTION_CORE__LATENCY_TRACKER_HPP_
#define PERCEPTION_CORE__LATENCY_TRACKER_HPP_

#include <cstddef>
#include <deque>

namespace perception_core
{

struct LatencyStatistics
{
  std::size_t sample_count{0};
  double mean_ms{0.0};
  double minimum_ms{0.0};
  double maximum_ms{0.0};
  double percentile_95_ms{0.0};
};

class LatencyTracker
{
public:
  explicit LatencyTracker(std::size_t window_size = 200);

  void observe(double latency_ms);
  void clear() noexcept;
  LatencyStatistics statistics() const;
  std::size_t windowSize() const noexcept;

private:
  std::size_t window_size_;
  std::deque<double> samples_;
};

}  // namespace perception_core

#endif  // PERCEPTION_CORE__LATENCY_TRACKER_HPP_
