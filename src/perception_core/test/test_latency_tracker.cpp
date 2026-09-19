#include <limits>

#include <gtest/gtest.h>

#include "perception_core/latency_tracker.hpp"

TEST(LatencyTracker, ComputesBoundedRollingPercentiles)
{
  perception_core::LatencyTracker tracker(3);
  for (const double sample : {10.0, 20.0, 30.0, 40.0}) {
    tracker.observe(sample);
  }
  const auto statistics = tracker.statistics();
  EXPECT_EQ(statistics.sample_count, 3U);
  EXPECT_DOUBLE_EQ(statistics.minimum_ms, 20.0);
  EXPECT_DOUBLE_EQ(statistics.maximum_ms, 40.0);
  EXPECT_DOUBLE_EQ(statistics.mean_ms, 30.0);
  EXPECT_DOUBLE_EQ(statistics.percentile_50_ms, 30.0);
  EXPECT_DOUBLE_EQ(statistics.percentile_95_ms, 40.0);
}

TEST(LatencyTracker, RejectsInvalidSamplesAndCanClear)
{
  perception_core::LatencyTracker tracker;
  EXPECT_THROW(tracker.observe(-1.0), std::invalid_argument);
  EXPECT_THROW(tracker.observe(std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
  tracker.observe(1.0);
  tracker.clear();
  EXPECT_EQ(tracker.statistics().sample_count, 0U);
  EXPECT_THROW(perception_core::LatencyTracker invalid(0), std::invalid_argument);
}
