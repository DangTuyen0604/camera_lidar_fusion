#include <gtest/gtest.h>

#include "perception_core/latency_tracker.hpp"

TEST(LatencyTracker, ComputesRollingStatistics)
{
  perception_core::LatencyTracker tracker(3);
  tracker.observe(10.0);
  tracker.observe(20.0);
  tracker.observe(30.0);
  tracker.observe(40.0);

  const auto statistics = tracker.statistics();
  EXPECT_EQ(statistics.sample_count, 3U);
  EXPECT_DOUBLE_EQ(statistics.minimum_ms, 20.0);
  EXPECT_DOUBLE_EQ(statistics.maximum_ms, 40.0);
  EXPECT_DOUBLE_EQ(statistics.mean_ms, 30.0);
  EXPECT_DOUBLE_EQ(statistics.percentile_95_ms, 40.0);
}

TEST(LatencyTracker, RejectsInvalidSamples)
{
  perception_core::LatencyTracker tracker;
  EXPECT_THROW(tracker.observe(-1.0), std::invalid_argument);
  EXPECT_THROW(perception_core::LatencyTracker invalid(0), std::invalid_argument);
}
