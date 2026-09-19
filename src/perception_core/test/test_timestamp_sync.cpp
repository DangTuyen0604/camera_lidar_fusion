#include <gtest/gtest.h>

#include "perception_core/timestamp_sync.hpp"

using perception_core::SensorStream;
using perception_core::SyncFault;
using perception_core::TimestampSyncMonitor;

TEST(TimestampSync, AcceptsApproximateMatchWithinTolerance)
{
  TimestampSyncMonitor monitor(50.0, 10, {"camera", "camera", "lidar"});
  EXPECT_EQ(monitor.observe(SensorStream::Image, 1000000000, "camera"), SyncFault::None);
  EXPECT_EQ(monitor.observe(SensorStream::CameraInfo, 1010000000, "camera"), SyncFault::None);
  EXPECT_EQ(monitor.observe(SensorStream::PointCloud, 1049000000, "lidar"), SyncFault::None);
  EXPECT_EQ(monitor.observeMatch(1000000000, 1010000000, 1049000000), SyncFault::None);
  EXPECT_EQ(monitor.counters().synchronized_pairs, 1U);
  EXPECT_NEAR(monitor.latestCameraLidarOffsetMs(), 49.0, 1.0e-12);
}

TEST(TimestampSync, DetectsDelayAboveFiftyMilliseconds)
{
  TimestampSyncMonitor monitor(50.0, 10, {"camera", "camera", "lidar"});
  monitor.observe(SensorStream::Image, 1000000000, "camera");
  monitor.observe(SensorStream::CameraInfo, 1000000000, "camera");
  EXPECT_EQ(monitor.observe(SensorStream::PointCloud, 1051000000, "lidar"), SyncFault::Delayed);
  EXPECT_EQ(monitor.observeMatch(1000000000, 1000000000, 1051000000), SyncFault::Delayed);
}

TEST(TimestampSync, AccountsForOutOfOrderAndWrongFrame)
{
  TimestampSyncMonitor monitor(50.0, 10, {"camera", "camera", "lidar"});
  EXPECT_EQ(monitor.observe(SensorStream::Image, 20, "camera"), SyncFault::None);
  EXPECT_EQ(monitor.observe(SensorStream::Image, 10, "camera"), SyncFault::OutOfOrder);
  EXPECT_EQ(monitor.observe(SensorStream::PointCloud, 20, "map"), SyncFault::WrongFrame);
  EXPECT_EQ(monitor.counters().dropped_messages, 2U);
  EXPECT_EQ(monitor.counters().out_of_order_messages, 1U);
  EXPECT_EQ(monitor.counters().wrong_frame_messages, 1U);
}
