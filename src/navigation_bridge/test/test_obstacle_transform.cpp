#include <limits>

#include <gtest/gtest.h>

#include "navigation_bridge/detection_obstacle_bridge.hpp"

TEST(DetectionObstacleBridge, KeepsOnlyValidConfidentNearbyDetections)
{
  fusion_interfaces::msg::FusedDetectionArray message;
  message.detections.resize(4);
  message.detections[0].valid = true;
  message.detections[0].detection.confidence = 0.9F;
  message.detections[0].position.x = 3.0;
  message.detections[0].position.y = 4.0;
  message.detections[1].valid = false;
  message.detections[2].valid = true;
  message.detections[2].detection.confidence = 0.2F;
  message.detections[3].valid = true;
  message.detections[3].detection.confidence = 0.9F;
  message.detections[3].position.x = 100.0;

  const auto result = navigation_bridge::DetectionObstacleBridge::extractObstacles(
    message, 0.5, 20.0);
  ASSERT_EQ(result.size(), 1U);
  EXPECT_FLOAT_EQ(result.front().x, 3.0F);
  EXPECT_FLOAT_EQ(result.front().y, 4.0F);
}

TEST(DetectionObstacleBridge, RejectsInvalidThresholds)
{
  fusion_interfaces::msg::FusedDetectionArray message;
  EXPECT_THROW(
    navigation_bridge::DetectionObstacleBridge::extractObstacles(message, 2.0, 20.0),
    std::invalid_argument);
}

TEST(DetectionObstacleBridge, RejectsInvalidNonFiniteConfidenceAndRange)
{
  fusion_interfaces::msg::FusedDetectionArray message;
  message.detections.resize(5);
  for (auto & detection : message.detections) {
    detection.valid = true;
    detection.detection.confidence = 0.9F;
    detection.position.x = 1.0;
  }
  message.detections[0].valid = false;
  message.detections[1].detection.confidence = 0.1F;
  message.detections[2].position.x = std::numeric_limits<double>::quiet_NaN();
  message.detections[3].position.x = 100.0;

  navigation_bridge::BridgeConfig config;
  config.minimum_confidence = 0.5;
  config.minimum_range = 0.2;
  config.maximum_range = 20.0;
  navigation_bridge::RejectionCounters counters;
  const auto result = navigation_bridge::DetectionObstacleBridge::extractFootprints(
    message, config, &counters);
  EXPECT_FALSE(result.empty());
  EXPECT_EQ(counters.accepted, 1U);
  EXPECT_EQ(counters.invalid, 1U);
  EXPECT_EQ(counters.confidence, 1U);
  EXPECT_EQ(counters.non_finite, 1U);
  EXPECT_EQ(counters.range, 1U);
}

TEST(DetectionObstacleBridge, UsesClassSpecificFootprints)
{
  const auto person = navigation_bridge::DetectionObstacleBridge::footprintSize("person");
  const auto pallet = navigation_bridge::DetectionObstacleBridge::footprintSize("pallet");
  const auto box = navigation_bridge::DetectionObstacleBridge::footprintSize("box");
  EXPECT_DOUBLE_EQ(person.first, 0.60);
  EXPECT_DOUBLE_EQ(person.second, 0.60);
  EXPECT_DOUBLE_EQ(pallet.first, 1.20);
  EXPECT_DOUBLE_EQ(pallet.second, 0.80);
  EXPECT_DOUBLE_EQ(box.first, 0.60);
  EXPECT_DOUBLE_EQ(box.second, 0.60);
}

TEST(DetectionObstacleBridge, RejectsBadFootprintConfiguration)
{
  fusion_interfaces::msg::FusedDetectionArray message;
  navigation_bridge::BridgeConfig config;
  config.minimum_range = 5.0;
  config.maximum_range = 1.0;
  EXPECT_THROW(
    navigation_bridge::DetectionObstacleBridge::extractFootprints(message, config),
    std::invalid_argument);
}
