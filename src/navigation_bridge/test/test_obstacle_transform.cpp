#include <algorithm>
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
  message.detections.resize(7);
  for (auto & detection : message.detections) {
    detection.valid = true;
    detection.detection.confidence = 0.9F;
    detection.position.x = 1.0;
  }
  message.detections[0].valid = false;
  message.detections[1].detection.confidence = 0.1F;
  message.detections[2].position.x = std::numeric_limits<double>::quiet_NaN();
  message.detections[3].position.x = 100.0;
  message.detections[4].position.y = std::numeric_limits<double>::infinity();
  message.detections[5].position.z = -std::numeric_limits<double>::infinity();

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
  EXPECT_EQ(counters.non_finite, 3U);
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

TEST(DetectionObstacleBridge, AcceptsForwardDistanceInCameraOpticalZ)
{
  fusion_interfaces::msg::FusedDetectionArray message;
  message.detections.resize(1);
  auto & detection = message.detections.front();
  detection.valid = true;
  detection.detection.class_name = "person";
  detection.detection.confidence = 0.9F;
  detection.position.z = 2.0;

  navigation_bridge::BridgeConfig config;
  navigation_bridge::RejectionCounters counters;
  const auto candidates = navigation_bridge::DetectionObstacleBridge::extractCandidates(
    message, config, &counters);

  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_FLOAT_EQ(candidates.front().position.z, 2.0F);
  EXPECT_EQ(candidates.front().class_name, "person");
  EXPECT_EQ(counters.accepted, 1U);
}

TEST(DetectionObstacleBridge, ExpandsFootprintAfterTargetFrameTransform)
{
  const navigation_bridge::ObstaclePoint center{2.0F, 1.0F, 0.2F};
  const auto footprint = navigation_bridge::DetectionObstacleBridge::expandFootprint(
    center, "person", 0.10);
  ASSERT_FALSE(footprint.empty());
  auto x_limits = std::minmax_element(
    footprint.begin(), footprint.end(),
    [](const auto & left, const auto & right) {return left.x < right.x;});
  auto y_limits = std::minmax_element(
    footprint.begin(), footprint.end(),
    [](const auto & left, const auto & right) {return left.y < right.y;});
  EXPECT_FLOAT_EQ(x_limits.first->x, 1.7F);
  EXPECT_FLOAT_EQ(x_limits.second->x, 2.3F);
  EXPECT_FLOAT_EQ(y_limits.first->y, 0.7F);
  EXPECT_FLOAT_EQ(y_limits.second->y, 1.3F);
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
