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
