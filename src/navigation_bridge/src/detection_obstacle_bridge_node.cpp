#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "fusion_interfaces/msg/fused_detection_array.hpp"
#include "navigation_bridge/detection_obstacle_bridge.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace navigation_bridge
{

class DetectionObstacleBridgeNode : public rclcpp::Node
{
public:
  DetectionObstacleBridgeNode()
  : Node("detection_obstacle_bridge")
  {
    const auto input_topic = declare_parameter<std::string>(
      "input_topic", "/fusion/detections_3d");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/navigation/detection_obstacles");
    minimum_confidence_ = declare_parameter<double>("minimum_confidence", 0.35);
    maximum_range_ = declare_parameter<double>("maximum_range", 50.0);

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<fusion_interfaces::msg::FusedDetectionArray>(
      input_topic, 10,
      std::bind(&DetectionObstacleBridgeNode::callback, this, std::placeholders::_1));
  }

private:
  void callback(const fusion_interfaces::msg::FusedDetectionArray::ConstSharedPtr message)
  {
    const auto obstacles = DetectionObstacleBridge::extractObstacles(
      *message, minimum_confidence_, maximum_range_);
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = message->header;
    cloud.height = 1;
    cloud.width = static_cast<std::uint32_t>(obstacles.size());
    cloud.is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(obstacles.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    for (const auto & obstacle : obstacles) {
      *x = obstacle.x;
      *y = obstacle.y;
      *z = obstacle.z;
      ++x;
      ++y;
      ++z;
    }
    publisher_->publish(cloud);
  }

  double minimum_confidence_{0.35};
  double maximum_range_{50.0};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<fusion_interfaces::msg::FusedDetectionArray>::SharedPtr subscription_;
};

}  // namespace navigation_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<navigation_bridge::DetectionObstacleBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
