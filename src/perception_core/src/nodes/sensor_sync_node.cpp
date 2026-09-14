#include <algorithm>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <unordered_map>

#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace perception_core
{

class SyncAuditNode : public rclcpp::Node
{
public:
  SyncAuditNode()
  : Node("sensor_sync_node")
  {
    camera_frame_id_ = declare_parameter<std::string>(
      "camera_frame_id", "camera_optical_frame");
    lidar_frame_id_ = declare_parameter<std::string>(
      "lidar_frame_id", "velodyne");

    const auto image_topic = declare_parameter<std::string>(
      "image_topic", "/kitti/camera/image_raw");
    const auto camera_info_topic = declare_parameter<std::string>(
      "camera_info_topic", "/kitti/camera/camera_info");
    const auto pointcloud_topic = declare_parameter<std::string>(
      "pointcloud_topic", "/kitti/velodyne/points");
    const auto overlay_topic = declare_parameter<std::string>(
      "overlay_topic", "/kitti/camera/lidar_overlay");

    const auto qos = rclcpp::SensorDataQoS();
    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
        check_camera_frame(message->header.frame_id, "image");
        observe(message->header.stamp, kImage);
      });
    camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic, qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
        check_camera_frame(message->header.frame_id, "camera_info");
        if (message->width == 0 || message->height == 0) {
          RCLCPP_ERROR(get_logger(), "CameraInfo has an invalid image size");
        }
        observe(message->header.stamp, kCameraInfo);
      });
    pointcloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic, qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        if (message->header.frame_id != lidar_frame_id_) {
          RCLCPP_ERROR(
            get_logger(), "point cloud frame mismatch: expected=%s actual=%s",
            lidar_frame_id_.c_str(), message->header.frame_id.c_str());
        }
        if (message->point_step < 16 || message->width == 0) {
          RCLCPP_ERROR(get_logger(), "PointCloud2 has an invalid layout");
        }
        observe(message->header.stamp, kPointCloud);
      });
    overlay_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      overlay_topic, qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
        check_camera_frame(message->header.frame_id, "overlay");
        observe(message->header.stamp, kOverlay);
      });

    RCLCPP_INFO(
      get_logger(),
      "Auditing exact timestamp synchronization for image, CameraInfo, "
      "PointCloud2 and projection overlay");
  }

private:
  static constexpr std::uint8_t kImage = 1U << 0;
  static constexpr std::uint8_t kCameraInfo = 1U << 1;
  static constexpr std::uint8_t kPointCloud = 1U << 2;
  static constexpr std::uint8_t kOverlay = 1U << 3;
  static constexpr std::uint8_t kComplete =
    kImage | kCameraInfo | kPointCloud | kOverlay;

  void check_camera_frame(const std::string & frame_id, const char * source)
  {
    if (frame_id != camera_frame_id_) {
      RCLCPP_ERROR(
        get_logger(), "%s frame mismatch: expected=%s actual=%s", source,
        camera_frame_id_.c_str(), frame_id.c_str());
    }
  }

  void observe(const builtin_interfaces::msg::Time & stamp, std::uint8_t mask)
  {
    const auto stamp_ns =
      static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
    auto & observed_mask = observations_[stamp_ns];
    observed_mask |= mask;

    if (observed_mask != kComplete) {
      return;
    }

    ++synchronized_frames_;
    observations_.erase(stamp_ns);
    if (synchronized_frames_ == 1 || synchronized_frames_ % 50 == 0) {
      RCLCPP_INFO(
        get_logger(), "Validated %zu exactly synchronized frames",
        synchronized_frames_);
    }

    completed_stamps_.push_back(stamp_ns);
    while (completed_stamps_.size() > 10) {
      completed_stamps_.pop_front();
    }

    if (observations_.size() > 50) {
      const auto cutoff = completed_stamps_.empty() ? stamp_ns : completed_stamps_.front();
      for (auto iterator = observations_.begin(); iterator != observations_.end(); ) {
        if (iterator->first < cutoff) {
          iterator = observations_.erase(iterator);
        } else {
          ++iterator;
        }
      }
    }
  }

  std::string camera_frame_id_;
  std::string lidar_frame_id_;
  std::unordered_map<std::int64_t, std::uint8_t> observations_;
  std::deque<std::int64_t> completed_stamps_;
  std::size_t synchronized_frames_{0};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr overlay_subscription_;
};

}  // namespace perception_core

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception_core::SyncAuditNode>());
  rclcpp::shutdown();
  return 0;
}
