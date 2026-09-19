#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "fusion_interfaces/msg/sync_status.hpp"
#include "message_filters/subscriber.hpp"
#include "message_filters/sync_policies/approximate_time.hpp"
#include "message_filters/synchronizer.hpp"
#include "perception_core/timestamp_sync.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace perception_core
{

class SensorSyncNode : public rclcpp::Node
{
public:
  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  using PointCloud = sensor_msgs::msg::PointCloud2;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    Image, CameraInfo, PointCloud>;

  SensorSyncNode()
  : Node("sensor_sync_node")
  {
    const std::string image_topic = declare_parameter<std::string>(
      "image_topic", "/camera/image");
    const std::string camera_info_topic = declare_parameter<std::string>(
      "camera_info_topic", "/camera/camera_info");
    const std::string pointcloud_topic = declare_parameter<std::string>(
      "pointcloud_topic", "/lidar/points");
    const std::string synced_image_topic = declare_parameter<std::string>(
      "synced_image_topic", "/fusion/synced/image");
    const std::string synced_camera_info_topic = declare_parameter<std::string>(
      "synced_camera_info_topic", "/fusion/synced/camera_info");
    const std::string synced_pointcloud_topic = declare_parameter<std::string>(
      "synced_pointcloud_topic", "/fusion/synced/points");
    const std::string status_topic = declare_parameter<std::string>(
      "status_topic", "/fusion/sync_status");
    const int queue_size = declare_parameter<int>("queue_size", 20);
    sync_tolerance_ms_ = declare_parameter<double>("sync_tolerance_ms", 50.0);
    camera_frame_id_ = declare_parameter<std::string>(
      "camera_frame_id", "camera_optical_frame");
    lidar_frame_id_ = declare_parameter<std::string>(
      "lidar_frame_id", "velodyne");

    if (queue_size <= 0 || !std::isfinite(sync_tolerance_ms_) || sync_tolerance_ms_ < 0.0) {
      throw std::invalid_argument("Invalid synchronization parameters");
    }

    const auto sensor_qos = rclcpp::SensorDataQoS();
    image_publisher_ = create_publisher<Image>(synced_image_topic, sensor_qos);
    camera_info_publisher_ = create_publisher<CameraInfo>(
      synced_camera_info_topic, sensor_qos);
    pointcloud_publisher_ = create_publisher<PointCloud>(
      synced_pointcloud_topic, sensor_qos);
    status_publisher_ = create_publisher<fusion_interfaces::msg::SyncStatus>(status_topic, 10);
    monitor_ = std::make_unique<TimestampSyncMonitor>(
      sync_tolerance_ms_, static_cast<std::size_t>(queue_size),
      std::array<std::string, 3>{camera_frame_id_, camera_frame_id_, lidar_frame_id_});

    image_subscription_.subscribe(this, image_topic, sensor_qos.get_rmw_qos_profile());
    camera_info_subscription_.subscribe(
      this, camera_info_topic, sensor_qos.get_rmw_qos_profile());
    pointcloud_subscription_.subscribe(
      this, pointcloud_topic, sensor_qos.get_rmw_qos_profile());
    image_subscription_.registerCallback(
      [this](const Image::ConstSharedPtr & message) {
        observeInput(SensorStream::Image, message->header);
      });
    camera_info_subscription_.registerCallback(
      [this](const CameraInfo::ConstSharedPtr & message) {
        observeInput(SensorStream::CameraInfo, message->header);
      });
    pointcloud_subscription_.registerCallback(
      [this](const PointCloud::ConstSharedPtr & message) {
        observeInput(SensorStream::PointCloud, message->header);
      });

    synchronizer_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(queue_size), image_subscription_, camera_info_subscription_,
      pointcloud_subscription_);
    synchronizer_->setMaxIntervalDuration(
      rclcpp::Duration::from_seconds(sync_tolerance_ms_ / 1000.0));
    synchronizer_->registerCallback(std::bind(
        &SensorSyncNode::synchronizedCallback, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    RCLCPP_INFO(
      get_logger(), "Synchronizing image, CameraInfo and PointCloud2 within %.1f ms",
      sync_tolerance_ms_);
  }

private:
  static std::int64_t stampNanoseconds(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
           static_cast<std::int64_t>(stamp.nanosec);
  }

  void observeInput(SensorStream stream, const std_msgs::msg::Header & header)
  {
    const auto fault = monitor_->observe(stream, stampNanoseconds(header.stamp), header.frame_id);
    if (fault != SyncFault::None) {
      publishStatus(header, fault);
    }
  }

  void publishStatus(const std_msgs::msg::Header & header, SyncFault fault)
  {
    fusion_interfaces::msg::SyncStatus status;
    status.header = header;
    status.status = static_cast<std::uint8_t>(fault);
    status.healthy = fault == SyncFault::None;
    status.camera_lidar_offset_ms = static_cast<float>(monitor_->latestCameraLidarOffsetMs());
    const auto & counters = monitor_->counters();
    status.synchronized_pairs = counters.synchronized_pairs;
    status.dropped_messages = counters.dropped_messages;
    status.out_of_order_messages = counters.out_of_order_messages;
    status.wrong_frame_messages = counters.wrong_frame_messages;
    switch (fault) {
      case SyncFault::None: status.message = "timestamps synchronized"; break;
      case SyncFault::Delayed: status.message = "timestamp tolerance exceeded"; break;
      case SyncFault::OutOfOrder: status.message = "out-of-order timestamp"; break;
      case SyncFault::WrongFrame: status.message = "unexpected frame id"; break;
    }
    status_publisher_->publish(status);
  }

  void synchronizedCallback(
    const Image::ConstSharedPtr & image,
    const CameraInfo::ConstSharedPtr & camera_info,
    const PointCloud::ConstSharedPtr & pointcloud)
  {
    if (image->header.frame_id != camera_frame_id_ ||
      camera_info->header.frame_id != camera_frame_id_ ||
      pointcloud->header.frame_id != lidar_frame_id_)
    {
      publishStatus(image->header, SyncFault::WrongFrame);
      return;
    }
    const std::int64_t image_ns = stampNanoseconds(image->header.stamp);
    const std::int64_t camera_info_ns = stampNanoseconds(camera_info->header.stamp);
    const std::int64_t pointcloud_ns = stampNanoseconds(pointcloud->header.stamp);
    const auto fault = monitor_->observeMatch(image_ns, camera_info_ns, pointcloud_ns);
    if (fault != SyncFault::None) {
      publishStatus(image->header, fault);
      return;
    }

    image_publisher_->publish(*image);
    camera_info_publisher_->publish(*camera_info);
    pointcloud_publisher_->publish(*pointcloud);
    publishStatus(image->header, SyncFault::None);
  }

  double sync_tolerance_ms_{50.0};
  std::string camera_frame_id_;
  std::string lidar_frame_id_;
  std::unique_ptr<TimestampSyncMonitor> monitor_;
  message_filters::Subscriber<Image> image_subscription_;
  message_filters::Subscriber<CameraInfo> camera_info_subscription_;
  message_filters::Subscriber<PointCloud> pointcloud_subscription_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer_;
  rclcpp::Publisher<Image>::SharedPtr image_publisher_;
  rclcpp::Publisher<CameraInfo>::SharedPtr camera_info_publisher_;
  rclcpp::Publisher<PointCloud>::SharedPtr pointcloud_publisher_;
  rclcpp::Publisher<fusion_interfaces::msg::SyncStatus>::SharedPtr status_publisher_;
};

}  // namespace perception_core

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception_core::SensorSyncNode>());
  rclcpp::shutdown();
  return 0;
}
