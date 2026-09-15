#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>

#include "message_filters/subscriber.hpp"
#include "message_filters/sync_policies/approximate_time.hpp"
#include "message_filters/synchronizer.hpp"
#include "perception_core/lidar_projector.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace perception_core
{
namespace
{

Eigen::Matrix4d transformMatrix(const geometry_msgs::msg::TransformStamped & transform)
{
  const auto & rotation = transform.transform.rotation;
  Eigen::Quaterniond quaternion(rotation.w, rotation.x, rotation.y, rotation.z);
  if (!quaternion.coeffs().allFinite() || quaternion.norm() < 1.0e-12) {
    throw std::runtime_error("TF contains an invalid quaternion");
  }
  quaternion.normalize();
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  matrix.topLeftCorner<3, 3>() = quaternion.toRotationMatrix();
  matrix(0, 3) = transform.transform.translation.x;
  matrix(1, 3) = transform.transform.translation.y;
  matrix(2, 3) = transform.transform.translation.z;
  return matrix;
}

std::vector<PointXYZI> readPoints(const sensor_msgs::msg::PointCloud2 & cloud)
{
  sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
  sensor_msgs::PointCloud2ConstIterator<float> intensity(cloud, "intensity");
  std::vector<PointXYZI> points;
  points.reserve(static_cast<std::size_t>(cloud.width) * cloud.height);
  for (; x != x.end(); ++x, ++y, ++z, ++intensity) {
    points.push_back({Eigen::Vector3d(*x, *y, *z), *intensity});
  }
  return points;
}

cv::Scalar depthColor(double depth, double minimum, double maximum)
{
  const double normalized = std::clamp((depth - minimum) / (maximum - minimum), 0.0, 1.0);
  return cv::Scalar(255.0 * (1.0 - normalized), 255.0 * normalized, 40.0);
}

}  // namespace

class LidarProjectionNode : public rclcpp::Node
{
public:
  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  using PointCloud = sensor_msgs::msg::PointCloud2;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    Image, CameraInfo, PointCloud>;

  LidarProjectionNode()
  : Node("lidar_projection_node"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    const auto image_topic = declare_parameter<std::string>(
      "image_topic", "/fusion/synced/image");
    const auto camera_info_topic = declare_parameter<std::string>(
      "camera_info_topic", "/fusion/synced/camera_info");
    const auto pointcloud_topic = declare_parameter<std::string>(
      "pointcloud_topic", "/fusion/synced/points");
    const auto overlay_topic = declare_parameter<std::string>(
      "overlay_topic", "/fusion/lidar_overlay");
    minimum_depth_ = declare_parameter<double>("minimum_depth", 1.0);
    maximum_depth_ = declare_parameter<double>("maximum_depth", 80.0);
    point_radius_ = declare_parameter<int>("point_radius", 2);
    const int queue_size = declare_parameter<int>("queue_size", 10);
    if (minimum_depth_ <= 0.0 || maximum_depth_ <= minimum_depth_ ||
      point_radius_ <= 0 || queue_size <= 0)
    {
      throw std::invalid_argument("Invalid projection parameters");
    }

    const auto qos = rclcpp::SensorDataQoS();
    publisher_ = create_publisher<Image>(overlay_topic, qos);
    image_subscription_.subscribe(this, image_topic, qos.get_rmw_qos_profile());
    camera_info_subscription_.subscribe(this, camera_info_topic, qos.get_rmw_qos_profile());
    pointcloud_subscription_.subscribe(this, pointcloud_topic, qos.get_rmw_qos_profile());
    synchronizer_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(queue_size), image_subscription_, camera_info_subscription_,
      pointcloud_subscription_);
    synchronizer_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.05));
    synchronizer_->registerCallback(std::bind(
        &LidarProjectionNode::callback, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  }

private:
  void callback(
    const Image::ConstSharedPtr & image,
    const CameraInfo::ConstSharedPtr & camera_info,
    const PointCloud::ConstSharedPtr & cloud)
  {
    if (publisher_->get_subscription_count() == 0) {
      return;
    }
    try {
      CalibrationData calibration;
      calibration.image_width = static_cast<int>(camera_info->width);
      calibration.image_height = static_cast<int>(camera_info->height);
      for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 4; ++column) {
          calibration.projection_matrix(row, column) = camera_info->p[row * 4 + column];
        }
      }
      const auto transform = tf_buffer_.lookupTransform(
        camera_info->header.frame_id, cloud->header.frame_id,
        rclcpp::Time(cloud->header.stamp), rclcpp::Duration::from_seconds(0.05));
      calibration.lidar_to_camera = transformMatrix(transform);

      const LidarProjector projector(std::move(calibration));
      const auto projected = projector.project(
        readPoints(*cloud), static_cast<int>(camera_info->width),
        static_cast<int>(camera_info->height), minimum_depth_, maximum_depth_);
      auto converted = cv_bridge::toCvCopy(image, "bgr8");
      for (const auto & point : projected) {
        cv::circle(
          converted->image,
          cv::Point(static_cast<int>(std::lround(point.u)),
          static_cast<int>(std::lround(point.v))),
          point_radius_, depthColor(point.depth, minimum_depth_, maximum_depth_), -1,
          cv::LINE_AA);
      }
      converted->header = image->header;
      publisher_->publish(*converted->toImageMsg());
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Projection TF: %s", error.what());
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Projection failed: %s", error.what());
    }
  }

  double minimum_depth_{1.0};
  double maximum_depth_{80.0};
  int point_radius_{2};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  message_filters::Subscriber<Image> image_subscription_;
  message_filters::Subscriber<CameraInfo> camera_info_subscription_;
  message_filters::Subscriber<PointCloud> pointcloud_subscription_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer_;
  rclcpp::Publisher<Image>::SharedPtr publisher_;
};

}  // namespace perception_core

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception_core::LidarProjectionNode>());
  rclcpp::shutdown();
  return 0;
}
