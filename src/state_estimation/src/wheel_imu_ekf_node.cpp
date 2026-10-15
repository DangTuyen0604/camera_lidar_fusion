#include <cmath>
#include <memory>
#include <string>

#include <Eigen/Geometry>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "state_estimation/wheel_imu_ekf.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace state_estimation
{

// ROS front end of WheelImuEkf. Wheel odometry (twist only) and the IMU go
// in; the fused odometry and the odom -> base TF come out, replacing what the
// wheel odometry publisher would otherwise provide. Every consumer keeps
// reading /odom; the raw wheel odometry moves to /wheel/odom.
class WheelImuEkfNode : public rclcpp::Node
{
public:
  explicit WheelImuEkfNode(const rclcpp::NodeOptions & options)
  : Node("wheel_imu_ekf", options), tf_buffer_(get_clock()), tf_listener_(tf_buffer_),
    tf_broadcaster_(*this)
  {
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    const auto wheel_topic = declare_parameter<std::string>("wheel_odom_topic", "/wheel/odom");
    const auto imu_topic = declare_parameter<std::string>("imu_topic", "/imu");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/odom");

    EkfParameters p;
    p.gyro_noise_density = declare_parameter("gyro_noise_density", p.gyro_noise_density);
    p.accel_noise_density = declare_parameter("accel_noise_density", p.accel_noise_density);
    p.accel_vibration = declare_parameter("accel_vibration", p.accel_vibration);
    p.gyro_bias_random_walk = declare_parameter("gyro_bias_random_walk", p.gyro_bias_random_walk);
    p.accel_bias_random_walk =
      declare_parameter("accel_bias_random_walk", p.accel_bias_random_walk);
    p.gravity = declare_parameter("gravity", p.gravity);
    p.pitch_std = declare_parameter("pitch_std", p.pitch_std);
    p.initial_speed_std = declare_parameter("initial_speed_std", p.initial_speed_std);
    p.initial_gyro_bias_std = declare_parameter("initial_gyro_bias_std", p.initial_gyro_bias_std);
    p.initial_accel_bias_std =
      declare_parameter("initial_accel_bias_std", p.initial_accel_bias_std);
    p.wheel_speed_std = declare_parameter("wheel_speed_std", p.wheel_speed_std);
    p.wheel_speed_scale = declare_parameter("wheel_speed_scale", p.wheel_speed_scale);
    p.stationary_speed = declare_parameter("stationary_speed", p.stationary_speed);
    p.stationary_yaw_rate = declare_parameter("stationary_yaw_rate", p.stationary_yaw_rate);
    p.stationary_yaw_rate_std =
      declare_parameter("stationary_yaw_rate_std", p.stationary_yaw_rate_std);
    p.stationary_dwell = declare_parameter("stationary_dwell", p.stationary_dwell);
    p.gate_chi2 = declare_parameter("gate_chi2", p.gate_chi2);
    p.min_slip_speed = declare_parameter("min_slip_speed", p.min_slip_speed);
    p.max_rejection_time = declare_parameter("max_rejection_time", p.max_rejection_time);
    p.max_imu_dt = declare_parameter("max_imu_dt", p.max_imu_dt);
    p.use_accelerometer = declare_parameter("use_accelerometer", p.use_accelerometer);
    p.no_accel_speed_noise = declare_parameter("no_accel_speed_noise", p.no_accel_speed_noise);
    ekf_ = WheelImuEkf(p);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic, 20);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {onImu(*msg);});
    wheel_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      wheel_topic, 20,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {onWheel(*msg);});
    diagnostics_timer_ = create_wall_timer(
      std::chrono::seconds(1), [this]() {publishDiagnostics();});
  }

private:
  static double seconds(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<double>(stamp.sec) + 1e-9 * static_cast<double>(stamp.nanosec);
  }

  // Rotation from the IMU frame into the base frame (URDF, looked up once).
  bool imuRotation(const std::string & frame)
  {
    if (imu_frame_ == frame) {
      return true;
    }
    try {
      const auto t = tf_buffer_.lookupTransform(base_frame_, frame, tf2::TimePointZero);
      const auto & q = t.transform.rotation;
      imu_to_base_ = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
      imu_frame_ = frame;
      return true;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for %s -> %s: %s", base_frame_.c_str(),
        frame.c_str(), error.what());
      return false;
    }
  }

  void onImu(const sensor_msgs::msg::Imu & msg)
  {
    if (!imuRotation(msg.header.frame_id)) {
      return;
    }
    const Eigen::Vector3d rate = imu_to_base_ * Eigen::Vector3d(
      msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
    const Eigen::Vector3d accel = imu_to_base_ * Eigen::Vector3d(
      msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
    const double stamp = seconds(msg.header.stamp);
    if (ekf_.initialized() && stamp < ekf_.time() - 1.0) {
      // Simulation reset or bag loop: time went backwards.
      RCLCPP_WARN(get_logger(), "IMU time jumped back %.2f s; resetting", ekf_.time() - stamp);
      ekf_.reset(stamp);
    }
    ekf_.predict(ImuSample{stamp, rate.z(), accel.x(), rate.y()});
    publish(msg.header.stamp);
  }

  void onWheel(const nav_msgs::msg::Odometry & msg)
  {
    if (!ekf_.initialized()) {
      return;  // The IMU starts the filter; the wheels alone cannot.
    }
    const auto result = ekf_.update(
      WheelSample{seconds(msg.header.stamp), msg.twist.twist.linear.x,
        msg.twist.twist.angular.z});
    ++wheel_updates_;
    speed_rejections_ += result.speed_accepted ? 0 : 1;
    stationary_updates_ += result.stationary ? 1 : 0;
    resynchronizations_ += result.resynchronized ? 1 : 0;
    slipping_ = !result.speed_accepted;
  }

  void publish(const builtin_interfaces::msg::Time & stamp)
  {
    const auto & x = ekf_.state();
    const auto & P = ekf_.covariance();
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x(WheelImuEkf::kX);
    odom.pose.pose.position.y = x(WheelImuEkf::kY);
    const double yaw = x(WheelImuEkf::kYaw);
    odom.pose.pose.orientation.z = std::sin(0.5 * yaw);
    odom.pose.pose.orientation.w = std::cos(0.5 * yaw);
    // 6x6 row-major (x, y, z, roll, pitch, yaw); the planar states only.
    const int pose_index[3] = {0, 1, 5};
    const int state_index[3] = {WheelImuEkf::kX, WheelImuEkf::kY, WheelImuEkf::kYaw};
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        odom.pose.covariance[pose_index[r] * 6 + pose_index[c]] = P(state_index[r], state_index[c]);
      }
    }
    odom.pose.covariance[2 * 6 + 2] = odom.pose.covariance[3 * 6 + 3] =
      odom.pose.covariance[4 * 6 + 4] = 1e6;  // z, roll, pitch: not estimated
    odom.twist.twist.linear.x = x(WheelImuEkf::kSpeed);
    odom.twist.twist.angular.z = ekf_.yawRate();
    odom.twist.covariance[0] = P(WheelImuEkf::kSpeed, WheelImuEkf::kSpeed);
    odom.twist.covariance[35] = P(WheelImuEkf::kGyroBias, WheelImuEkf::kGyroBias);
    odom_pub_->publish(odom);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odom.header;
      transform.child_frame_id = base_frame_;
      transform.transform.translation.x = odom.pose.pose.position.x;
      transform.transform.translation.y = odom.pose.pose.position.y;
      transform.transform.rotation = odom.pose.pose.orientation;
      tf_broadcaster_.sendTransform(transform);
    }
  }

  void publishDiagnostics()
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "wheel_imu_ekf";
    status.hardware_id = base_frame_;
    if (!ekf_.initialized()) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "no IMU data";
    } else if (slipping_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "wheel speed rejected (slip)";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "ok";
    }
    const auto add = [&status](const std::string & key, double value) {
        diagnostic_msgs::msg::KeyValue item;
        item.key = key;
        item.value = std::to_string(value);
        status.values.push_back(item);
      };
    add("gyro_bias_rad_s", ekf_.state()(WheelImuEkf::kGyroBias));
    add("accel_bias_m_s2", ekf_.state()(WheelImuEkf::kAccelBias));
    add("wheel_updates", static_cast<double>(wheel_updates_));
    add("speed_rejections", static_cast<double>(speed_rejections_));
    add("stationary_updates", static_cast<double>(stationary_updates_));
    add("resynchronizations", static_cast<double>(resynchronizations_));
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
  }

  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_{true};
  WheelImuEkf ekf_;
  std::string imu_frame_;
  Eigen::Matrix3d imu_to_base_{Eigen::Matrix3d::Identity()};
  bool slipping_{false};
  std::size_t wheel_updates_{0};
  std::size_t speed_rejections_{0};
  std::size_t stationary_updates_{0};
  std::size_t resynchronizations_{0};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_sub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

}  // namespace state_estimation

RCLCPP_COMPONENTS_REGISTER_NODE(state_estimation::WheelImuEkfNode)
