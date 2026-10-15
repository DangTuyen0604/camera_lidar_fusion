#include "state_estimation/wheel_imu_ekf.hpp"

#include <algorithm>
#include <cmath>

namespace state_estimation
{

namespace
{

double wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double square(double value) {return value * value;}

}  // namespace

WheelImuEkf::WheelImuEkf(EkfParameters parameters)
: parameters_(parameters) {}

void WheelImuEkf::reset(double stamp)
{
  x_.setZero();
  P_.setZero();
  P_(kSpeed, kSpeed) = square(parameters_.initial_speed_std);
  P_(kGyroBias, kGyroBias) = square(parameters_.initial_gyro_bias_std);
  P_(kAccelBias, kAccelBias) = square(parameters_.initial_accel_bias_std);
  P_(kPitch, kPitch) = square(parameters_.pitch_std);
  P_(kPitchRateBias, kPitchRateBias) = square(parameters_.initial_gyro_bias_std);
  time_ = stamp;
  last_speed_accept_ = stamp;
  stationary_since_ = -1.0;
  has_imu_ = false;
  last_imu_ = ImuSample{};
  initialized_ = true;
}

void WheelImuEkf::predict(const ImuSample & sample)
{
  if (!initialized_) {
    reset(sample.stamp);
  }
  last_imu_ = sample;
  has_imu_ = true;
  const double dt = sample.stamp - time_;
  if (dt <= 0.0) {
    return;  // Duplicate or out-of-order sample: keep it only as the latest input.
  }
  imu_period_ = std::min(dt, parameters_.max_imu_dt);
  propagate(imu_period_);
  time_ = sample.stamp;
}

void WheelImuEkf::propagate(double dt)
{
  const auto & p = parameters_;
  const double accel_gain = p.use_accelerometer ? 1.0 : 0.0;
  const double w = last_imu_.yaw_rate - x_(kGyroBias);
  const double pitch = x_(kPitch);
  const double gravity_slope = accel_gain * p.gravity * std::cos(pitch);  // d a / d pitch
  const double a = accel_gain *
    (last_imu_.forward_accel - x_(kAccelBias) + p.gravity * std::sin(pitch));
  const double yaw = x_(kYaw);
  const double v = x_(kSpeed);
  const double heading = yaw + 0.5 * w * dt;
  const double c = std::cos(heading);
  const double s = std::sin(heading);
  const double distance = v * dt + 0.5 * a * dt * dt;

  x_(kX) += distance * c;
  x_(kY) += distance * s;
  x_(kYaw) = wrapAngle(yaw + w * dt);
  x_(kSpeed) = v + a * dt;
  x_(kPitch) = pitch + (last_imu_.pitch_rate - x_(kPitchRateBias)) * dt;

  // d distance / d a = dt^2 / 2; a depends on b_a (-1) and pitch (g cos).
  const double half_dt2 = 0.5 * dt * dt;
  Covariance F = Covariance::Identity();
  F(kX, kYaw) = -distance * s;
  F(kX, kSpeed) = c * dt;
  F(kX, kGyroBias) = 0.5 * distance * s * dt;
  F(kX, kAccelBias) = -half_dt2 * c * accel_gain;
  F(kX, kPitch) = half_dt2 * c * gravity_slope;
  F(kY, kYaw) = distance * c;
  F(kY, kSpeed) = s * dt;
  F(kY, kGyroBias) = -0.5 * distance * c * dt;
  F(kY, kAccelBias) = -half_dt2 * s * accel_gain;
  F(kY, kPitch) = half_dt2 * s * gravity_slope;
  F(kYaw, kGyroBias) = -dt;
  F(kSpeed, kAccelBias) = -dt * accel_gain;
  F(kSpeed, kPitch) = dt * gravity_slope;
  F(kPitch, kPitchRateBias) = -dt;

  // Input noise (z gyro, accelerometer, y gyro) mapped through the same model.
  Eigen::Matrix<double, kStateSize, 3> G = Eigen::Matrix<double, kStateSize, 3>::Zero();
  G(kX, 0) = -0.5 * distance * s * dt;
  G(kY, 0) = 0.5 * distance * c * dt;
  G(kYaw, 0) = dt;
  G(kX, 1) = half_dt2 * c;
  G(kY, 1) = half_dt2 * s;
  G(kSpeed, 1) = dt;
  G(kPitch, 2) = dt;
  const double gyro_var = square(p.gyro_noise_density) / (2.0 * dt);
  const double accel_var = p.use_accelerometer ?
    square(p.accel_noise_density) / (2.0 * dt) + square(p.accel_vibration) :
    square(p.no_accel_speed_noise);
  const Eigen::Matrix3d input_cov = Eigen::Vector3d(gyro_var, accel_var, gyro_var).asDiagonal();

  P_ = F * P_ * F.transpose() + G * input_cov * G.transpose();
  P_(kGyroBias, kGyroBias) += square(p.gyro_bias_random_walk) * dt;
  P_(kPitchRateBias, kPitchRateBias) += square(p.gyro_bias_random_walk) * dt;
  P_(kAccelBias, kAccelBias) += square(p.accel_bias_random_walk) * dt;
}

bool WheelImuEkf::fuseScalar(
  const Row & H, double z, double predicted, double r, bool gated, double tolerance,
  double & nis)
{
  const double innovation = z - predicted;
  const double s = (H * P_ * H.transpose())(0, 0) + r;
  nis = innovation * innovation / s;
  if (gated && nis > parameters_.gate_chi2 && std::abs(innovation) > tolerance) {
    return false;
  }
  const Eigen::Matrix<double, kStateSize, 1> K = P_ * H.transpose() / s;
  x_ += K * innovation;
  x_(kYaw) = wrapAngle(x_(kYaw));
  // Joseph form: stays symmetric positive semi-definite in finite precision.
  const Covariance I_KH = Covariance::Identity() - K * H;
  P_ = I_KH * P_ * I_KH.transpose() + K * r * K.transpose();
  return true;
}

WheelUpdateResult WheelImuEkf::update(const WheelSample & sample)
{
  WheelUpdateResult result;
  if (!initialized_) {
    reset(sample.stamp);
  }
  if (has_imu_ && sample.stamp > time_) {
    propagate(std::min(sample.stamp - time_, parameters_.max_imu_dt));
    time_ = sample.stamp;
  }
  const auto & p = parameters_;

  Row H = Row::Zero();
  H(kSpeed) = 1.0;
  const double speed_var = square(p.wheel_speed_std + p.wheel_speed_scale * std::abs(sample.speed));
  const bool overdue = time_ - last_speed_accept_ > p.max_rejection_time;
  if (overdue) {
    // Slip that outlasts the gate's patience, or an IMU speed that drifted:
    // reopen the speed to the wheels instead of rejecting them forever. A
    // drift is most likely an accelerometer bias change, of about
    // innovation / rejection time, so reopen that too.
    const double innovation = sample.speed - x_(kSpeed);
    P_(kSpeed, kSpeed) += square(innovation);
    if (p.use_accelerometer) {
      P_(kAccelBias, kAccelBias) += square(innovation / (time_ - last_speed_accept_));
    }
    result.resynchronized = true;
  }
  result.speed_accepted = fuseScalar(
    H, sample.speed, x_(kSpeed), speed_var, !overdue, p.min_slip_speed, result.speed_nis);
  if (result.speed_accepted) {
    last_speed_accept_ = time_;
    // Level floor while driving normally: anchors pitch, b_a takes the offset.
    H.setZero();
    H(kPitch) = 1.0;
    double nis = 0.0;
    fuseScalar(H, 0.0, x_(kPitch), square(p.pitch_std), false, 0.0, nis);
  }

  const bool still = std::abs(sample.speed) < p.stationary_speed &&
    std::abs(sample.yaw_rate) < p.stationary_yaw_rate;
  if (!still) {
    stationary_since_ = -1.0;
  } else if (stationary_since_ < 0.0) {
    stationary_since_ = time_;
  }
  if (has_imu_ && still && time_ - stationary_since_ >= p.stationary_dwell) {
    // Zero angular-rate update: w_z - b_gz = w_wheel (~0), and the chassis
    // does not pitch either. The gyro sample's own noise is part of the residual.
    const double rate_var = square(p.stationary_yaw_rate_std) +
      square(p.gyro_noise_density) / (2.0 * imu_period_);
    double nis = 0.0;
    H.setZero();
    H(kGyroBias) = -1.0;
    result.stationary = fuseScalar(
      H, sample.yaw_rate, last_imu_.yaw_rate - x_(kGyroBias), rate_var, true, 0.0, nis);
    result.stationary_rejected = !result.stationary;
    H.setZero();
    H(kPitchRateBias) = -1.0;
    fuseScalar(H, 0.0, last_imu_.pitch_rate - x_(kPitchRateBias), rate_var, true, 0.0, nis);
  }
  return result;
}

}  // namespace state_estimation
