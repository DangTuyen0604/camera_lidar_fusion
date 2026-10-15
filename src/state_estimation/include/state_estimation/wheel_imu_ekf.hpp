#ifndef STATE_ESTIMATION__WHEEL_IMU_EKF_HPP_
#define STATE_ESTIMATION__WHEEL_IMU_EKF_HPP_

#include <Eigen/Dense>

namespace state_estimation
{

// Planar odometry of a differential-drive robot from wheel encoders and a
// 6-axis IMU, as an extended Kalman filter on
//
//   x = [p_x, p_y, yaw, v, b_gz, b_a, pitch, b_gy]
//
// v is the forward speed, b_gz / b_gy the z / y gyro biases [rad/s], b_a the
// forward accelerometer bias [m/s^2] and pitch the chassis pitch [rad], ROS
// convention (positive nose down). The IMU drives the prediction:
//
//   yaw'   = yaw + (w_z - b_gz) dt
//   pitch' = pitch + (w_y - b_gy) dt
//   v'     = v + a dt,   a = f_x - b_a + g sin(pitch)
//   p'     = p + (v dt + a dt^2 / 2) [cos, sin](yaw + w dt / 2)
//
// f_x is the measured specific force. g sin(pitch) removes the gravity that
// leaks into it when the chassis pitches, as it does pushing an obstacle
// (0.2 deg there is 0.04 m/s^2). A constant pitch and b_a affect f_x alike
// and cannot be told apart; only g sin(pitch) - b_a is observed. A level
// floor constraint, pitch = 0 +- pitch_std, is fused whenever the wheel speed
// is accepted, so b_a takes the static offset and the pitch follows the y
// gyro while the wheels slip, which is exactly when the leak matters. (A
// Gauss-Markov pitch decaying to zero failed here: it forgot the pitch of a
// sustained push, and its driving noise inflated the speed variance until the
// slip gate opened.)
//
// The wheel speed is a measurement, v_wheel = v. Comparing it with the
// integrated accelerometer makes b_a observable and exposes wheel slip: a
// wheel speed whose normalized innovation squared exceeds the chi-square gate
// is rejected, so a robot whose wheels spin against a pallet keeps the IMU's
// speed instead of "driving" through it. Wheel noise grows with speed (slip
// and radius error are proportional to it).
//
// Heading comes from the gyro alone. While the robot turns, a gyro bias and a
// wheel track/scrub error are not separable: fusing a systematic wheel yaw
// error as if it were white noise makes the filter learn a wrong bias that
// reproduces the wheel heading error. The wheel yaw rate is therefore fused
// only while the encoders report standstill, as a zero angular-rate update
// (w_z - b_gz = 0, and w_y - b_gy = 0), which is when the biases are learned.
// Standstill must last stationary_dwell first: when the wheels stop, the
// chassis still rocks for a few hundred ms, and at low speed the quantized
// encoders read zero between counts. Because the heading is correlated with
// the bias (by the time since it was last observed), one contaminated update
// would also "correct" the heading; the update is gated for the same reason.
struct EkfParameters
{
  // IMU noise densities (per sqrt(Hz)); a sample averaged over dt then has
  // std = density / sqrt(2 dt). Defaults: TDK InvenSense MPU-6500, the
  // OpenAMRobot's IMU (PS-MPU-6500A-01 rev 1.3).
  double gyro_noise_density{1.75e-4};        // 0.01 deg/s/sqrt(Hz) [rad/s/sqrt(Hz)]
  double accel_noise_density{2.94e-3};       // 300 ug/sqrt(Hz) [m/s^2/sqrt(Hz)]
  // Chassis vibration seen by the accelerometer [m/s^2 rms per sample]. It is
  // much larger than the sensor noise but integrates to ~zero velocity, so
  // only a fraction of it is modelled as random-walk speed noise.
  double accel_vibration{0.05};
  double gyro_bias_random_walk{1.4e-4};      // ~2 K/h warm-up at 0.24 deg/s/K [rad/s / sqrt(s)]
  double accel_bias_random_walk{1.0e-3};     // [m/s^2 / sqrt(s)]
  double gravity{9.80665};                   // [m/s^2]
  double pitch_std{0.0035};                  // level floor constraint [rad] (0.2 deg)

  double initial_speed_std{0.05};            // [m/s]
  // The firmware removes the gyro offset at boot; 1 deg/s also covers a boot
  // while the robot was moving. The accelerometer offset is not removed.
  double initial_gyro_bias_std{0.0175};      // [rad/s]
  double initial_accel_bias_std{0.3};        // 30 mg: MPU-6500 zero-g +-60 mg as 2 sigma

  // Wheel speed std = base + scale * |speed|.
  // 1024 counts per wheel revolution read at 50 Hz: one count is 0.034 m/s.
  double wheel_speed_std{0.01};              // [m/s]
  double wheel_speed_scale{0.03};
  // Standstill: both wheel readings below these. The encoders are then
  // exact up to quantization, so w = 0 is a strong measurement.
  double stationary_speed{0.002};            // [m/s]
  double stationary_yaw_rate{0.005};         // [rad/s]
  double stationary_yaw_rate_std{0.002};     // [rad/s]
  double stationary_dwell{0.25};             // [s]

  double gate_chi2{10.83};                   // chi-square, 1 dof, p = 0.001
  // A wheel speed within min_slip_speed of the filter's is never rejected.
  // While the wheels are rejected the speed is the integrated accelerometer,
  // which drifts faster than its noise model says (residual pitch leak and
  // bias). Without this floor the gate keeps rejecting the wheels after the
  // slip ends. It must stay between that drift and the slip speed minus it,
  // which is the limit of slip detection from these sensors alone: slower
  // slips pass, and longer ones end in a resynchronization; localization
  // against the map corrects both.
  double min_slip_speed{0.1};                // [m/s]
  // After this long with every wheel speed rejected, the speed covariance is
  // inflated to the innovation and the wheels are trusted again: the gate
  // must not lock the filter onto a diverged IMU-only speed forever. Longer
  // than a typical push against an obstacle, which must not end this way.
  double max_rejection_time{5.0};            // [s]
  double max_imu_dt{0.1};                    // longer IMU gaps are clamped [s]

  // false: ignore the accelerometer (a = 0) and let the speed random-walk
  // with no_accel_speed_noise instead, i.e. wheels + gyro only, as the usual
  // robot_localization setup for a ground robot. Kept for comparison.
  bool use_accelerometer{true};
  double no_accel_speed_noise{2.0};          // [m/s^2 rms per sample]
};

struct ImuSample
{
  double stamp{0.0};          // [s]
  double yaw_rate{0.0};       // z gyro, robot frame [rad/s]
  double forward_accel{0.0};  // x specific force, robot frame [m/s^2]
  double pitch_rate{0.0};     // y gyro, robot frame [rad/s]
};

struct WheelSample
{
  double stamp{0.0};
  double speed{0.0};       // [m/s]
  double yaw_rate{0.0};    // [rad/s]
};

struct WheelUpdateResult
{
  bool speed_accepted{false};
  bool stationary{false};           // a zero angular-rate update was applied
  bool stationary_rejected{false};  // standstill, but the gyro disagreed
  bool resynchronized{false};       // max_rejection_time forced the speed back in
  double speed_nis{0.0};
};

class WheelImuEkf
{
public:
  static constexpr int kStateSize = 8;
  using State = Eigen::Matrix<double, kStateSize, 1>;
  using Covariance = Eigen::Matrix<double, kStateSize, kStateSize>;
  enum Index
  {
    kX = 0, kY = 1, kYaw = 2, kSpeed = 3, kGyroBias = 4, kAccelBias = 5, kPitch = 6,
    kPitchRateBias = 7
  };

  explicit WheelImuEkf(EkfParameters parameters = EkfParameters{});

  // Start at the origin of the odometry frame, at rest.
  void reset(double stamp);
  bool initialized() const noexcept {return initialized_;}

  // Propagate with the IMU sample that covers (time(), sample.stamp].
  void predict(const ImuSample & sample);
  // Fuse one encoder reading. A reading ahead of the filter is reached by
  // holding the last IMU sample; one behind it is applied now (the IMU runs
  // at twice the wheel rate, so it is at most one IMU period late).
  WheelUpdateResult update(const WheelSample & sample);

  double time() const noexcept {return time_;}
  const State & state() const noexcept {return x_;}
  const Covariance & covariance() const noexcept {return P_;}
  double yawRate() const noexcept {return last_imu_.yaw_rate - x_(kGyroBias);}
  const EkfParameters & parameters() const noexcept {return parameters_;}

private:
  using Row = Eigen::Matrix<double, 1, kStateSize>;

  void propagate(double dt);
  // Scalar measurement z with prediction H x and variance r. Rejected when
  // gated, its NIS exceeds the gate and |z - prediction| > tolerance.
  bool fuseScalar(
    const Row & H, double z, double predicted, double r, bool gated, double tolerance,
    double & nis);

  EkfParameters parameters_;
  bool initialized_{false};
  bool has_imu_{false};
  double time_{0.0};
  double last_speed_accept_{0.0};
  double stationary_since_{-1.0};  // < 0: the wheels are moving
  double imu_period_{0.01};  // last IMU sample spacing, for the gyro noise [s]
  ImuSample last_imu_{};
  State x_{State::Zero()};
  Covariance P_{Covariance::Zero()};
};

}  // namespace state_estimation

#endif  // STATE_ESTIMATION__WHEEL_IMU_EKF_HPP_
