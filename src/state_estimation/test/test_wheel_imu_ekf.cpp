#include <algorithm>
#include <cmath>
#include <functional>
#include <random>

#include <gtest/gtest.h>

#include "state_estimation/wheel_imu_ekf.hpp"

namespace
{

using state_estimation::EkfParameters;
using state_estimation::ImuSample;
using state_estimation::WheelImuEkf;
using state_estimation::WheelSample;

constexpr double kPi = 3.14159265358979323846;

struct Truth
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double v{0.0};
};

// Commanded true motion: forward speed and yaw rate at time t.
using Profile = std::function<void(double t, double & v, double & w)>;

struct Errors
{
  double gyro_bias{0.0};          // [rad/s]
  double accel_bias{0.0};         // [m/s^2]
  double wheel_speed_scale{1.0};  // wheel radius error
  double wheel_yaw_scale{1.0};    // track width error / in-place scrub
  double gyro_noise{0.0};         // per-sample std
  double accel_noise{0.0};
  // Wheels report this speed while the robot stands still, in [begin, end).
  double slip_begin{-1.0};
  double slip_end{-1.0};
  double slip_speed{0.0};
  // Chassis pitch while slipping [rad, nose down positive], reached and left
  // over 0.1 s: the pushing chassis tilts and gravity leaks into the
  // accelerometer, while the y gyro sees the tilt happen.
  double slip_pitch{0.0};
  double pitch_y_gyro_bias{0.0};
  // Wheels report wheel_lag_scale x the true speed in [begin, end): they and
  // the IMU disagree slightly while accelerating (MuJoCo: ~0.08 m/s).
  double wheel_lag_begin{-1.0};
  double wheel_lag_end{-1.0};
  double wheel_lag_scale{1.0};
  // Added to the accelerometer bias from this time on.
  double accel_bias_step_time{-1.0};
  double accel_bias_step{0.0};
  // The chassis keeps turning at settle_rate in [begin, end) with the wheels
  // stopped (rocking after an in-place turn).
  double settle_begin{-1.0};
  double settle_end{-1.0};
  double settle_rate{0.0};
};

struct Run
{
  Truth truth;
  Truth wheel_only;  // dead reckoning of the wheel twist, as /odom used to be
  WheelImuEkf ekf;
  std::size_t speed_rejections{0};
  std::size_t resynchronizations{0};
};

// 1 kHz truth; IMU at 100 Hz reports the average over its period (dv/dt,
// dyaw/dt), wheels at 50 Hz report the period's travel / dt.
Run simulate(double duration, const Profile & profile, const Errors & e, EkfParameters p = {})
{
  Run run{Truth{}, Truth{}, WheelImuEkf(p)};
  std::mt19937 rng(7);
  std::normal_distribution<double> unit(0.0, 1.0);
  const double dt = 0.001;
  run.ekf.reset(0.0);
  double imu_v0 = 0.0;
  double imu_yaw0 = 0.0;
  double pitch = 0.0;
  double imu_pitch0 = 0.0;
  double wheel_s = 0.0;
  double wheel_yaw = 0.0;
  const int steps = static_cast<int>(std::lround(duration / dt));
  for (int k = 1; k <= steps; ++k) {
    const double t = k * dt;
    double v = 0.0;
    double w = 0.0;
    profile(t, v, w);
    const bool slipping = t >= e.slip_begin && t < e.slip_end;
    const bool settling = t >= e.settle_begin && t < e.settle_end;
    const double true_v = slipping ? 0.0 : v;
    const double true_w = slipping ? 0.0 : (settling ? e.settle_rate : w);
    auto & tr = run.truth;
    tr.yaw += true_w * dt;
    tr.x += true_v * std::cos(tr.yaw) * dt;
    tr.y += true_v * std::sin(tr.yaw) * dt;
    tr.v = true_v;
    const double pitch_target = slipping ? e.slip_pitch : 0.0;
    pitch += std::clamp(pitch_target - pitch, -std::abs(e.slip_pitch) * dt / 0.1,
        std::abs(e.slip_pitch) * dt / 0.1);
    const double lag = t >= e.wheel_lag_begin && t < e.wheel_lag_end ? e.wheel_lag_scale : 1.0;
    wheel_s += (slipping ? e.slip_speed : v * lag) * e.wheel_speed_scale * dt;
    wheel_yaw += w * e.wheel_yaw_scale * dt;

    if (k % 10 == 0) {
      const double bias =
        e.accel_bias + (e.accel_bias_step_time >= 0.0 && t >= e.accel_bias_step_time ?
        e.accel_bias_step : 0.0);
      ImuSample imu;
      imu.stamp = t;
      imu.yaw_rate = (tr.yaw - imu_yaw0) / 0.01 + e.gyro_bias + e.gyro_noise * unit(rng);
      // Specific force: dv/dt minus gravity along the pitched x axis.
      imu.forward_accel = (tr.v - imu_v0) / 0.01 - 9.80665 * std::sin(pitch) + bias +
        e.accel_noise * unit(rng);
      imu.pitch_rate = (pitch - imu_pitch0) / 0.01 + e.pitch_y_gyro_bias + e.gyro_noise * unit(rng);
      imu_yaw0 = tr.yaw;
      imu_v0 = tr.v;
      imu_pitch0 = pitch;
      run.ekf.predict(imu);
    }
    if (k % 20 == 0) {
      WheelSample wheel{t, wheel_s / 0.02, wheel_yaw / 0.02};
      auto & wo = run.wheel_only;
      wo.yaw += wheel.yaw_rate * 0.02;
      wo.x += wheel.speed * std::cos(wo.yaw) * 0.02;
      wo.y += wheel.speed * std::sin(wo.yaw) * 0.02;
      wheel_s = 0.0;
      wheel_yaw = 0.0;
      const auto result = run.ekf.update(wheel);
      run.speed_rejections += result.speed_accepted ? 0 : 1;
      run.resynchronizations += result.resynchronized ? 1 : 0;
    }
  }
  return run;
}

double positionError(const WheelImuEkf & ekf, const Truth & truth)
{
  return std::hypot(ekf.state()(WheelImuEkf::kX) - truth.x, ekf.state()(WheelImuEkf::kY) - truth.y);
}

double angleError(double a, double b)
{
  return std::abs(std::atan2(std::sin(a - b), std::cos(a - b)));
}

}  // namespace

TEST(WheelImuEkf, LearnsGyroBiasAtStandstill)
{
  Errors e;
  e.gyro_bias = 0.0175;  // 1 deg/s, the BMI088 zero-rate offset
  e.gyro_noise = 0.0014;
  const auto run = simulate(20.0, [](double, double & v, double & w) {v = w = 0.0;}, e);
  EXPECT_NEAR(run.ekf.state()(WheelImuEkf::kGyroBias), e.gyro_bias, 0.001);
  // Integrating the raw gyro would have turned 20 deg; the filter holds still.
  EXPECT_LT(angleError(run.ekf.state()(WheelImuEkf::kYaw), 0.0), 0.01);
  EXPECT_LT(positionError(run.ekf, run.truth), 1e-3);
}

TEST(WheelImuEkf, TracksAStraightLineWithAccelerationAndBraking)
{
  const Profile profile = [](double t, double & v, double & w) {
      v = t < 1.0 ? 0.5 * t : (t < 9.0 ? 0.5 : std::max(0.0, 0.5 - 0.5 * (t - 9.0)));
      w = 0.0;
    };
  Errors e;
  e.accel_bias = 0.1;
  e.accel_noise = 0.011;
  const auto run = simulate(11.0, profile, e);
  EXPECT_NEAR(run.truth.x, 4.5, 1e-6);
  EXPECT_LT(positionError(run.ekf, run.truth), 0.01);
  EXPECT_NEAR(run.ekf.state()(WheelImuEkf::kSpeed), 0.0, 0.01);
  EXPECT_EQ(run.speed_rejections, 0u);
}

TEST(WheelImuEkf, GyroCorrectsWheelHeadingErrorOnACircle)
{
  // 5 s at rest (as docked at start), then 2 laps of a 1 m radius circle.
  // The wheels over-report the turn by 2 % (a track width error / scrub);
  // the gyro has a 0.5 deg/s bias.
  const Profile profile = [](double t, double & v, double & w) {
      v = t < 5.0 ? 0.0 : 0.3;
      w = t < 5.0 ? 0.0 : 0.3;
    };
  Errors e;
  e.wheel_yaw_scale = 1.02;
  e.gyro_bias = 0.0087;
  e.gyro_noise = 0.0014;
  const double duration = 5.0 + 2.0 * 2.0 * kPi / 0.3;
  const auto run = simulate(duration, profile, e);
  const double wheel_heading = angleError(run.wheel_only.yaw, run.truth.yaw);
  const double ekf_heading = angleError(run.ekf.state()(WheelImuEkf::kYaw), run.truth.yaw);
  EXPECT_GT(wheel_heading, 0.2);   // 2 % of 4 pi rad
  EXPECT_LT(ekf_heading, 0.05);
  EXPECT_LT(ekf_heading, 0.25 * wheel_heading);
  EXPECT_LT(
    positionError(run.ekf, run.truth),
    0.25 * std::hypot(run.wheel_only.x - run.truth.x, run.wheel_only.y - run.truth.y));
}

TEST(WheelImuEkf, RejectsWheelSpeedWhileTheWheelsSlip)
{
  // Driving at 0.3 m/s into a pallet at t = 4 s: the robot stops dead, the
  // wheels keep turning at 0.25 m/s for 1.5 s.
  const Profile profile = [](double t, double & v, double & w) {
      v = t < 0.6 ? 0.5 * t : 0.3;
      w = 0.0;
    };
  Errors e;
  e.slip_begin = 4.0;
  e.slip_end = 5.5;
  e.slip_speed = 0.25;
  e.accel_noise = 0.011;
  const auto gated = simulate(5.5, profile, e);
  EXPECT_GT(gated.speed_rejections, 60u);  // most of the 75 slipping wheel updates
  EXPECT_EQ(gated.resynchronizations, 0u);
  EXPECT_LT(positionError(gated.ekf, gated.truth), 0.03);

  EkfParameters ungated;
  ungated.gate_chi2 = INFINITY;
  const auto trusting = simulate(5.5, profile, e, ungated);
  EXPECT_GT(positionError(trusting.ekf, trusting.truth), 0.15);
  // Wheel odometry alone "drives" the full 0.375 m into the pallet.
  EXPECT_NEAR(gated.wheel_only.x - gated.truth.x, 0.375, 0.01);
}

TEST(WheelImuEkf, RecoversAfterAPushWithoutResynchronizing)
{
  // 2 s docked, then 3 s pushing a wall (wheels at 0.22 m/s) and backing off
  // 0.75 m, with the chassis pitched while pushing. The wheels must be
  // trusted again when the slip ends, and not by a resync mid-push.
  const Profile profile = [](double t, double & v, double & w) {
      v = t < 2.0 ? 0.0 : (t < 2.6 ? 0.5 * (t - 2.0) :
        (t < 9.0 ? 0.3 : (t < 9.5 || t >= 12.0 ? 0.0 : -0.3)));
      w = 0.0;
    };
  Errors e;
  e.slip_begin = 6.0;
  e.slip_end = 9.0;
  e.slip_speed = 0.22;
  e.slip_pitch = -0.0039;  // -0.22 deg, as measured pushing a wall in MuJoCo
  e.pitch_y_gyro_bias = 0.01;
  e.accel_bias = 0.15;
  e.accel_noise = 0.011;
  e.gyro_noise = 0.0014;
  const auto run = simulate(14.0, profile, e);
  EXPECT_EQ(run.resynchronizations, 0u);
  EXPECT_GT(run.speed_rejections, 140u);  // of the 150 slipping updates
  EXPECT_NEAR(run.wheel_only.x - run.truth.x, 0.66, 0.02);
  EXPECT_LT(positionError(run.ekf, run.truth), 0.03);
  EXPECT_NEAR(run.ekf.state()(WheelImuEkf::kSpeed), 0.0, 0.01);
}

TEST(WheelImuEkf, AcceptsSmallWheelImuDisagreementWhileAccelerating)
{
  // Starting off, the wheels under-report the speed by up to 0.075 m/s for
  // 0.5 s. That is no slip to reject: rejecting it would leave the speed to
  // the IMU from a wrong start and lock the gate until the resync.
  const Profile profile = [](double t, double & v, double & w) {
      v = t < 2.0 ? 0.0 : std::min(0.3, 0.5 * (t - 2.0));
      w = 0.0;
    };
  Errors e;
  e.wheel_lag_begin = 2.1;
  e.wheel_lag_end = 2.6;
  e.wheel_lag_scale = 0.75;
  e.accel_noise = 0.011;
  const auto run = simulate(8.0, profile, e);
  EXPECT_EQ(run.resynchronizations, 0u);
  EXPECT_LT(run.speed_rejections, 5u);
  EXPECT_NEAR(run.ekf.state()(WheelImuEkf::kSpeed), 0.3, 0.01);
}

TEST(WheelImuEkf, IgnoresChassisRockingWhenTheWheelsStop)
{
  // Spin at 0.6 rad/s for 10 s, stop; the chassis rocks back at -0.1 rad/s
  // for 0.3 s with the wheels already still. A zero angular-rate update in
  // that window would pull the bias and, through their correlation, the
  // heading.
  const Profile profile = [](double t, double & v, double & w) {
      v = 0.0;
      w = (t >= 2.0 && t < 12.0) ? 0.6 : 0.0;
    };
  Errors e;
  e.gyro_bias = 0.005;
  e.gyro_noise = 0.0014;
  e.settle_begin = 12.0;
  e.settle_end = 12.3;
  e.settle_rate = -0.1;
  const auto run = simulate(16.0, profile, e);
  EXPECT_LT(angleError(run.ekf.state()(WheelImuEkf::kYaw), run.truth.yaw), 0.005);
  EXPECT_NEAR(run.ekf.state()(WheelImuEkf::kGyroBias), e.gyro_bias, 0.0005);
}

TEST(WheelImuEkf, ResynchronizesWhenTheImuSpeedDiverges)
{
  // An accelerometer bias step the filter has not seen yet makes the IMU
  // speed run away; the gate rejects the (correct) wheels until
  // max_rejection_time, then must trust them again.
  const Profile profile = [](double t, double & v, double & w) {
      v = t < 0.6 ? 0.5 * t : 0.3;
      w = 0.0;
    };
  Errors e;
  e.accel_bias_step_time = 3.0;
  e.accel_bias_step = 1.5;
  const auto run = simulate(16.0, profile, e);
  EXPECT_GE(run.resynchronizations, 1u);
  EXPECT_NEAR(run.ekf.state()(WheelImuEkf::kSpeed), 0.3, 0.02);
  EXPECT_NEAR(run.ekf.state()(WheelImuEkf::kAccelBias), 1.5, 0.1);
}

TEST(WheelImuEkf, CovarianceStaysSymmetricPositiveDefinite)
{
  const Profile profile = [](double t, double & v, double & w) {
      v = 0.4 * std::sin(0.3 * t);
      w = 0.8 * std::cos(0.2 * t);
    };
  Errors e;
  e.gyro_bias = 0.01;
  e.accel_bias = 0.05;
  e.gyro_noise = 0.0014;
  e.accel_noise = 0.011;
  e.slip_begin = 20.0;
  e.slip_end = 22.0;
  e.slip_speed = 0.2;
  const auto run = simulate(60.0, profile, e);
  const auto & P = run.ekf.covariance();
  EXPECT_LT((P - P.transpose()).cwiseAbs().maxCoeff(), 1e-12);
  const Eigen::SelfAdjointEigenSolver<WheelImuEkf::Covariance> solver(P);
  EXPECT_GT(solver.eigenvalues().minCoeff(), 0.0);
}

TEST(WheelImuEkf, IgnoresStaleImuSamplesAndClampsGaps)
{
  WheelImuEkf ekf;
  ekf.reset(0.0);
  ekf.predict({0.01, 0.0, 0.0});
  ekf.update({0.01, 0.04, 0.0});
  const auto before = ekf.state();
  ekf.predict({0.005, 0.0, 0.0});  // older than the filter: input only
  EXPECT_DOUBLE_EQ(ekf.time(), 0.01);
  EXPECT_TRUE(ekf.state().isApprox(before));
  ekf.predict({5.0, 0.0, 0.0});  // 5 s gap: propagated over max_imu_dt only
  EXPECT_DOUBLE_EQ(ekf.time(), 5.0);
  EXPECT_GT(before(WheelImuEkf::kSpeed), 0.03);
  // Heading ~0: x advances by v dt + a dt^2 / 2 with a = -b_a, dt = 0.1 s.
  const double a = -before(WheelImuEkf::kAccelBias);
  EXPECT_NEAR(
    ekf.state()(WheelImuEkf::kX),
    before(WheelImuEkf::kX) + 0.1 * before(WheelImuEkf::kSpeed) + 0.5 * a * 0.01, 1e-6);
}
