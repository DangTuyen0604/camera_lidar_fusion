// Run WheelImuEkf over a recorded sensor log, without ROS.
//
//   ekf_replay <log.csv> <estimate.csv> [--no-accel] [--no-gate] [--set name=value]...
//
// Input lines (others, e.g. "truth,..." or "#..." comments, are ignored):
//   imu,<t>,<yaw_rate>,<forward_accel>[,<pitch_rate>]
//   wheel,<t>,<speed>,<yaw_rate>
// Output: one row per IMU sample with the state, its standard deviations and
// whether the last wheel speed was accepted. Timing goes to stderr.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "state_estimation/wheel_imu_ekf.hpp"

namespace
{

using state_estimation::EkfParameters;
using state_estimation::WheelImuEkf;

void setParameter(EkfParameters & p, const std::string & assignment)
{
  const auto split = assignment.find('=');
  if (split == std::string::npos) {
    throw std::invalid_argument("--set expects name=value, got " + assignment);
  }
  const std::string name = assignment.substr(0, split);
  const double value = std::stod(assignment.substr(split + 1));
  const std::map<std::string, double *> fields = {
    {"gyro_noise_density", &p.gyro_noise_density},
    {"accel_noise_density", &p.accel_noise_density},
    {"accel_vibration", &p.accel_vibration},
    {"gyro_bias_random_walk", &p.gyro_bias_random_walk},
    {"accel_bias_random_walk", &p.accel_bias_random_walk},
    {"gravity", &p.gravity},
    {"pitch_std", &p.pitch_std},
    {"initial_speed_std", &p.initial_speed_std},
    {"initial_gyro_bias_std", &p.initial_gyro_bias_std},
    {"initial_accel_bias_std", &p.initial_accel_bias_std},
    {"wheel_speed_std", &p.wheel_speed_std},
    {"wheel_speed_scale", &p.wheel_speed_scale},
    {"stationary_speed", &p.stationary_speed},
    {"stationary_yaw_rate", &p.stationary_yaw_rate},
    {"stationary_yaw_rate_std", &p.stationary_yaw_rate_std},
    {"stationary_dwell", &p.stationary_dwell},
    {"gate_chi2", &p.gate_chi2},
    {"min_slip_speed", &p.min_slip_speed},
    {"max_rejection_time", &p.max_rejection_time},
    {"max_imu_dt", &p.max_imu_dt},
    {"no_accel_speed_noise", &p.no_accel_speed_noise},
  };
  const auto field = fields.find(name);
  if (field == fields.end()) {
    throw std::invalid_argument("unknown parameter " + name);
  }
  *field->second = value;
}

std::vector<double> numbers(const std::string & line)
{
  std::vector<double> values;
  std::stringstream stream(line);
  std::string cell;
  std::getline(stream, cell, ',');  // record type
  while (std::getline(stream, cell, ',')) {
    values.push_back(std::stod(cell));
  }
  return values;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::cerr << "usage: ekf_replay <log.csv> <estimate.csv> [--no-accel] [--no-gate] "
      "[--set name=value]...\n";
    return 2;
  }
  EkfParameters parameters;
  try {
    for (int i = 3; i < argc; ++i) {
      const std::string option = argv[i];
      if (option == "--no-accel") {
        parameters.use_accelerometer = false;
      } else if (option == "--no-gate") {
        parameters.gate_chi2 = INFINITY;
      } else if (option == "--set" && i + 1 < argc) {
        setParameter(parameters, argv[++i]);
      } else {
        throw std::invalid_argument("unknown option " + option);
      }
    }
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 2;
  }

  std::ifstream input(argv[1]);
  std::ofstream output(argv[2]);
  if (!input || !output) {
    std::cerr << "cannot open " << (!input ? argv[1] : argv[2]) << '\n';
    return 1;
  }
  output << "t,x,y,yaw,v,gyro_bias,accel_bias,pitch,std_x,std_y,std_yaw,std_v,speed_accepted\n";

  WheelImuEkf ekf(parameters);
  bool speed_accepted = true;
  std::size_t imu_count = 0;
  std::size_t wheel_count = 0;
  std::size_t rejected = 0;
  std::size_t resynchronized = 0;
  std::chrono::nanoseconds predict_time{0};
  std::chrono::nanoseconds update_time{0};
  std::string line;
  char row[512];
  while (std::getline(input, line)) {
    if (line.rfind("imu,", 0) == 0) {
      const auto v = numbers(line);
      const auto start = std::chrono::steady_clock::now();
      ekf.predict({v.at(0), v.at(1), v.at(2), v.size() > 3 ? v[3] : 0.0});
      predict_time += std::chrono::steady_clock::now() - start;
      ++imu_count;
      const auto & x = ekf.state();
      const auto & P = ekf.covariance();
      std::snprintf(
        row, sizeof(row), "%.4f,%.5f,%.5f,%.6f,%.5f,%.7f,%.6f,%.6f,%.5f,%.5f,%.6f,%.5f,%d\n",
        ekf.time(), x(WheelImuEkf::kX), x(WheelImuEkf::kY), x(WheelImuEkf::kYaw),
        x(WheelImuEkf::kSpeed), x(WheelImuEkf::kGyroBias), x(WheelImuEkf::kAccelBias),
        x(WheelImuEkf::kPitch),
        std::sqrt(P(0, 0)), std::sqrt(P(1, 1)), std::sqrt(P(2, 2)), std::sqrt(P(3, 3)),
        speed_accepted ? 1 : 0);
      output << row;
    } else if (line.rfind("wheel,", 0) == 0) {
      if (!ekf.initialized()) {
        continue;  // As the node: the IMU starts the filter.
      }
      const auto v = numbers(line);
      const auto start = std::chrono::steady_clock::now();
      const auto result = ekf.update({v.at(0), v.at(1), v.at(2)});
      update_time += std::chrono::steady_clock::now() - start;
      ++wheel_count;
      speed_accepted = result.speed_accepted;
      rejected += result.speed_accepted ? 0 : 1;
      resynchronized += result.resynchronized ? 1 : 0;
    }
  }
  const auto mean_us = [](std::chrono::nanoseconds total, std::size_t count) {
      return count ? 1e-3 * static_cast<double>(total.count()) / static_cast<double>(count) : 0.0;
    };
  std::cerr << "imu " << imu_count << " (predict " << mean_us(predict_time, imu_count)
            << " us), wheel " << wheel_count << " (update " << mean_us(update_time, wheel_count)
            << " us), speed rejected " << rejected << ", resynchronized " << resynchronized
            << '\n';
  return 0;
}
