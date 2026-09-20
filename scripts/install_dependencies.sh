#!/usr/bin/env bash
set -eo pipefail

if [[ "$(. /etc/os-release && printf '%s' "$VERSION_CODENAME")" != "noble" ]]; then
  echo "This project targets Ubuntu 24.04 (noble) and ROS 2 Jazzy." >&2
  exit 1
fi

sudo apt-get update
sudo apt-get install -y \
  build-essential \
  curl \
  git \
  libeigen3-dev \
  libopencv-dev \
  libyaml-cpp-dev \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-numpy \
  python3-opencv \
  python3-pip \
  python3-rosdep \
  python3-venv \
  python3-yaml \
  shellcheck \
  ros-jazzy-ament-lint-auto \
  ros-jazzy-ament-lint-common \
  ros-jazzy-ament-cmake-gtest \
  ros-jazzy-ament-cmake-pytest \
  ros-jazzy-cv-bridge \
  ros-jazzy-desktop \
  ros-jazzy-rosbag2 \
  ros-jazzy-tf2-msgs \
  ros-jazzy-tf2-ros \
  unzip

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi

rosdep update
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
. /opt/ros/jazzy/setup.bash
set -u
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python3 -m venv --system-site-packages "$project_root/.venv"
"$project_root/.venv/bin/python" -m pip install --upgrade pip
"$project_root/.venv/bin/python" -m pip install -r "$project_root/requirements.txt"

echo "Dependencies installed. Activate Python tools with: source .venv/bin/activate"
