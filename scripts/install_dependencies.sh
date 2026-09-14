#!/usr/bin/env bash
set -eo pipefail

if [[ "$(. /etc/os-release && printf '%s' "$VERSION_CODENAME")" != "noble" ]]; then
  echo "This project targets Ubuntu 24.04 (noble) and ROS 2 Jazzy." >&2
  exit 1
fi

sudo apt-get update
sudo apt-get install -y \
  curl \
  python3-colcon-common-extensions \
  python3-numpy \
  python3-opencv \
  python3-rosdep \
  python3-yaml \
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
. /opt/ros/jazzy/setup.bash
set -u
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
