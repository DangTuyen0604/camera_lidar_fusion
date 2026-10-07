#!/usr/bin/env bash
# Run the ROS 2 Jazzy workspace in Docker with X11 GUI (MuJoCo viewer, RViz).
# Intended for hosts that are not Ubuntu 24.04, e.g. Ubuntu 22.04.
#
#   scripts/run_docker_gui.sh                 # interactive shell
#   scripts/run_docker_gui.sh ros2 launch fusion_bringup bringup_sim.launch.py sim_gui:=true use_rviz:=true
#
# Options (before the command):
#   --gpu            use the NVIDIA GPU (needs nvidia-container-toolkit)
#   --rebuild-image  rebuild the Docker image (after package.xml/Dockerfile changes)
#   --rebuild-ws     rerun colcon build before the command
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"

image=camera-lidar-fusion:runtime-cpu
use_gpu=false
rebuild_image=false
rebuild_ws=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --gpu) use_gpu=true; shift ;;
    --rebuild-image) rebuild_image=true; shift ;;
    --rebuild-ws) rebuild_ws=true; shift ;;
    -h|--help) sed -n '2,11p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) break ;;
  esac
done
cmd=("$@")
[[ ${#cmd[@]} -gt 0 ]] || cmd=(bash)

if [[ -z "${DISPLAY:-}" ]]; then
  echo 'DISPLAY is not set; run this from a desktop session.' >&2
  exit 1
fi

if [[ "$rebuild_image" == true ]] || ! docker image inspect "$image" >/dev/null 2>&1; then
  docker build -f docker/Dockerfile --target runtime-cpu -t "$image" .
fi

# Build artifacts live in .docker_ws so they never mix with a host build, and
# the container runs as the host user so the files stay owned by them.
ws=/workspace/.docker_ws
mkdir -p .docker_ws/home
xhost "+SI:localuser:$(id -un)" >/dev/null

run_args=(
  --rm --init --net=host --ipc=host
  --user "$(id -u):$(id -g)"
  -e HOME="$ws/home"
  -e ROS_LOG_DIR="$ws/log/ros"
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  -e DISPLAY
  -e MUJOCO_GL=glfw
  -e QT_X11_NO_MITSHM=1
  -e REBUILD_WS="$rebuild_ws"
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw
  -v "$project_root":/workspace
  -w /workspace
)
if [[ -t 0 ]]; then
  run_args+=(-it)
fi
if [[ "$use_gpu" == true ]]; then
  # PRIME render offload: on hybrid laptops the X server runs on the iGPU, so
  # GLX would otherwise pick Mesa (software in the container) over NVIDIA.
  run_args+=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all
    -e __NV_PRIME_RENDER_OFFLOAD=1 -e __GLX_VENDOR_LIBRARY_NAME=nvidia)
else
  run_args+=(-e LIBGL_ALWAYS_SOFTWARE=1)
fi

# Runs inside the container, so its variables must not expand here.
# shellcheck disable=SC2016
inner='
set -e
source /opt/ros/jazzy/setup.bash
ws=/workspace/.docker_ws
if [[ "$REBUILD_WS" == true || ! -f $ws/install/setup.bash ]]; then
  colcon --log-base $ws/log build --symlink-install \
    --build-base $ws/build --install-base $ws/install
fi
source $ws/install/setup.bash
exec "$@"
'
exec docker run "${run_args[@]}" --entrypoint bash "$image" -c "$inner" bash "${cmd[@]}"
