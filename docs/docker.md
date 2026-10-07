# Docker

Two build targets are provided:

- `runtime-cpu`: ROS 2 Jazzy, projection, ONNX Runtime CPU inference, fusion,
  calibration monitoring, MuJoCo/Nav2 and mission runtime (camera rendered
  in software with `MUJOCO_GL=osmesa`).
- `training`: runtime plus PyTorch, torchvision and Ultralytics for dataset
  preparation, YOLO training and ONNX export.

Dataset, model, bag, training runs and benchmark results are bind-mounted; they
are never copied into the final workflow as generated artifacts.

```bash
docker compose -f docker/compose.yaml --profile runtime-cpu build
docker compose -f docker/compose.yaml --profile runtime-cpu up
docker compose -f docker/compose.yaml --profile training run --rm training
```

Run the complete build/headless smoke gate later with
`./docker/smoke_test.sh`. The script owns cleanup and checks that the perception
topic is visible from inside the container.

## GUI on a non-Noble host (e.g. Ubuntu 22.04)

`scripts/run_docker_gui.sh` runs the `runtime-cpu` image with X11 forwarding,
mounts the repository at `/workspace`, and builds it with colcon into
`.docker_ws/` on first use, so source edits on the host are picked up without
rebuilding the image. The container runs as the host user.

```bash
scripts/run_docker_gui.sh ros2 launch fusion_bringup bringup_sim.launch.py \
  sim_gui:=true use_rviz:=true mission_autostart:=true
```

Rendering is software (llvmpipe) by default. Pass `--gpu` to use an NVIDIA GPU
(requires `nvidia-container-toolkit`). Use `--rebuild-ws` after adding packages
and `--rebuild-image` after changing `package.xml` dependencies or the
Dockerfile. Run ROS tools inside the container rather than from a host ROS 2
Humble install.
