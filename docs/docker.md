# Docker

Two build targets are provided:

- `runtime-cpu`: ROS 2 Jazzy, projection, ONNX Runtime CPU inference, fusion,
  calibration monitoring, Gazebo/Nav2 and mission runtime.
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
