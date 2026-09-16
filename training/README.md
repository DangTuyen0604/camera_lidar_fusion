# YOLO training workspace

Dataset contents and generated `runs/` are intentionally ignored by Git. Keep
the dataset and training YAML files under `configs/` so a run can be reproduced.
The wrappers require ROS 2 Jazzy, Ultralytics, ONNX Runtime and OpenCV.

## End-to-end round trip

The Day 3 smoke run used the following commands from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source .venv-day3/bin/activate

python3 training/scripts/extract_images_from_rosbag.py \
  bags/day3_kitti_0005_complete \
  training/datasets/day3_roundtrip/source_images \
  --topic /kitti/camera/image_raw --limit 8

# Create matching YOLO .txt annotations in source_labels before splitting.
python3 training/scripts/validate_labels.py \
  training/datasets/day3_roundtrip/source_labels \
  --images training/datasets/day3_roundtrip/source_images --num-classes 3

python3 training/scripts/split_dataset.py \
  training/datasets/day3_roundtrip/source_images \
  training/datasets/day3_roundtrip \
  --labels training/datasets/day3_roundtrip/source_labels \
  --validation-ratio 0.25 --seed 42 --copy

python3 training/scripts/train_yolo.py \
  training/configs/day3_roundtrip_training.yaml

python3 training/scripts/evaluate_yolo.py \
  runs/detect/day3_roundtrip/weights/best.pt \
  training/configs/day3_roundtrip_dataset.yaml \
  --imgsz 320 --output runs/detect/day3_roundtrip/evaluation.json

python3 training/scripts/export_model.py \
  runs/detect/day3_roundtrip/weights/best.pt --format onnx \
  --output-metadata \
  runs/detect/day3_roundtrip/weights/best.metadata.json
```

`train_yolo.py` writes the exact Ultralytics command to
`runs/detect/<run-name>/reproduce_train.sh`. `evaluate_yolo.py` persists
precision, recall, mAP50 and mAP50-95. `export_model.py` loads the exported
model with ONNX Runtime and records its input/output shapes and class metadata.

The committed Day 3 configuration is a deliberately small CPU smoke test. Its
metrics verify plumbing and artifact production; they are not a production
model-quality baseline.
