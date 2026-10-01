# Detection model

`yolov8n-opencv.onnx` is the COCO-pretrained YOLOv8n model referenced by
OpenCV's official DNN model manifest. The binary is intentionally ignored by
Git because it is downloaded separately.

- Source: `https://github.com/CVHub520/X-AnyLabeling/releases/download/v0.1.0/yolov8n.onnx`
- SHA-1: `68f864475d06e2ec4037181052739f268eeac38d`
- Input: `1 x 3 x 640 x 640`
- Output: `1 x 84 x 8400`
- Classes: COCO 80 classes

# Warehouse model (trained by you)

Put the trained warehouse detector here as `warehouse_yolov8n.onnx` (ignored
by Git like the other weights). Training images: `tools/generate_synthetic_dataset.py`
plus real images, merged and split on Roboflow.

The ONNX loader (`yolo_detector/model_loader.py`) requires an Ultralytics
export with a **static** input and the class names in its metadata:

    yolo export model=best.pt format=onnx imgsz=640 opset=12

Class names must be `person`, `box`, `pallet`: the costmap bridge sizes each
obstacle by name. Their order does not matter (Roboflow sorts them
alphabetically); the names are read from the model.

Use it:

    ros2 run yolo_detector yolo_detector_node --ros-args \
      -p model_path:=$PWD/models/warehouse_yolov8n.onnx
