# Day 2 starting point

This is an audit only. No perception code was changed during Day 1.

## Existing nodes and topics

- Gazebo publishes `/camera/image_raw` and `/lidar/points`.
- `sensor_sync_node` synchronizes camera and PointCloud2 data into
  `/fusion/synced/image` and `/fusion/synced/points`.
- `yolo_detector_node` consumes the synchronized image when launched with
  `fusion_bringup/config/yolo_params.yaml`.
- `object_fusion_node` publishes `/fusion/detections_3d`.
- `detection_obstacle_bridge_node` converts `/fusion/detections_3d` into
  `/navigation/detection_obstacles` and its clearing topic.
- Existing coverage includes `test_perception_pipeline.py`, warehouse smoke
  and end-to-end tests, and navigation bridge tests.

## Clear blocker

The default simulation sensor topics and `fusion_bringup/config/sync_params.yaml`
do not match: the simulation uses `/camera/image_raw` and `/lidar/points`, while
that configuration currently selects KITTI topics. Stage 3 intentionally does
not launch perception, so `/fusion/detections_3d` is absent in the navigation-
only launch.

## First gate

Launch the existing perception pipeline against the warehouse simulation with
explicit simulation-topic overrides, then prove synchronized image/point-cloud
traffic and a live `/fusion/detections_3d` topic. Do not tune models or change
the navigation stack until that topic contract passes.
