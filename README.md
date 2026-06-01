# ROS2_Yolo

ROS 2 Jazzy workspace for video reception and YOLO-based object detection.

## Packages

- `video_rx_pkg2`
  - Receives video frames and publishes ROS topics.
- `yolo_detector_pkg`
  - Runs Ultralytics `.pt` YOLO inference and publishes detection results.
- `sentinel_interfaces`
  - Custom messages and services used by the workspace.
- `sentinel_bringup`
  - Bringup/config package for launching the system.
- `image_preprocess_pkg`
  - Image preprocessing utilities/node.

## Requirements

- Ubuntu with ROS 2 Jazzy installed
- Python 3.12
- `cv_bridge`
- `ultralytics`
- `numpy`
- `opencv-python` or system OpenCV bindings

## Workspace Layout

```text
/ros2_ws
  ├── src/
  ├── build/
  ├── install/
  └── log/
```

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd /ros2_ws
colcon build --symlink-install
source /ros2_ws/install/setup.bash
```

## YOLO Models

The current launch uses these model files:

- `/ros2_ws/src/yolo_detector_pkg/model/drone.pt` for EO drone detection
- `/ros2_ws/src/yolo_detector_pkg/model/yolo11l.pt` for EO person detection
- `/ros2_ws/src/yolo_detector_pkg/model/yolo11n.pt` for IR person detection

## Run

Start the full pipeline:

```bash
./run_yolo.sh
```

## Topics

### Input

- `/video/eo/preprocessed`
  - `sensor_msgs/msg/Image`
- `/video/eo/preprocessed/frame_info`
  - `sentinel_interfaces/msg/FrameInfo`

### Output

- `/detections/eo/drone`
- `/detections/eo/person`
- `/detections/eo`
- `/detections/ir`
  - `sentinel_interfaces/msg/Detection2DArray`
- `/tracks/eo`
- `/tracks/ir`
  - `sentinel_interfaces/msg/TrackedDetection2DArray`

## Check Detection Results

```bash
ros2 topic echo /detections/eo
```

Example:

```yaml
detections:
- class_name: person
  score: 0.88
  x1: 120.0
  y1: 45.0
  x2: 300.0
  y2: 220.0
```

## Troubleshooting

### `/detections` is not publishing

Check whether the image input topic exists:

```bash
ros2 topic info /video/eo/preprocessed
```

If `Publisher count: 0`, the YOLO node has no input image, so it cannot publish detections.

### Model load failure

Check:

- the `.pt` model file path is correct
- the model file exists
- `ultralytics` is installed in the Python environment used by `ros2 run`

## Git Notes

This repository ignores generated files and large model artifacts through `.gitignore`:

- `build/`
- `install/`
- `log/`
- `__pycache__/`
- `*.pt`

If you want to version large model files, use Git LFS instead of normal Git tracking.
