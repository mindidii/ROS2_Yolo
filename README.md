# ROS2_Yolo

ROS 2 Jazzy workspace for video reception and YOLO-based object detection.

## Packages

- `video_rx_pkg`
  - Receives video frames and publishes ROS topics.
- `yolo_detector_pkg`
  - Runs ONNX-based YOLO inference and publishes detection results.
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
- `onnxruntime`
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

## YOLO Model

The detector expects an ONNX model file.

Recommended example:

- `/ros2_ws/src/yolo_detector_pkg/model/anti.onnx`

If your model is stored somewhere else, pass it with the `model_path` parameter when running the node.

## Run

### 1. Start the video publisher

Start the node or launch file that publishes:

- `/video/raw`
- `/video/frame_info`

### 2. Start the YOLO detector

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
ros2 run yolo_detector_pkg yolo_detector_node --ros-args -p model_path:=/ros2_ws/src/yolo_detector_pkg/model/anti.onnx
```

## Topics

### Input

- `/video/raw`
  - `sensor_msgs/msg/Image`
- `/video/frame_info`
  - `sentinel_interfaces/msg/FrameInfo`

### Output

- `/detections`
  - `sentinel_interfaces/msg/Detection2DArray`
- `/yolo/status`
  - `sentinel_interfaces/msg/YoloStatus`

## Services

- `/yolo/enable`
  - `sentinel_interfaces/srv/SetBoolFlag`
- `/yolo/set_threshold`
  - `sentinel_interfaces/srv/SetThreshold`

## Check Detection Results

### Check node status

```bash
ros2 topic echo /yolo/status
```

Expected fields:

- `enabled`
- `model_loaded`
- `conf_threshold`
- `last_error`

### Check detection results

```bash
ros2 topic echo /detections
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
ros2 topic info /video/raw
```

If `Publisher count: 0`, the YOLO node has no input image, so it cannot publish detections.

### Model load failure

Check:

- the ONNX file path is correct
- the model file exists
- `onnxruntime` is installed in the Python environment used by `ros2 run`

## Git Notes

This repository ignores generated files and large model artifacts through `.gitignore`:

- `build/`
- `install/`
- `log/`
- `__pycache__/`
- `*.onnx`
- `*.pt`

If you want to version large model files, use Git LFS instead of normal Git tracking.
