# Lab 8 — YOLO Detection (ROS 2)

Subscribe to Ultralytics YOLO results and inspect camera TF.

## Quick start (Docker)

```bash
bash ros2-docker/run_lab_dev.sh lab8
```

Default launch: **webcam** (`usb_cam`) → **YOLOv8n on CPU** (`ultralytics_ros`) → `deliverable_1`.

Build only:

```bash
bash ros2-docker/run_lab_dev.sh lab8 --build-only
```

## Launch options

```bash
# Default: webcam + CPU YOLOv8n
ros2 launch lab_8 deliverable_1.launch.yaml

# Without webcam (provide your own image topic)
ros2 launch lab_8 deliverable_1.launch.yaml use_webcam:=false input_topic:=/camera/color/image_raw

# Different model / device
ros2 launch lab_8 deliverable_1.launch.yaml yolo_model:=yolov8n.pt device:=cpu
```

## macOS notes

- Webcam passthrough into Docker Desktop is limited; prefer feeding a rosbag or host image topic when `/dev/video0` is unavailable.
- OpenCV/RViz GUI windows need **XQuartz** (`DISPLAY=:0`). See [ros2-docker/README.md](../../ros2-docker/README.md).

## Key files

| File | Purpose |
|------|---------|
| `src/deliverable_1.cpp` | Subscribe to `/yolo_result`, print detections + TF |
| `include/deliverable_1.hpp` | Header stub for student helpers |
| `launch/deliverable_1.launch.yaml` | usb_cam + ultralytics tracker + node |

## Dependencies

- **ultralytics_ros** — pre-built in the Docker image (`Alpaca-zip/ultralytics_ros`, `humble-devel`)
- **ultralytics** (pip) + YOLOv8n weights (downloaded on first run)
- **usb_cam**, GTSAM, tf2
