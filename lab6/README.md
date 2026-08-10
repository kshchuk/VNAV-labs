# Lab 6 — Pose Estimation (ROS 2)

Robust pose estimation from RGB-D correspondences using OpenGV (5-point, 8-point, 2-point with known rotation, Arun 3D-3D).

Depends on **Lab 5** feature trackers (`lab_5` package).

## Quick start (Docker)

```bash
bash ros2-docker/run_lab_dev.sh lab6
```

Build only (also builds `lab_5`):

```bash
bash ros2-docker/run_lab_dev.sh lab6 --build-only
```

Implement the deliverables in `src/pose_estimation.cpp` before estimates appear in RViz2.

## Rosbag

Download and convert the office dataset (ROS 1 `.bag` → ROS 2 rosbag2):

```bash
bash ros2-docker/download_lab6_bags.sh
```

If the cached MIT Dropbox zip only contains Lab 5 bags, the script prints manual download steps. You can also set a direct URL:

```bash
VNAV_LAB6_BAG_URL='https://.../vnav-lab6-office.bag' bash ros2-docker/download_lab6_bags.sh
```

## Launch

Default (office bag + RViz2):

```bash
ros2 launch lab_6 video_tracking.launch.yaml dataset:=vnav-lab6-office
```

Select pose estimator (`pose_estimator` parameter):

| Value | Algorithm |
| --- | --- |
| `0` | Nister 5-point (2D-2D) |
| `1` | Longuet-Higgins 8-point (2D-2D) |
| `2` | OpenGV 2-point (known rotation) |
| `3` | Arun 3D-3D |

Example:

```bash
ros2 launch lab_6 video_tracking.launch.yaml pose_estimator:=0 use_ransac:=true show_images:=false
```

## macOS GUI

RViz2 and optional OpenCV windows need **XQuartz** + `DISPLAY=:0`. Without XQuartz, macOS defaults to `show_images:=false`; set `VNAV_LAB6_SHOW_IMAGES=1` after configuring X11.

See [ros2-docker/README.md](../../ros2-docker/README.md#x11-lab-5-opencv-windows-etc).

## Key files

| File | Purpose |
|------|---------|
| `src/pose_estimation.cpp` | ROS node + pose estimation deliverables |
| `include/lab6_utils.h` | Camera model, pose conversion helpers |
| `launch/video_tracking.launch.yaml` | Bag playback + RViz2 |
| `config/default.rviz` | RViz layout |

## Dependencies

- **OpenGV** — pre-built in the Docker image (`ros2-docker/Dockerfile`)
- **lab_5** — SIFT/AKAZE/ORB/BRISK trackers from Lab 5
