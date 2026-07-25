# Lab 5 — Feature Tracking (ROS 2)

Computer vision pipeline: detect → describe → match features (SIFT, AKAZE, ORB, BRISK, LK).

## Quick start (Docker)

```bash
bash ros2-docker/run_lab_dev.sh lab5
```

Default launch runs **two-frame** mode on bundled sample images (`box.png`, `box_in_scene.png`).

Build only:

```bash
bash ros2-docker/run_lab_dev.sh lab5 --build-only
```

Student package builds with **C++20** and **clang-19** in Docker (OpenCV 4.5 on Humble does not compile as C++26).

## Launch modes

### Two images (Part 1)

```bash
ros2 launch lab_5 two_frames_tracking.launch.yaml descriptor:=SIFT show_images:=true
```

Implement deliverables in `src/feature_tracker.cpp` and the tracker classes under `src/` before matches appear.

### Video / rosbag

1. Download bags from [MIT VNAV lab data (Dropbox)](https://www.dropbox.com/sh/5xkr1kpygubs6fa/AABZgm2cDuDB1cnSwR7yzZuza?dl=0) — recommended:
   - `30fps_424x240_2018-10-01-18-35-06`
   - `vnav-lab5-smooth-trajectory`

2. Extract into `VNAV-labs/lab5/bags/` (or set `VNAV_LAB5_BAGS`).

```bash
bash ros2-docker/download_lab5_bags.sh   # prints Dropbox link + instructions
```

3. Launch:

```bash
bash ros2-docker/run_lab_dev.sh lab5 -- \
  ros2 launch lab_5 video_tracking.launch.yaml \
  bag_name:=30fps_424x240_2018-10-01-18-35-06
```

## macOS OpenCV windows

`show_images:=true` needs **XQuartz** + `DISPLAY=:0` on the host. Without XQuartz, macOS defaults to `show_images:=false`; set `VNAV_LAB5_SHOW_IMAGES=1` after configuring X11.

See [ros2-docker/README.md](../../ros2-docker/README.md#x11-lab-5-opencv-windows-etc).

## Key files

| File | Purpose |
|------|---------|
| `include/feature_tracker.h` | Abstract tracker interface |
| `src/sift_feature_tracker.cpp` | SIFT (Deliverable 3+) |
| `src/track_features.cpp` | ROS node + image/bag modes |
| `launch/two_frames_tracking.launch.yaml` | Static image pair |
| `launch/video_tracking.launch.yaml` | Rosbag playback |
