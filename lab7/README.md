# Lab 7 — GTSAM Factor Graphs (ROS 2)

Practice nonlinear least-squares and maximum-likelihood estimation with **GTSAM**.

## Quick start (Docker)

```bash
bash ros2-docker/run_lab_dev.sh lab7
```

Default launch opens **Deliverable 2/3** with RViz2.

Build only:

```bash
bash ros2-docker/run_lab_dev.sh lab7 --build-only
```

GTSAM is installed in the Docker image (`ros-humble-gtsam` or `libgtsam-dev`).

## Deliverables

| Command | Purpose |
|---------|---------|
| `ros2 run lab_7 deliverable_1` | GTSAM intro (2D pose + GPS-like factor) |
| `ros2 launch lab_7 deliverable_2_3.launch.yaml` | 3D pose estimation + MoCap factor (+ RViz) |
| `ros2 launch lab_7 deliverable_4.launch.yaml` | Computer-vision factors (+ RViz) |
| `ros2 run lab_7 deliverable_5` | Optional: SO(3) MLE with Langevin noise |

Useful parameters for Deliverable 2/3:

```bash
ros2 launch lab_7 deliverable_2_3.launch.yaml max_solver_iterations:=10 use_mocap:=true
```

## macOS GUI

RViz2 needs **XQuartz** + `DISPLAY=:0` on the host. See [ros2-docker/README.md](../../ros2-docker/README.md#x11-lab-5-opencv-windows-etc).

## Key files

| File | Purpose |
|------|---------|
| `src/deliverable_1.cpp` | Unary GPS-like factor + Pose2 graph |
| `src/deliverable_2_3.cpp` | Pose3 odometry / MoCap optimization |
| `include/deliverable_2_3.h` | Custom `MoCapPosition3Factor` |
| `src/deliverable_4.cpp` | Projection / landmark factors |
| `src/deliverable_5.cpp` | Optional SO(3) MLE |
| `launch/*.launch.yaml` | Nodes + RViz layouts |
