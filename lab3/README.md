# Lab 3 — Geometric Controller + Gazebo + Foxglove

Lab 3 implements a geometric tracking controller for a quadrotor. Simulation runs in **Gazebo Classic** (headless) inside Docker; visualization matches the original **Unity/TESSE** workflow via **Foxglove Studio**.

## Quick start

```bash
bash ros2-docker/run_lab_dev.sh lab3
```

1. Open [Foxglove Studio](https://app.foxglove.dev) → **Foxglove WebSocket** → `ws://localhost:8765`
2. **Layout → Import from file…** → `VNAV-labs/lab3/gazebo_quadrotor_pkg/config/lab3_foxglove.json`

### Foxglove panels (Unity-like)

| Panel | Content |
|-------|---------|
| **3D** (left) | Outdoor scene markers, reference circle path, drone cube, desired sphere, TF trails |
| **Image** (top-right) | Third-person chase camera `/third_person/rgb/image_raw` (same topic name as TESSE) |
| **Plot** (bottom-right) | Actual position `x/y/z` from `/current_state` |

**3D panel colors:** blue trail = actual drone, orange trail = desired path, green ring = reference trajectory (R=5 m, z=2 m).

Implement the controller in `controller_pkg/src/controller_node.cpp`, rebuild, and re-launch.

## Architecture

```
Gazebo (headless)
  ├─ third_person camera → /third_person/rgb/image_raw
  └─ odometry → quadrotor_bridge → /current_state + TF base_link
traj_publisher → desired_state + TF av-desired
lab3_viz_publisher → /visuals (MarkerArray, environment + trails)
foxglove_bridge → ws://localhost:8765
controller_node
```

| Topic | Type | Role |
|-------|------|------|
| `/third_person/rgb/image_raw` | `sensor_msgs/Image` | Chase camera (TESSE-compatible) |
| `/visuals` | `visualization_msgs/MarkerArray` | Scene, drone, trails |
| `/current_state` | `nav_msgs/Odometry` | Actual pose |
| `desired_state` | `trajectory_msgs/MultiDOFJointTrajectoryPoint` | Reference |
| `rotor_speed_cmds` | `mav_msgs/Actuators` | Motor commands |
| `/tf` | `tf2_msgs/TFMessage` | `world` → `base_link`, `av-desired` |

## TESSE (legacy)

The original MIT lab used the TESSE Unity simulator. Those packages remain for reference but are not used by the default workflow.
