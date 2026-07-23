# Lab 4 — Trajectory Generation and Planning (Gazebo)

Lab 4 uses **Gazebo Classic** (same stack as Lab 3) instead of the Unity/TESSE simulator.
Waypoint gates come from CSV (`traj_vertices_publisher`); the controller comes from Lab 3.

## Quick start

```bash
bash ros2-docker/run_lab_dev.sh lab4
```

1. Open [Foxglove Studio](https://app.foxglove.dev) → **Foxglove WebSocket** → `ws://localhost:8765`
2. **Layout → Import from file…** → `VNAV-labs/lab4/config/lab4_foxglove.json`

### Foxglove panels

| Panel | Content |
|-------|---------|
| **3D** | Gazebo scene, drone, desired target, **orange gate markers** + path from CSV |
| **Image** | Chase camera `/third_person/rgb/image_raw` |
| **Plot** | Position `x/y/z` from `/current_state` |

**3D:** blue trail = actual drone, orange trail = desired, red cubes = race gates.

## Architecture

```
traj_vertices_publisher  →  /desired_traj_vertices
        ↓
simple_traj_planner  OR  trajectory_generation_node  →  /desired_state  +  TF av-desired
        ↓
controller_node  →  rotor_speed_cmds  →  Gazebo quadrotor_bridge  →  /current_state
```

Implement the assignments in:

- `planner_pkg/src/simple_traj_planner.cpp` (Part 0 — fly to first gate)
- `trajectory_generation_pkg/src/trajectory_generation_node.cpp` (Parts 1.1–1.3)

Until Part 0 is done, the drone hovers but does not receive meaningful `/desired_state` commands.

## Launch options

```bash
# Full stack — simple planner (default, Part 0)
bash ros2-docker/run_lab_dev.sh lab4

# Polynomial optimizer (Parts 1.x)
bash ros2-docker/run_lab_dev.sh lab4 -- \
  ros2 launch planner_pkg lab4_stack.gazebo.launch.py planner:=optimized

# CSV publisher only (no Gazebo)
bash ros2-docker/run_lab_dev.sh lab4 -- \
  ros2 launch planner_pkg traj_gen.launch.py
```

## Waypoint data

1. `VNAV_LAB4_DATA` environment variable
2. `~/vnav/tesse/lab4/lab4_Data` (Unity build from [lab4.zip](https://vnav.mit.edu/material/lab4.zip))
3. Bundled sample: `VNAV-labs/lab4/data/` — square course at **z = 2 m** in Gazebo

CSV format: **no header**; names like `red_square_drone_door [0]` (Unity export convention).

## Dependencies (Docker)

- Lab 3 `gazebo_quadrotor_pkg`, `controller_pkg`
- [fishberg/mav_trajectory_generation](https://github.com/fishberg/mav_trajectory_generation)
- **Clang 19** + **Kitware CMake ≥ 3.30** for C++26 student packages
