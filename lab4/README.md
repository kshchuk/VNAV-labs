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
| **3D** | Gazebo campus (`lab4.world`), drone, desired target, **12 gates**, semi-transparent walls |
| **Image** | Chase camera `/third_person/rgb/image_raw` |
| **Plot** | Actual (`/current_state`) and desired (`/desired_state`) position |

**3D:** blue trail = actual drone, **orange sphere + trail** = moving desired target, **red/orange gate line** = static race course (does not move).

### Troubleshooting: desired looks stationary

1. **Confirm the optimized planner is running** (not Part 0 simple planner):
   ```bash
   ros2 node list | grep trajectory_generation
   ```
   You should see `/trajectory_generation_node`. If you see `/simple_traj_planner` instead, relaunch with the default optimized planner or `planner:=optimized`.

2. **Rebuild after editing C++** — `run_lab_dev.sh` rebuilds on start, or run:
   ```bash
   bash ros2-docker/run_lab_dev.sh lab4 --build-only
   bash ros2-docker/run_lab_dev.sh lab4
   ```

3. **Check `/desired_state` is moving** (first ~24 s of sim):
   ```bash
   ros2 topic echo /desired_state --field transforms
   ```
   `translation.x/y` should change over time. After the ~24 s lap completes, desired returns to gate 0 `(0, 0, 2)` and stays there.

4. **Re-import the Foxglove layout** after updates — `VNAV-labs/lab4/config/lab4_foxglove.json` plots desired vs actual. The static red gate path in 3D is the course layout, not the desired pose.

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
# Full stack — optimized planner (default, Part 1)
bash ros2-docker/run_lab_dev.sh lab4

# Simple planner (Part 0 only — holds first gate)
bash ros2-docker/run_lab_dev.sh lab4 -- \
  ros2 launch planner_pkg lab4_stack.gazebo.launch.py planner:=simple

# CSV publisher only (no Gazebo)
bash ros2-docker/run_lab_dev.sh lab4 -- \
  ros2 launch planner_pkg traj_gen.launch.py
```

## Waypoint data

The bundled course matches the **MIT Unity lab4** indoor/outdoor campus from [lab4.zip](https://vnav.mit.edu/material/lab4.zip):

- **12 gates** through buildings (not the old 5×5 m square)
- **Collision geometry** in `lab4.world` (walls, colliders, doors, roofs)
- Drone spawns at Unity `QuadUAV` pose: **(0.4, 8.0, 2.0)** in Gazebo `world`

Data resolution order:

1. `VNAV_LAB4_DATA` environment variable
2. `~/vnav/tesse/lab4/lab4_Data` (Unity build from lab4.zip)
3. Bundled sample: `VNAV-labs/lab4/data/`

CSV format: **no header**; gate rows named `red_square_drone_door (N)` (Unity export convention).

### Regenerate Gazebo world from CSV

After editing the Unity CSV, regenerate collision geometry:

```bash
python3 VNAV-labs/lab4/scripts/generate_lab4_world.py
```

This writes `VNAV-labs/lab3/gazebo_quadrotor_pkg/worlds/lab4.world`.

## Dependencies (Docker)

- Lab 3 `gazebo_quadrotor_pkg`, `controller_pkg`
- [fishberg/mav_trajectory_generation](https://github.com/fishberg/mav_trajectory_generation)
- **Clang 19** + **Kitware CMake ≥ 3.30** for C++26 student packages
