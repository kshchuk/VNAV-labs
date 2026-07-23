# Lab 4 — Trajectory Generation and Planning

Lab 4 builds on the Lab 3 controller. Packages:

- `planner_pkg` — waypoint CSV publisher + simple planner
- `trajectory_generation` — polynomial optimization via `mav_trajectory_generation`
- `controller_pkg` (Lab 3) — used by full-stack launch files

## Quick start

```bash
bash ros2-docker/run_lab_dev.sh lab4
```

Default launch publishes race-course vertices from CSV on `/desired_traj_vertices`.

### Launch options

```bash
# Vertices only (default)
bash ros2-docker/run_lab_dev.sh lab4

# Simple planner + controller (needs /current_state publisher for closed loop)
bash ros2-docker/run_lab_dev.sh lab4 -- \
  ros2 launch planner_pkg static_point_test.launch.py

# Optimized trajectory + controller
bash ros2-docker/run_lab_dev.sh lab4 -- \
  ros2 launch trajectory_generation traj_following.launch.py
```

### Simulator data

Waypoint CSV path (in order of precedence):

1. `VNAV_LAB4_DATA` environment variable
2. `~/vnav/tesse/lab4/lab4_Data` (Unity simulator build from [vnav.mit.edu/material/lab4.zip](https://vnav.mit.edu/material/lab4.zip))
3. Bundled sample: `VNAV-labs/lab4/data/` (5 waypoints for smoke testing)

The CSV has **no header row**. Object names look like `red_square_drone_door [0]`.

```bash
export VNAV_LAB4_DATA=/path/to/lab4_Data
bash ros2-docker/run_lab_dev.sh lab4
```

## Dependencies (pre-installed in Docker)

- [fishberg/mav_comm](https://github.com/fishberg/mav_comm) — `mav_msgs`, `mav_planning_msgs`
- [fishberg/mav_trajectory_generation](https://github.com/fishberg/mav_trajectory_generation) — polynomial trajectory library
- Lab 3 `controller_pkg`
- **Clang 19** — `planner_pkg` and `trajectory_generation` compile as **C++26** (`-std=c++26`)
