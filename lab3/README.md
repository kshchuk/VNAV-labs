# Lab 3 — Geometric Controller + Gazebo

Lab 3 implements a geometric tracking controller for a quadrotor. Simulation runs in **Gazebo Classic** inside the ROS 2 Docker container — no Unity/TESSE or UTM VM required.

## Quick start

```bash
# macOS: start XQuartz first, then allow connections
open -a XQuartz
xhost +localhost

bash ros2-docker/run_lab_dev.sh lab3
```

Gazebo opens with a quadrotor. Implement the controller in `controller_pkg/src/controller_node.cpp`, rebuild inside the container, and re-launch.

## Architecture

```
Gazebo Classic (quadrotor model)
        ↕  odometry / wrench
quadrotor_bridge  →  /current_state
        ↑  rotor_speed_cmds
controller_node  ←  desired_state  ←  traj_publisher
```

| Topic | Type | Direction |
|-------|------|-----------|
| `/current_state` | `nav_msgs/Odometry` | bridge → controller |
| `desired_state` | `trajectory_msgs/MultiDOFJointTrajectoryPoint` | traj_publisher → controller |
| `rotor_speed_cmds` | `mav_msgs/Actuators` | controller → bridge → Gazebo |

Physical constants in the controller (`m=1.0`, `cf=1e-3`, `d=0.3`, …) match the Gazebo model and bridge.

## Packages

| Package | Role |
|---------|------|
| `gazebo_quadrotor_pkg` | Gazebo world, URDF, topic bridge |
| `controller_pkg` | Student controller + trajectory publisher |

Build:

```bash
colcon build --symlink-install --packages-select gazebo_quadrotor_pkg controller_pkg
```

Launch manually:

```bash
ros2 launch controller_pkg lab3_stack.gazebo.launch.yaml
```

Headless (no GUI):

```bash
ros2 launch controller_pkg lab3_stack.gazebo.launch.yaml gui:=false
```

## macOS notes

- Install [XQuartz](https://www.xquartz.org/) for the Gazebo window.
- Docker runs `linux/amd64` — first Gazebo start may be slow under emulation.
- If the window does not appear: check `echo $DISPLAY`, run `xhost +localhost`.

## TESSE (legacy)

The original MIT lab used the TESSE Unity simulator (`tesse_ros_bridge`, `tesse-interface`). Those packages remain in the repo for reference but are **not** used by the default workflow.
