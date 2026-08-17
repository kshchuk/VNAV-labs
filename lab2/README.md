# Lab 2 — Two Drones (ROS 2 + Foxglove)

Two AVs in the `world` frame. Static TF for Part 1; animated trajectories from `frames_publisher_node` after Deliverable 2. Markers and trails go out on `/visuals`.

## Quick start (Docker)

```bash
bash ros2-docker/run_lab_dev.sh lab2
```

Default launch is **static** TF (`static:=true`). Open [Foxglove Studio](https://app.foxglove.dev) → **Foxglove WebSocket** → `ws://localhost:8765`. In a **3D** panel enable **TF** and add **Markers** on `/visuals`.

Animated drones (after completing `frames_publisher_node.cpp`):

```bash
bash ros2-docker/run_lab_dev.sh lab2 -- \
  ros2 launch two_drones_pkg two_drones.launch.yaml static:=false
```

Without visualization:

```bash
bash ros2-docker/run_lab_dev.sh lab2 -- \
  ros2 launch two_drones_pkg two_drones.launch.yaml static:=true foxglove:=false
```

Lab 2 does **not** need XQuartz — Foxglove runs natively on the host.

## Launch arguments

| Arg | Default | Effect |
|-----|---------|--------|
| `static` | `false` in the YAML; Docker default is `true` | `true`: two `static_transform_publisher`s. `false`: `frames_publisher_node` |
| `foxglove` | `true` | Start `foxglove_bridge` on port 8765 |

`if` / `unless` in `two_drones.launch.yaml` pick static vs animated publishers.

## Topics and frames

| Node | Publishes | Subscribes |
|------|-----------|------------|
| `av1broadcaster` / `av2broadcaster` | `/tf` (`world` → `av1` / `av2`) | — |
| `frames_publisher_node` | `/tf` (dynamic) | — |
| `plots_publisher_node` | `/visuals` (`MarkerArray`) | `/tf` |
| `foxglove_bridge` | WebSocket | relevant ROS topics |

Frames: `world`, `av1`, `av2`. Drone meshes and trails: `/visuals`.

## Theory writeup

Deliverables 1, 4, and 5 (nodes/topics, SE(3) trajectories, quaternion identities):

- [`lab2_theory.tex`](lab2_theory.tex)
- [`lab2_theory.pdf`](lab2_theory.pdf)

Official handout: [vnav.mit.edu lab 2](https://vnav.mit.edu/labs_2023/lab2/exercises.html).

## Key files

| File | Purpose |
|------|---------|
| `two_drones_pkg/src/frames_publisher_node.cpp` | Dynamic TF for AV1 (circle) and AV2 (parabola) |
| `two_drones_pkg/src/plots_publisher_node.cpp` | Markers + trajectory trails on `/visuals` |
| `two_drones_pkg/launch/two_drones.launch.yaml` | Static vs animated launch |
