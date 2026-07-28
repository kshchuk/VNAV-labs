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

## Controller gain tuning (Part 6)

Gains in [`controller_pkg/config/params.yaml`](controller_pkg/config/params.yaml) were tuned **empirically** in Gazebo (Docker, macOS) using second-order control theory. Controller code and trajectory were unchanged — only gains.

### Method

1. **Initial estimate (control-theory seed)**  
   Position loop `F = -kx·ex - kv·ev + mg·e₃ + m·ad` modeled as second-order oscillation:
   - `ωn_pos = √(kx/m)`, `ζ_pos = kv / (2·ωn_pos)`  
   Attitude loop `τ = -kr·er - komega·eomega + ...`:
   - `ωn_att = √(kr)`, `ζ_att = komega / (2·ωn_att)`  

   Inner (attitude) loop should be faster than outer (position). For circle R=5 m, `timeScale=2.0` (~2.5 m/s), target `ωn_pos ≈ 2–4 rad/s`.

2. **Two-stage validation**
   - **Hover:** `STATIC_POSE=1` in `traj_publisher.cpp` — target `(0,0,2)`, check altitude stability.
   - **Circle:** `STATIC_POSE=0` — simple circle; first ~10 s is transient (desired jumps from `(0,0,2)` to `(0,5,2)`), then evaluate steady-state tracking.

3. **Iterative loop with runtime metrics**  
   Temporary logger in `controlLoop()` wrote NDJSON (`ex_xy`, `ex_z`, `|ev|`, `|er|`, `z`, `f`, `motor_max`). Logs confirmed or rejected hypotheses and guided gain changes.

### Iterations and results

| Iteration | kx | kv | kr | komega | Hover | Circle (steady-state) |
|-----------|----|----|----|--------|-------|------------------------|
| Initial (student) | 1 | 8 | 5 | 1 | slow | unstable, altitude loss |
| Seed | 16 | 8 | 40 | 8 | `\|ex\| < 0.02` m | `ex_xy` ≈ 1.3 m, z oscillations |
| Aggressive | 25 | 10 | 80 | 16 | — | excessive tilt, worse altitude |
| **Final** | **8** | **6** | **90** | **18** | stable | **`ex_xy` ≈ 0.07 m**, **z ≈ 2.0** |

**Conclusion:** excessive `kx` caused aggressive roll for horizontal error → reduced vertical thrust component. Softer position loop + stiff attitude loop (`kr=90`) gave stable circle tracking.

### Final gains

```yaml
kx: 8.0      # ωn_pos ≈ 2.8 rad/s
kv: 6.0      # ζ_pos ≈ 1.1 (slightly overdamped)
kr: 90.0     # ωn_att ≈ 9.5 rad/s
komega: 18.0 # ζ_att ≈ 0.95
```

After changing gains: `docker rm -f ros2-vnav-lab3-dev && bash ros2-docker/run_lab_dev.sh lab3`

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
