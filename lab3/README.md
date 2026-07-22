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

## Налаштування коефіцієнтів контролера (Part 6)

Коефіцієнти в [`controller_pkg/config/params.yaml`](controller_pkg/config/params.yaml) підібрані **емпірично** в Gazebo (Docker, macOS) з опорою на теорію другого порядку. Код контролера та траєкторія не змінювались — лише gains.

### Метод

1. **Початкова оцінка (control-theory seed)**  
   Позиційний контур `F = -kx·ex - kv·ev + mg·e₃ + m·ad` моделюється як коливання другого порядку:
   - `ωn_pos = √(kx/m)`, `ζ_pos = kv / (2·ωn_pos)`  
   Контур орієнтації `τ = -kr·er - komega·eomega + ...`:
   - `ωn_att = √(kr)`, `ζ_att = komega / (2·ωn_att)`  

   Внутрішній (attitude) контур має бути швидшим за зовнішній (position). Для кола R=5 m, `timeScale=2.0` (швидкість ~2.5 m/s) цільова `ωn_pos ≈ 2–4 rad/s`.

2. **Двоетапна валідація**
   - **Hover:** `STATIC_POSE=1` у `traj_publisher.cpp` — ціль `(0,0,2)`, перевірка стабільності висоти.
   - **Circle:** `STATIC_POSE=0` — просте коло; перші ~10 с є перехідним процесом (стрибок desired з `(0,0,2)` на `(0,5,2)`), далі оцінюється steady-state tracking.

3. **Ітераційний цикл з runtime-метриками**  
   Тимчасовий логер у `controlLoop()` записував NDJSON (`ex_xy`, `ex_z`, `|ev|`, `|er|`, `z`, `f`, `motor_max`). За логами підтверджували/відхиляли гіпотези та змінювали gains.

### Ітерації та результати

| Ітерація | kx | kv | kr | komega | Hover | Circle (steady-state) |
|----------|----|----|----|--------|-------|------------------------|
| Початкові (студент) | 1 | 8 | 5 | 1 | повільно | нестабільно, втрата висоти |
| Seed | 16 | 8 | 40 | 8 | `\|ex\| < 0.02` m | `ex_xy` ≈ 1.3 m, осциляції z |
| Aggressive | 25 | 10 | 80 | 16 | — | надмірний нахил, гірша висота |
| **Фінальні** | **8** | **6** | **90** | **18** | стабільно | **`ex_xy` ≈ 0.07 m**, **z ≈ 2.0** |

**Висновок:** завеликий `kx` змушував дрон різко нахилятись за горизонтальною помилкою → падала вертикальна складова тяги. М’якший position loop + жорсткий attitude loop (`kr=90`) дав стабільне коло.

### Фінальні коефіцієнти

```yaml
kx: 8.0      # ωn_pos ≈ 2.8 rad/s
kv: 6.0      # ζ_pos ≈ 1.1 (легко overdamped)
kr: 90.0     # ωn_att ≈ 9.5 rad/s
komega: 18.0 # ζ_att ≈ 0.95
```

Після зміни gains: `docker rm -f ros2-vnav-lab3-dev && bash ros2-docker/run_lab_dev.sh lab3`

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
