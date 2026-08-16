# Lab 9 — ORB-SLAM3 & Kimera-VIO (Docker)

Compare feature-based (ORB-SLAM3) and visual-inertial (Kimera-VIO) systems on EuRoC.

These run in **separate Docker images** (not the main `ros2-vnav:humble` lab container).

## Quick start

```bash
# 1. Download EuRoC MH_01_easy (~2–3 GB)
bash ros2-docker/download_euroc.sh

# 2. Build images (multi-hour the first time)
bash ros2-docker/run_orbslam.sh --build-only
bash ros2-docker/run_kimera.sh --build-only

# 3. Run
bash ros2-docker/run_orbslam.sh
bash ros2-docker/run_kimera.sh
```

Dataset default path: `$HOME/vnav/data/MH_01_easy` (override with `VNAV_LAB9_DATA`).

## Compatibility wrapper

```bash
bash VNAV-labs/lab9/run_docker.sh kimera
bash VNAV-labs/lab9/run_docker.sh orbslam
```

## Outputs

- Kimera logs / trajectories: `VNAV-labs/lab9/output/kimera/`
- ORB-SLAM3 trajectory: `VNAV-labs/lab9/output/orbslam/`

## macOS

- Needs Docker Desktop + (for GUI) XQuartz with `DISPLAY=:0`
- Images build as `linux/amd64` via QEMU on Apple Silicon (slow)

## Key files

| File | Purpose |
|------|---------|
| `Dockerfile_ORBSLAM3` | OpenCV 4.4, Pangolin, ORB-SLAM3 |
| `Dockerfile_KIMERA` | GTSAM, OpenGV, DBoW2, Kimera-RPGO, Kimera-VIO |
| `params/` | Kimera VIO parameters |
| `run_orbslam.sh` / `run_kimera.sh` | In-container entrypoints |
