#!/usr/bin/env bash
# Compatibility wrapper: prefer ros2-docker Lab 9 runners (no sudo).
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
NAV_ROOT=$(cd -- "$SCRIPT_DIR/../.." && pwd)
ROS2_DOCKER="$NAV_ROOT/ros2-docker"

usage() {
  cat <<EOF
Usage: run_docker.sh <kimera|orbslam|IMAGE_TAG>

Prefer the ros2-docker wrappers (macOS / Docker Desktop friendly):
  bash ros2-docker/run_kimera.sh
  bash ros2-docker/run_orbslam.sh

This script forwards:
  kimera  -> ros2-docker/run_kimera.sh
  orbslam -> ros2-docker/run_orbslam.sh
  other   -> docker run with Lab 9 mounts (no sudo)
EOF
}

TARGET=${1:-}
if [[ -z "$TARGET" || "$TARGET" == "--help" || "$TARGET" == "-h" ]]; then
  usage
  exit 0
fi
shift || true

case "$TARGET" in
  kimera|vnav-kimera)
    exec bash "$ROS2_DOCKER/run_kimera.sh" "$@"
    ;;
  orbslam|orbslam3|vnav-orbslam3)
    exec bash "$ROS2_DOCKER/run_orbslam.sh" "$@"
    ;;
esac

# Generic image run (legacy path)
DATA_DIR="${VNAV_LAB9_DATA:-$HOME/vnav/data}"
HOST_OS=$(uname -s)
CONTAINER_DISPLAY=${DISPLAY:-:0}
if [[ "$HOST_OS" == "Darwin" ]]; then
  CONTAINER_DISPLAY=host.docker.internal:0
fi

mkdir -p "$SCRIPT_DIR/output"
docker run --rm -it \
  -e DISPLAY="$CONTAINER_DISPLAY" \
  -e QT_X11_NO_MITSHM=1 \
  -v "$DATA_DIR:/datasets:ro" \
  -v "$SCRIPT_DIR/output:/output:rw" \
  -v "$SCRIPT_DIR/params:/kimera_params:ro" \
  "$@" "$TARGET"
