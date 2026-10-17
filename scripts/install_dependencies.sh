#!/usr/bin/env bash
set -euo pipefail

if [[ "${ROS_DISTRO:-}" != "humble" ]]; then
  echo "Expected ROS_DISTRO=humble. Run: source /opt/ros/humble/setup.bash" >&2
  exit 2
fi

source /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "22.04" ]]; then
  echo "Warning: tested on Ubuntu 22.04; detected ${PRETTY_NAME:-unknown}." >&2
fi

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

check_only=false
if [[ "${1:-}" == "--check-only" ]]; then
  check_only=true
elif [[ $# -gt 0 ]]; then
  echo "Usage: $0 [--check-only]" >&2
  exit 2
fi

if ! $check_only; then
  sudo_cmd=()
  if [[ $EUID -ne 0 ]]; then
    sudo_cmd=(sudo)
    if [[ -t 0 ]]; then
      sudo -v
    elif ! sudo -n true 2>/dev/null; then
      echo "Dependency installation needs sudo. Run this script in an interactive terminal, or use --check-only." >&2
      exit 4
    fi
  fi

  "${sudo_cmd[@]}" apt-get update
  "${sudo_cmd[@]}" apt-get install -y \
    build-essential cmake git \
    libboost-all-dev libeigen3-dev liboctomap-dev libpcl-dev \
    python3-colcon-common-extensions python3-numpy python3-pil python3-rosdep python3-yaml \
    ros-humble-gtsam \
    ros-humble-octomap ros-humble-octomap-msgs ros-humble-octomap-ros \
    ros-humble-octomap-rviz-plugins \
    ros-humble-pcl-conversions ros-humble-rviz2 \
    ros-humble-rosbag2-compression-zstd \
    ros-humble-tf2-geometry-msgs

  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    "${sudo_cmd[@]}" rosdep init
  fi
  rosdep update
  rosdep install --from-paths "$repo_root" --ignore-src -r -y
fi

for command in cmake colcon git ros2; do
  if ! command -v "$command" >/dev/null 2>&1; then
    echo "Missing required command after dependency setup: $command" >&2
    exit 3
  fi
done

python3 -c 'import numpy; import PIL; import yaml'

gtsam_found=false
if ldconfig -p 2>/dev/null | grep -q 'libgtsam'; then
  gtsam_found=true
elif compgen -G "/opt/ros/$ROS_DISTRO/lib/libgtsam*.so*" >/dev/null; then
  gtsam_found=true
elif compgen -G "/opt/ros/$ROS_DISTRO/lib/*/libgtsam*.so*" >/dev/null; then
  gtsam_found=true
elif compgen -G "/usr/local/lib/libgtsam*.so*" >/dev/null; then
  gtsam_found=true
fi

if ! $gtsam_found; then
  cat >&2 <<'EOF'
GTSAM was not found. On ROS 2 Humble, run:
  sudo apt-get install ros-humble-gtsam
Then source /opt/ros/humble/setup.bash and rebuild the workspace.
EOF
  exit 3
fi

echo "Dependency check passed. Build from the workspace root with:"
echo "  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"
