#!/usr/bin/env bash
set -euo pipefail

if [[ $# -eq 1 && ( "$1" == "-h" || "$1" == "--help" ) ]]; then
  echo "Usage: $0 /absolute/path/to/area_a_demo [playback_rate]"
  exit 0
fi

if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "Usage: $0 /absolute/path/to/area_a_demo [playback_rate]" >&2
  exit 2
fi

bag_path="$(realpath "$1")"
rate="${2:-1.0}"
play_bag="$bag_path"
temporary_bag=""

if [[ ! -f "$bag_path/metadata.yaml" ]]; then
  echo "Not a ROS 2 bag directory: $bag_path" >&2
  exit 2
fi
if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 is unavailable. Source ROS 2 and the workspace first." >&2
  exit 2
fi

for package in inland_loam mapconversion octomap_rviz_plugins; do
  if ! ros2 pkg prefix "$package" >/dev/null 2>&1; then
    echo "Missing ROS package: $package. Run scripts/install_dependencies.sh, build, and source the workspace." >&2
    exit 3
  fi
done

for target in inland_loam/inland_loam mapconversion/map_conversion_oct_node; do
  package="${target%%/*}"
  executable="${target#*/}"
  prefix="$(ros2 pkg prefix "$package")"
  node_path="$prefix/lib/$package/$executable"
  if [[ ! -x "$node_path" ]]; then
    echo "Executable not found: $node_path. Build and source the intended workspace." >&2
    exit 3
  fi
  if ! libraries="$(ldd "$node_path" 2>&1)"; then
    echo "Cannot check runtime libraries for $node_path: $libraries" >&2
    exit 3
  fi
  if [[ "$libraries" == *"not found"* ]]; then
    echo "Missing runtime libraries for $node_path:" >&2
    printf '%s\n' "$libraries" | grep 'not found' >&2
    echo "Run scripts/install_dependencies.sh, then source /opt/ros/humble/setup.bash and the workspace install/local_setup.bash." >&2
    exit 3
  fi
  echo "Using $node_path"
done
echo "Demo bag: $bag_path (playback rate: $rate)"

shopt -s nullglob
compressed_files=("$bag_path"/*.db3.zstd)
database_files=("$bag_path"/*.db3)
if (( ${#compressed_files[@]} > 0 && ${#database_files[@]} == 0 )); then
  temporary_bag="$(mktemp -d -t inland_loam_demo_bag_XXXXXX)"
  cp "$bag_path/metadata.yaml" "${compressed_files[@]}" "$temporary_bag/"
  play_bag="$temporary_bag"
fi

cleanup() {
  if [[ -n "$temporary_bag" && -d "$temporary_bag" ]]; then
    rm -rf -- "$temporary_bag"
  fi
}
trap cleanup EXIT

ros2 launch inland_loam demo.launch.py \
  bag_path:="$play_bag" rate:="$rate" use_rviz:=true
