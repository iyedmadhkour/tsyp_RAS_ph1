#!/usr/bin/env bash
# Run against /opt/ros/jazzy to reproduce failure, or the overlay install to verify.
set -euo pipefail
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
slam_prefix="$(realpath "${1:?Usage: test_slam_grid_extents.sh SLAM_INSTALL_PREFIX}")"
test_dir="$(mktemp -d "${TMPDIR:-/tmp}/livingmap-slam-grid-test.XXXXXX")"
echo "Diagnostic artifacts: $test_dir"
g++ -std=c++17 -O1 -I"$slam_prefix/include" \
  "$project_dir/tests/native/verify_slam_grid_extents.cpp" \
  -L"$slam_prefix/lib" -lkartoSlamToolbox -lboost_serialization \
  -Wl,-rpath,"$slam_prefix/lib" -o "$test_dir/verify_slam_grid_extents"
# Explicit library selection also makes a baseline comparison independent of overlays.
LD_LIBRARY_PATH="$slam_prefix/lib:${LD_LIBRARY_PATH:-}" "$test_dir/verify_slam_grid_extents"
