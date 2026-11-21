#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/_env.sh"
if [[ ! -f install/_local_setup_util_sh.py ]]; then
  echo "[LivingMap] Workspace install is missing/incomplete; rebuilding it now."
  ./scripts/build.sh
  source "$(dirname "$0")/_env.sh"
fi
command -v ros2 >/dev/null || { echo "ROS 2 Jazzy required. For core validation here run: ./tools/headless_demo.py"; exit 2; }
mkdir -p runtime
# VMware shared-memory locks can survive an interrupted Fast DDS process and
# prevent reliable discovery on the next judge run. The demo is entirely local,
# so UDP loopback is deterministic and avoids that host-specific failure mode.
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
if pgrep -f '^gz sim server$' >/dev/null 2>&1; then
  echo "A stale Gazebo server is running. Run ./scripts/reset_demo.sh before starting a new demo."
  exit 2
fi
cleanup_demo() {
  pkill -f '^gz sim server$' 2>/dev/null || true
  pkill -f "$LIVING_MAP_ROOT/install/" 2>/dev/null || true
}
trap cleanup_demo EXIT INT TERM
fallback=false
check_slam_overlay=false
speed="${LIVING_MAP_SPEED:-normal}"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --fallback) fallback=true ;;
    --check-slam-overlay) check_slam_overlay=true ;;
    --speed) [[ $# -ge 2 ]] || { echo "--speed needs a value: normal, fast or very_fast"; exit 2; }; speed="$2"; shift ;;
    --speed=*) speed="${1#--speed=}" ;;
    *) echo "Usage: $0 [--fallback] [--speed normal|fast|very_fast]"; exit 2 ;;
  esac
  shift
done
speed="${speed//-/_}"
case "$speed" in
  normal|fast|very_fast) ;;
  *) echo "Unknown speed '$speed'; choose normal, fast or very_fast"; exit 2 ;;
esac

# The Karto endpoint fix is opt-in and outside /opt/ros. Source it only after
# the normal Jazzy and LivingMap environments so it takes precedence here.
if [[ -n "${LIVING_MAP_SLAM_OVERLAY:-}" ]]; then
  overlay_prefix="$(realpath -e "$LIVING_MAP_SLAM_OVERLAY")" || {
    echo "SLAM overlay path does not exist: $LIVING_MAP_SLAM_OVERLAY" >&2; exit 2;
  }
  if [[ ! -d "$overlay_prefix" || ! -f "$overlay_prefix/local_setup.bash" ]]; then
    echo "SLAM overlay must be a ROS install prefix containing local_setup.bash: $overlay_prefix" >&2
    exit 2
  fi
  overlay_restore_nounset=0
  case $- in *u*) overlay_restore_nounset=1; set +u ;; esac
  source "$overlay_prefix/local_setup.bash"
  if [[ "$overlay_restore_nounset" -eq 1 ]]; then set -u; fi
  unset overlay_restore_nounset
  selected_prefix="$(ros2 pkg prefix slam_toolbox)"
  selected_prefix="$(realpath -e "$selected_prefix")"
  if [[ "$selected_prefix" != "$overlay_prefix" ]]; then
    echo "SLAM overlay selection failed: expected $overlay_prefix, got $selected_prefix" >&2
    exit 2
  fi
  echo "[LivingMap] SLAM Toolbox overlay: $selected_prefix"
elif [[ "$check_slam_overlay" == true ]]; then
  echo "--check-slam-overlay requires LIVING_MAP_SLAM_OVERLAY to name an install prefix" >&2
  exit 2
fi
if [[ "$check_slam_overlay" == true ]]; then
  echo "[LivingMap] Overlay selection verified; Gazebo was not started."
  exit 0
fi
export LIVING_MAP_SPEED="$speed"
echo "[LivingMap] Robot speed preset: $speed"
if [[ "$fallback" == true ]]; then
  export LIVING_MAP_DEMO_MODE=guided
  echo "[LivingMap] DEMO FALLBACK requested: deterministic Writer waypoints (frontier exploration disabled)."
else
  export LIVING_MAP_DEMO_MODE=autonomous
fi
ros2 launch living_map_bringup demo.launch.py demo_fallback:="$fallback" speed:="$speed" auto_explore:="${LIVING_MAP_AUTO_EXPLORE:-false}" rviz:="${LIVING_MAP_RVIZ:-true}"
