#!/usr/bin/env bash
set -Eeuo pipefail

# Bounded, simulation-only validation for review/pr1-writer-stuck-detection.
# Builds into a timestamped directory under $HOME and never touches the
# workspace's build/install/log/runtime directories or saved evidence.

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
EXPECTED_BRANCH="review/pr1-writer-stuck-detection"
EXPECTED_COMMIT="0f79291f1bcddaa750462de7b7ac425eaa540b8e"
STAMP="$(date +%Y%m%d-%H%M%S)"
RUN_ROOT="${LIVINGMAP_VALIDATION_ROOT:-$HOME/livingmap-pr1-validation}/$STAMP"
BUILD="$RUN_ROOT/build"
INSTALL="$RUN_ROOT/install"
LOGS="$RUN_ROOT/log"

die() { printf 'FAIL: %s\nEvidence: %s\n' "$*" "$RUN_ROOT" >&2; exit 1; }
need() { command -v "$1" >/dev/null 2>&1 || die "required command not found: $1"; }

[[ -d "$ROOT/.git" ]] || die "run this script from the LivingMap checkout"
cd "$ROOT"
[[ "$(git branch --show-current)" == "$EXPECTED_BRANCH" ]] || die "expected branch $EXPECTED_BRANCH"
git merge-base --is-ancestor "$EXPECTED_COMMIT" HEAD || die "HEAD is not based on expected review commit $EXPECTED_COMMIT"
if [[ -n "$(git status --porcelain --untracked-files=no)" ]]; then
  die "tracked working-tree changes exist; review them before validation"
fi
[[ -f /opt/ros/jazzy/setup.bash ]] || die "ROS 2 Jazzy is not installed at /opt/ros/jazzy"
need colcon
need pytest
need setsid
need timeout

ACTIVE_LAUNCH_PID=""
ACTIVE_STATE_PID=""
ACTIVE_STUCK_PID=""
ACTIVE_BAG_PID=""
cleanup_case() {
  set +e
  for pid in "${ACTIVE_STATE_PID:-}" "${ACTIVE_STUCK_PID:-}"; do
    [[ -n "$pid" ]] && kill -TERM "$pid" 2>/dev/null
  done
  for pid in "${ACTIVE_BAG_PID:-}" "${ACTIVE_LAUNCH_PID:-}"; do
    [[ -n "$pid" ]] || continue
    if [[ "$pid" == "$ACTIVE_BAG_PID" ]]; then
      kill -INT -- "-$pid" 2>/dev/null
    else
      kill -INT "$pid" 2>/dev/null
    fi
    for _ in {1..20}; do
      kill -0 "$pid" 2>/dev/null || break
      sleep 1
    done
    if kill -0 "$pid" 2>/dev/null; then
      kill -TERM -- "-$pid" 2>/dev/null
      sleep 2
      kill -0 "$pid" 2>/dev/null && kill -KILL -- "-$pid" 2>/dev/null
    fi
    wait "$pid" 2>/dev/null
  done
  for pid in "${ACTIVE_STATE_PID:-}" "${ACTIVE_STUCK_PID:-}"; do
    [[ -n "$pid" ]] && wait "$pid" 2>/dev/null
  done
  ACTIVE_LAUNCH_PID="" ACTIVE_STATE_PID="" ACTIVE_STUCK_PID="" ACTIVE_BAG_PID=""
  set -e
}
trap cleanup_case EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

mkdir -p "$BUILD" "$INSTALL" "$LOGS" "$RUN_ROOT/home" "$RUN_ROOT/cases"
set +u
source /opt/ros/jazzy/setup.bash
set -u
[[ "${ROS_DISTRO:-}" == jazzy ]] || die "ROS_DISTRO is not jazzy"
export HOME="$RUN_ROOT/home"
export ROS_DOMAIN_ID="$((100 + $$ % 100))"
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export PYTHONDONTWRITEBYTECODE=1

{
  printf 'branch=%s\n' "$EXPECTED_BRANCH"
  printf 'commit=%s\n' "$EXPECTED_COMMIT"
  printf 'ros_distro=%s\n' "$ROS_DISTRO"
  printf 'started=%s\n' "$(date -Is)"
  printf 'run_root=%s\n' "$RUN_ROOT"
} | tee "$RUN_ROOT/manifest.txt"

echo 'Building workspace into the isolated validation directory...'
timeout 900s colcon --log-base "$LOGS" build --base-paths src --build-base "$BUILD" \
  --install-base "$INSTALL" --symlink-install 2>&1 | tee "$RUN_ROOT/build.log"
set +u
source "$INSTALL/setup.bash"
set -u
# Some ROS package manifests omit an explicit ament build type. In an isolated
# install their marker is present, but colcon's generated setup may not add the
# individual prefix to AMENT_PREFIX_PATH. Expose only this temporary overlay.
for prefix in "$INSTALL"/*; do
  [[ -d "$prefix/share/ament_index/resource_index/packages" ]] || continue
  export AMENT_PREFIX_PATH="$prefix${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
done

WRITER_PREFIX="$(ros2 pkg prefix living_map_writer 2>/dev/null)" || die "ROS cannot discover living_map_writer"
[[ "$WRITER_PREFIX" == "$INSTALL"/* ]] || die "ROS resolved living_map_writer outside isolated install: $WRITER_PREFIX"
export LIVINGMAP_EXPECTED_BUILD="$BUILD"
export LIVINGMAP_EXPECTED_INSTALL="$INSTALL"
python3 - <<'PY' | tee "$RUN_ROOT/package-resolution.txt"
import inspect, os
from living_map_writer import node, progress
roots = (os.environ["LIVINGMAP_EXPECTED_BUILD"], os.environ["LIVINGMAP_EXPECTED_INSTALL"])
for name, module in (("Writer node", node), ("STUCK detector", progress)):
    path = inspect.getfile(module)
    print(f"{name}: {path}")
    if not any(root in path for root in roots):
        raise SystemExit(f"{name} did not load from the isolated build/install")
if not all(hasattr(progress.StuckDetector, item) for item in ("command", "odometry", "evaluate")):
    raise SystemExit("command/odometry STUCK detector API is missing")
PY

echo 'Running focused Writer detector, wiring, watchdog, and navigation tests...'
timeout 180s pytest -p no:cacheprovider -q \
  tests/test_writer_stuck_detection.py \
  tests/test_writer_node_stuck_wiring.py \
  tests/test_writer_watchdog_stuck_integration.py \
  tests/test_writer_stuck_integration_regressions.py \
  tests/test_writer_goal_watchdog.py \
  tests/test_writer_navigation.py 2>&1 | tee "$RUN_ROOT/pytest.log"

run_case() {
  local name="$1" limit_s="$2" fallback="$3" static_writer="$4" expected="$5"
  local case_dir="$RUN_ROOT/cases/$name"
  local runtime="$case_dir/runtime"
  mkdir -p "$runtime" "$case_dir/home"

  if [[ "$static_writer" == true ]]; then
    mkdir -p "$runtime/scenario"
    python3 - "$runtime/scenario" <<'PY'
import shutil, sys, xml.etree.ElementTree as ET
from pathlib import Path
out = Path(sys.argv[1])
tree = ET.parse("src/living_map_sim/worlds/mine.sdf")
writer = next(item for item in tree.findall(".//world/include")
              if item.findtext("name") == "writer")
ET.SubElement(writer, "static").text = "true"
pose = [float(value) for value in writer.findtext("pose").split()]
# Align the static robot with the fallback's first (1.8, -1.8) waypoint so
# the physical-stall case includes forward commands, not just a turn request.
pose[5] = -0.7853981633974483
writer.find("pose").text = " ".join(map(str, pose))
tree.write(out / "mine.sdf", encoding="unicode", xml_declaration=True)
shutil.copyfile("config/events.yaml", out / "events.yaml")
PY
  fi

  export HOME="$case_dir/home" LIVING_MAP_RUNTIME_DIR="$runtime"
  export GZ_SIM_RESOURCE_PATH="$ROOT/src/living_map_sim/models"
  local fallback_arg=false
  [[ "$fallback" == true ]] && fallback_arg=true
  setsid bash -c 'trap - INT; exec ros2 launch living_map_bringup demo.launch.py "$@"' _ \
    "demo_fallback:=$fallback_arg" auto_explore:=true speed:=very_fast rviz:=false \
    >"$case_dir/launch.log" 2>&1 &
  local launch_pid=$!
  ACTIVE_LAUNCH_PID="$launch_pid"
  ros2 topic echo /living_map/writer_explorer_state std_msgs/msg/String \
    --qos-durability transient_local >"$case_dir/state-topic.log" 2>&1 &
  local state_pid=$!
  ACTIVE_STATE_PID="$state_pid"
  ros2 topic echo /living_map/writer_stuck std_msgs/msg/Bool \
    --qos-durability transient_local >"$case_dir/stuck-topic.log" 2>&1 &
  local stuck_pid=$!
  ACTIVE_STUCK_PID="$stuck_pid"
  setsid bash -c 'trap - INT; exec ros2 bag record -o "$1" --topics "$2" "$3" "$4" "$5" "$6"' _ \
    "$case_dir/bag" /writer/odom /writer/cmd_vel /living_map/writer_explorer_state \
    /living_map/writer_stuck /clock >"$case_dir/bag-record.log" 2>&1 &
  local bag_pid=$!
  ACTIVE_BAG_PID="$bag_pid"

  local result=timeout
  for ((second=0; second<limit_s; second++)); do
    local state_home=false stuck=false watchdog=false
    rg -q 'data: HOME' "$case_dir/state-topic.log" 2>/dev/null && state_home=true || true
    rg -q 'data: true' "$case_dir/stuck-topic.log" 2>/dev/null && stuck=true || true
    rg -q 'Navigation watchdog cancelling' "$case_dir/launch.log" 2>/dev/null && watchdog=true || true
    case "$expected" in
      home)
        if [[ "$stuck" == true ]] || rg -q 'WRITER STUCK:' "$case_dir/launch.log"; then result=false_stuck; break; fi
        if [[ "$state_home" == true ]]; then result=home; break; fi
        ;;
      stuck)
        if [[ "$stuck" == true ]] || rg -q 'WRITER STUCK:' "$case_dir/launch.log"; then result=stuck; break; fi
        ;;
      watchdog)
        if [[ "$watchdog" == true ]] && { [[ "$stuck" == true ]] || rg -q 'WRITER STUCK:' "$case_dir/launch.log"; }; then
          result=watchdog_then_stuck; break
        fi
        ;;
    esac
    if ! kill -0 "$launch_pid" 2>/dev/null; then result=launch_exited; break; fi
    sleep 1
  done

  cleanup_case

  if [[ "$expected" == home || "$expected" == stuck ]]; then
    python3 - "$case_dir/bag" <<'PY' | tee "$case_dir/command-analysis.txt"
import sys
from pathlib import Path
import rosbag2_py
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=str(Path(sys.argv[1]) / "bag_0.mcap"), storage_id="mcap"),
            rosbag2_py.ConverterOptions("cdr", "cdr"))
types = {item.name: get_message(item.type) for item in reader.get_all_topics_and_types()}
turns = zeros = translations = odometry = 0
first = last = None
path_length = 0.0
max_deviation = 0.0
while reader.has_next():
    topic, payload, _stamp = reader.read_next()
    message = deserialize_message(payload, types[topic])
    if topic == "/writer/cmd_vel":
        assert isinstance(message, Twist)
        linear, angular = message.linear.x, message.angular.z
        translations += abs(linear) > 0.02
        turns += abs(angular) > 0.1
        zeros += abs(linear) <= 0.02 and abs(angular) <= 0.1
    elif topic == "/writer/odom":
        assert isinstance(message, Odometry)
        point = (message.pose.pose.position.x, message.pose.pose.position.y)
        first = point if first is None else first
        if last is not None:
            path_length += ((point[0]-last[0])**2 + (point[1]-last[1])**2)**0.5
        last = point
        odometry += 1
        max_deviation = max(max_deviation, ((point[0]-first[0])**2 + (point[1]-first[1])**2)**0.5)
distance = 0.0 if first is None or last is None else ((last[0]-first[0])**2 + (last[1]-first[1])**2)**0.5
print(f"cmd_translation_samples={translations}")
print(f"cmd_rotation_samples={turns}")
print(f"cmd_zero_samples={zeros}")
print(f"odom_samples={odometry}")
print(f"odom_path_length_m={path_length:.2f}")
print(f"odom_start_to_end_m={distance:.2f}")
print(f"odom_max_deviation_m={max_deviation:.2f}")
scenario = Path(sys.argv[1]).parent.name
if scenario == "physical-immobility":
    if not translations or odometry < 10 or max_deviation > 0.3:
        raise SystemExit("static Writer case did not show forward commands with stationary odometry")
elif not translations or not turns or not zeros or odometry < 10 or path_length < 1.0 or distance > 1.0:
    raise SystemExit("normal route did not demonstrate translation, rotation, stops, travel and return to HOME")
PY
  fi

  local case_status=PASS
  if [[ "$result" != "$expected" && ! ( "$expected" == watchdog && "$result" == watchdog_then_stuck ) ]]; then
    case_status=FAIL
    FAILURES=$((FAILURES + 1))
  fi
  printf '%s=%s expected=%s\n' "$name" "$result" "$expected" | tee -a "$RUN_ROOT/results.txt"
  printf '%s_status=%s\n' "$name" "$case_status" | tee -a "$RUN_ROOT/results.txt"
  printf 'Evidence: %s\n' "$case_dir"
}

FAILURES=0
echo 'Gazebo scenario 1: normal movement, turns, stops, no false STUCK, and HOME.'
run_case normal-home 360 true false home
echo 'Gazebo scenario 2: static Writer, continued motion commands, physical STUCK.'
run_case physical-immobility 120 true true stuck
echo 'Gazebo scenario 3: Nav2 watchdog cancellation followed by continued immobility and STUCK.'
run_case watchdog-immobility 210 false true watchdog

printf 'complete=true\nfinished=%s\n' "$(date -Is)" | tee -a "$RUN_ROOT/manifest.txt"
if (( FAILURES )); then
  printf '\nPARTIAL: %s Gazebo scenario(s) did not meet the expected result.\nEvidence: %s\n' \
    "$FAILURES" "$RUN_ROOT"
  exit 1
fi
printf '\nPASS: isolated build, focused tests, and all bounded Gazebo scenarios.\nEvidence: %s\n' "$RUN_ROOT"
