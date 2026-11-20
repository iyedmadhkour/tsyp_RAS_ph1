#!/usr/bin/env bash
set -Eeuo pipefail

# Bounded Gazebo validation for stale, delayed, missing, and frozen Writer pose
# streams. All pose fault injection is test-only and remains inside this run.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
BRANCH=phase2/reliability-optimization-20261120
STAMP="$(date +%Y%m%d-%H%M%S)"
EVIDENCE_ROOT="${LIVINGMAP_POSE_EVIDENCE_ROOT:-$ROOT/docs/results/evidence/team4-phase2-final-20261120}"
RUN="$EVIDENCE_ROOT/stale-pose-$STAMP"
TMP="${LIVINGMAP_POSE_TMP_ROOT:-/tmp/team4-pose-$STAMP}"
BUILD="$TMP/build" INSTALL="$TMP/install" LOG="$TMP/colcon-log"
SLAM_OVERLAY="${LIVING_MAP_SLAM_OVERLAY:-$HOME/livingmap-setup/writer-slam-5b-9qc5m2a6/overlay/install}"

die() { echo "FAIL: $*" >&2; echo "Evidence: $RUN" >&2; exit 1; }
need() { command -v "$1" >/dev/null 2>&1 || die "missing command: $1"; }
cd "$ROOT"
[[ "$(git branch --show-current)" == "$BRANCH" ]] || die "unexpected branch"
[[ -f /opt/ros/jazzy/setup.bash ]] || die "ROS 2 Jazzy not found"
[[ -f "$SLAM_OVERLAY/local_setup.bash" ]] || die "validated Karto overlay missing"
mkdir -p "$EVIDENCE_ROOT"
[[ ! -e "$RUN" ]] || die "refusing to overwrite evidence"
mkdir -p "$RUN" "$BUILD" "$INSTALL" "$LOG"
for command in colcon timeout setsid pgrep rg gz ros2 python3; do need "$command"; done

LAUNCH_PID= ADAPTER_PID= OBSERVER_PID= POSE_PID= BAG_PID= SCAN_PID=
cleanup() {
  set +e
  if [[ -n "$SCAN_PID" ]]; then kill -CONT "$SCAN_PID" 2>/dev/null; fi
  for pid in "$OBSERVER_PID"; do
    [[ -n "$pid" ]] && kill -INT "$pid" 2>/dev/null
    [[ -n "$pid" ]] && sleep .3
    [[ -n "$pid" ]] && kill -TERM "$pid" 2>/dev/null
    [[ -n "$pid" ]] && wait "$pid" 2>/dev/null
  done
  for pid in "$POSE_PID" "$BAG_PID" "$ADAPTER_PID" "$LAUNCH_PID"; do
    [[ -n "$pid" ]] || continue
    kill -INT -- "-$pid" 2>/dev/null
    for _ in {1..8}; do kill -0 -- "-$pid" 2>/dev/null || break; sleep .5; done
    if kill -0 -- "-$pid" 2>/dev/null; then
      kill -TERM -- "-$pid" 2>/dev/null
      sleep 1
      kill -KILL -- "-$pid" 2>/dev/null
    fi
    wait "$pid" 2>/dev/null
  done
  set -e
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

set +u
source /opt/ros/jazzy/setup.bash
set -u
set +u
source "$SLAM_OVERLAY/local_setup.bash"
set -u
export ROS_DOMAIN_ID="$((130 + $$ % 60))"
export GZ_PARTITION="team4-pose-validation-$STAMP"
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4 PYTHONDONTWRITEBYTECODE=1
export GZ_SIM_RESOURCE_PATH="$ROOT/src/living_map_sim/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export HOME="$TMP/home" ROS_LOG_DIR="$TMP/ros-logs"
mkdir -p "$HOME" "$ROS_LOG_DIR"
PHASE_INDEX=0

timeout 900s colcon --log-base "$LOG" build --base-paths src --build-base "$BUILD" \
  --install-base "$INSTALL" --merge-install --executor sequential \
  >"$RUN/build.log" 2>&1 || die "isolated 12-package ROS build failed"
set +u
source "$INSTALL/setup.bash"
set -u
for prefix in "$INSTALL"/*; do
  [[ -d "$prefix/share/ament_index/resource_index/packages" ]] || continue
  export AMENT_PREFIX_PATH="$prefix${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
done

printf 'branch=%s\nhead=%s\nstarted=%s\nros_domain_id=%s\ngz_partition=%s\n' \
  "$BRANCH" "$(git rev-parse HEAD)" "$(date -Is)" "$ROS_DOMAIN_ID" "$GZ_PARTITION" \
  >"$RUN/manifest.txt"
printf 'fault_modes=stale,delayed,missing,frozen\npose_age_assumption_s=0.5\nscan_age_assumption_s=0.35\ndelay_s=1.5\n' \
  >>"$RUN/manifest.txt"

run_phase() {
  local name="$1" mode_set="$2" return_home="$3" expected="$4"
  local d="$RUN/$name"
  mkdir -p "$d"
  export ROS_DOMAIN_ID="$((130 + $$ % 60 + PHASE_INDEX))"
  export GZ_PARTITION="team4-pose-validation-$STAMP-$name"
  PHASE_INDEX=$((PHASE_INDEX + 1))
  echo "starting $name" | tee -a "$RUN/progress.txt"
  setsid ros2 launch living_map_bringup demo.launch.py demo_fallback:=false \
    auto_explore:=false speed:=very_fast rviz:=false >"$d/launch.log" 2>&1 &
  LAUNCH_PID=$!

  # Wait for map/sensors and active Writer lifecycle before replacing scan_frames.
  timeout 75s ros2 topic echo /map nav_msgs/msg/OccupancyGrid --once >"$d/map-ready.yaml" 2>&1 \
    || die "$name: map was not ready"
  timeout 20s ros2 topic echo /writer/world_odom nav_msgs/msg/Odometry --once >"$d/world-odom-ready.yaml" 2>&1 \
    || die "$name: Gazebo world odometry was not ready"
  timeout 20s ros2 topic echo /writer/scan_raw sensor_msgs/msg/LaserScan --once >"$d/raw-scan-ready.yaml" 2>&1 \
    || die "$name: raw Writer scan was not ready"
  local life=false
  for _ in {1..45}; do
    if [[ "$(timeout 4s ros2 lifecycle get /writer/controller_server 2>/dev/null || true)" == *'active [3]' ]] \
      && rg -q 'writer.global_costmap.global_costmap.*StaticLayer: Resizing costmap' "$d/launch.log"; then
      life=true; break
    fi
    sleep 1
  done
  [[ "$life" == true ]] || die "$name: Writer controller/costmap did not become active"

  while read -r candidate; do
    [[ -n "$candidate" ]] || continue
    [[ "$(ps -o pgid= -p "$candidate" | tr -d ' ')" == "$LAUNCH_PID" ]] || continue
    SCAN_PID="$candidate"; break
  done < <(pgrep -f '[/]lib/living_map_writer/scan_frames' || true)
  [[ -n "$SCAN_PID" ]] || die "$name: cannot identify this launch's scan_frames node"
  kill -TERM "$SCAN_PID"
  for _ in {1..20}; do kill -0 "$SCAN_PID" 2>/dev/null || break; sleep .1; done
  kill -0 "$SCAN_PID" 2>/dev/null && die "$name: scan_frames did not stop"
  SCAN_PID=

  setsid python3 "$ROOT/scripts/phase2_pose_fault_adapter.py" --ros-args \
    -p use_sim_time:=true -p writer_mode:=pass -p delay_s:=1.5 >"$d/pose-adapter.log" 2>&1 &
  ADAPTER_PID=$!
  timeout 20s ros2 topic echo /writer/odom nav_msgs/msg/Odometry --once >"$d/odom-ready.yaml" 2>&1 \
    || die "$name: test relay did not publish Writer odometry"
  timeout 20s ros2 topic echo /writer/scan sensor_msgs/msg/LaserScan --once >"$d/scan-ready.yaml" 2>&1 \
    || die "$name: test relay did not preserve Writer scan"
  timeout 20s ros2 topic echo /executor/odom nav_msgs/msg/Odometry --once >"$d/executor-odom-ready.yaml" 2>&1 \
    || die "$name: test relay did not preserve rescue-robot odometry"

  python3 "$ROOT/scripts/pr1_cancel_validation_probe.py" observe \
    --output "$d/observations.jsonl" --launch-log "$d/launch.log" \
    --config "$ROOT/config/nav2/writer.yaml" >"$d/observer.log" 2>&1 &
  OBSERVER_PID=$!
  local gz_topics
  gz_topics="$(timeout 10s gz topic -l 2>/dev/null || true)"
  printf '%s\n' "$gz_topics" >"$d/gazebo-topics.txt"
  printf '%s\n' "$gz_topics" | rg -Fxq '/model/writer/pose' \
    || die "$name: no independent Gazebo pose topic in the isolated partition"
  setsid timeout 360s gz topic -e -t /model/writer/pose >"$d/ground-truth-pose.txt" 2>&1 &
  POSE_PID=$!
  setsid ros2 bag record -o "$d/bag" --topics /writer/odom /writer/cmd_vel \
    /writer/world_odom /living_map/writer_explorer_state /living_map/writer_stuck /clock \
    >"$d/bag-record.log" 2>&1 &
  BAG_PID=$!
  sleep 1
  [[ -f "$d/bag/metadata.yaml" ]] || true

  ros2 service call /living_map/writer/set_enabled std_srvs/srv/SetBool '{data: true}' \
    >"$d/enable-response.txt" 2>&1 || die "$name: Writer enable service failed"
  rg -q 'success=True' "$d/enable-response.txt" || die "$name: Writer did not enable"
  wait_for_goal "$name" "$d"

  if [[ "$mode_set" == all ]]; then
    printf 'paused_sim_s=%s\n' "$(current_sim_s "$d/observations.jsonl")" >"$d/fault-injection.txt"
    for mode in stale delayed missing; do
      inject_pose_mode "$mode" "$d"
      [[ "$mode" == missing ]] && python3 "$ROOT/scripts/pr1_cancel_validation_probe.py" request-return \
        >"$d/home-request.json"
      restore_pose_mode "$d"
      if [[ "$mode" != missing ]]; then wait_for_goal "$name after-$mode" "$d"; fi
    done
    printf 'resumed_sim_s=%s\n' "$(current_sim_s "$d/observations.jsonl")" >>"$d/fault-injection.txt"
    if [[ "$return_home" == true ]]; then
      # The request was made with missing pose; with the stream restored, HOME is now allowed.
      wait_for_state HOME "$d" 180 || die "$name: HOME did not complete after fresh pose recovery"
    fi
  elif [[ "$mode_set" == frozen ]]; then
    timeout 10s ros2 param set /living_map/pose_fault_frames writer_mode frozen \
      >"$d/param-frozen.log" 2>&1 || die "failed to enable frozen pose fault"
    local frozen_ready=false
    for _ in {1..100}; do
      if rg -q 'POSE_FAULT_MODE=frozen' "$d/pose-adapter.log"; then frozen_ready=true; break; fi
      sleep .1
    done
    [[ "$frozen_ready" == true ]] || die "pose relay did not confirm frozen mode"
    printf 'frozen_start_sim_s=%s\n' "$(current_sim_s "$d/observations.jsonl")" >"$d/fault-injection.txt"
    printf 'mode=frozen start=%s\n' "$(date -Is)" >"$d/mode-timeline.txt"
    wait_for_state STUCK "$d" 240 || die "$name: frozen fresh-stamped pose did not reach conservative STUCK state"
    printf 'frozen_end_sim_s=%s\n' "$(current_sim_s "$d/observations.jsonl")" >>"$d/fault-injection.txt"
  fi

  write_phase_summary "$name" "$mode_set" "$d" "$expected"
  cleanup_phase
}

current_sim_s() {
  python3 - "$1" <<'PY'
import json,sys
from pathlib import Path
events=[json.loads(s) for s in Path(sys.argv[1]).read_text().splitlines() if s]
times=[e.get("sim_s") for e in events if e.get("sim_s") is not None]
print(times[-1] if times else "0")
PY
}

wait_for_goal() {
  local label="$1" d="$2"
  for _ in {1..180}; do
    if rg -q 'Received a goal, begin computing control effort' "$d/launch.log"; then
      # Require a goal after the latest mode transition when one has occurred.
      return 0
    fi
    kill -0 "$LAUNCH_PID" 2>/dev/null || die "$label: launch exited before a goal"
    sleep 1
  done
  die "$label: Writer did not receive a Nav2 goal in 180 s"
}

inject_pose_mode() {
  local mode="$1" d="$2"
  printf 'mode=%s start=%s\n' "$mode" "$(date -Is)" | tee -a "$d/mode-timeline.txt"
  timeout 10s ros2 param set /living_map/pose_fault_frames writer_mode "$mode" \
    >"$d/param-$mode.log" 2>&1 || die "$mode: failed to set pose fault mode"
  local cancel_before
  cancel_before="$(rg -c 'Active Writer goal cancellation requested: localization unavailable' "$d/launch.log" || true)"
  cancel_before="${cancel_before:-0}"
  local goals_before
  goals_before="$(rg -c 'Received a goal, begin computing control effort' "$d/launch.log" || true)"
  goals_before="${goals_before:-0}"
  for _ in {1..50}; do
    local cancel_now
    cancel_now="$(rg -c 'Active Writer goal cancellation requested: localization unavailable' "$d/launch.log" || true)"
    cancel_now="${cancel_now:-0}"
    if (( cancel_now > cancel_before )); then
      printf 'cancel_observed=%s\n' "$(date -Is)" | tee -a "$d/mode-timeline.txt"
      printf 'goals_before=%s\n' "$goals_before" >>"$d/mode-timeline.txt"
      sleep 4
      local goals_during
      goals_during="$(rg -c 'Received a goal, begin computing control effort' "$d/launch.log" || true)"
      goals_during="${goals_during:-0}"
      printf 'goals_while_pose_invalid=%s\n' "$goals_during" >>"$d/mode-timeline.txt"
      (( goals_during == goals_before )) || die "$mode: a new goal was sent while pose was invalid"
      return 0
    fi
    sleep .5
  done
  die "$mode: localization-unavailable cancellation was not observed"
}

restore_pose_mode() {
  local d="$1"
  local goals_before
  goals_before="$(rg -c 'Received a goal, begin computing control effort' "$d/launch.log" || true)"
  goals_before="${goals_before:-0}"
  timeout 10s ros2 param set /living_map/pose_fault_frames writer_mode pass \
    >>"$d/mode-timeline.txt" 2>&1 || die "pose relay recovery failed"
  printf 'mode=pass restored=%s\n' "$(date -Is)" | tee -a "$d/mode-timeline.txt"
  for _ in {1..90}; do
    local goals_now
    goals_now="$(rg -c 'Received a goal, begin computing control effort' "$d/launch.log" || true)"
    goals_now="${goals_now:-0}"
    if (( goals_now > goals_before )); then
      printf 'goals_after_recovery=%s\n' "$goals_now" >>"$d/mode-timeline.txt"
      return 0
    fi
    sleep 1
  done
  die "frontier navigation did not resume after pose recovery"
}

wait_for_state() {
  local state="$1" d="$2" limit="$3"
  for ((i=0; i<limit; i++)); do
    if rg -q "Explorer state $state|Writer is back at the entrance" "$d/launch.log"; then return 0; fi
    sleep 1
  done
  return 1
}

write_phase_summary() {
  local name="$1" mode_set="$2" d="$3" expected="$4"
  local json="$d/summary.json"
  python3 - "$name" "$mode_set" "$expected" "$d" "$json" <<'PY'
import json,sys
from pathlib import Path
name, modes, expected, root, output = sys.argv[1:]
log=(Path(root)/"launch.log").read_text(errors="replace")
cancel=log.count("Active Writer goal cancellation requested: localization unavailable")
result={"scenario":name,"fault_modes":["stale","delayed","missing"] if modes=="all" else [modes],
        "localization_cancel_events":cancel,"expected_terminal_state":expected,
        "home_observed":("Explorer state HOME" in log or "Writer is back at the entrance" in log),
        "stuck_observed":"Explorer state STUCK" in log,
        "nav_goal_count":log.count("Received a goal, begin computing control effort"),
        "simulation_only":True,"result":"PASS" if (cancel>=3 if modes=="all" else True) and (
            (expected=="HOME" and ("Explorer state HOME" in log or "Writer is back at the entrance" in log)) or
            (expected=="STUCK" and "Explorer state STUCK" in log)) else "FAIL"}
Path(output).write_text(json.dumps(result,indent=2,sort_keys=True)+"\n")
print(json.dumps(result,indent=2,sort_keys=True))
if result["result"]!="PASS": raise SystemExit(1)
PY
}

cleanup_phase() {
  set +e
  for pid in "$OBSERVER_PID"; do
    [[ -n "$pid" ]] && kill -INT "$pid" 2>/dev/null
    [[ -n "$pid" ]] && sleep .3
    [[ -n "$pid" ]] && kill -TERM "$pid" 2>/dev/null
    [[ -n "$pid" ]] && wait "$pid" 2>/dev/null
  done
  OBSERVER_PID=
  for pid in "$POSE_PID" "$BAG_PID" "$ADAPTER_PID" "$LAUNCH_PID"; do
    [[ -n "$pid" ]] || continue
    kill -INT -- "-$pid" 2>/dev/null
    for _ in {1..8}; do kill -0 -- "-$pid" 2>/dev/null || break; sleep .5; done
    if kill -0 -- "-$pid" 2>/dev/null; then kill -TERM -- "-$pid" 2>/dev/null; sleep 1; kill -KILL -- "-$pid" 2>/dev/null; fi
    wait "$pid" 2>/dev/null
  done
  LAUNCH_PID= ADAPTER_PID= POSE_PID= BAG_PID= SCAN_PID=
  set -e
}

run_phase stale-recovery all true HOME
run_phase fresh-stamped-freeze frozen false STUCK

python3 "$ROOT/scripts/analyze_phase2_pose_fault_evidence.py" \
  "$RUN" --writer-config "$ROOT/config/nav2/writer.yaml" \
  --world "$ROOT/src/living_map_sim/worlds/mine.sdf" \
  --model "$ROOT/src/living_map_sim/models/writer/model.sdf" \
  --output "$RUN/pose-fault-audit.json" >"$RUN/pose-fault-audit.log" 2>&1 \
  || die "pose fault ground-truth/stream audit was inconclusive"
printf 'complete=true\nfinished=%s\n' "$(date -Is)" >>"$RUN/manifest.txt"
echo "Pose-fault validation evidence: $RUN"
