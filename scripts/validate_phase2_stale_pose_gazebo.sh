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
# Keep every top-level failure message with the run evidence. The previous
# interrupted attempt retained ROS logs but lost the driver's first failure.
exec > >(tee -a "$RUN/harness.log") 2>&1

set +u
source /opt/ros/jazzy/setup.bash
set -u
set +u
source "$SLAM_OVERLAY/local_setup.bash"
set -u
for command in colcon timeout setsid pgrep rg gz ros2 python3; do need "$command"; done

LAUNCH_PID= ADAPTER_PID= OBSERVER_PID= POSE_PID= BAG_PID= SCAN_PID=
stop_process_group() {
  local pid="$1"
  [[ -n "$pid" ]] || return 0
  kill -INT -- "-$pid" 2>/dev/null || true
  for _ in {1..8}; do kill -0 -- "-$pid" 2>/dev/null || break; sleep .5; done
  if kill -0 -- "-$pid" 2>/dev/null; then
    kill -TERM -- "-$pid" 2>/dev/null || true
    sleep 1
    kill -KILL -- "-$pid" 2>/dev/null || true
  fi
  wait "$pid" 2>/dev/null || true
}

stop_launch() {
  local pid="$1" state=
  [[ -n "$pid" ]] || return 0
  # Signal the launch manager first. Sending SIGINT to its entire process
  # group kills gz directly while ros2 launch is still running; its
  # respawn=True policy can then start a replacement Gazebo during teardown.
  kill -INT "$pid" 2>/dev/null || true
  for _ in {1..40}; do
    state="$(ps -o stat= -p "$pid" 2>/dev/null | tr -d ' ' || true)"
    [[ -z "$state" || "$state" == Z* ]] && break
    sleep .25
  done
  state="$(ps -o stat= -p "$pid" 2>/dev/null | tr -d ' ' || true)"
  if [[ -n "$state" && "$state" != Z* ]]; then
    kill -TERM "$pid" 2>/dev/null || true
    sleep 1
    state="$(ps -o stat= -p "$pid" 2>/dev/null | tr -d ' ' || true)"
    if [[ -n "$state" && "$state" != Z* ]]; then
      kill -KILL -- "-$pid" 2>/dev/null || true
    fi
  fi
  wait "$pid" 2>/dev/null || true
}

cleanup() {
  set +e
  if [[ -n "$SCAN_PID" ]]; then kill -CONT "$SCAN_PID" 2>/dev/null; fi
  for pid in "$OBSERVER_PID"; do
    [[ -n "$pid" ]] && kill -INT "$pid" 2>/dev/null
    [[ -n "$pid" ]] && sleep .3
    [[ -n "$pid" ]] && kill -TERM "$pid" 2>/dev/null
    [[ -n "$pid" ]] && wait "$pid" 2>/dev/null
  done
  stop_launch "$LAUNCH_PID"
  stop_process_group "$POSE_PID"
  stop_process_group "$BAG_PID"
  stop_process_group "$ADAPTER_PID"
  set -e
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

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
printf 'fault_modes=stale,delayed,missing,frozen_translation,frozen_rotation\nspeed_preset=normal\npose_age_assumption_s=0.5\nscan_age_assumption_s=0.35\ndelay_s=1.5\n' \
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
    auto_explore:=false speed:=normal rviz:=false >"$d/launch.log" 2>&1 &
  LAUNCH_PID=$!

  # Confirm Gazebo sensor input first. A previous map timeout showed
  # slam_toolbox stuck in Configuring; scan_frames exited later during normal
  # harness teardown and was not the initiating failure.
  timeout 20s ros2 topic echo /writer/world_odom nav_msgs/msg/Odometry --once >"$d/world-odom-ready.yaml" 2>&1 \
    || { capture_startup_diagnostics "$d"; die "$name: Gazebo world odometry was not ready"; }
  timeout 20s ros2 topic echo /writer/scan_raw sensor_msgs/msg/LaserScan --once >"$d/raw-scan-ready.yaml" 2>&1 \
    || { capture_startup_diagnostics "$d"; die "$name: raw Writer scan was not ready"; }
  echo "$name: Gazebo odometry and raw scan are publishing"
  wait_for_slam_active "$name" "$d"
  timeout 20s ros2 topic echo /map nav_msgs/msg/OccupancyGrid \
    --qos-durability transient_local --once >"$d/map-ready.yaml" 2>&1 \
    || { capture_startup_diagnostics "$d"; die "$name: SLAM did not publish a latched map after activation"; }
  echo "$name: SLAM active and map received"
  local life=false
  for _ in {1..45}; do
    if [[ "$(timeout 4s ros2 lifecycle get /writer/controller_server 2>/dev/null || true)" == *'active [3]' ]] \
      && rg -q 'writer.global_costmap.global_costmap.*StaticLayer: Resizing costmap' "$d/launch.log"; then
      life=true; break
    fi
    sleep 1
  done
  [[ "$life" == true ]] || die "$name: Writer controller/costmap did not become active"
  echo "$name: Writer controller and static costmap active"

  while read -r candidate; do
    [[ -n "$candidate" ]] || continue
    [[ "$(ps -o pgid= -p "$candidate" | tr -d ' ')" == "$LAUNCH_PID" ]] || continue
    SCAN_PID="$candidate"; break
  done < <(pgrep -f '[/]lib/living_map_writer/scan_frames' || true)
  [[ -n "$SCAN_PID" ]] || die "$name: cannot identify this launch's scan_frames node"
  kill -TERM "$SCAN_PID"
  for _ in {1..20}; do
    state="$(ps -o stat= -p "$SCAN_PID" 2>/dev/null | tr -d ' ' || true)"
    [[ -z "$state" || "$state" == Z* ]] && break
    sleep .1
  done
  state="$(ps -o stat= -p "$SCAN_PID" 2>/dev/null | tr -d ' ' || true)"
  [[ -z "$state" || "$state" == Z* ]] \
    || die "$name: scan_frames did not stop within 2 s (state=$state)"
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
  for _ in {1..50}; do
    # rosbag2 writes metadata.yaml when recording is finalized, not at start.
    # During the run, the reliable bounded readiness signal is that every
    # requested topic has been discovered and subscribed.
    rg -q 'All requested topics are subscribed' "$d/bag-record.log" && break
    sleep .1
  done
  rg -q 'All requested topics are subscribed' "$d/bag-record.log" \
    || die "$name: ROS bag did not subscribe to all requested topics"
  echo "$name: fault relay and bag recorder ready"

  # Startup is a legitimate stationary pose case: fresh advancing estimates
  # with no commanded movement must not trip the integrity latch.
  local stationary_start stationary_end stationary_log_lines
  sleep 1
  stationary_start="$(current_sim_s "$d/observations.jsonl")"
  stationary_log_lines="$(wc -l <"$d/launch.log")"
  stationary_end="$stationary_start"
  for _ in {1..50}; do
    sleep .1
    stationary_end="$(current_sim_s "$d/observations.jsonl")"
    python3 - "$stationary_start" "$stationary_end" <<'PY' && break
import sys
raise SystemExit(0 if float(sys.argv[2])-float(sys.argv[1]) >= .75 else 1)
PY
  done
  python3 - "$stationary_start" "$stationary_end" <<'PY' \
    || die "$name: stationary observation window did not span 0.75 simulation seconds"
import sys
raise SystemExit(0 if float(sys.argv[2])-float(sys.argv[1]) >= .75 else 1)
PY
  printf 'start_sim_s=%s\nend_sim_s=%s\n' "$stationary_start" "$stationary_end" \
    >"$d/stationary-window.txt"
  echo "$name: checked legitimate stationary window ($stationary_start to $stationary_end sim s)"
  if tail -n "+$((stationary_log_lines + 1))" "$d/launch.log" \
      | rg -q 'pose integrity could not be established'; then
    die "$name: stationary robot falsely tripped pose integrity"
  fi
  if rg -q 'pose integrity could not be established' "$d/launch.log"; then
    die "$name: stationary startup falsely tripped pose integrity"
  fi

  timeout 10s ros2 service call /living_map/writer/set_enabled std_srvs/srv/SetBool '{data: true}' \
    >"$d/enable-response.txt" 2>&1 \
    || { capture_startup_diagnostics "$d"; die "$name: Writer enable service did not answer within 10 s"; }
  rg -q 'success=True' "$d/enable-response.txt" || die "$name: Writer did not enable"
  wait_for_goal "$name" "$d"

  if [[ "$mode_set" == all ]]; then
    printf 'paused_sim_s=%s\n' "$(current_sim_s "$d/observations.jsonl")" >"$d/fault-injection.txt"
    for mode in stale delayed missing; do
      inject_pose_mode "$mode" "$d"
      [[ "$mode" == missing ]] && python3 "$ROOT/scripts/pr1_cancel_validation_probe.py" request-return \
        >"$d/home-request.json"
      restore_pose_mode "$mode" "$d"
      if [[ "$mode" != missing ]]; then wait_for_goal "$name after-$mode" "$d"; fi
    done
    printf 'resumed_sim_s=%s\n' "$(current_sim_s "$d/observations.jsonl")" >>"$d/fault-injection.txt"
    if [[ "$return_home" == true ]]; then
      # The request was made with missing pose; with the stream restored, HOME is now allowed.
      wait_for_state HOME "$d" 180 || die "$name: HOME did not complete after fresh pose recovery"
    fi
  elif [[ "$mode_set" == frozen-translation || "$mode_set" == frozen-rotation ]]; then
    local motion_detail
    if [[ "$mode_set" == frozen-translation ]]; then
      motion_detail="$(wait_for_translation "$name" "$d" 180)"
    else
      motion_detail="$(wait_for_rotation "$name" "$d" 180)"
    fi
    printf 'pre_fault_motion=%s\n' "$motion_detail" | tee -a "$d/mode-timeline.txt"
    inject_frozen_pose_and_recover "$name" "$d" "$return_home" "${mode_set//-/_}"
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

wait_for_slam_active() {
  local label="$1" d="$2" active=false state=
  for _ in {1..20}; do
    state="$(timeout 3s ros2 lifecycle get /slam_toolbox 2>&1 || true)"
    if [[ "$state" == *'active [3]'* ]]; then active=true; break; fi
    sleep .5
  done
  if [[ "$active" != true ]]; then
    printf '%s\n' "$state" >"$d/slam-state-before-recovery.txt"
    case "$state" in
      *'unconfigured [1]'*)
        timeout 12s ros2 lifecycle set /slam_toolbox configure \
          >"$d/slam-configure-recovery.txt" 2>&1 || true
        if rg -q 'Transitioning successful' "$d/slam-configure-recovery.txt"; then
          timeout 12s ros2 lifecycle set /slam_toolbox activate \
            >"$d/slam-activate-recovery.txt" 2>&1 || true
        fi
        ;;
      *'inactive [2]'*)
        timeout 12s ros2 lifecycle set /slam_toolbox activate \
          >"$d/slam-activate-recovery.txt" 2>&1 || true
        ;;
      *)
        capture_startup_diagnostics "$d"
        die "$label: SLAM lifecycle did not reach active; state was: $state"
        ;;
    esac
    for _ in {1..20}; do
      state="$(timeout 3s ros2 lifecycle get /slam_toolbox 2>&1 || true)"
      if [[ "$state" == *'active [3]'* ]]; then active=true; break; fi
      sleep .5
    done
  fi
  if [[ "$active" != true ]]; then
    capture_startup_diagnostics "$d"
    die "$label: one bounded SLAM lifecycle recovery did not reach active state"
  fi
  printf '%s\n' "$state" >"$d/slam-state-ready.txt"
}

capture_startup_diagnostics() {
  local d="$1"
  timeout 5s ros2 node list >"$d/startup-node-list.txt" 2>&1 || true
  timeout 5s ros2 lifecycle get /slam_toolbox >"$d/startup-slam-state.txt" 2>&1 || true
  timeout 5s ros2 topic info --verbose /writer/scan_raw >"$d/startup-raw-scan-info.txt" 2>&1 || true
  timeout 5s ros2 topic info --verbose /writer/scan >"$d/startup-relayed-scan-info.txt" 2>&1 || true
  timeout 5s ros2 topic info --verbose /map >"$d/startup-map-info.txt" 2>&1 || true
  timeout 5s ros2 topic info --verbose /clock >"$d/startup-clock-info.txt" 2>&1 || true
  timeout 5s gz topic -l >"$d/startup-gazebo-topics.txt" 2>&1 || true
  ps -eo pid,ppid,pgid,stat,comm,args >"$d/startup-processes.txt"
}

inject_pose_mode() {
  local mode="$1" d="$2"
  printf 'mode=%s start=%s\n' "$mode" "$(date -Is)" | tee -a "$d/mode-timeline.txt"
  # Snapshot evidence before changing the relay: a safety callback may latch
  # synchronously before `ros2 param set` returns.
  local cancel_before integrity_before
  cancel_before="$(rg -c 'Active Writer goal cancellation requested: localization unavailable' "$d/launch.log" || true)"
  cancel_before="${cancel_before:-0}"
  integrity_before="$(rg -c 'Navigation locked out; no further goals will be sent: pose integrity could not be established' "$d/launch.log" || true)"
  integrity_before="${integrity_before:-0}"
  set_pose_fault_mode "$mode" "$d" \
    || die "$mode: relay did not confirm requested mode after bounded parameter call"
  for _ in {1..50}; do
    local cancel_now integrity_now
    cancel_now="$(rg -c 'Active Writer goal cancellation requested: localization unavailable' "$d/launch.log" || true)"
    cancel_now="${cancel_now:-0}"
    integrity_now="$(rg -c 'Navigation locked out; no further goals will be sent: pose integrity could not be established' "$d/launch.log" || true)"
    integrity_now="${integrity_now:-0}"
    if (( cancel_now > cancel_before || integrity_now > integrity_before )); then
      if (( integrity_now > integrity_before )); then
        printf 'pose_integrity_lockout_observed=%s\n' "$(date -Is)" | tee -a "$d/mode-timeline.txt"
      else
        printf 'localization_cancel_observed=%s\n' "$(date -Is)" | tee -a "$d/mode-timeline.txt"
      fi
      local goals_during goals_after_lockout
      goals_during="$(goals_after_fault_mode "$mode" "$d")"
      printf 'goals_after_fault_mode=%s\n' "$goals_during" >>"$d/mode-timeline.txt"
      sleep 4
      goals_after_lockout="$(goals_after_fault_mode "$mode" "$d")"
      printf 'goals_after_lockout_window=%s\n' "$goals_after_lockout" >>"$d/mode-timeline.txt"
      (( goals_after_lockout == goals_during )) \
        || die "$mode: a new Nav2 goal was sent after the pose fault was latched"
      return 0
    fi
    sleep .5
  done
  die "$mode: localization-unavailable cancellation was not observed"
}

set_pose_fault_mode() {
  local mode="$1" d="$2" before after client_status=received
  before="$(rg -c -F "POSE_FAULT_MODE=$mode" "$d/pose-adapter.log" || true)"
  before="${before:-0}"
  if ! timeout 10s ros2 param set /living_map/pose_fault_frames writer_mode "$mode" \
      >"$d/param-$mode.log" 2>&1; then
    client_status=timed_out_or_transport_error
  fi
  for _ in {1..50}; do
    after="$(rg -c -F "POSE_FAULT_MODE=$mode" "$d/pose-adapter.log" || true)"
    after="${after:-0}"
    if (( after > before )); then
      printf 'mode=%s\nclient_status=%s\nconfirmed_by=pose-adapter-log\n' \
        "$mode" "$client_status" >"$d/param-$mode-confirmation.txt"
      [[ "$client_status" == received ]] \
        || printf 'late_application_confirmed=true\n' >>"$d/param-$mode-confirmation.txt"
      return 0
    fi
    sleep .1
  done
  capture_startup_diagnostics "$d"
  printf 'mode=%s\nclient_status=%s\nconfirmed_by=none\n' "$mode" "$client_status" \
    >"$d/param-$mode-confirmation.txt"
  return 1
}

goals_after_fault_mode() {
  python3 - "$1" "$2" <<'PY'
import re,sys
from pathlib import Path
mode,root=sys.argv[1:]
adapter=(Path(root)/"pose-adapter.log").read_text(errors="replace")
launch=(Path(root)/"launch.log").read_text(errors="replace")
match=re.search(r"\[WARN\] \[([0-9.]+)\].*POSE_FAULT_MODE="+re.escape(mode),adapter)
if not match:
    raise SystemExit("fault relay activation timestamp not found")
fault_time=float(match.group(1))
goals=[float(t) for t in re.findall(
    r"\[INFO\] \[([0-9.]+)\].*\[writer\.controller_server\]: Received a goal, begin computing control effort",
    launch)]
print(sum(t >= fault_time for t in goals))
PY
}

restore_pose_mode() {
  local mode="$1" d="$2"
  local goals_before
  goals_before="$(rg -c 'Received a goal, begin computing control effort' "$d/launch.log" || true)"
  goals_before="${goals_before:-0}"
  set_pose_fault_mode pass "$d" || die "pose relay recovery was not confirmed"
  printf 'mode=pass restored=%s\n' "$(date -Is)" | tee -a "$d/mode-timeline.txt"
  if rg -q 'pose integrity could not be established' "$d/launch.log"; then
    local acked=false
    for _ in {1..60}; do
      local response
      response="$(timeout 4s ros2 service call /writer/ack_pose_integrity_recovery \
        std_srvs/srv/Trigger '{}' 2>&1 || true)"
      if [[ "$response" == *'success=True'* ]]; then
        printf '%s\n' "$response" >"$d/recovery-ack-$mode.log"
        acked=true
        break
      fi
      printf '%s\n' "$response" >"$d/recovery-ack-$mode-last-attempt.log"
      sleep .3
    done
    [[ "$acked" == true ]] || die "$mode: safe pose-integrity recovery acknowledgment did not become ready"
  fi
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

wait_for_rotation() {
  local label="$1" d="$2" limit="$3"
  for ((i=0; i<limit; i++)); do
    if python3 - "$d/observations.jsonl" <<'PY'
import json,sys
from pathlib import Path
import math
events=[]
for line in Path(sys.argv[1]).read_text().splitlines()[-2500:]:
    try: event=json.loads(line)
    except ValueError: continue
    events.append(event)
commands=[e for e in events if e.get("event")=="writer_cmd_vel"
          and e.get("sim_s") is not None
          and abs(float(e.get("data",{}).get("angular_z",0.0)))>.1
          and abs(float(e.get("data",{}).get("linear_x",0.0)))<.02]
poses=[e for e in events if e.get("event")=="gazebo_world_odom"
       and e.get("sim_s") is not None]
def delta(a,b): return abs(math.atan2(math.sin(a-b),math.cos(a-b)))
for command in reversed(commands):
    ct=float(command["sim_s"])
    samples=[(float(e["sim_s"]),float(e["data"]["yaw"])) for e in poses
             if ct<=float(e["sim_s"])<=ct+2.0]
    if len(samples)>=2 and max(delta(a[1],b[1]) for a in samples for b in samples)>.02:
        print(f"sim_s={ct:.3f}; yaw_range_rad={max(delta(a[1],b[1]) for a in samples for b in samples):.4f}")
        raise SystemExit(0)
raise SystemExit(1)
PY
    then return 0; fi
    kill -0 "$LAUNCH_PID" 2>/dev/null || die "$label: launch exited before a rotation command"
    sleep 1
  done
  die "$label: no in-place rotation command observed within ${limit}s"
}

wait_for_translation() {
  local label="$1" d="$2" limit="$3"
  for ((i=0; i<limit; i++)); do
    if python3 - "$d/observations.jsonl" <<'PY'
import json,sys
from pathlib import Path
import math
events=[]
for line in Path(sys.argv[1]).read_text().splitlines()[-2500:]:
    try: event=json.loads(line)
    except ValueError: continue
    events.append(event)
commands=[e for e in events if e.get("event")=="writer_cmd_vel"
          and e.get("sim_s") is not None
          and abs(float(e.get("data",{}).get("linear_x",0.0)))>.1]
poses=[e for e in events if e.get("event")=="gazebo_world_odom"
       and e.get("sim_s") is not None]
for command in reversed(commands):
    ct=float(command["sim_s"])
    samples=[(float(e["sim_s"]),float(e["data"]["x"]),float(e["data"]["y"]))
             for e in poses if ct<=float(e["sim_s"])<=ct+2.0]
    if len(samples)>=2:
        displacement=max(math.hypot(a[1]-b[1],a[2]-b[2])
                         for a in samples for b in samples)
        if displacement>.02:
            print(f"sim_s={ct:.3f}; displacement_m={displacement:.4f}")
            raise SystemExit(0)
raise SystemExit(1)
PY
    then return 0; fi
    kill -0 "$LAUNCH_PID" 2>/dev/null || die "$label: launch exited before forward motion"
    sleep 1
  done
  die "$label: no measured forward movement observed within ${limit}s"
}

inject_frozen_pose_and_recover() {
  local label="$1" d="$2" return_home="$3" fault_mode="$4"
  set_pose_fault_mode "$fault_mode" "$d" \
    || die "$label: pose relay did not confirm $fault_mode mode"
  printf 'frozen_start_sim_s=%s\n' "$(current_sim_s "$d/observations.jsonl")" >"$d/fault-injection.txt"
  printf 'mode=%s start=%s\n' "$fault_mode" "$(date -Is)" >>"$d/mode-timeline.txt"
  wait_for_state NAVIGATION_UNCERTAIN "$d" 45 \
    || die "$label: independent pose mismatch did not latch navigation lockout"
  printf 'lockout_sim_s=%s\n' "$(current_sim_s "$d/observations.jsonl")" >>"$d/fault-injection.txt"
  printf 'lockout=%s\n' "$(date -Is)" | tee -a "$d/mode-timeline.txt"
  local goals_locked goals_after
  goals_locked="$(rg -c 'Frontier selected x=' "$d/launch.log" || true)"
  goals_locked="${goals_locked:-0}"
  local frozen_start frozen_end
  frozen_start="$(current_sim_s "$d/observations.jsonl")"
  frozen_end="$frozen_start"
  # Keep the invalid but freshly-stamped stream alive for enough simulated
  # time to establish timestamp advancement and post-cancel ground truth.
  # The bound is wall-clock limited and does not relax any detector threshold.
  for _ in {1..120}; do
    sleep .1
    frozen_end="$(current_sim_s "$d/observations.jsonl")"
    python3 - "$frozen_start" "$frozen_end" <<'PY' && break
import sys
raise SystemExit(0 if float(sys.argv[2])-float(sys.argv[1]) >= 1.25 else 1)
PY
  done
  python3 - "$frozen_start" "$frozen_end" <<'PY' \
    || die "$label: simulation did not advance 1.25 s under the frozen-pose fault"
import sys
raise SystemExit(0 if float(sys.argv[2])-float(sys.argv[1]) >= 1.25 else 1)
PY
  goals_after="$(rg -c 'Frontier selected x=' "$d/launch.log" || true)"
  goals_after="${goals_after:-0}"
  printf 'frontier_goals_at_lockout=%s\nfrontier_goals_after_lockout_window=%s\n' \
    "$goals_locked" "$goals_after" >>"$d/mode-timeline.txt"
  (( goals_after == goals_locked )) \
    || die "$label: a new frontier goal was sent while pose-integrity lockout was latched"
  printf 'frozen_end_sim_s=%s\n' "$frozen_end" >>"$d/fault-injection.txt"
  if [[ "$return_home" == true ]]; then
    python3 "$ROOT/scripts/pr1_cancel_validation_probe.py" request-return \
      >"$d/home-request.json" || die "$label: could not request HOME while locked out"
  fi
  set_pose_fault_mode pass "$d" || die "$label: could not confirm restoration of valid pose"
  printf 'mode=pass restored=%s\n' "$(date -Is)" | tee -a "$d/mode-timeline.txt"
  local recovered=false
  for _ in {1..60}; do
    local response
    response="$(timeout 4s ros2 service call /writer/ack_pose_integrity_recovery \
      std_srvs/srv/Trigger '{}' 2>&1 || true)"
    if [[ "$response" == *'success=True'* ]]; then
      printf '%s\n' "$response" >"$d/recovery-ack.txt"
      recovered=true
      break
    fi
    printf '%s\n' "$response" >"$d/recovery-ack-last-attempt.txt"
    sleep .3
  done
  [[ "$recovered" == true ]] || die "$label: explicit recovery criteria/acknowledgment did not become ready"
  if [[ "$return_home" == true ]]; then
    wait_for_state HOME "$d" 180 || die "$label: HOME did not complete after acknowledged recovery"
  fi
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
integrity=log.count("Navigation locked out; no further goals will be sent: pose integrity could not be established")
cancel_requests=log.count("Active Writer goal cancellation requested:")
result={"scenario":name,"fault_modes":["stale","delayed","missing"] if modes=="all" else [modes],
        "localization_cancel_events":cancel,"expected_terminal_state":expected,
        "pose_integrity_lockouts":integrity,"cancel_requests":cancel_requests,
        "home_observed":("Explorer state HOME" in log or "Writer is back at the entrance" in log),
        "stuck_observed":"Explorer state STUCK" in log,
        "nav_goal_count":log.count("Received a goal, begin computing control effort"),
        "simulation_only":True,"result":"PASS" if
        ((cancel + integrity >= 3) if modes=="all" else True) and (
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
  stop_launch "$LAUNCH_PID"
  stop_process_group "$POSE_PID"
  stop_process_group "$BAG_PID"
  stop_process_group "$ADAPTER_PID"
  LAUNCH_PID= ADAPTER_PID= POSE_PID= BAG_PID= SCAN_PID=
  set -e
}

run_phase stale-recovery all true HOME
run_phase fresh-stamped-frozen-translation frozen-translation true HOME
run_phase fresh-stamped-frozen-rotation frozen-rotation true HOME

python3 "$ROOT/scripts/analyze_phase2_pose_fault_evidence.py" \
  "$RUN" --writer-config "$ROOT/config/nav2/writer.yaml" \
  --world "$ROOT/src/living_map_sim/worlds/mine.sdf" \
  --model "$ROOT/src/living_map_sim/models/writer/model.sdf" \
  --output "$RUN/pose-fault-audit.json" >"$RUN/pose-fault-audit.log" 2>&1 \
  || die "pose fault ground-truth/stream audit was inconclusive"
printf 'complete=true\nfinished=%s\n' "$(date -Is)" >>"$RUN/manifest.txt"
echo "Pose-fault validation evidence: $RUN"
