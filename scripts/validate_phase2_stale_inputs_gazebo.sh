#!/usr/bin/env bash
set -Eeuo pipefail

# Bounded integration scenario: pause the Gazebo sensor/odometry bridge, verify
# the active goal is canceled and HOME waits for fresh inputs, then resume it.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
EXPECTED_BRANCH="phase2/reliability-optimization-20261120"
STAMP="$(date +%Y%m%d-%H%M%S)"
EVIDENCE_BASE="${LIVINGMAP_ADVANCED_EVIDENCE_ROOT:-$ROOT/docs/results/evidence/team4-phase2-advanced-20261120}"
RUN_ROOT="$EVIDENCE_BASE/stale-inputs-$STAMP"
TMP_ROOT="${LIVINGMAP_ADVANCED_TMP_ROOT:-/tmp/team4-phase2-stale-$STAMP}"
SLAM_OVERLAY="${LIVING_MAP_SLAM_OVERLAY:-$HOME/livingmap-setup/writer-slam-5b-9qc5m2a6/overlay/install}"
HELPER="$ROOT/scripts/pr1_cancel_validation_probe.py"

die() { printf 'FAIL: %s\nEvidence: %s\n' "$*" "$RUN_ROOT" >&2; exit 1; }
need() { command -v "$1" >/dev/null 2>&1 || die "missing command: $1"; }
cd "$ROOT"
[[ "$(git branch --show-current)" == "$EXPECTED_BRANCH" ]] || die "unexpected branch"
[[ -f /opt/ros/jazzy/setup.bash ]] || die "ROS 2 Jazzy is unavailable"
[[ -f "$SLAM_OVERLAY/local_setup.bash" ]] || die "validated SLAM overlay missing: $SLAM_OVERLAY"
for tool in colcon timeout setsid pgrep rg gz; do need "$tool"; done
pgrep -f '^gz sim server$' >/dev/null 2>&1 && die "Gazebo is already active; preserving the existing process"
mkdir -p "$EVIDENCE_BASE"
[[ ! -e "$RUN_ROOT" ]] || die "refusing to reuse evidence path"
mkdir "$RUN_ROOT"
mkdir -p "$TMP_ROOT"/{build,install,logs}

LAUNCH_PID="" OBSERVER_PID="" SCAN_PID="" POSE_PID=""
cleanup() {
  set +e
  if [[ -n "$POSE_PID" ]]; then
    kill -INT -- "-$POSE_PID" 2>/dev/null
    for _ in {1..10}; do kill -0 "$POSE_PID" 2>/dev/null || break; sleep .2; done
    kill -TERM -- "-$POSE_PID" 2>/dev/null
    wait "$POSE_PID" 2>/dev/null
  fi
  if [[ -n "$SCAN_PID" ]]; then kill -CONT "$SCAN_PID" 2>/dev/null; fi
  if [[ -n "$OBSERVER_PID" ]]; then
    kill -INT "$OBSERVER_PID" 2>/dev/null
    sleep .5
    kill -TERM "$OBSERVER_PID" 2>/dev/null
    wait "$OBSERVER_PID" 2>/dev/null
  fi
  if [[ -n "$LAUNCH_PID" ]]; then
    kill -INT -- "-$LAUNCH_PID" 2>/dev/null
    for _ in {1..20}; do kill -0 "$LAUNCH_PID" 2>/dev/null || break; sleep 1; done
    if kill -0 "$LAUNCH_PID" 2>/dev/null; then
      kill -TERM -- "-$LAUNCH_PID" 2>/dev/null
      sleep 2
      kill -KILL -- "-$LAUNCH_PID" 2>/dev/null
    fi
    wait "$LAUNCH_PID" 2>/dev/null
  fi
  LAUNCH_PID="" OBSERVER_PID="" SCAN_PID="" POSE_PID=""
  set -e
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

set +u
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID="$((150 + $$ % 40))"
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export PYTHONDONTWRITEBYTECODE=1
export GALLIUM_DRIVER=d3d12
export MESA_D3D12_DEFAULT_ADAPTER_NAME="${MESA_D3D12_DEFAULT_ADAPTER_NAME:-NVIDIA}"
export GZ_SIM_RESOURCE_PATH="$ROOT/src/living_map_sim/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

{
  printf 'branch=%s\nhead=%s\nstarted=%s\nros_distro=%s\nseed=none (deterministic fault timing)\n' \
    "$EXPECTED_BRANCH" "$(git rev-parse HEAD)" "$(date -Is)" "${ROS_DISTRO:-unknown}"
  printf 'simulation_assumptions=0.5s pose age; 0.35s scan age; 0.5s costmap timeout; 10Hz scan; bridge paused by SIGSTOP\n'
  printf 'ground_truth_topic=/model/writer/pose; source=Gazebo model pose publisher; frame=world\n'
} | tee "$RUN_ROOT/manifest.txt"

timeout 600s colcon --log-base "$TMP_ROOT/logs" build --base-paths src \
  --build-base "$TMP_ROOT/build" --install-base "$TMP_ROOT/install" \
  --merge-install --executor sequential 2>&1 | tee "$RUN_ROOT/build.log"
set +u
source "$TMP_ROOT/install/setup.bash"
source "$SLAM_OVERLAY/local_setup.bash"
set -u
writer_prefix="$(realpath -e "$(ros2 pkg prefix living_map_writer)")"
[[ "$writer_prefix" == "$(realpath -e "$TMP_ROOT/install")" ]] \
  || die "Writer package did not resolve from isolated build: $writer_prefix"
export HOME="$TMP_ROOT/home" ROS_LOG_DIR="$TMP_ROOT/ros-logs"
mkdir -p "$HOME" "$ROS_LOG_DIR"

setsid ros2 launch living_map_bringup demo.launch.py \
  demo_fallback:=false auto_explore:=true speed:=normal rviz:=false \
  >"$RUN_ROOT/launch.log" 2>&1 &
LAUNCH_PID=$!
python3 "$HELPER" observe --output "$RUN_ROOT/observations.jsonl" \
  --launch-log "$RUN_ROOT/launch.log" --config "$ROOT/config/nav2/writer.yaml" \
  >"$RUN_ROOT/observer.log" 2>&1 &
OBSERVER_PID=$!

# Gazebo's model pose publisher is independent of the paused ROS odometry/scan
# bridge. Preserve raw simulation-stamped ground truth for outage analysis.
pose_topic_ready=false
for _ in {1..90}; do
  if timeout 3s gz topic -l 2>/dev/null | rg -Fxq '/model/writer/pose'; then
    pose_topic_ready=true
    break
  fi
  kill -0 "$LAUNCH_PID" 2>/dev/null || die "Gazebo exited before ground-truth topic discovery"
  sleep 1
done
[[ "$pose_topic_ready" == true ]] || die "Gazebo Writer ground-truth pose topic unavailable"
setsid timeout 360s gz topic -e -t /model/writer/pose \
  >"$RUN_ROOT/ground-truth-pose.txt" 2>&1 &
POSE_PID=$!
sleep 1
kill -0 "$POSE_PID" 2>/dev/null || die "ground-truth pose recorder exited during startup"

selected_before=0
for _ in {1..150}; do
  selected_before="$(rg -c 'Frontier selected x=' "$RUN_ROOT/launch.log" 2>/dev/null || true)"
  if [[ "$selected_before" -ge 1 ]]; then break; fi
  kill -0 "$LAUNCH_PID" 2>/dev/null || die "Gazebo exited before frontier selection"
  sleep 1
done
[[ "$selected_before" -ge 1 ]] || die "no frontier selected within 150 s"
for _ in {1..10}; do
  while read -r candidate_pid; do
    [[ -n "$candidate_pid" ]] || continue
    candidate_pgid="$(ps -o pgid= -p "$candidate_pid" | tr -d ' ' || true)"
    if [[ "$candidate_pgid" == "$LAUNCH_PID" ]]; then
      SCAN_PID="$candidate_pid"
      break
    fi
  done < <(pgrep -f '[/]lib/living_map_writer/scan_frames' || true)
  [[ -n "$SCAN_PID" ]] && break
  sleep 1
done
[[ -n "$SCAN_PID" ]] || die "could not identify this launch's scan_frames process"
# Nav2 lifecycle activation can lag the first selector tick. Watch at 20 Hz,
# and pause the sensor/odom bridge immediately after the Writer controller
# receives a goal so this short Gazebo route cannot finish first.
active_goal=false
for _ in {1..1800}; do
  if rg -q '\[writer\.controller_server\].*Received a goal, begin computing control effort' \
      "$RUN_ROOT/launch.log"; then
    active_goal=true
    selected_before="$(rg -c 'Frontier selected x=' "$RUN_ROOT/launch.log" 2>/dev/null || true)"
    kill -STOP "$SCAN_PID"
paused_sim_s="$(python3 - "$RUN_ROOT/observations.jsonl" <<'PY'
import json, sys
from pathlib import Path
events=[json.loads(line) for line in Path(sys.argv[1]).read_text().splitlines()]
values=[event.get("sim_s") for event in events if event.get("sim_s") is not None]
print(values[-1] if values else "unknown")
PY
)"
    break
  fi
  kill -0 "$LAUNCH_PID" 2>/dev/null || die "Gazebo exited before an active Nav2 goal"
  sleep 0.05
done
[[ "$active_goal" == true ]] || die "Nav2 did not activate an accepted goal within 90 s"
printf 'scan_frames_pid=%s\npaused_at=%s\npaused_sim_s=%s\n' "$SCAN_PID" "$(date -Is)" "$paused_sim_s" \
  | tee "$RUN_ROOT/fault-injection.txt"

cancel_seen=false
for _ in {1..35}; do
  rg -q 'Active Writer goal cancellation requested: LiDAR data stale|Active Writer goal cancellation requested: localization unavailable' \
    "$RUN_ROOT/launch.log" && cancel_seen=true || true
  [[ "$cancel_seen" == true ]] && break
  sleep 1
done
[[ "$cancel_seen" == true ]] || die "stale input did not request active-goal cancellation"

# Request battery return while all pose/scan frames are absent. A safe HOME goal
# must wait for fresh pose and LiDAR; it may not assume the current goal stopped.
python3 "$HELPER" request-return | tee "$RUN_ROOT/home-request.json"
sleep 4
selected_during="$(rg -c 'Frontier selected x=' "$RUN_ROOT/launch.log" 2>/dev/null || true)"
[[ "$selected_during" -eq "$selected_before" ]] \
  || die "exploration selected a new frontier while localization/sensor data were absent"
kill -CONT "$SCAN_PID"
SCAN_PID=""
resumed_sim_s="$(python3 - "$RUN_ROOT/observations.jsonl" <<'PY'
import json, sys
from pathlib import Path
events=[json.loads(line) for line in Path(sys.argv[1]).read_text().splitlines()]
values=[event.get("sim_s") for event in events if event.get("sim_s") is not None]
print(values[-1] if values else "unknown")
PY
)"
printf 'resumed_at=%s\nresumed_sim_s=%s\n' "$(date -Is)" "$resumed_sim_s" \
  | tee -a "$RUN_ROOT/fault-injection.txt"

home=false
for _ in {1..180}; do
  if rg -q 'Explorer state HOME|Writer is back at the entrance' "$RUN_ROOT/launch.log"; then
    home=true
    break
  fi
  kill -0 "$LAUNCH_PID" 2>/dev/null || die "Gazebo exited before HOME"
  sleep 1
done
[[ "$home" == true ]] || die "fresh inputs resumed but HOME was not reached within 180 s"
sleep 1
cleanup
trap - EXIT INT TERM

python3 - "$RUN_ROOT/observations.jsonl" "$RUN_ROOT/launch.log" "$RUN_ROOT/results.json" <<'PY'
import json, sys
from pathlib import Path
events=[json.loads(line) for line in Path(sys.argv[1]).read_text().splitlines()]
log=Path(sys.argv[2]).read_text(errors="replace")
summaries=[e.get("data",{}) for e in events if e.get("event")=="summary"]
observed_final=next((e.get("data",{}) for e in reversed(events)
                     if e.get("event")=="final"),{})
counts=[s.get("counts",{}) for s in summaries]
scan_count=max((c.get("watchdog_scan_unavailable",0) for c in counts),default=0)
pose_count=max((c.get("watchdog_pose_unavailable",0) for c in counts),default=0)
home=any("Explorer state HOME" in line or "Writer is back at the entrance" in line
         for line in log.splitlines())
cancel=("Active Writer goal cancellation requested: LiDAR data stale" in log
        or "Active Writer goal cancellation requested: localization unavailable" in log)
if not cancel or scan_count + pose_count < 1:
    raise SystemExit("missing evidence that stale-input watchdog requested cancellation")
if not home:
    raise SystemExit("missing HOME completion evidence after input recovery")
result={"scenario":"stale_scan_and_localization_bridge_pause",
        "result":"PASS","stale_cancel_requested":cancel,
        "watchdog_scan_unavailable_count":scan_count,
        "watchdog_pose_unavailable_count":pose_count,
        "home_after_fresh_inputs":home,
        "swept_boundary_during_observed_intervals": {
            "status": "INCONCLUSIVE" if observed_final.get("odom_gap_count",0)
                or observed_final.get("unmapped_motion_samples",0) else "PASS",
            "minimum_uncertain_swept_x_m": observed_final.get(
                "minimum_swept_uncertain_footprint_x"),
            "entrance_x_m": observed_final.get("entrance_x"),
            "odom_gap_count_over_0_5s": observed_final.get("odom_gap_count"),
            "map_transform_missing_samples": observed_final.get(
                "boundary_transform_missing"),
            "unmapped_motion_samples": observed_final.get(
                "unmapped_motion_samples")},
        "physical_stop_guaranteed":False,
        "limitation":"Gazebo bridge was paused; motor stopping remains hardware-contract dependent."}
Path(sys.argv[3]).write_text(json.dumps(result,indent=2,sort_keys=True)+"\n")
print(json.dumps(result,indent=2,sort_keys=True))
PY
printf 'complete=true\nfinished=%s\n' "$(date -Is)" | tee -a "$RUN_ROOT/manifest.txt"
printf '\nPASS: stale bridge cancellation and HOME recovery. Evidence: %s\n' "$RUN_ROOT"
