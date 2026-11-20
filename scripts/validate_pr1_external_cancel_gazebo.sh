#!/usr/bin/env bash
set -Eeuo pipefail

# Runs one bounded end-to-end cancel/recovery scenario with an isolated ROS
# build and the explicitly supplied patched SLAM overlay. Existing evidence is
# never deleted or reused.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
EXPECTED_BRANCHES=(review/pr1-writer-stuck-detection phase2/reliability-optimization-20261120)
EXPECTED_BASE="0fdf3449a5df970f74dccce922aa650c4b687c55"
STAMP="$(date +%Y%m%d-%H%M%S)"
EVIDENCE_BASE="${LIVINGMAP_VALIDATION_ROOT:-$HOME/livingmap-pr1-evidence-20261120}"
RUN_ROOT="$EVIDENCE_BASE/external-cancel-$STAMP"
BUILD="$RUN_ROOT/build"
INSTALL="$RUN_ROOT/install"
LOGS="$RUN_ROOT/colcon-log"
RUNTIME="$RUN_ROOT/runtime"
HELPER="$ROOT/scripts/pr1_cancel_validation_probe.py"
SLAM_OVERLAY="${LIVING_MAP_SLAM_OVERLAY:-$HOME/livingmap-setup/writer-slam-5b-9qc5m2a6/overlay/install}"

die() { printf 'FAIL: %s\nEvidence: %s\n' "$*" "$RUN_ROOT" >&2; exit 1; }
need() { command -v "$1" >/dev/null 2>&1 || die "required command not found: $1"; }

[[ -d "$ROOT/.git" ]] || die "run from the LivingMap checkout"
cd "$ROOT"
CURRENT_BRANCH="$(git branch --show-current)"
branch_allowed=false
for expected_branch in "${EXPECTED_BRANCHES[@]}"; do
  [[ "$CURRENT_BRANCH" == "$expected_branch" ]] && branch_allowed=true
done
[[ "$branch_allowed" == true ]] || die "unexpected branch: $CURRENT_BRANCH"
git merge-base --is-ancestor "$EXPECTED_BASE" HEAD || die "expected review checkpoint is not an ancestor"
[[ -f /opt/ros/jazzy/setup.bash ]] || die "ROS 2 Jazzy is unavailable"
[[ -f "$SLAM_OVERLAY/local_setup.bash" ]] || die "validated SLAM overlay missing: $SLAM_OVERLAY"
for tool in colcon timeout setsid pgrep rg; do need "$tool"; done
if pgrep -f '^gz sim server$' >/dev/null 2>&1; then
  die "another Gazebo server is active; preserve it and retry in an idle session"
fi
mkdir -p "$EVIDENCE_BASE"
[[ ! -e "$RUN_ROOT" ]] || die "timestamped evidence directory already exists; refusing to reuse it"
mkdir "$RUN_ROOT"
mkdir -p "$BUILD" "$INSTALL" "$LOGS" "$RUNTIME" \
  "$RUN_ROOT/home" "$RUN_ROOT/ros-logs"

LAUNCH_PID=""
OBSERVER_PID=""
HOLD_PID=""
TF_PID=""
cleanup() {
  set +e
  if [[ -n "$OBSERVER_PID" ]]; then
    kill -INT "$OBSERVER_PID" 2>/dev/null
    for _ in {1..10}; do kill -0 "$OBSERVER_PID" 2>/dev/null || break; sleep 0.2; done
    kill -TERM "$OBSERVER_PID" 2>/dev/null
    wait "$OBSERVER_PID" 2>/dev/null
  fi
  if [[ -n "$HOLD_PID" ]]; then
    kill -INT "$HOLD_PID" 2>/dev/null
    for _ in {1..10}; do kill -0 "$HOLD_PID" 2>/dev/null || break; sleep 0.2; done
    kill -TERM "$HOLD_PID" 2>/dev/null
    wait "$HOLD_PID" 2>/dev/null
  fi
  if [[ -n "$TF_PID" ]]; then
    kill -INT "$TF_PID" 2>/dev/null
    for _ in {1..10}; do kill -0 "$TF_PID" 2>/dev/null || break; sleep 0.2; done
    kill -TERM "$TF_PID" 2>/dev/null
    wait "$TF_PID" 2>/dev/null
  fi
  if [[ -n "$LAUNCH_PID" ]]; then
    kill -INT -- "-$LAUNCH_PID" 2>/dev/null
    for _ in {1..25}; do kill -0 "$LAUNCH_PID" 2>/dev/null || break; sleep 1; done
    if kill -0 "$LAUNCH_PID" 2>/dev/null; then
      kill -TERM -- "-$LAUNCH_PID" 2>/dev/null
      sleep 2
      kill -KILL -- "-$LAUNCH_PID" 2>/dev/null
    fi
    wait "$LAUNCH_PID" 2>/dev/null
  fi
  LAUNCH_PID="" OBSERVER_PID="" HOLD_PID="" TF_PID=""
  set -e
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

set +u
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID="$((120 + $$ % 60))"
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export PYTHONDONTWRITEBYTECODE=1
export GALLIUM_DRIVER=d3d12
export MESA_D3D12_DEFAULT_ADAPTER_NAME="${MESA_D3D12_DEFAULT_ADAPTER_NAME:-NVIDIA}"
export GZ_SIM_RESOURCE_PATH="$ROOT/src/living_map_sim/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

{
  printf 'branch=%s\nbase=%s\nhead=%s\n' "$CURRENT_BRANCH" "$EXPECTED_BASE" "$(git rev-parse HEAD)"
  printf 'started=%s\nros_distro=%s\nros_domain_id=%s\nrun_root=%s\nslam_overlay=%s\n' \
    "$(date -Is)" "${ROS_DISTRO:-unknown}" "$ROS_DOMAIN_ID" "$RUN_ROOT" "$SLAM_OVERLAY"
} | tee "$RUN_ROOT/manifest.txt"

echo 'Building all project packages into an isolated validation prefix.'
timeout 600s colcon --log-base "$LOGS" build --base-paths src \
  --build-base "$BUILD" --install-base "$INSTALL" --merge-install \
  --executor sequential 2>&1 | tee "$RUN_ROOT/build.log"
set +u
source "$INSTALL/setup.bash"
source "$SLAM_OVERLAY/local_setup.bash"
set -u
resolved_slam="$(ros2 pkg prefix slam_toolbox)"
[[ "$(realpath -e "$resolved_slam")" == "$(realpath -e "$SLAM_OVERLAY")" ]] \
  || die "ROS did not resolve the requested patched SLAM overlay: $resolved_slam"
resolved_writer="$(ros2 pkg prefix living_map_writer)"
[[ "$(realpath -e "$resolved_writer")" == "$(realpath -e "$INSTALL")" ]] \
  || die "Writer package did not resolve from isolated build: $resolved_writer"
printf 'slam_prefix=%s\nwriter_prefix=%s\n' "$resolved_slam" "$resolved_writer" \
  | tee "$RUN_ROOT/package-prefixes.txt"

export HOME="$RUN_ROOT/home"
export ROS_LOG_DIR="$RUN_ROOT/ros-logs"
export LIVING_MAP_RUNTIME_DIR="$RUNTIME"
mkdir -p "$HOME" "$ROS_LOG_DIR" "$RUNTIME"

setsid ros2 launch living_map_bringup demo.launch.py \
  demo_fallback:=false auto_explore:=true speed:=normal rviz:=false \
  >"$RUN_ROOT/launch.log" 2>&1 &
LAUNCH_PID=$!
python3 "$HELPER" observe --output "$RUN_ROOT/observations.jsonl" \
  --launch-log "$RUN_ROOT/launch.log" --config "$ROOT/config/nav2/writer.yaml" \
  >"$RUN_ROOT/observer.log" 2>&1 &
OBSERVER_PID=$!
if [[ "${LIVINGMAP_CONTROLLED_SAME_ZONE_RETRY:-false}" == true ]]; then
  python3 "$HELPER" hold-after-cancel --launch-log "$RUN_ROOT/launch.log" \
    --paused-file "$RUN_ROOT/controlled-pause.json" \
    --resume-file "$RUN_ROOT/controlled-resume.request" \
    --transform-file "$RUN_ROOT/map-to-odom.json" \
    --cancel-first-frontier \
    >"$RUN_ROOT/hold-controller.log" 2>&1 &
  HOLD_PID=$!
fi

echo 'Waiting up to 150 s for an autonomous frontier goal and the patched SLAM process.'
selected=false
slam_verified=false
for _ in {1..150}; do
  rg -q 'Frontier selected x=' "$RUN_ROOT/launch.log" 2>/dev/null && selected=true || true
  if [[ "$slam_verified" == false ]]; then
    slam_pid="$(pgrep -n -f '[a]sync_slam_toolbox_node' 2>/dev/null || true)"
    if [[ -n "$slam_pid" && -r "/proc/$slam_pid/maps" ]]; then
      maps="$RUN_ROOT/slam-process-maps.txt"
      grep -E 'lib(async_slam_toolbox|toolbox_common|kartoSlamToolbox)' "/proc/$slam_pid/maps" >"$maps" || true
      if [[ $(grep -cF "$(realpath -e "$SLAM_OVERLAY")" "$maps" || true) -ge 3 ]]; then
        printf 'pid=%s\n' "$slam_pid" >"$RUN_ROOT/slam-process.txt"
        slam_verified=true
      fi
    fi
  fi
  if [[ "$selected" == true && "$slam_verified" == true ]]; then break; fi
  kill -0 "$LAUNCH_PID" 2>/dev/null || die "Gazebo launch exited before ready"
  sleep 1
done
[[ "$selected" == true ]] || die "no frontier goal appeared within 150 s"
[[ "$slam_verified" == true ]] || die "running SLAM process libraries did not resolve to the patched overlay"

if [[ "${LIVINGMAP_CONTROLLED_SAME_ZONE_RETRY:-false}" == true ]]; then
  echo 'Waiting for the preconnected probe to cancel the first frontier goal.'
  for _ in {1..180}; do
    [[ -s "$RUN_ROOT/cancel-response.json" ]] && break
    kill -0 "$HOLD_PID" 2>/dev/null || die "first-frontier cancellation controller exited early"
    sleep 0.25
  done
  [[ -s "$RUN_ROOT/cancel-response.json" ]] || die "first frontier was not canceled within 45 s"
  cp "$RUN_ROOT/cancel-response.json" "$RUN_ROOT/cancel-service.log"
else
echo 'Requesting cancellation of the active Nav2 goal.'
cancel_count=0
for _ in {1..8}; do
  cancel_output="$(timeout 12s python3 "$HELPER" cancel-all 2>&1)" || {
    printf '%s\n' "$cancel_output" >>"$RUN_ROOT/cancel-service.log"
    sleep 1
    continue
  }
  printf '%s\n' "$cancel_output" | tee -a "$RUN_ROOT/cancel-service.log"
  cancel_count="$(python3 -c 'import json,sys; print(json.loads(sys.argv[1])["goals_canceling"])' "$cancel_output")"
  (( cancel_count > 0 )) && break
  sleep 1
done
(( cancel_count > 0 )) || die "Nav2 did not report an active goal for cancellation"
fi

echo 'Waiting for cancellation evidence and the published retry deadline.'
cancel_observed=false
retry_deadline=""
for _ in {1..45}; do
  if rg -q 'frontier goal at .*cancelled externally \(no reason\)' "$RUN_ROOT/launch.log" 2>/dev/null; then
    cancel_observed=true
  fi
  if [[ -f "$RUN_ROOT/observations.jsonl" ]]; then
    retry_deadline="$(python3 - "$RUN_ROOT/observations.jsonl" <<'PY'
import json, sys
from pathlib import Path
for line in reversed(Path(sys.argv[1]).read_text().splitlines()):
    event=json.loads(line)
    if event.get('event') == 'summary':
        data=event.get('data', {})
        counts=data.get('counts', {})
        policy=data.get('frontier_policy', {})
        if counts.get('frontier_cancelled', 0) and policy.get('cancellation_only_zones', 0):
            print(policy.get('next_retry_at_s') or '')
            break
PY
    )"
  fi
  if [[ "$cancel_observed" == true && -n "$retry_deadline" ]]; then break; fi
  sleep 1
done
[[ "$cancel_observed" == true ]] || die "unexplained cancellation was not observed"
[[ -n "$retry_deadline" ]] || die "summary did not report cancellation-only cooldown deadline"

echo "Waiting for simulation time to pass retry deadline $retry_deadline."
if [[ "${LIVINGMAP_CONTROLLED_SAME_ZONE_RETRY:-false}" == true ]]; then
  echo 'Holding the unchanged Writer pose during the cancellation-only cooldown.'
  for _ in {1..40}; do
    [[ -f "$RUN_ROOT/controlled-pause.json" ]] && break
    kill -0 "$HOLD_PID" 2>/dev/null || die "controlled pause controller exited early"
    sleep 0.25
  done
  [[ -f "$RUN_ROOT/controlled-pause.json" ]] || die "Writer did not pause after external cancellation"
  [[ -s "$RUN_ROOT/map-to-odom.json" ]] || die "map-to-odom snapshot was not captured"
  lifecycle_inactive=false
  for _ in {1..8}; do
    lifecycle_state="$(timeout 10s ros2 lifecycle get /slam_toolbox 2>&1 || true)"
    printf '%s\n' "$lifecycle_state" | tee -a "$RUN_ROOT/slam-freeze.txt"
    if rg -qi 'inactive' <<<"$lifecycle_state"; then
      lifecycle_inactive=true
      break
    fi
    if rg -qi 'active' <<<"$lifecycle_state"; then
      timeout 12s ros2 lifecycle set /slam_toolbox deactivate \
        2>&1 | tee -a "$RUN_ROOT/slam-freeze.txt" || true
    fi
    sleep 1
  done
  [[ "$lifecycle_inactive" == true ]] || die "SLAM did not enter inactive state"
  read -r tf_x tf_y tf_z tf_yaw < <(python3 - "$RUN_ROOT/map-to-odom.json" <<'PY'
import json, sys
d=json.load(open(sys.argv[1]))
print(d['x'], d['y'], d['z'], d['yaw'])
PY
  )
  ros2 run tf2_ros static_transform_publisher --x "$tf_x" --y "$tf_y" \
    --z "$tf_z" --yaw "$tf_yaw" --frame-id map --child-frame-id writer/odom \
    >"$RUN_ROOT/static-map-to-odom.log" 2>&1 &
  TF_PID=$!
  sleep 1
fi
deadline_passed=false
for _ in {1..180}; do
  health="$(python3 - "$RUN_ROOT/observations.jsonl" <<'PY'
import json, sys
from pathlib import Path
for line in reversed(Path(sys.argv[1]).read_text().splitlines()):
    event=json.loads(line)
    if event.get('event') == 'health':
        print(json.dumps(event))
        break
PY
  )"
  if [[ -n "$health" ]]; then
    sim_time="$(python3 -c 'import json,sys; print(json.loads(sys.argv[1]).get("sim_s") or 0)' "$health")"
    if python3 - "$sim_time" "$retry_deadline" <<'PY'
import sys
raise SystemExit(0 if float(sys.argv[1]) >= float(sys.argv[2]) + 2.0 else 1)
PY
    then deadline_passed=true; break; fi
  fi
  sleep 1
done
[[ "$deadline_passed" == true ]] || die "simulation clock did not pass the cancellation retry deadline"
if [[ "${LIVINGMAP_CONTROLLED_SAME_ZONE_RETRY:-false}" == true ]]; then
  touch "$RUN_ROOT/controlled-resume.request"
  for _ in {1..40}; do
    rg -q 'controlled_resume_confirmed' "$RUN_ROOT/hold-controller.log" 2>/dev/null && break
    kill -0 "$HOLD_PID" 2>/dev/null || die "controlled resume controller exited early"
    sleep 0.25
  done
  rg -q 'controlled_resume_confirmed' "$RUN_ROOT/hold-controller.log" \
    || die "Writer did not confirm exploration resumption after cooldown"
fi

echo 'Allowing up to 20 additional simulation seconds for post-cooldown exploration.'
post_cooldown_selection=false
for _ in {1..75}; do
  selected_after_expiry="$(python3 - "$RUN_ROOT/observations.jsonl" "$retry_deadline" <<'PY'
import json, sys
from pathlib import Path
deadline=float(sys.argv[2])
events=[json.loads(line) for line in Path(sys.argv[1]).read_text().splitlines()]
print(int(any(e.get('event')=='frontier_selected' and e.get('sim_s') is not None
              and float(e['sim_s']) >= deadline for e in events)))
PY
  )"
  if [[ "$selected_after_expiry" == 1 ]]; then
    post_cooldown_selection=true
    break
  fi
  health="$(python3 - "$RUN_ROOT/observations.jsonl" <<'PY'
import json, sys
from pathlib import Path
for line in reversed(Path(sys.argv[1]).read_text().splitlines()):
    event=json.loads(line)
    if event.get('event')=='health':
        print(event.get('sim_s') or 0)
        break
PY
  )"
  if python3 - "$health" "$retry_deadline" <<'PY'
import sys
raise SystemExit(0 if float(sys.argv[1]) >= float(sys.argv[2]) + 20.0 else 1)
PY
  then break; fi
  sleep 1
done
printf 'post_cooldown_selection_observed=%s\n' "$post_cooldown_selection" \
  | tee "$RUN_ROOT/post-cooldown-window.txt"

echo 'Requesting safe HOME after the cancellation backoff elapsed.'
python3 "$HELPER" request-return | tee "$RUN_ROOT/home-request.json"
home=false
for _ in {1..180}; do
  rg -q 'Explorer state HOME|Writer is back at the entrance' "$RUN_ROOT/launch.log" 2>/dev/null && home=true || true
  [[ "$home" == true ]] && break
  kill -0 "$LAUNCH_PID" 2>/dev/null || die "Gazebo launch exited before HOME"
  sleep 1
done
[[ "$home" == true ]] || die "Writer did not publish HOME within the 180 s bound"
sleep 1

cleanup
trap - EXIT INT TERM

python3 - "$RUN_ROOT/observations.jsonl" "$RUN_ROOT/launch.log" \
  "$RUN_ROOT/results.json" <<'PY'
import json, math, re, sys
from pathlib import Path
events=[json.loads(line) for line in Path(sys.argv[1]).read_text().splitlines()]
log=Path(sys.argv[2]).read_text(errors='replace')
cancel=next((e for e in events if e.get('event')=='frontier_cancelled'), None)
if cancel is None:
    raise SystemExit('missing correlated cancellation/summary evidence')
summary=next((e for e in events if e.get('event')=='summary'
              and e.get('sim_s') is not None
              and e['sim_s'] >= cancel['sim_s']
              and e.get('data',{}).get('counts',{}).get('frontier_cancelled',0)
              and e.get('data',{}).get('frontier_policy',{}).get('cooling_zones',0)),None)
if summary is None:
    raise SystemExit('missing cancellation summary with an active cooldown')
policy=summary['data']['frontier_policy']
counts=summary['data']['counts']
baseline=next((e for e in reversed(events)
              if e.get('event')=='summary' and e.get('sim_s') is not None
              and e['sim_s'] <= cancel['sim_s']
              and e.get('data',{}).get('counts',{}).get('frontier_cancelled',0)
                  < counts.get('frontier_cancelled',0)),None)
before_counts={} if baseline is None else baseline['data'].get('counts',{})
before_policy={} if baseline is None else baseline['data'].get('frontier_policy',{})
deadline=policy.get('next_retry_at_s')
if not deadline or summary['sim_s'] is None:
    raise SystemExit('missing retry deadline or simulation timestamp')
delay=float(deadline)-float(summary['sim_s'])
if not 13.0 <= delay <= 17.0:
    raise SystemExit(f'cancellation cooldown was {delay:.2f}s, expected configured 15s')
if (counts.get('frontier_failed', 0) != before_counts.get('frontier_failed', 0)
        or policy.get('failure_zones', 0) != before_policy.get('failure_zones', 0)):
    raise SystemExit('external cancellation consumed the unreachable-goal failure budget')
if policy.get('cancellation_only_zones', 0) < 1 or policy.get('cooling_zones', 0) < 1:
    raise SystemExit('cancellation-only suppression was not active')
cx,cy=cancel['data']['x'],cancel['data']['y']
for event in events:
    if event.get('event')!='frontier_selected' or event.get('sim_s') is None:
        continue
    if float(summary['sim_s']) <= float(event['sim_s']) < float(deadline):
        if math.hypot(event['data']['x']-cx,event['data']['y']-cy) <= 1.5:
            raise SystemExit('cancelled spatial goal was selected again before cooldown expiry')
post=[e for e in events if e.get('event')=='frontier_selected'
      and e.get('sim_s') is not None and float(e['sim_s']) >= float(deadline)]
same_zone_retry=any(math.hypot(e['data']['x']-cx,e['data']['y']-cy) <= 1.5
                    for e in post)
if not post:
    raise SystemExit('no frontier goal was selected after the cancellation cooldown elapsed')
if __import__('os').environ.get('LIVINGMAP_CONTROLLED_SAME_ZONE_RETRY') == 'true' and not same_zone_retry:
    raise SystemExit('controlled scenario did not reselect the cancelled frontier zone')
final=next((e['data'] for e in reversed(events) if e.get('event')=='final'), {})
if final.get('state')!='HOME':
    raise SystemExit(f"final observer state was {final.get('state')!r}, not HOME")
if final.get('stuck') or re.search(r'WRITER STUCK:|Navigation watchdog cancelling',log):
    raise SystemExit('external cancellation caused STUCK or an unexpected watchdog cancellation')
if (final.get('boundary_crossed')
        or final.get('minimum_footprint_x', math.inf) < final.get('entrance_x',-1.0)
        or final.get('minimum_uncertain_footprint_x', math.inf) < final.get('entrance_x',-1.0)
        or final.get('minimum_swept_footprint_x', math.inf) < final.get('entrance_x',-1.0)
        or final.get('minimum_swept_uncertain_footprint_x', math.inf)
           < final.get('entrance_x',-1.0)):
    raise SystemExit('padded Writer footprint crossed the entrance boundary')
if final.get('odom_samples', 0) < 10:
    raise SystemExit('insufficient odometry samples to verify entrance clearance')
if final.get('odom_gap_count', 0) != 0:
    raise SystemExit('odometry gaps exceeded the 0.5 s simulation observation assumption')
if final.get('unmapped_motion_samples', 0) != 0:
    raise SystemExit('robot motion was observed without map-frame transform alignment')
result={"cancel_reason":cancel['data']['reason'],"cancel_sim_s":summary['sim_s'],
        "retry_deadline_s":deadline,"cooldown_s":delay,"counts":counts,
        "frontier_policy":policy,"final_state":final.get('state'),
        "post_cooldown_selection_observed":bool(post),
        "same_zone_retry_observed":same_zone_retry,
        "stuck":final.get('stuck'),"minimum_padded_footprint_x":final.get('minimum_footprint_x'),
        "minimum_uncertain_footprint_x":final.get('minimum_uncertain_footprint_x'),
        "minimum_swept_footprint_x":final.get('minimum_swept_footprint_x'),
        "minimum_swept_uncertain_footprint_x":final.get('minimum_swept_uncertain_footprint_x'),
        "position_uncertainty_xy":final.get('position_uncertainty_xy'),
        "entrance_x":final.get('entrance_x'),"odom_samples":final.get('odom_samples'),
        "boundary_transform_missing":final.get('boundary_transform_missing'),
        "unmapped_motion_samples":final.get('unmapped_motion_samples'),
        "odom_gap_count":final.get('odom_gap_count'),
        "maximum_odom_gap_s":final.get('maximum_odom_gap_s'),
        "path_length_m":final.get('path_length_m'),"boundary_crossed":final.get('boundary_crossed')}
result["controlled_same_zone_retry"] = (__import__('os').environ.get(
    'LIVINGMAP_CONTROLLED_SAME_ZONE_RETRY') == 'true' and same_zone_retry)
Path(sys.argv[3]).write_text(json.dumps(result,indent=2,sort_keys=True)+'\n')
print(json.dumps(result,indent=2,sort_keys=True))
PY

printf 'complete=true\nfinished=%s\n' "$(date -Is)" | tee -a "$RUN_ROOT/manifest.txt"
printf '\nPASS: external cancellation pacing, failure-budget separation, patched SLAM runtime, and safe HOME.\nEvidence: %s\n' "$RUN_ROOT"
