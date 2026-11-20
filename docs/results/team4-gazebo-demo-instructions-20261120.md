# Team 4 Reproducible Gazebo Demonstration Instructions

Date: 2026-11-20. These procedures are for isolated simulation only; never use
the fault adapter against a physical robot. They do not replace HIL acceptance.

## Reproduce the latest dedicated pose-fault run

From the repository root on branch `phase2/reliability-optimization-20261120`
in Ubuntu/WSL with ROS 2 Jazzy, Gazebo and the already-built local Karto overlay
available:

```bash
cd /home/iyed/LivingMap-Team4
LIVINGMAP_POSE_EVIDENCE_ROOT="$PWD/docs/results/evidence/team4-phase2-final-20261120" \
  scripts/validate_phase2_stale_pose_gazebo.sh
```

The script builds 12 ROS packages into an isolated `/tmp/team4-pose-*` overlay,
allocates its own ROS domain and Gazebo partition, checks map/sensor/lifecycle
readiness, records independent Gazebo model poses and ROS bags, and runs two
bounded phases:

1. stale, delayed and missing Writer odometry during an active goal; verifies
   cancellation, no goal dispatch while invalid, recovery, and HOME after the
   missing-pose case;
2. fresh-stamped frozen Writer pose while the simulated model remains free to
   move; verifies the STUCK outcome and independently audits continuous padded
   footprint containment.

**Expected result on the current checkpoint:** the first phase passes; the
second reaches STUCK but the independent footprint audit reports
`INCONCLUSIVE`/safety failure because the model crosses the configured entrance
plane. The top-level script therefore exits nonzero. This is an intentionally
preserved failure, not a pass condition. Do not rerun repeatedly until the
software response is changed and reviewed; one failed attempt with complete
evidence is already available at:

`docs/results/evidence/team4-phase2-final-20261120/stale-pose-20261120-222259/`

The first attempt at `.../stale-pose-20261120-221733/` is retained but not
accepted: its frozen interval marker began before the adapter confirmed the
fault mode. Earlier environment/setup attempts remain separate.

## Review the final evidence

```bash
cat docs/results/evidence/team4-phase2-final-20261120/stale-pose-20261120-222259/stale-recovery/summary.json
cat docs/results/evidence/team4-phase2-final-20261120/stale-pose-20261120-222259/fresh-stamped-freeze/summary.json
cat docs/results/evidence/team4-phase2-final-20261120/stale-pose-20261120-222259/pose-fault-audit.json
```

`ground-truth-pose.txt` is Gazebo's `/model/writer/pose`, independent of the
injected Writer odometry. `bag/bag_0.mcap` includes Writer odometry, command,
Gazebo world odometry, state and `/clock`. `launch.log` has action/watchdog/
STUCK events and timestamps. Fault intervals are recorded in
`fault-injection.txt` and `mode-timeline.txt`. The analyzer uses the unchanged
configured footprint/padding/entrance values, the 5 cm test uncertainty, the
model's 1.2 m/s speed cap, and a full padded-footprint circumradius to bound
inter-sample motion.

## Re-run software checks without repeating Gazebo

```bash
cd /home/iyed/LivingMap-Team4
set +u
source /opt/ros/jazzy/setup.bash
source /tmp/team4-final-pose-20261120-222259/install/setup.bash
set -u
COVERAGE_FILE=/tmp/team4-final-pose-20261120-222259/final-checks/.coverage \
  .venv/bin/python -m pytest -p no:cacheprovider -q tests
```

The recorded result is 506/506 passed. The same isolated build output is in the
pose evidence root's `build.log`; it records 12/12 packages finished.

## Prior STUCK regression demonstrations

The three accepted isolated cases are preserved at
`docs/results/evidence/team4-ultimate-validation-20261120/final-gazebo-recovery-isolated/20261120-213332/`:

- `normal-home`: navigation, turns, intentional stops and HOME without false
  STUCK;
- `forward-immobilization`: forward commands with no physical model movement,
  followed by STUCK;
- `watchdog-immobility`: watchdog cancellation/retry then independent STUCK.

These earlier successful cases do not override the later fresh-stamped frozen
localization failure.

## Cleanup and safety

The validation runner uses bounded timeouts and process groups. It stops only
its own launch, relay, observer, bag and Gazebo processes on exit. Before
restarting a run, check the manifest's ROS domain and Gazebo partition and
confirm prior session processes are gone. Preserve all failed evidence. Never
change footprint, entrance boundary, fault thresholds or evidence files to
manufacture a passing result.
