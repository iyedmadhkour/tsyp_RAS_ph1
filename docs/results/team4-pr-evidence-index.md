# Team 4 PR evidence index

Branch: `phase2/reliability-optimization-20261120`
Evidence scope: software tests, isolated ROS 2 build, and bounded Gazebo runs.
Gazebo results are simulation evidence, not physical robot acceptance.

## Completion and validation reports

- [Updated Phase 1/Phase 2 checklist](team4-final-phase1-phase2-checklist-20261120.md)
- [Final frozen-pose validation report](team4-frozen-pose-safety-validation-report-20261121.md)
- [Earlier frozen-pose failure report](team4-final-software-validation-20261120.md) — records the original entrance-plane crossing and its observed ground-truth measurements.

## Latest final run

Run directory: `evidence/team4-phase2-final-20261120/stale-pose-20261121-211121/`

- Full automated suite: [524-test log](evidence/team4-phase2-final-20261120/stale-pose-20261121-211121/pytest-full-final.log)
- Focused pose-integrity/navigation/STUCK/watchdog suite: [121-test log](evidence/team4-phase2-final-20261120/stale-pose-20261121-211121/pytest-focused-final.log)
- Isolated ROS 2 Jazzy build: [12-package build log](evidence/team4-phase2-final-20261120/stale-pose-20261121-211121/build.log)
- Corrected combined result: [final audit JSON](evidence/team4-phase2-final-20261120/stale-pose-20261121-211121/pose-fault-audit-final.json) and [audit log](evidence/team4-phase2-final-20261120/stale-pose-20261121-211121/pose-fault-audit-final.log)
- Scenario summaries: [stale/delayed/missing pose and recovery](evidence/team4-phase2-final-20261120/stale-pose-20261121-211121/stale-recovery/summary.json), [frozen translation](evidence/team4-phase2-final-20261120/stale-pose-20261121-211121/fresh-stamped-frozen-translation/summary.json), and [frozen rotation](evidence/team4-phase2-final-20261120/stale-pose-20261121-211121/fresh-stamped-frozen-rotation/summary.json)
- Independent ground-truth traces and MCAP bags remain beside each scenario summary in the same run directory.

## ROS package metadata audit

The seven changed package manifests are valid XML and each still exports
`ament_python` as its build type. Their explicit `buildtool_depend` lists are
empty. This checkout's `rosdep` data has no `ament_python` rule, while the
documented ROS Jazzy development setup installs `ros-dev-tools` and
`python3-colcon-common-extensions`; the installed Colcon environment exposes
the `ros_ament_python` extension. A bounded isolated build of all 12 workspace
packages completed successfully without installing software. Thus the
manifest changes work with the documented development setup; standalone
package/release dependency generation outside that setup was not validated.

## Earlier boundary-crossing failure — retained

Run directory: `evidence/team4-phase2-final-20261120/stale-pose-20261120-222259/`

- [Earlier failing ground-truth trace](evidence/team4-phase2-final-20261120/stale-pose-20261120-222259/fresh-stamped-freeze/ground-truth-pose.txt)
- [Earlier frozen-pose summary](evidence/team4-phase2-final-20261120/stale-pose-20261120-222259/fresh-stamped-freeze/summary.json)
- [Earlier raw MCAP bag](evidence/team4-phase2-final-20261120/stale-pose-20261120-222259/fresh-stamped-freeze/bag/bag_0.mcap)
- [Earlier combined audit](evidence/team4-phase2-final-20261120/stale-pose-20261120-222259/pose-fault-audit.json) and the associated [fault-injection record](evidence/team4-phase2-final-20261120/stale-pose-20261120-222259/fresh-stamped-freeze/fault-injection.txt)

The original report describes this run as a safety failure based on the
independent ground-truth crossing. Its machine-readable combined result is
`INCONCLUSIVE`; that label is retained as recorded and is not rewritten as
`FAIL` or `PASS`. The later passing run is a separate result and does not erase
this evidence.

## Preservation note

All 293 tracked evidence files and their Git history are preserved. This index
points reviewers to the final passing run and the earlier adverse result; it
does not replace or relabel intermediate attempts. The stated pose, scan,
timestamp, uncertainty, and speed limits are simulation assumptions. Team 1
hardware contracts, measured stopping behavior, and hardware-in-loop acceptance
remain outstanding.
