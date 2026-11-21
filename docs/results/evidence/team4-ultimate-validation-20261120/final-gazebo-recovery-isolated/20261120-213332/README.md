# Final isolated STUCK/watchdog regression run

- Branch/source HEAD: `phase2/reliability-optimization-20261120` / `f1c05a792c9382c936c5a8d86166b07a9b076bac`
- ROS: Jazzy; base domain 190; per-case domains 190–192 and unique Gazebo partitions.
- Result: all three bounded STUCK scenarios passed; see `results.txt` and each case's `timeline.txt` and `command-analysis.txt`.
- Build: 12 packages passed (`build.log`); focused tests: 165 passed (`pytest.log`).
- Watchdog readiness: `cases/watchdog-immobility/readiness/` captures map, scan, odometry, lifecycle, service enable, and domain/partition.
- Full suite, ROS action fault injection and refreshed stale-input/HOME check: `final-checks/`.
- The independent continuous boundary audit, raw ground-truth poses and cancel residual movement are in `final-checks/stale-evidence/stale-inputs-20261120-214701/`.
- The earlier timeout and its raw evidence remain in the sibling `stuck-regression-rerun/20261120-195818/` directory; it is not a passing run.

`build/`, `install/`, `log/`, temporary HOME and runtime ROS logs are retained locally but ignored by Git. Scenario bags and raw logs are preserved.
