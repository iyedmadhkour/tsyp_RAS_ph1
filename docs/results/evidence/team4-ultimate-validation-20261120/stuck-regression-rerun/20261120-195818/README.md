# STUCK regression rerun

This bounded rerun was executed on `phase2/reliability-optimization-20261120`
at HEAD `d8038360847442b578946f3fe5f3d2bb9efff024` with ROS 2 Jazzy. The
script's `manifest.txt` records `0f79291f1bcddaa750462de7b7ac425eaa540b8e`
as its required ancestor/base commit, not the tested HEAD. The script built
and resolved the Writer and detector from this isolated run's build/install
overlays.

| Check | Result | Evidence |
| --- | --- | --- |
| Isolated ROS build | PASS, 12/12 packages | `build.log` |
| Focused Writer regressions | PASS, 165 tests | `pytest.log` |
| Normal navigation and HOME | PASS; no false STUCK | `cases/normal-home/` |
| Static forward immobilization | PASS; 455 forward command samples, zero odometry displacement, STUCK emitted | `cases/physical-immobility/` |
| Watchdog then continued immobilization | FAIL to reach test path; 210 s timeout, no state or STUCK events; Nav2 reported no map and out-of-bounds costmaps | `cases/watchdog-immobility/` |

The watchdog timeout is a setup/validation failure for this run. Earlier
watchdog-then-STUCK recorded passes remain preserved in the phase 2 evidence
directories referenced by the parent completion report. This rerun makes no
hardware stopping claim. No production code changed.

The `build/`, `install/`, `log/`, and per-scenario `home/` directories were
retained locally for reproducibility but are generated build/runtime state and
are excluded from this commit. The scenario logs, topic captures, bags,
configuration, and summary files are committed alongside this note.
