# Team 4 Task 5B — isolated SLAM grid-extent correction

**PASS for the reproduced grid-edge defect and the scenarios below.** Validated on 2026-11-16 in Ubuntu 24.04.5 WSL2, ROS 2 Jazzy and NVIDIA D3D12. This follows the [original investigation](team4-slam-investigation.md); it does not establish complete mapping, improved localization accuracy or physical-robot safety. Tasks 6 and 7 were not started.

## Dependency and correction

The installed package is `ros-jazzy-slam-toolbox` version `2.8.5-1noble.20260915.204151`. The isolated build uses official SLAM Toolbox **2.8.5**, revision `ec8f7635dea317b531c419f798f87d90a336f32e`. Its original Karto header is byte-identical to the installed header. See the [official source](https://github.com/SteveMacenski/slam_toolbox/blob/ec8f7635dea317b531c419f798f87d90a336f32e/lib/karto_sdk/include/karto_sdk/Karto.h).

In `lib/karto_sdk/include/karto_sdk/Karto.h`, `OccupancyGrid::ComputeDimensions` allocates `Round(span / resolution)` cells. `CoordinateConverter::WorldToGrid` also rounds: the positive maximum endpoint can receive index equal to that cell count, outside the grid. `RayTrace` then cannot register its occupied hit. A deterministic four-scan reproduction produces a 40 × 40 grid for cardinal 1 m returns at 5 cm resolution: negative endpoints survive, positive endpoints at index 40 do not.

The patch adds **one cell to each allocated dimension**, so indices zero through the maximum rounded index are valid. It leaves origin, resolution, coordinate conversion, scan admission and occupancy thresholds unchanged. This is inclusive endpoint allocation, not an arbitrary safety margin. The complete `slam_toolbox` package was rebuilt because the grid construction is used by the mapper through its shared libraries; changing an installed header alone would not fix a running mapper.

Pinned checksums:

| Artifact | SHA-256 |
| --- | --- |
| Official source archive | `c480942995404009cc833b32e10ca2538c1a322b1cda94cf2744c947271fe52a` |
| Original `Karto.h` | `277fd3ddbb4c655408660e7bab8ff39f49875af709bd7a83ff3d029190d44052` |
| Patched `Karto.h` | `4cc087b6f083f3d814cee1ecfc54d9765112a8bedc92968ef87477a59298f5e6` |

No system ROS file, Writer algorithm, Nav2 configuration, entrance protection, message/service interface or unrelated team source changed. `dpkg -V ros-jazzy-slam-toolbox` returned no differences.

## Reproduce and activate

Use an existing Ubuntu 24.04/Jazzy development installation with the project's dependencies and `colcon`, CMake, g++, GNU patch and Boost serialization development library already available. The script installs no packages and needs no sudo. From the checkout:

```bash
source /opt/ros/jazzy/setup.bash
overlay="$HOME/livingmap-slam-2.8.5-gridfix"
python3 scripts/build_slam_overlay.py "$overlay" --jobs 2
```

Use a new dedicated directory under Linux home, outside the checkout. Alternatively pass `--archive /path/to/slam_toolbox-ec8f7635.tar.gz` to reuse a cached archive with the exact checksum above. Without it, the script downloads the pinned archive once, with three attempts. It preserves failed download artifacts, verifies archive/header checksums, extracts with Python's data filter, applies the tracked patch with zero fuzz, and records every prepared source file's hash. An unknown nonempty directory or changed recorded source is rejected without cleanup. Reinvoking an intact prepared workspace performs an incremental build; no existing build directory is deleted. Dependency upgrades require a fresh review and workspace.

The completed workspace on this PC is `/home/iyed/livingmap-setup/writer-slam-5b-9qc5m2a6/overlay`. It was reused, not rebuilt unnecessarily. Its package build completed in 3 min 31 s. The build script automatically runs the native regression. These commands also run it explicitly:

```bash
./scripts/test_slam_grid_extents.sh /opt/ros/jazzy   # expected exit 1: eight reproduced failures
./scripts/test_slam_grid_extents.sh "$overlay/install"  # expected exit 0: eight passes
```

For simulation, set `overlay` to the workspace actually built, then either
source environments in this order in a dedicated terminal:

```bash
source scripts/_env.sh
source "$overlay/install/local_setup.bash"
export GALLIUM_DRIVER=d3d12
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
export ROS_DOMAIN_ID=173
export LIVING_MAP_RUNTIME_DIR="$(mktemp -d "$HOME/livingmap-slam-mission.XXXXXX")"
export ROS_LOG_DIR="$LIVING_MAP_RUNTIME_DIR/ros-logs"
ros2 pkg prefix slam_toolbox
ros2 launch living_map_bringup demo.launch.py rviz:=false auto_explore:=false demo_fallback:=false speed:=normal
```

Confirm the printed package prefix is the overlay's `install`, wait for readiness, then use the documented dashboard autonomous full-demo/return control. The validation observer used `/api/demo` with `{"action":"full_demo","outcome":"return"}` only after lifecycle, map, LiDAR and entrance-mask readiness. Stop with Ctrl-C and allow shutdown to complete. These settings apply only to that shell. A fresh shell sourced with just the normal project environment uses stock ROS again.

The standard demo entry point now accepts the overlay explicitly without
changing the default runtime:

```bash
export LIVING_MAP_SLAM_OVERLAY="$overlay/install"
./scripts/run_demo.sh --check-slam-overlay # verifies prefix, does not start Gazebo
./scripts/run_demo.sh --speed normal
```

The first command was verified against the existing validated overlay. The
second starts the ordinary bounded demo and can be used after readiness checks.
Do not set this variable automatically for physical hardware.

During both fresh missions, `/proc/<slam-pid>/exe` and `/proc/<slam-pid>/maps` confirmed that `async_slam_toolbox_node`, `libasync_slam_toolbox.so`, `libtoolbox_common.so` and `libkartoSlamToolbox.so` all came from the isolated install. Package-prefix selection alone was not treated as proof. The renderer check reports **D3D12 (NVIDIA GeForce RTX 4060 Laptop GPU), Accelerated: yes**, OpenGL 4.6.

## Tests and unchanged measurement method

The native test calls actual Karto `OccupancyGrid::CreateFromScans`, compiled and linked against the selected prefix. Its eight cases cover axis alignment, positive/negative translations, diagonal/arbitrary rotations, nonintegral span, and coarser/finer resolutions. Every case checks all five returned endpoints, including negative extrema, unchanged minimum origin, coordinate round trips within half a cell and a free scan center.

- Stock library: **8/8 fail as expected**, retaining only 2–3 of 5 endpoints per case.
- Patched library: **8/8 pass**, retaining all 5 endpoints per case; verified again after resuming.
- Full project suite with the overlay active: **281 passed, 0 failed, 0 skipped**, final run 7.88 s. This includes radio, beacon persistence/deployment, battery reserve/odometry, STUCK/DESTROYED, HOME, entrance and navigation regressions. The eight SLAM native cases are separate from the 281 pytest cases.
- Only affected ROS package: `slam_toolbox`; isolated build passed. No LivingMap production package changed or required rebuilding.
- Final source-manifest/hash checks, Python/shell syntax and `git diff --check` passed.

Ground truth and thresholds were kept from Task 5: unchanged `mine.sdf` SHA-256 `b6b0038604cd6f9fa392a9ff40a73c2578ae0aa88b338173c669b0a75f25e74a`; **2,030 exposed interior wall-face samples**, spaced 5 cm; and **59,091 conservative footprint-safe reachable center cells**, representing 147.7275 m². Entrance-prohibited space and obstacles are excluded from the reachable reference. The two visual victim neighborhoods are excluded only from wall-geometry errors; their interiors remain in the floor denominator.

Predicted walls are occupied (>20) cell centers adjacent to free (0–20) cells, within the same world ROI. Wall RMSE is distance to analytical collision-box surfaces; completeness is the reverse nearest-neighbor distance from reference wall samples, within 10 cm. Registration uses exact map-stamp `map <- writer/odom` TF and the fixed world spawn offset, with no fitted/ICP alignment. Fixed-anchor RMSE is also retained below. The simulator supplies ideal physics odometry, so this is not an independent noisy-odometry localization benchmark.

Positive-edge completeness uses **608 unique samples** from that same frozen reference: the top inner face at world y = 4.825 and right inner face at x = 9.825. There are 416 top and 193 right samples, sharing one corner. The explicit face subset was fixed for this comparison, not chosen per map. Observed floor coverage counts known cells; free-classified, occupied-on-free and unknown cells are retained separately in the evidence. High observed coverage does not prove complete or correct walls.

## Completed missions and measurements

All six compared missions used the same mine, normal-speed documented autonomous/return procedure and existing final Writer/Nav2 behavior. The three stock-library missions are preserved Task 5 runs after the navigation fallback change. The first patched mission was recovered from the interrupted session; two fresh repeats were completed on resume. Different routes and timing prevent paired efficiency or universal accuracy claims.

| SLAM / run | Wall RMSE, TF / fixed (cm) | Walls within 10 cm | Positive edges within 10 cm | Observed floor | HOME time (s) | Successful / aborted outer goals |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Stock 1 | 2.411 / 2.519 | 96.995% | 100% | 99.4940% | 174.264 | 8 / 2 |
| Stock 2 | 3.337 / 3.570 | 67.192% | 4.934% | 99.4991% | 160.298 | 7 / 0 |
| Stock 3 | 2.988 / 2.860 | 97.241% | 100% | 99.5058% | 190.333 | 7 / 0 |
| Patched, saved 1 | 3.213 / 3.680 | 96.946% | 100% | 99.5160% | 264.494 | 8 / 1 |
| Patched, fresh 1 | 2.863 / 3.239 | 97.094% | 100% | 99.5008% | 210.327 | 8 / 0 |
| Patched, fresh 2 | 2.107 / 2.896 | 97.094% | 100% | 99.5025% | 210.382 | 8 / 0 |

The demonstrated improvement is endpoint retention and consistent positive-edge completeness: all 608 samples are within 10 cm in each completed patched mission. Mean overall wall completeness changed from 87.143% to 97.044%. This small repeated sample supports the specific fix, not a universal completeness guarantee. Mean TF-registered RMSE changed from 2.912 to 2.728 cm, while fixed-anchor RMSE increased from 2.983 to 3.272 cm: **no general localization/accuracy improvement is claimed**. Mean observed floor coverage was essentially unchanged, 99.4996% versus 99.5064%; final free-classified coverage is 99.3045%. Mean HOME time increased from 174.965 to 228.401 s. Navigation-speed improvement is not claimed.

All three completed patched missions reached HOME with **zero sampled padded or published-footprint entrance crossings**, across 12,753 pose samples and 2,064 published polygons. Minimum published x was -0.4932 m, inside the x >= -1.0 m boundary. The local/global entrance probe was lethal for all 1,086 samples from first mission telemetry onward. Two initial local-map probes before mission readiness were 99 and 0, then became 100 before movement was authorized. The readiness gate remained enabled. These discrete checks are not continuous swept-volume proof.

The saved patched mission had one aborted outer goal during a logged simulation-clock rewind/TF-buffer clear: the controller could not transform the robot pose, and the mission subsequently recovered and returned HOME. The initiator of the clock rewind was not established. Primary planner-failure counts for the three patched runs were 21, 2 and 8; zero controller-patience and halt-result errors occurred. Each fresh run had one transient collision-ahead warning and still completed all eight outer goals. Startup TF/no-map and idle rescue-robot costmap warnings remain preserved. No safety check was disabled and no navigation parameter was retuned for this mapping change.

Two additional preserved attempts were interrupted and are **not completed-mission passes**: `patched-2-partial-killed` has no terminal summary; `patched-2` lost its launch stack to SIGINT about 107 s into the mission, after which its observer waited to the 420 s limit. The initiator is not recorded. Its partial wall/positive-edge/floor measurements are 71.330% / 62.007% / 87.6039%; these are excluded from completed-run aggregates. They are not silently relabeled as successful missions or an established mapping/navigation defect.

## Review, preservation and limitations

The original WIP commit `e980c96` is retained. Its patch contained valid unified-diff blank context lines that Git flagged as trailing whitespace. The follow-up encodes the same change with a zero-context hunk, applied by GNU patch. Applying it to pristine source produced bytes identical to the already-built source and installed header; no rebuild was needed. The old build receipt and patch hash are retained, and its receipt was updated only after proving this equivalence. The old patch-created `.orig` backup remains outside Git. Historical WIP formatting warnings are preserved in history; the final patch passes whitespace checks.

The complete Task 5B tracked file set is:

- `patches/slam_toolbox-2.8.5-grid-extents.patch`
- `scripts/build_slam_overlay.py`
- `scripts/test_slam_grid_extents.sh`
- `tests/native/verify_slam_grid_extents.cpp`
- `docs/results/team4-slam-grid-fix.md`

Full local evidence is under `/home/iyed/livingmap-setup/writer-slam-5b-9qc5m2a6/`: source/archive manifests, build output, stock and patched native results, full pytest logs, process/library and renderer proofs, `comparison.json`, `metrics/`, `boundary-check-review.json`, `evaluate.py`, raw maps/scans/costmaps and every interrupted run. The frozen evaluators remain at `writer-sequential-_vr7jms6/task2/accuracy.py` / `METHOD.md` and `exploration-coverage-is163ep4/coverage_reference.py` beneath the same setup directory. Re-run `python .../writer-slam-5b-9qc5m2a6/evaluate.py` in the project environment to reproduce the table from preserved captures. Original stock runs remain under `writer-five-xtouqie6/task4/after-{1,2}/validation` and `writer-five-xtouqie6/final-2026-11-16/validation`.

Raw logs, cached sources, binaries, maps, build/install directories, runtime storage, virtual environments and credentials are excluded from Git. Publication verification and a repository bundle are retained locally separately after the final commit. Only the personal feature branch is authorized for a normal push; local and remote main and organizer upstream remain unchanged.

Remaining limits: the lower passage's occluded faces and sensor-visible victim interiors are not repaired by an extent fix; complete-mine exploration and real-hardware localization remain unproven. The overlay is opt-in and pinned to 2.8.5, not an upstream release or a system package replacement. Longer mission times, the saved run's clock rewind, transient navigation warnings and startup mask initialization remain visible for later work. Tasks 6 and 7 require a separate continuation; neither was started here.
