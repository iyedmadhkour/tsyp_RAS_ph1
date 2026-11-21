# Team 4 SLAM mapping investigation — 2026-11-16

**Status: PARTIAL.** Missing-wall causes were separated and an installed Karto grid-extent defect was reproduced. No SLAM settings or production mapping code changed. There is no claimed mapping improvement. Correcting the dependency remains outside this Team 4-only change.

## Method and repeated measurements

The unchanged Gazebo `mine.sdf` collision boxes define ground truth. Sample exposed, permitted interior wall faces every 5 cm: **2,030 reference samples**. Compare these with full-resolution occupied map cell centers adjacent to known free space. Report both predicted-wall distance to analytical wall surfaces (RMSE) and reciprocal reference-wall distance to mapped walls (completeness). The 10 cm band is descriptive, not an invented acceptance requirement.

Wall geometry metrics exclude the two visual victim neighborhoods, which generate LiDAR returns despite lacking physical collisions. The separate coverage metric retains them: **59,091 conservative footprint-safe reachable center samples**, or 147.7275 m², using the same padded robot geometry, entrance restriction and grid guard in every run. Observed coverage counts known cells; it is not a guarantee of correct free-space classification or complete wall visibility.

Maps use exact map-stamp `map <- writer/odom` registration. Fixed-spawn-anchor results are also reported; no fitted/ICP alignment is used. Physics-derived ideal 2D odometry limits conclusions about real hardware localization.

| Mission | Registered wall RMSE | Fixed-anchor RMSE | Reference walls within 10 cm | Observed safe-center coverage | Free-classified safe-center coverage |
| --- | ---: | ---: | ---: | ---: | ---: |
| Navigation baseline | 3.622 cm | 3.144 cm | 68.621% | 99.5126% | 99.3214% |
| Navigation after 1 | 2.411 cm | 2.519 cm | 96.995% | 99.4940% | 99.3112% |
| Navigation after 2 | 3.337 cm | 3.570 cm | 67.192% | 99.4991% | 99.3146% |
| Final validation | 2.988 cm | 2.860 cm | 97.241% | 99.5058% | 99.3062% |

SLAM settings were unchanged throughout. These are repeat observations; the earlier baseline used the original planner and the later missions used the validated fallback. Different routes prevent causal performance claims. The older 3.218 cm / 93.251% measurement remains a result for its original mission, not a universal guarantee. High floor coverage conceals large variation in wall completeness. Without a mapping change, an improved-after measurement is **not applicable**.

## Confirmed and supported causes

### LiDAR saw the missing outer walls

The baseline capture contains 486 exact-stamp scans. All 405 missing top-wall samples and all 146 missing right-wall samples have a raw endpoint within 5 cm. In the second post-change capture, all 401 missing top-wall and 177 missing right-wall samples do too. Those latter samples had nearby returns in 44–100 and 74–135 distinct captured scans, respectively, using a 10 cm neighborhood.

These gaps cannot primarily be explained by never observing the walls. Comparing against every occupied map cell, instead of only free-adjacent occupied cells, gives similar completeness: 68.768%, 96.995%, 67.389%. The metric's adjacency filter is not the cause.

Raw capture is approximately 3 Hz of the 10 Hz sensor stream. Only exact-stamp odometry-to-scan transforms are accepted; 1, 1 and 2 initial inexact samples were rejected. A nearby raw endpoint proves a return near a reference wall sample; it does not prove that SLAM accepted that scan. Full independent 3D sensor pose was not captured.

### The installed rasterizer excludes positive extrema

In the installed `karto_sdk/Karto.h`, `OccupancyGrid::ComputeDimensions` rounds bounding span divided by resolution to a cell count. `CoordinateConverter::WorldToGrid` also rounds to a cell index. A positive-maximum endpoint can therefore receive index equal to the count, outside the valid range. `RayTrace` only increments its occupied hit when the index is valid.

A native diagnostic invoked the actual installed `OccupancyGrid::CreateFromScans` with four repeated scans, five cardinal 1 m ranges, 5 cm resolution, `min_pass_through=2` and `occupancy_threshold=0.1`. It produced a 40 × 40 grid:

| Endpoint | Grid index | Actual result |
| --- | --- | --- |
| (-1, 0) | (0, 20) | Valid, occupied |
| (0, -1) | (20, 0) | Valid, occupied |
| (1, 0) | (40, 20) | Invalid, endpoint discarded |
| (0, 1) | (20, 40) | Invalid, endpoint discarded |

All four diagnostic assertions executed successfully. No installed header, library or mapper was modified. The initial diagnostic link command lacked the existing Boost serialization library; adding that link flag resolved it without installing anything.

The live evidence matches this defect: baseline and after-2 grids have 194 rows and 477 columns, with mostly free cells on their positive edges. After-1 has an extra row and column, with 460 occupied top-row cells and 194 occupied right-column cells. Small changes in scan matching and extrema can change which returns fit. This is strong evidence for the source of the positive-edge losses, supported by the native reproduction. Exact accepted/corrected internal scan histories were not recorded, so it is not an attribution of every missing map cell.

### Lower passage faces have a separate visibility limitation

Sixty-one `wall8` reference samples are missing in all three analyzed captures. In after-2, 58 of its 63 missing samples have no captured endpoint within 10 cm. The 0.65 m gap to the outer wall is narrower than the configured padded footprint's 0.66 m minimum width. This supports occlusion and viewpoint limitations there. Capture subsampling prevents claiming that no full-rate ray ever observed a face. Entrance or footprint safety was not weakened to gain access.

## Decision and required follow-up

The configured frames, topic, 720 samples, 10 Hz rate and 0.08–12 m limits agree with the simulated LiDAR. SLAM uses 5 cm cells and 0.20 m / 0.20 rad admission thresholds; its occupancy parameters match installed defaults. Loop closing is intentionally disabled for ideal odometry. Frame corrections are small and fixed-anchor results show the same missing-wall pattern.

Changing occupancy thresholds cannot restore an out-of-bounds endpoint. No evidence justifies random filtering, range, loop-closure or scan-matching tuning. The appropriate next step is an isolated upstream Karto extent correction that includes both extrema, with translated/rotated-bound tests and real scan replay, then repeated Gazebo comparison. It requires separately approved dependency work. Fake scan points, fabricated wall cells and system-library edits were not used.

Nav2's live obstacle layer and entrance mask remain active. Incomplete static walls are still a real limitation; these results do not establish physical-robot safety.

## Validation and preserved evidence

Final combined validation: **281 project tests passed, 0 failed, 0 skipped**; all three affected packages built. The initial build omitted `--merge-install` and stopped before building; the corrected command matched the existing workspace layout. A restricted test attempt stalled and was terminated; the same complete suite passed with normal local access. Both attempts are preserved.

The final NVIDIA D3D12 mission reached HOME in **190.333 s**, with seven successful outer navigation goals, no aborted goals, and zero sampled padded or published-footprint crossings. There were 575 published-footprint samples, 3,542 pose samples and 1,410 LiDAR messages. All 216 inspected boundary costmap samples were lethal. Sixteen primary planner errors were followed by successful plans; no controller-patience, collision-ahead or halt-result errors occurred. One Writer startup no-map warning remains recorded. Sampling is not continuous swept-volume proof.

Local evidence root: `/home/iyed/livingmap-setup/writer-five-xtouqie6/`. Task 5 contains the full report, `mapping-comparison.json`, `map-metrics/`, `scan-wall-metrics/`, the raw-scan analysis script and `native/` diagnostic source/build/results. `final-2026-11-16/` contains the final mission, measurements and test/build logs. Earlier reports, maps and logs remain intact and outside Git. The frozen methods are `writer-sequential-_vr7jms6/task2/accuracy.py` and `METHOD.md`, plus `exploration-coverage-is163ep4/coverage_reference.py` under the same setup directory.

Repository changes for this task are documentation only. Tasks 6 and 7 were not started.
