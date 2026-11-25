# Course-paper experiment data

Simulation data and metrics behind the results of the Project of Mechatronics paper:
10 autonomous mapping runs and 3 navigation sessions (45 waypoint goals), recorded on
2026-10-10. All data come from simulation.

## Setup

| Item | Value |
|---|---|
| Code | the code on master (comment changes since) |
| Tests | `colcon test`: 15 tests, 0 errors, 0 failures |
| Machine | AMD Ryzen 5 5600H (6 cores, 12 threads), 15.0 GiB RAM, NVIDIA GeForce RTX 3050 Laptop GPU 4 GB |
| OS | Ubuntu 24.04.5 LTS, kernel 7.0.0-34-generic |
| Software | ROS 2 Jazzy (rclpy 7.1.12), Gazebo Harmonic (gz-sim 8.15.0), Nav2 1.3.13, slam_toolbox 2.8.5, ros_gz 1.0.24, rosbag2 0.26.11 (mcap) |
| Simulation | headless, robot spawned at the origin (yaw 0), DART physics at 1 kHz, real-time factor target 1.0 |

## Runs

| Set | Runs | Outcome |
|---|---|---|
| Pilot (not counted) | `pilot/mapping`, `pilot/navigation` (1 loop) | both completed; used to validate the tooling |
| Mapping | `mapping/run_01` to `run_10` | 9 returned to the start; **run_04 hit the 600 s cap** (robot failure, see notes) |
| Navigation | `navigation/session_1` to `session_3`, 3 loops each, then one global-localizer trial | 45/45 goals reached, 3/3 localizer trials converged |
| Not counted | `mapping/infra_run_09_*`, `navigation/infra_session_1_*`, `navigation/infra_session_3_*` (2) | infrastructure or tooling aborts, each relaunched (see notes) |

Real-time factor (sim time / wall time): mapping 0.992 ± 0.004 (0.981–0.995 over 10 runs);
navigation sessions 0.993, 0.992, 0.993. `runs.csv` lists every run with its wall-clock
start, end and outcome.

## What was recorded

- **Ground truth:** `paper/experiments/gt_logger.py` reads Gazebo's
  `/world/industrial_room/dynamic_pose/info` over gz-transport (about 60 Hz, simulation
  timestamps) and logs the pose of the model `industrial_amr`, whose frame is
  `base_footprint`. It also logs `/world/industrial_room/stats` for the real-time factor.
  ros_gz_bridge isn't used here because its Pose_V → TFMessage conversion drops the
  entity names and stamps.
- **Robot side:** a rosbag (mcap) of `/clock`, `/map`, `/tf`, `/tf_static`, `/rosout`,
  `/odom`, `/amcl_pose`, the `navigate_to_pose` action status and feedback, and
  `/collision_monitor_state`. Receive times are wall clock; `/clock` maps them to
  simulation time.
- The robot stack was launched exactly as described in the top-level README.

## Contents

```
runs.csv                     every run, counted or not, with outcome
pilot/{mapping,navigation}/  shakedown runs (not counted)
mapping/run_01..10/          mapping runs
navigation/session_1..3/     navigation sessions (3 loops each)
results/                     metrics: mapping_runs.csv, mapping_summary.json, mapping_curves.csv,
                             nav_goals.csv, nav_summary.json, figure metadata
screenshots/                 raw Gazebo screenshots for Fig. 1
```

Each run folder holds `gt_pose.csv`, `world_stats.csv`, `launch.log` and the other process
logs, `events_live.csv`, and the files extracted from the bag by `extract_bag.py`:
`clock.csv`, `est_pose.csv`, `amcl_pose.csv`, `nav_status.csv`, `nav_feedback.csv`,
`cm_state.csv`, `events.csv` and `maps.npz`. Mapping runs also have the saved
`map.pgm/.yaml`. The raw bags (422 MB) are not in the repository and are available on
request; the extracted files are enough to recompute every metric, figure and table.

## Reproduce

Recompute the metrics from the files in this folder:

```bash
python3 paper/experiments/metrics_mapping.py
python3 paper/experiments/metrics_navigation.py
```

Record a new set of runs (workspace built and sourced):

```bash
python3 paper/experiments/run_experiments.py --data-dir <new_dir> mapping --runs 10
python3 paper/experiments/run_experiments.py --data-dir <new_dir> navigation --sessions 3 --loops 3
python3 paper/experiments/extract_bag.py <new_dir>/mapping/run_* <new_dir>/navigation/session_*
```

## Notes

- **run_04 (robot failure, counted).** Exploration finished normally (65 s, final map
  79.25 m²). During the return to start, at sim time 83.3 s, the ground-truth pose jumped by
  about 0.10 m and 0.48 rad in under a second next to the small rotated crate near the south
  wall. The robot then stayed at (1.12, −3.34, −1.04 rad) for the rest of the run, where its
  ground-truth footprint overlaps the crate by 1.2 cm. The collision monitor was in its
  "approach" state from 69 s on (8 activations), and the spin, back-up and wait recoveries
  did not move the robot. Nav2 aborted the return goal, the explorer does not retry it, and
  the run ended at the 600 s cap. The cause was not isolated (candidates: an in-place turn in
  the 0.28 m gap between the crate and the wall, or a physics contact effect). Return and
  total times are reported over the 9 runs that returned.
- **run_03: keepout filter inactive (found afterwards).** A later audit of the lifecycle logs
  (`research/rotation_center/analysis/compare.py`) showed that the reply to
  `/filter_mask_server/change_state` was lost, so `lifecycle_manager_costmap_filters` never
  activated and the charger keepout filter was off for this run. The robot stayed 1.31 m from
  the keepout zone, so no reported result changes.
- **Closest approach.** Over the other 9 mapping runs, the minimum ground-truth footprint
  clearance to any obstacle was 1.7–20 cm (`results/mapping_runs.csv`, `clear_obstacle`);
  over the navigation missions it was 20 cm. The keepout zone was never entered: minimum
  footprint clearance 0.63 m in mapping and 0.076 m in navigation (near WP1); charger plate
  ≥ 0.49 m.
- **Runs not counted.**
  - infra_run_09: a lost DDS lifecycle reply (`failed to send response to
    /controller_server/change_state (timeout)`) meant Nav2 never became active within 120 s.
    It was relaunched as run_09.
  - infra_session_1: the same kind of hang in localization. `map_server` and
    `filter_mask_server` didn't send their `change_state` responses, so the lifecycle manager
    stopped at "Configuring map_server". It was relaunched as session_1.
  - infra_session_3 (×2, tooling): the stack came up, but the orchestrator's `/rosout`
    subscriber (then volatile) missed the early "Managed nodes are active" message, and the
    runs were aborted at 120 s. The subscriber is now transient-local; the third attempt ran
    normally.
- **Ground truth vs Nav2 success.** Nav2's goal checker judges the AMCL estimate, so the true
  final error is the tolerance plus the localization error: 27 of the 45 reached goals are
  within 0.20 m and 0.15 rad of the waypoint against ground truth (`results/nav_summary.json`).
- **AMCL offset.** The mean AMCL − ground-truth difference over the missions is +2.5 to
  +3.0 cm in x and +0.3 to +0.8 cm in y per session; it is included in the navigation errors.
