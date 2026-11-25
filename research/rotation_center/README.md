# Rotation-centre study: protocol (pre-registered)

Written on 2026-11-16, before any run of this study. Each run stores the sha256 of this file
in its `provenance.json`. Changes after the first counted run go to **Deviations** at the end,
with a date and a reason. The results go in `REPORT.md`, not here.

## Question

The robot is a differential drive. It turns about the midpoint of its drive axle, which is
0.20 m behind the chassis centre (`wheel_x_offset = -0.20` in `properties.xacro`). The
software treats the chassis centre as the turning point:

- `base_link` and `base_footprint` sit at the chassis centre.
- The Nav2 footprint is ±0.30 × ±0.25 m around that point.
- Several components model the robot as turning about `base_footprint`: MPPI's `DiffDrive`
  motion model, the collision monitor's motion simulation, the Spin behaviour's collision
  check, and AMCL's odometry model.
- Gazebo's DiffDrive odometry (`gz::math::DiffDriveOdometry`) integrates the axle's motion
  and publishes the result as `base_footprint`. It has no offset parameter.

Consequence (derived, not yet measured): after a pure turn of Δθ, the true `base_footprint`
has moved 0.40·|sin(Δθ/2)| m while odometry reports no motion. The course-paper evaluation
(`paper/data`, 2026-10-10) also found two problems:

- Only 27 of 45 goals that Nav2 reported as reached were within the 0.20 m / 0.15 rad
  tolerance against ground truth.
- One of 10 mapping runs got stuck against a crate.

**Does the conventional fix improve true goal accuracy, localization and robustness?** The
fix puts `base_link`/`base_footprint` on the axle midpoint and makes the footprint
asymmetric; the physical robot stays the same. The answer is reported whatever it is,
including a null or negative result.

## Conditions

| | Frame (`base_footprint`) | Nav2 footprint (x; y) | Code |
|---|---|---|---|
| **A0** | chassis centre | [−0.30, 0.30]; [−0.25, 0.25] | `paper/data`, 2026-10-10, commit `0bebd95` (reference only) |
| **A** | chassis centre | [−0.30, 0.30]; [−0.25, 0.25] | master `0bebd95` + cosmetic uncommitted diffs, re-run now |
| **B** | drive-axle midpoint | [−0.10, 0.50]; [−0.25, 0.25] | branch `rotation-center-study` |

**What B changes.** Every link, visual, collision and inertial origin moves +0.20 m in x
relative to `base_footprint`, which is verified by `analysis/check_urdf_shift.py`. The
physical body, mass distribution, wheels and sensors stay identical relative to each other.
The footprint is the same 0.60 × 0.50 m rectangle, expressed about the axle.

**What B does not change:** world, spawn pose (`base_footprint` at the world origin, so
map = world), saved map, keepout mask, waypoints, and every Nav2, SLAM and explorer
parameter, including the 0.55 m inflation radius.

**Side effects that come with B and are not tuned away:**

- **Circumscribed radius 0.39 → 0.56 m.** This exceeds the 0.55 m inflation radius, so
  MPPI's cost critic warns and checks the full footprint everywhere. It stays correct but
  costs more CPU.
- **Inscribed radius 0.25 → 0.10 m.** The band the global planner treats as lethal narrows.
- **The local costmap's forward reach shrinks.** The 3 × 3 m window is centred on the axle,
  so it reaches 1.0 m beyond the front edge instead of 1.2 m.
- **The body ends up in a different place at each goal.** Goals are poses of
  `base_footprint`, so in B the body ends 0.20 m further along the goal heading than in A.
  Planning checked the body clearance at every waypoint: it is 0.46 m or more in both
  conditions.

## Hypotheses

- **H1 (mechanism).** During an in-place spin, in A the true `base_footprint` moves on a
  circle of radius 0.20 m, and odometry misses that motion: the odometry error follows
  0.40·|sin(Δθ/2)| m. In B both are near zero.
  - *Supported if:* in A the median over spins of the peak true displacement is within
    0.40 ± 0.04 m and the RMS deviation of the odometry error from the predicted curve is
    ≤ 0.03 m; in B the median peak displacement is ≤ 0.05 m.
- **H2 (goal accuracy, primary).** B has a higher share of goals within 0.20 m and 0.15 rad
  of the waypoint against ground truth, and a lower final position error.
  - *Tests:* Fisher's exact test on the within-tolerance counts out of all goals, and the
    Mann–Whitney U test on the final position error of goals Nav2 reported as reached. Both
    two-sided, α = 0.05.
  - Effect sizes come with 95% cluster-bootstrap confidence intervals: 10,000 resamples of
    whole sessions, seed 20261116. Goals within one session aren't independent.
- **H3 (localization).** B has a lower AMCL position RMSE against ground truth over the
  missions. *Test:* Mann–Whitney U on the per-session RMSE.
- **H4 (robustness).** B has fewer progress-checker failures, recoveries and
  collision-monitor interventions, and its minimum ground-truth body clearance is no lower.
  These are reported with tests where counts allow. Stuck runs are counted and reported
  only, because 15 runs can't test a rare event.
- **Secondary (mapping).** In B, exploration time, coverage and map error (distance from
  occupied cells to the true surfaces) are no worse than in A.

H2 is the primary test. H1, H3, H4 and the mapping comparison are reported with their tests
but treated as supporting evidence, without a multiple-comparison correction.

## Protocol

Same machine and software for A and B. The versions are recorded per run; ROS packages are
the same versions as in A0, and only the kernel differs (7.0.0-34 → 7.0.0-38). Runs are
headless, without RViz, at a real-time-factor target of 1.0, with `base_footprint` spawned
at the origin with yaw 0.

Per condition, in this order:

1. **Pilot** (not counted): one mapping run, one navigation session with 1 loop, one spin
   test.
2. **Spin test:** 3 sessions. `navigation.launch.py` with the saved map, then
   `paper/experiments/spin_test.py`:
   - NavigateToPose to (2.8, 0.5, 0) in the map frame. Its nearest obstacle is barrel 2,
     1.87 m away.
   - A 2 s pause, then Nav2 Spin +2π, a 2 s pause, then Spin −2π, then a 2 s pause.
   - That gives 6 spins per condition.
3. **Mapping:** 15 runs of `mapping.launch.py` (frontier exploration and return to start),
   with a 600 s simulation-time cap per run.
4. **Navigation:** 10 sessions. `navigation.launch.py` runs the 5-waypoint circuit
   (`waypoints.yaml`) 3 times per session, which gives 150 goals per condition. Each
   session ends with the existing global-localizer trial. The cap is 300 s of simulation
   time per loop.

Command (A; for B, `--condition B` and `--expect-axle-x 0.0`):

```bash
cd ~/industrial_amr_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash
E=paper/experiments/run_experiments.py; D=research/rotation_center/data/A
P=research/rotation_center/README.md; O="--data-dir $D --condition A --expect-axle-x -0.20 --prereg $P"
python3 $E $O pilot
python3 $E $O spin --sessions 3
python3 $E $O mapping --runs 15
python3 $E $O navigation --sessions 10 --loops 3
```

`--expect-axle-x` makes the orchestrator render the installed URDF before every run and
refuse to start if the drive axle isn't where the condition says. That way data can't be
recorded under the wrong label.

**Stop rules**

- **Infrastructure failures** don't count: Nav2 not active within 120 s, a lifecycle hang,
  or a stalled clock. The orchestrator renames the run `infra_*` and retries it, as in the
  course paper.
- **A robot failure in B's pilot** stops the study before any counted B run. Examples: the
  robot can't leave the spawn point, or it collides on the first goal. Such a failure is
  reported, and nothing is tuned to make B work.
- **Robot failures in counted runs** count as results: a stuck run, a goal Nav2 aborts, or
  a run that hits the cap.

## Metrics

Ground truth is the Gazebo model pose (`gt_logger.py`, about 60 Hz). Its frame is the
model frame, which is `base_footprint` in each condition.

**Spin test.** Each spin is the interval between its "spin k start" and "spin k done"
markers.

- **Peak true displacement:** max over the spin of |p_gt(t) − p_gt(t0)|.
- **Odometry error:** e(t) = |trans(odom(t0)⁻¹·odom(t)) − trans(gt(t0)⁻¹·gt(t))|.
  Both relative poses are in the robot's frame at spin start, so the drift accumulated
  between spawn and t0 cancels out.
- **Predicted error:** 0.40·|sin(Δθ_gt/2)| m in A and 0 in B, where Δθ_gt is the true
  heading change since t0.
- **AMCL error:** the same quantity with AMCL in place of odometry.

**Navigation, per goal.** The goal ends when Nav2 reports a terminal status. The waypoint
of goal i is (i mod 5) + 1.

- **Final position error:** |p_gt − p_waypoint|.
- **Final yaw error:** |wrap(yaw_gt − yaw_waypoint)|.
- **Within tolerance:** Nav2 reported success and both errors are within 0.20 m / 0.15 rad.
- **Time per goal and true path length.**
- **Physical-point errors (secondary):** the distance from the waypoint position to the axle
  midpoint and to the chassis centre. Both points come from the ground-truth pose and the
  wheel-joint x in the run's `robot_description.urdf`.

**Navigation, per session:**

- AMCL position and yaw RMSE against ground truth over the mission.
- Nav2 recoveries and collision-monitor interventions.
- Progress-checker failures.
- MPPI "Control loop missed its desired rate" warnings.
- Real-time factor.
- Minimum ground-truth body clearance to obstacles, the charger plate and the keepout zone.

**Mapping, per run:** the course-paper definitions in `paper/experiments/metrics_mapping.py`:
exploration, return and total time; known area; coverage of the 80 m² room; map error;
progress failures, goal timeouts and aborts; recoveries; minimum body clearance; and
whether the robot got back to the start. Body clearances use the footprint from each run's
own `nav2_params.yaml` snapshot.

## Layout

```
data/{A,B}/{pilot,spin,mapping,navigation}/...   one directory per run; runs.csv per condition
analysis/check_urdf_shift.py                     A vs B URDF: everything moves +0.20 m in x
analysis/compare.py                              metrics, statistics -> results/
analysis/figures.py                              figures -> figures/
results/                                         per-condition metrics, comparison.json
REPORT.md                                        method, results, discussion, limitations
```

## Deviations

All dated 2026-11-17, after the counted runs of both conditions had started. None changes a
hypothesis, a threshold, a test or a parameter of the robot or the world. The text above
this section is unchanged since before the first run. With this section's body reset to
"None so far.", the file hashes to `6d17f13c0904a3da91ec4e7e6215f4b8f1b4207dfe063f62b0b0b0444d8475d2`,
the sha256 recorded in every run's `provenance.json`.

1. **The orchestrator stopped after B navigation session 7.**
   - **What happened.** Its cleanup step matched `amcl` inside the name `appstreamcli`, a
     root process started by the daily apt job. It tried to SIGTERM it and got a
     PermissionError, so nothing was killed. A `finally: os._exit(0)` then hid the
     exception and exited with code 0.
   - **Fix.** Three changes in `paper/experiments/run_experiments.py`:
     - patterns match whole names only;
     - a refused kill is logged and skipped;
     - exceptions print a traceback and exit non-zero.
   - **Check.** On a live run, the new matcher selected exactly the same robot processes
     as the old one.
   - **Resumption.** The series resumed with `--first 8`. B navigation sessions 8–10 and
     the two replacement sessions (items 2 and 3) carry a different tooling hash in
     `provenance.json`. The change affects only process cleanup, not what a run does.
2. **B navigation session 1 was reclassified as an infrastructure failure and re-run.**
   - **Evidence.** The `/bt_navigator/get_state` reply was lost (the lifecycle hang in the
     stop rules), so `waypoint_runner` waited forever and never sent a goal. The robot
     didn't move in 921 s.
   - **Why it slipped through.** The orchestrator logged it as `cap` because the runner
     never exited.
   - **Action.** Renamed `infra_session_1_20261117T002301` and re-run as `session_1` after
     session 10.
3. **A navigation session 1 was reclassified as an infrastructure failure and re-run.**
   - **Evidence.** The `/costmap_filter_info_server/change_state` reply was lost, so
     `lifecycle_manager_costmap_filters` never activated. The keepout filter was off for the
     whole session.
   - **Why it slipped through.** The orchestrator only waits for the navigation and
     localization managers, so it logged the run as `completed`.
   - **Action.** Renamed `infra_session_1_20261116T224512` and re-run as `session_1` on
     `master` (`0bebd95`, `--expect-axle-x -0.20`) after the B series. Its URDF and Nav2
     parameter snapshots are byte-identical to the other A runs. This is the only A run
     made after B.
4. **A lifecycle audit was added to `analysis/compare.py`, and all runs were audited.**
   - **The rule.** A counted A or B run must have every lifecycle manager report "Managed
     nodes are active"; `compare.py` refuses to run otherwise. Items 2 and 3 were found
     this way.
   - **A navigation session 10.** The only lost reply was `/amcl/get_state`, after the
     mission, which kept the global-localizer trial from starting. The navigation part
     stays counted. Its localizer trial is reported as "not started", not as "did not
     converge".
   - **A0 (course-paper data) also has two such runs:**
     - `mapping/run_03` had the keepout filter off;
     - the uncounted `pilot/navigation` did too.
     A0 is reference only and was not changed; `comparison.json` lists both under
     `lifecycle_audit`.
5. **Analysis additions made after seeing A's data.** These are descriptive only and change
   no pre-registered test:
   - the split of the tolerance check into position and yaw;
   - the slip of the axle midpoint during spins;
   - the absolute spin rotation;
   - whether the localizer trial started;
   - the generated `summary_table.md`;
   - figure layout.
