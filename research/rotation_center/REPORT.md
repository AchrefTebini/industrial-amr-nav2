# Rotation-centre study: report

Study run 2026-11-16 to 2026-11-17. The protocol and hypotheses were fixed beforehand in
`README.md`; changes made after the first counted run are listed there under **Deviations**.
Every number below comes from `results/comparison.json` or the per-run CSVs in `results/`
(regenerate with `analysis/compare.py`). `results/summary_table.md` has all metrics side by
side.

## Summary

The robot turns about its drive-axle midpoint, 0.20 m behind the chassis centre, but the
software treated the chassis centre as the turning point. Moving `base_link` and
`base_footprint` onto the axle (condition B), with the same physical robot, was compared
with the original frame (condition A) over 6 spins, 15 mapping runs and 150 navigation goals
per condition.

| Hypothesis | Result |
|---|---|
| H1 mechanism | **Supported.** In A, the frame swings 0.406 m during a spin and odometry misses it, as predicted (0.40 m). In B the swing is gone. |
| H2 goal accuracy (primary) | **Split.** The share of goals within 0.20 m and 0.15 rad did not improve (71 % → 66 %, p = 0.46). The final position error did (median 0.117 → 0.103 m, p = 0.003). |
| H3 localization | **Supported.** AMCL position RMSE fell from 0.100 to 0.061 m (p = 0.0002). |
| H4 robustness | **Not supported.** No failures or recoveries in either condition, but B's body came closer to obstacles and entered the charger's keepout margin. |
| Mapping (secondary) | **No worse**, except a 0.18 m² smaller known area. All 30 runs returned to the start. |

In short, the fix does what it is meant to do for odometry, localization and the accuracy of
the controlled point. But with every other parameter unchanged, the body ends up closer to
obstacles. That's because the footprint now reaches 0.50 m ahead of the controlled point
instead of 0.30 m, the inflation radius (0.55 m) is below the new circumscribed radius
(0.56 m), and the waypoints now place the body 0.20 m further forward. Adopting the fix
would mean re-tuning the inflation radius and re-checking the waypoints. This study kept them
fixed on purpose.

## Method

- **Conditions.**
  - A: `master`, frame at the chassis centre, footprint x ∈ [−0.30, 0.30] m.
  - B: branch `rotation-center-study`, frame on the axle midpoint, footprint
    x ∈ [−0.10, 0.50] m.
  - `analysis/check_urdf_shift.py` confirms that B is A with every link, visual, collision
    and inertial origin moved +0.20 m in x relative to `base_footprint`, and nothing else
    changed.
  - World, map, keepout mask, waypoints and all other Nav2, SLAM and explorer parameters
    are identical.
  - A0 is the 2026-10-10 course-paper data (same code as A); it is a reference only.
- **Runs.** Headless Gazebo Harmonic, ROS 2 Jazzy, Nav2 1.3.13, real-time factor 0.993 (A)
  and 0.996 (B), median over navigation sessions.
  - Spin test: 3 sessions per condition. NavigateToPose to (2.8, 0.5, 0), then Nav2 Spin
    +2π and −2π, which gives 6 spins.
  - Mapping: 15 frontier-exploration runs per condition.
  - Navigation: 10 sessions × 3 loops of the 5-waypoint circuit, which gives 150 goals per
    condition.
  - A pilot preceded each series and isn't counted.
- **Ground truth.** The Gazebo model pose (about 60 Hz), whose frame is `base_footprint`.
  A goal counts as "within tolerance" when Nav2 reported success and the true final pose is
  within 0.20 m and 0.15 rad of the waypoint.
- **Statistics.** Pre-registered.
  - Fisher's exact test on the within-tolerance counts.
  - Mann–Whitney U on errors, RMSE and per-run counts. All tests two-sided.
  - 95 % cluster-bootstrap confidence intervals that resample whole sessions (10,000
    resamples, seed 20261116).
  - No correction for multiple comparisons; H2 is the primary test.
- **Run integrity.**
  - Infrastructure failures were renamed `infra_*` and re-run: 4 in A and 3 in B, including
    one in each condition that was reclassified after the audit described under
    Deviations.
  - Every counted run passes an audit: all Nav2 lifecycle managers reported active.

## Results

### H1: the mechanism (Fig. `figures/fig_spin.pdf`)

| Spin test (6 spins each) | A | B |
|---|---|---|
| Peak true displacement of `base_footprint` (median) | 0.4061 m | 0.0381 m |
| Odometry error, maximum (median) | 0.4061 m | 0.0381 m |
| RMS deviation of the odometry error from 0.40·\|sin(Δθ/2)\| (A) or 0 (B) | 0.0047 m | 0.0264 m |
| Maximum slip of the axle midpoint (median) | 0.0383 m | 0.0381 m |
| AMCL error, maximum (median) | 0.343 m | 0.068 m |

- **The prediction holds.** In A, `base_footprint` traces a circle of radius 0.20 m around
  the axle during each spin, while odometry reports no motion at all. The odometry error
  follows 0.40·|sin(Δθ/2)| m with an RMS deviation of 4.7 mm.
- **All three pre-registered H1 thresholds are met:**
  - A median peak within 0.40 ± 0.04 m;
  - A RMS deviation ≤ 0.03 m;
  - B median peak ≤ 0.05 m.
- **B's residual displacement is wheel slip.**
  - The axle midpoint itself slides up to 3.8 cm during a spin, in both conditions (0.0383
    and 0.0381 m), and ends 7 mm from where it started.
  - In B, `base_footprint` *is* the axle midpoint, so this slip is all that remains.
  - Odometry can't see the slip in either condition, because it integrates wheel rotation.
- **AMCL inherits the error.** During spins in A, AMCL is off by up to 0.343 m (median of the
  per-spin maxima), against 0.068 m in B.

### H2: goal accuracy (Fig. `figures/fig_goal_error.pdf`)

| 150 goals per condition | A | B | B − A [95 % CI] | p |
|---|---|---|---|---|
| Within 0.20 m and 0.15 rad | 106 (71 %) | 99 (66 %) | −0.047 [−0.127, +0.027] | 0.457 (Fisher) |
| Position within 0.20 m | 146 (97 %) | 150 (100 %) | | |
| Yaw within 0.15 rad | 110 (73 %) | 99 (66 %) | | |
| Final position error, median | 0.117 m | 0.103 m | mean −0.017 m [−0.024, −0.011] | 0.003 (MWU) |
| Final yaw error, median | 0.141 rad | 0.141 rad | | 0.565 (MWU) |

- **The primary count didn't move, and the reason is the yaw check.**
  - Nav2's goal checker stops when the AMCL yaw estimate is within 0.15 rad of the goal.
    So true final yaw errors pile up at the threshold: 102 of 150 goals in A and 105 of
    150 in B end between 0.13 and 0.17 rad.
  - Whether a goal lands inside the 0.15 rad tolerance therefore depends mostly on the sign
    of a 0.02 rad AMCL yaw error. The frame change doesn't affect that.
  - So the binary within-tolerance count measures the yaw check more than goal accuracy,
    and it can't show a position effect.
  - Position alone was within 0.20 m for 146 of 150 goals in A and 150 of 150 in B.
- **The position error fell at four of five waypoints.** Medians A → B:
  - WP1: 0.173 → 0.104 m;
  - WP3: 0.061 → 0.033 m;
  - WP4: 0.123 → 0.107 m;
  - WP5: 0.091 → 0.086 m;
  - WP2 got worse: 0.121 → 0.152 m. This was not investigated further.
- **What "error" means differs between conditions.** Waypoints are poses of
  `base_footprint`, so the controlled point is the chassis centre in A and the axle in B.
  - Measured on the chassis centre, B's error is larger: median 0.243 m against 0.117 m in
    A. The body stops about 0.20 m further along the goal heading.
  - Measured on the axle, A's error is larger: median 0.230 m against 0.103 m in B.
  - The position result is therefore about how accurately Nav2 places the point it
    controls, not about a better-placed body.

### H3: localization

AMCL position RMSE against ground truth, per session over the mission: median 0.100 m in A,
0.061 m in B (p = 0.0002, 10 sessions each). The largest per-session AMCL error halved
(median 0.268 → 0.135 m). In A, every turn moves `base_footprint` in a way the odometry
doesn't report, and AMCL's scan matching has to correct that afterwards.

### H4: robustness

| | A | B |
|---|---|---|
| Navigation: progress-checker failures / recoveries / collision-monitor interventions | 0 / 0 / 0 | 0 / 0 / 0 |
| Navigation: minimum body clearance to obstacles, per session | 0.212–0.261 m | 0.088–0.132 m |
| Navigation: minimum body clearance to the keepout zone, per session | 0.105–0.157 m | −0.063 to −0.017 m |
| Mapping: runs back at start | 15 / 15 | 15 / 15 |
| Mapping: minimum body clearance, median over runs (lowest run) | 0.149 m (0.076 m) | 0.108 m (−0.001 m) |
| Mapping: runs with clearance below 0.05 m | 0 | 4 |
| Mapping: collision-monitor interventions, total | 2 | 17 |

There were no failures in either condition, so the count-based tests don't distinguish them
(p = 0.88–1). Clearance did get worse in B, in two places that repeat across runs:

- **Keepout zone at WP1, every B session.**
  - The robot arrives at WP1 heading about +0.5 rad and turns in place to the goal yaw of 0.
  - In B the front-left corner, 0.56 m from the controlled point, swings 2–6 cm into the
    keepout zone during that turn.
  - The keepout zone is the charger plate plus a 0.3 m margin. B's body stayed at least
    0.293 m from the plate itself (A: 0.499 m).
  - The planning check before the study looked only at the final pose with the exact goal
    yaw (0.15 m clearance in both conditions) and missed this turn.
- **West wall on the way to WP4, and the small crate during mapping.**
  - The inflation layer keeps the *controlled point* away from obstacles.
  - In B that point is 0.20 m behind the body centre, and the 0.55 m inflation radius no
    longer covers the full body reach of 0.56 m. So the front of the body gets closer.
  - Four B mapping runs passed within 0.05 m of an obstacle (three at the small crate, one
    at the south wall). One of them (run_02) touched the small crate (−0.001 m) and
    triggered 15 of B's 17 collision-monitor interventions. It still finished.

### Mapping (secondary)

| 15 runs each, median | A | B | p (MWU) |
|---|---|---|---|
| Exploration time | 74.0 s | 66.7 s | 0.078 |
| Total time | 87.9 s | 81.8 s | 0.184 |
| Known area | 79.56 m² | 79.38 m² | 0.033 |
| Coverage of the 80 m² room | 0.975 | 0.975 | 1.0 |
| Map error (mean distance of occupied cells to true surfaces) | 0.0308 m | 0.0298 m | 0.199 |

Exploration time, coverage and map error are no worse in B. The known area is 0.18 m²
smaller (p = 0.033), while coverage of the room is equal. Coverage counts only known cells
inside the 10 × 8 m room, so the difference is in cells outside it. It was not investigated
further.

## Confounds and limitations

- **Run-to-run variation is large next to some effects.** A and A0 ran the same code and
  configuration, a month apart, on a newer kernel. They still differed:
  - in navigation, A0 had 4 progress failures and 5 recoveries, and A had none;
  - in mapping, A0 had a run stuck against a crate and 1.6 progress failures per run, while
    A had no stuck run and 0.53 progress failures per run.
  For this reason B is compared with A, which ran on the same day and machine, not with A0.
- **Order.**
  - A ran before B, except one A navigation session, re-run after B (see Deviations).
  - The real-time factor was 0.993 in A and 0.996 in B (median over navigation sessions).
  - B's full-footprint collision checking did not cost real time. MPPI missed-rate warnings
    were equally rare: none in navigation, and 5 over 15 mapping runs in each condition.
- **The goals mean different body placements.** Because waypoints were not moved, B places
  the body 0.20 m further forward at each goal. This explains the keepout result and part of
  the clearance result. A deployment would shift waypoints, or define goals for the axle.
- **Untuned side effects are part of B as tested.** These were pre-registered and left
  alone on purpose, so the results describe the frame change alone, not a re-tuned system:
  - the inflation radius stays at 0.55 m, now below the 0.56 m circumscribed radius;
  - the inscribed radius drops to 0.10 m;
  - the local costmap sees 0.2 m less ahead of the bumper.
- **Simulation only.** One world, one robot model, frictionless casters, and a chassis whose
  centre of mass sits at axle height. The 3.8 cm axle slip during fast spins depends on that
  physics model.
- **Sample size.** 150 goals and 15 mapping runs per condition. Rare events such as stuck
  runs can't be tested; none occurred in A or B.
- **Multiple tests.** About 20 comparisons were made, without correction. Only H2 was
  designated primary, and its two pre-registered tests disagree.

## Files

- `README.md`: pre-registered protocol, plus Deviations.
- `data/{A,B}/`: every run, with `runs.csv`, rosbags, ground truth, logs, and a
  `provenance.json` per run (commit, branch, URDF and parameter snapshots, package versions,
  tool hashes).
- `analysis/`:
  - `check_urdf_shift.py`: the A vs B URDF check;
  - `compare.py`: metrics, audit and statistics;
  - `figures.py`: the figures.
- `results/`: per-condition CSVs, `comparison.json`, `summary_table.md`.
- `figures/`:
  - `fig_spin.pdf`: (a, b) true path of `base_footprint` and its odometry estimate during
    the spins; (c) odometry error against the true heading change, with the prediction.
  - `fig_goal_error.pdf`: final position error per waypoint, A0, A and B.
