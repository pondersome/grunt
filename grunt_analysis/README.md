# grunt_analysis

Post-mission analysis tooling for grunt's outdoor and indoor runs. Reads
mcap bags produced by `diagnostic_bagger` (or any `ros2 bag record` output),
computes a set of composable metrics, and renders reports + plots.

This is a **bench-side / off-robot** tool. Nothing in here runs at runtime
on the robot; it's pure Python that consumes bag files. No ROS node, no
launch file, no colcon participation needed.

## Why this exists

Mission post-mortems used to be one-off `python3 -c '...'` blocks scattered
through chat sessions, with state cached in `/tmp/*.pkl` between commands.
That's fine for sleuthing a single symptom but doesn't compose: when a new
sensor or algorithm comes online (LiDAR, RTABMap pose graph, etc.), we'd
re-derive every prior analysis from scratch instead of layering on top.

This package keeps those analyses as named, composable functions that we
can grow with the platform.

## Layout

```
grunt_analysis/
├── README.md
├── pyproject.toml                  # `pip install -e .` to use
├── grunt_analysis/                 # the package
│   ├── __init__.py
│   ├── bag.py                      # load topics from mcap (summarized)
│   ├── geo.py                      # ENU at site anchor (matches GeographicLib)
│   ├── metrics.py                  # composable metric functions
│   ├── viz.py                      # SVG plot generation
│   ├── cli.py                      # main mission-report runner
│   ├── _deep_common.py             # shared helpers for deep-dive modules
│   ├── segdev.py                   # per-segment + planner + carrots
│   ├── planner_audit.py            # plan-vs-segment + EKF jumps
│   ├── onsets.py                   # fresh-onset + sonar marks
│   └── costmap_render.py           # actual costmap PNGs at events
├── scripts/                        # thin entry-points
│   ├── analyze_mission.py
│   ├── analyze_segments.py
│   ├── analyze_planner.py
│   ├── analyze_onsets.py
│   └── render_costmaps.py
└── tests/
    └── test_geo.py
```

## Tools — what to run when

Use this as the "where do I start" guide for a new bag. Most missions
get the **mission report** first; the other tools answer specific
questions when the report points at one.

| When you want to ... | Run | Outputs |
|---|---|---|
| Summarize a mission end-to-end (metrics, accuracy tube, cmd→actual) | `python -m grunt_analysis.cli` | `report.md`, `accuracy_tube.svg`, `cmd_vs_actual.svg` |
| Understand where actual path deviated from the ideal segment path | `python -m grunt_analysis.segdev` | `segdev_report.md`, `segment_overlay.svg`, `carrot_vectors.svg`, `deviation_strip.svg` |
| Check whether the planner is curving plans off the segment line | `python -m grunt_analysis.planner_audit` | `planner_audit.md`, `planner_only.svg` |
| Find what triggered a foliage-stuck event (vs cluster recounting) | `python -m grunt_analysis.onsets` | `onset_audit.md`, `planner_with_marks.svg` |
| See the actual recorded costmap at a moment (best evidence) | `python -m grunt_analysis.costmap_render` | `actual_costmap_<tag>_<label>.png` per event |

### Most useful for routine post-mission

Default ordering:

1. **`cli`** — the workhorse. Run this on every bag. Tells you everything
   that fits in a single page of metrics: datum alignment, EKF tracking,
   IMU health, RTK continuity, cmd_vel breakdown, control oscillation,
   crosstrack, interventions, collision events, plus the path-vs-truth
   accuracy tube.

2. **`segdev`** — run when the report shows non-trivial crosstrack or
   when the operator says "drifted to one side". Tells you per-segment
   bias (LEFT/RIGHT/mixed), abs_p90 deviation, and shows the carrot
   vector field so you can see what RPP was being told to chase.

3. **`onsets`** — run when there were collision events. Distinguishes
   FRESH onsets (no prior collision in 10 s) from cluster events, so
   the "% of plans correlate with collision activity" claim isn't
   inflated by extrication clusters. Adds the sonar mark overlay, which
   is the obstacle picture the planner is reacting to.

### Drill-down when above point at the planner / costmap

4. **`planner_audit`** — when `segdev` shows the planner's plans aren't
   tracking the segment line. Computes plan-vs-ideal-segment deviation
   (different from internal bow), lists the most-deviating plans, and
   checks for EKF position/yaw jumps that would distort plan framing.
   Produces a clean `planner_only.svg` with no actual-path occlusion.

5. **`costmap_render`** — when you need to see what the planner ACTUALLY
   saw, not a reconstruction. Reads recorded `OccupancyGrid` messages
   (already in `diagnostic_bagger`'s topic list) and renders PNGs at
   each fresh onset, plus any `--at <T>` time you pass. PNG (not SVG)
   because per-cell SVG would be 100k+ elements.

## Usage

The package's runtime deps (`mcap_ros2_support`, `matplotlib`, `numpy`,
`pyyaml`) are already installed system-wide on this rig as part of the
ROS / dev stack — so the simplest invocation is to just run from source:

```bash
cd ~/ros2_ws/src/grunt/grunt_analysis

# Routine mission report
PYTHONPATH=. python3 -m grunt_analysis.cli \
    ~/bag_files/grunt/<bag> \
    --mission-yaml ~/ros2_ws/grunt_missions/sites/ranchero/<mission>.yaml

# Drill-down (any subset)
PYTHONPATH=. python3 -m grunt_analysis.segdev <bag> --mission-yaml <m>
PYTHONPATH=. python3 -m grunt_analysis.onsets <bag> --mission-yaml <m>
PYTHONPATH=. python3 -m grunt_analysis.planner_audit <bag> --mission-yaml <m>
PYTHONPATH=. python3 -m grunt_analysis.costmap_render <bag> --mission-yaml <m>

# Render costmaps at specific times (in addition to fresh onsets)
PYTHONPATH=. python3 -m grunt_analysis.costmap_render <bag> \
    --mission-yaml <m> --at 120 --at 254 --at 329
```

Output (default — written next to the bag, or to `--out <dir>`):

| Module | Files |
|---|---|
| `cli` | `report.md`, `accuracy_tube.svg`, `cmd_vs_actual.svg` |
| `segdev` | `segdev_report.md`, `segment_overlay.svg`, `carrot_vectors.svg`, `deviation_strip.svg` |
| `planner_audit` | `planner_audit.md`, `planner_only.svg` |
| `onsets` | `onset_audit.md`, `planner_with_marks.svg` |
| `costmap_render` | `actual_costmap_<tag>_<label>.png` × N |

Override output dir with `--out /some/dir`. Pass `--site-yaml` to point
at a different site (defaults to
`~/ros2_ws/grunt_missions/sites/ranchero/site.yaml`).

### Why not `pip install -e .`?

Ubuntu 24.04's PEP 668 protection blocks editable installs into the system
Python without `--break-system-packages`. If you want a clean install, use a
venv:

```bash
python3 -m venv ~/.venv/grunt_analysis
source ~/.venv/grunt_analysis/bin/activate
pip install -e ~/ros2_ws/src/grunt/grunt_analysis
grunt-analyze-mission ~/bag_files/grunt/<bag>/
```

The `pyproject.toml` is set up so the venv path works whenever someone wants
it, but the `PYTHONPATH=.` from-source invocation is canonical for
day-to-day analysis work.

## Adding new metrics

Each metric is a function in `grunt_analysis/metrics.py`:

```python
def my_metric(parsed: dict[str, list], anchor: GeoAnchor) -> dict:
    """One-line summary.

    Returns a dict with at minimum a 'name' field and arbitrary results.
    """
    ...
```

Convention: metrics take the same `parsed` dict (output of
`bag.load_bag`) plus a site anchor object, and return a result dict.
The runner aggregates results into the report.

To add a new sensor's metrics:

1. Add the topic to `bag.TOPICS_OF_INTEREST` (see `bag.py`).
2. Add a parser in `bag._decode_message` if the topic type isn't already
   handled.
3. Write metric functions that consume the parsed data.
4. Add the metric to the runner's pipeline.

## Planned extensions

| Sensor / capability | What we'd add |
|---|---|
| **Unitree L2 LiDAR** | Scan-rate health, point density vs range, scan match residual to current pose, ground-plane outlier fraction. |
| **RTABMap** | Pose-graph optimization residuals, loop closure events, constraint quality, map-frame discontinuities at relocalization. |
| **AprilTag indoor relocalization** | Tag detection rate, residual between tag-derived pose and EKF, gap before relocalization on first detection at startup. |
| **Indoor SLAM transitions** | EKF→AMCL handoff jumps, map-relative crosstrack on saved maps. |
| **Long-horizon RTK quality** | Per-mission % FIXED, head_acc CDF, baseline distance to NTRIP base station correlation. |

Each of these adds new metric functions; the runner composes them.

## Conventions

- All distances in meters, all angles in radians internally; deg only for
  display.
- Site anchor lat/lon comes from `<missions_dir>/sites/<site>/site.yaml`.
- Local ENU is computed via flat-earth approximation (matches
  `GeographicLib::LocalCartesian` to mm at site scale; we don't need the
  full ECEF model for sub-km work).
- Time series passed around as `[(t, *fields)]` tuples — t in seconds since
  bag start unless explicitly bag-absolute.
- Plots use the matplotlib SVG backend by default for resolution-independence.

## Status

- 2026-05-01: initial extract from chat-session sleuthing scripts.
- 2026-05-02: deep-dive modules promoted from /tmp:
  - `segdev` — per-segment deviation + planner overlay + carrot vectors
  - `planner_audit` — plan-vs-segment deviation + EKF jump detection
  - `onsets` — fresh-onset filtering + sonar mark overlay
  - `costmap_render` — actual recorded `OccupancyGrid` PNGs
  - `_deep_common` — shared geometry + bag-streaming helpers
- Metrics: datum_alignment, ekf_tracking, imu_health, rtk_continuity,
  cmd_vel_breakdown, control_oscillation (S-turn signature),
  cmd_vel_response, rotation_mode_fraction, carrot_tracking,
  heading_obs_acceptance, crosstrack_error, interventions,
  collision_events.
- Visualizations: accuracy_tube (GPS path colored by carrier, EKF overlay,
  waypoints, intervention markers), cmd_vs_actual, segment_overlay,
  carrot_vectors, deviation_strip, planner_only, planner_with_marks,
  actual_costmap PNGs.

## Reference analyses

- `ponderdocs/nav/docs/outdoor_mission_analysis_2026-05-01.md` — first
  run using this pattern, two missions side-by-side.
- `ponderdocs/nav/docs/segment_deviation_2026-05-02.md` — deep-dive on
  the 2026-05-02 outbound bag (drove the deep-dive modules above).
