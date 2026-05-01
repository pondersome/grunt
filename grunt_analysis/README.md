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
│   ├── bag.py                      # load topics from mcap
│   ├── geo.py                      # ENU at site anchor (matches GeographicLib)
│   ├── metrics.py                  # composable metric functions
│   └── viz.py                      # SVG plot generation
├── scripts/
│   └── analyze_mission.py          # CLI runner — produces a report markdown
└── tests/
    └── test_geo.py                 # placeholder
```

## Usage

The package's runtime deps (`mcap_ros2_support`, `matplotlib`, `pyyaml`) are
already installed system-wide on this rig as part of the ROS / dev stack —
so the simplest invocation is to just run from source:

```bash
cd ~/ros2_ws/src/grunt/grunt_analysis
PYTHONPATH=. python3 -m grunt_analysis.cli \
    ~/bag_files/grunt/2026-05-01_13-22-43_run_gps_mission \
    --mission-yaml ~/ros2_ws/grunt_missions/sites/ranchero/260501-131659.yaml
```

Output (default — written next to the bag):

- `report.md` — metrics breakdown
- `accuracy_tube.svg` — vector path plot

Override output dir with `--out /some/dir`. Pass `--site-yaml` to point at a
different site (defaults to `~/ros2_ws/grunt_missions/sites/ranchero/site.yaml`).

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
- Metrics: datum_alignment, ekf_tracking, imu_health, rtk_continuity,
  cmd_vel_breakdown, control_oscillation (S-turn signature),
  crosstrack_error, interventions, collision_events.
- Visualizations: accuracy_tube (GPS path colored by carrier, EKF overlay,
  waypoints, intervention markers).

## Reference analyses

- `ponderdocs/nav/docs/outdoor_mission_analysis_2026-05-01.md` — first run
  using this pattern, two missions side-by-side.
