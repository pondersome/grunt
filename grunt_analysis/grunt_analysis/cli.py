"""CLI runner: load a bag, run all metrics, print a report.

Usage:
    grunt-analyze-mission <bag_dir> [--site-yaml PATH] [--mission-yaml PATH]
                                     [--out DIR]

By default writes report.md and accuracy_tube.svg next to the bag.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import yaml

from .bag import load_bag
from .geo import GeoAnchor
from . import metrics, viz


def _pick_default_site_yaml(bag_dir: Path) -> Path | None:
    """Best-effort: assume Ranchero site if no site_yaml is passed."""
    cand = Path.home() / "ros2_ws" / "grunt_missions" / "sites" / "ranchero" / "site.yaml"
    return cand if cand.exists() else None


def _format_report(label: str, results: list[dict],
                    bag_path: Path, anchor: GeoAnchor,
                    mission_yaml: Path | None) -> str:
    """Render a results list to markdown."""
    lines = [
        f"# Mission analysis — {label}",
        "",
        f"- bag: `{bag_path}`",
        f"- anchor: `{anchor.name}` @ ({anchor.lat:.7f}, {anchor.lon:.7f})",
    ]
    if mission_yaml:
        lines.append(f"- mission: `{mission_yaml.name}`")
    lines.extend(["", "## Metrics", ""])
    for r in results:
        name = r.get("name", "?")
        lines.append(f"### {name}")
        if "error" in r:
            lines.append(f"  - **error**: {r['error']}")
        else:
            for k, v in r.items():
                if k == "name":
                    continue
                if isinstance(v, list) and len(v) > 8:
                    lines.append(f"  - {k}: {len(v)} items "
                                  f"(first 3: {v[:3]} …)")
                elif isinstance(v, dict):
                    lines.append(f"  - {k}: {json.dumps(v)}")
                else:
                    lines.append(f"  - {k}: {v}")
        lines.append("")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(prog="grunt-analyze-mission",
                                 description=__doc__)
    p.add_argument("bag_dir", type=Path, help="bag directory or .mcap file")
    p.add_argument("--site-yaml", type=Path, default=None,
                   help="path to site.yaml with anchor_lat/lon "
                        "(default: ranchero)")
    p.add_argument("--mission-yaml", type=Path, default=None,
                   help="path to mission YAML — enables crosstrack metric "
                        "and waypoint overlay in plot")
    p.add_argument("--out", type=Path, default=None,
                   help="output directory (default: <bag_dir>)")
    args = p.parse_args(argv)

    site_yaml = args.site_yaml or _pick_default_site_yaml(args.bag_dir)
    if site_yaml is None:
        print("error: no --site-yaml and default ranchero site not found",
              file=sys.stderr)
        return 2
    anchor = GeoAnchor.from_site_yaml(site_yaml)

    waypoints_lla = None
    if args.mission_yaml:
        with open(args.mission_yaml) as f:
            mdata = yaml.safe_load(f)
        waypoints_lla = [(w["lat"], w["lon"]) for w in mdata["waypoints"]]

    print(f"loading bag from {args.bag_dir}")
    parsed = load_bag(args.bag_dir)

    print("running metrics...")
    results = metrics.run_all(parsed, anchor, waypoints_lla=waypoints_lla)

    out_dir = args.out or (args.bag_dir if args.bag_dir.is_dir()
                            else args.bag_dir.parent)
    out_dir.mkdir(parents=True, exist_ok=True)

    # Markdown report
    label = args.bag_dir.name
    report_path = out_dir / "report.md"
    report_path.write_text(_format_report(label, results,
                                            args.bag_dir, anchor,
                                            args.mission_yaml))
    print(f"wrote {report_path}")

    # Accuracy tube SVG
    svg_path = out_dir / "accuracy_tube.svg"
    viz.accuracy_tube({label: parsed}, anchor,
                       {label: waypoints_lla or []}, svg_path,
                       title=f"Accuracy tube — {label}")
    print(f"wrote {svg_path}")

    # cmd_vel response SVG (best-effort — needs both cmd_vel and odometry/local)
    try:
        cva_path = out_dir / "cmd_vs_actual.svg"
        viz.cmd_vs_actual(parsed, cva_path,
                           title=f"cmd→actual — {label}")
        print(f"wrote {cva_path}")
    except Exception as e:
        print(f"(skipped cmd_vs_actual: {e})")

    return 0


if __name__ == "__main__":
    sys.exit(main())
