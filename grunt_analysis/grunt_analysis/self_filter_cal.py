"""L2 self-return cuboid calibration.

Fits the tight axis-aligned cuboid that encloses the robot's own
returns in the L2 cloud, so `l2_self_filter` masks the body without
over-excluding the near field. The original hand-set cuboid reached
1.10 m forward — a near-field blind wedge that hid real obstacles
until ~1 m ahead.

Repeatable by design: re-run whenever the robot's shape changes (arm
re-stow / new arm pose, mast change, collision damage). The point
cloud is treated as ground truth — the URDF drifts (grunt's arm is
currently collision-bent and no longer matches its model).

Method:
  1. Aggregate raw L2 clouds, transform to base_link via tf_static.
  2. Drop ground: keep z >= chassis top-plate height (tf-derived). The
     L2 sees no body below the plate; below-plate near-robot points are
     ground. This also stops the body blob from merging with the
     ground sheet as one connected component in step 4.
  3. Voxel-occupancy grid — a voxel is "occupied" only with
     >= min_vox_pts points, so stray returns cannot create structure.
  4. Connected components; the body = components whose centroid lies in
     the near-robot envelope (world objects sit outside it, past a
     clear gap). Vertically-disconnected body parts — the mast/antenna
     column above the chassis — are picked up because their centroid is
     still near the origin.
  5. AABB of the body voxels + margin -> the cuboid.
  6. z_max raised to the ten-hut arm envelope if that is higher (so the
     box still masks the arm when it is raised, not just when stowed).

Run:
    python -m grunt_analysis.self_filter_cal <bag_dir>
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

from mcap_ros2.reader import read_ros2_messages

from .bag import discover_mcap
from .traversability import _tf_matrix, compose_static, decode_xyz

DEFAULT_CLOUD_TOPIC = "/grunt1/l2/points"
# the cuboid l2_self_filter ships with today — reported for comparison
OLD_CUBOID = dict(x=(-0.40, 1.10), y=(-0.45, 0.45), z=(0.00, 1.50))


def main(argv=None) -> int:
    p = argparse.ArgumentParser(
        description="Calibrate the l2_self_filter self-return cuboid")
    p.add_argument("bag_dir", help="raw-L2 bag (robot static, arm stowed)")
    p.add_argument("--out", help="output dir (default: next to the bag)")
    p.add_argument("--cloud-topic", default=DEFAULT_CLOUD_TOPIC)
    p.add_argument("--base-frame", default=None,
                   help="default: <cloud-prefix>/base_link")
    p.add_argument("--plate-frame", default=None,
                   help="ground-cut reference; default <prefix>/chassis/"
                        "top_plate")
    p.add_argument("--arm-base-frame", default=None,
                   help="default <prefix>/arm1/base_link")
    p.add_argument("--max-clouds", type=int, default=150)
    p.add_argument("--margin", type=float, default=0.10,
                   help="cuboid margin on every face, m (default 0.10)")
    p.add_argument("--vox", type=float, default=0.08,
                   help="voxel size, m (default 0.08)")
    p.add_argument("--min-vox-pts", type=int, default=3,
                   help="points needed for a voxel to count as occupied")
    p.add_argument("--env-x", type=float, default=1.2,
                   help="near-robot envelope half-extent in x, m — a "
                        "component is body if its centroid is inside it")
    p.add_argument("--env-y", type=float, default=0.9)
    p.add_argument("--tenhut-arm-reach", type=float, default=0.70,
                   help="arm vertical reach above its base in the "
                        "ten-hut (straight-up) pose, m")
    p.add_argument("--costmap-max-height", type=float, default=1.50,
                   help="voxel_layer max_obstacle_height, m — self-"
                        "returns above this are dropped by the costmap "
                        "anyway, so they must not inflate the cuboid")
    args = p.parse_args(argv)

    bag_dir = Path(args.bag_dir)
    out_dir = Path(args.out) if args.out else (
        bag_dir if bag_dir.is_dir() else bag_dir.parent)
    mcap = discover_mcap(bag_dir)
    label = bag_dir.name

    static_tf = {}
    clouds = []
    cloud_frame = None
    for m in read_ros2_messages(str(mcap), topics=[
            "/tf_static", args.cloud_topic]):
        if m.channel.topic == "/tf_static":
            for tr in m.ros_msg.transforms:
                static_tf[tr.child_frame_id] = (
                    tr.header.frame_id, _tf_matrix(tr.transform))
            continue
        if len(clouds) >= args.max_clouds:
            continue
        if cloud_frame is None:
            cloud_frame = m.ros_msg.header.frame_id
        xyz = decode_xyz(m.ros_msg)
        if xyz.shape[0]:
            clouds.append(xyz)

    if not clouds:
        print(f"no `{args.cloud_topic}` clouds in {label}", file=sys.stderr)
        return 1

    prefix = cloud_frame.split("/")[0]
    base = args.base_frame or f"{prefix}/base_link"
    plate_frame = args.plate_frame or f"{prefix}/chassis/top_plate"
    arm_frame = args.arm_base_frame or f"{prefix}/arm1/base_link"

    T = compose_static(static_tf, base, cloud_frame)
    if T is None:
        print(f"no static TF {cloud_frame} -> {base}", file=sys.stderr)
        return 1
    Tp = compose_static(static_tf, base, plate_frame)
    Ta = compose_static(static_tf, base, arm_frame)
    plate_z = float(Tp[2, 3]) if Tp is not None else 0.0
    arm_base_z = float(Ta[2, 3]) if Ta is not None else plate_z
    tenhut_top = arm_base_z + args.tenhut_arm_reach

    P = np.vstack(clouds)
    P = (P @ T[:3, :3].T + T[:3, 3])
    n_total = P.shape[0]

    # --- step 2: drop ground (below the chassis top plate) ---
    above = P[P[:, 2] >= plate_z]
    if above.shape[0] < 50:
        print("almost nothing above the plate — wrong plate frame?",
              file=sys.stderr)
        return 1

    # --- step 3: voxel-occupancy grid ---
    vox = args.vox
    orig = above.min(axis=0)
    idx = np.floor((above - orig) / vox).astype(np.int64)
    dims = idx.max(axis=0) + 1
    flat = (idx[:, 0] * dims[1] + idx[:, 1]) * dims[2] + idx[:, 2]
    counts = np.bincount(flat, minlength=int(np.prod(dims)))
    occ = (counts >= args.min_vox_pts).reshape(dims)

    # --- step 4: connected components, keep near-robot ones ---
    from scipy.ndimage import label as cc_label
    lab, n_lab = cc_label(occ, structure=np.ones((3, 3, 3)))
    body_mask = np.zeros(dims, dtype=bool)
    comps = []
    for k in range(1, n_lab + 1):
        vx = np.argwhere(lab == k)
        ctr = orig + (vx.mean(axis=0) + 0.5) * vox
        is_body = abs(ctr[0]) < args.env_x and abs(ctr[1]) < args.env_y
        comps.append((k, vx.shape[0], ctr, is_body))
        if is_body:
            body_mask[lab == k] = True

    body_vox = np.argwhere(body_mask)
    if body_vox.shape[0] == 0:
        print("no body component found inside the envelope — widen "
              "--env-x/--env-y or check the render", file=sys.stderr)
        return 1

    # --- step 5: AABB of the body voxels + margin ---
    # The costmap drops returns above max_obstacle_height, so self-
    # returns above it (the mast / GPS antenna) need no masking and must
    # not inflate the cuboid — split them off before the AABB.
    vox_cz = orig[2] + (body_vox[:, 2] + 0.5) * vox
    masked_vox = body_vox[vox_cz < args.costmap_max_height]
    mast_vox = body_vox[vox_cz >= args.costmap_max_height]
    if masked_vox.shape[0] == 0:
        masked_vox = body_vox
    lo = orig + masked_vox.min(axis=0) * vox
    hi = orig + (masked_vox.max(axis=0) + 1) * vox
    mg = args.margin
    cub_lo = lo - mg
    cub_hi = hi + mg
    # --- step 6: raise z_max to the ten-hut arm envelope ---
    z_self = cub_hi[2]
    cub_hi[2] = max(z_self, tenhut_top + mg)
    z_driver = ("stowed-body AABB" if z_self >= tenhut_top + mg
                else "ten-hut arm envelope")

    # how many self-returns the old vs new cuboid actually catch
    def inside(box, pts):
        return int(np.all([
            pts[:, i] >= box[0][i] for i in range(3)] + [
            pts[:, i] <= box[1][i] for i in range(3)], axis=0).sum())

    new_box = (cub_lo, cub_hi)
    body_pts_mask = np.zeros(above.shape[0], dtype=bool)
    body_flat = set((np.argwhere(body_mask)[:, 0] * dims[1]
                     + np.argwhere(body_mask)[:, 1]) * dims[2]
                    + np.argwhere(body_mask)[:, 2])
    body_pts_mask = np.array([f in body_flat for f in flat])
    self_pts = above[body_pts_mask]
    # the costmap-relevant self-returns (below max_obstacle_height) —
    # the cuboid only needs to enclose these; above are the mast/antenna
    self_lo = self_pts[self_pts[:, 2] < args.costmap_max_height]
    self_hi = self_pts[self_pts[:, 2] >= args.costmap_max_height]

    _render(P, self_pts, OLD_CUBOID, new_box, out_dir, label)

    # --- report ---
    L = [f"# L2 self-filter cuboid calibration — {label}", "",
         f"- bag: `{mcap.name}`  ({len(clouds)} clouds aggregated, "
         f"{n_total} pts)",
         f"- frames: cloud `{cloud_frame}` -> base `{base}`",
         f"- chassis top-plate z = {plate_z:+.3f} m (ground cut)",
         f"- arm base z = {arm_base_z:+.3f} m; ten-hut reach "
         f"{args.tenhut_arm_reach:.2f} m -> arm top {tenhut_top:+.3f} m",
         f"- voxel {vox:.2f} m, min {args.min_vox_pts} pts/voxel, "
         f"margin {mg:.2f} m", "",
         "## Connected components above the plate", "",
         f"{n_lab} components. Body = centroid inside the "
         f"|x|<{args.env_x}, |y|<{args.env_y} m envelope; all body "
         "components plus the 5 largest others:", "",
         "| comp | voxels | centroid (x,y,z) | body? |",
         "|---|---|---|---|"]
    body_comps = sorted((c for c in comps if c[3]), key=lambda c: -c[1])
    other_comps = sorted((c for c in comps if not c[3]),
                         key=lambda c: -c[1])[:5]
    for k, nv, ctr, is_body in body_comps + other_comps:
        L.append(f"| {k} | {nv} | ({ctr[0]:+.2f}, {ctr[1]:+.2f}, "
                 f"{ctr[2]:+.2f}) | {'YES' if is_body else 'no'} |")
    L += ["",
          f"Body = {int(body_mask.sum())} voxels / {self_pts.shape[0]} "
          f"points (components with centroid inside the "
          f"|x|<{args.env_x}, |y|<{args.env_y} m envelope).", "",
          "## Calibrated cuboid", "",
          "| axis | old | new | ",
          "|---|---|---|"]
    for i, ax in enumerate("xyz"):
        old = OLD_CUBOID[ax]
        L.append(f"| {ax} | [{old[0]:+.2f}, {old[1]:+.2f}] | "
                 f"[{cub_lo[i]:+.2f}, {cub_hi[i]:+.2f}] |")
    old_box = (np.array([OLD_CUBOID[a][0] for a in "xyz"]),
               np.array([OLD_CUBOID[a][1] for a in "xyz"]))
    L += ["",
          f"- z_max set by: **{z_driver}** (stowed-body AABB+margin "
          f"{z_self:.2f} m vs ten-hut+margin {tenhut_top + mg:.2f} m)",
          f"- costmap-relevant self-returns (z < "
          f"{args.costmap_max_height:.1f} m) enclosed: "
          f"new {inside(new_box, self_lo)}/{self_lo.shape[0]}, "
          f"old {inside(old_box, self_lo)}/{self_lo.shape[0]}",
          f"- {self_hi.shape[0]} self-returns above "
          f"{args.costmap_max_height:.1f} m (the mast / GPS antenna) — "
          f"the costmap's max_obstacle_height drops these, so the "
          f"cuboid deliberately does NOT enclose them",
          f"- forward reach: old {OLD_CUBOID['x'][1]:.2f} m -> "
          f"new {cub_hi[0]:.2f} m "
          f"(near-field blind wedge shrinks by "
          f"{OLD_CUBOID['x'][1] - cub_hi[0]:+.2f} m)", "",
          "## Apply — l2_self_filter parameters", "", "```yaml"]
    for i, ax in enumerate("xyz"):
        L.append(f"cuboid_{ax}_min: {cub_lo[i]:.3f}")
        L.append(f"cuboid_{ax}_max: {cub_hi[i]:.3f}")
    L += ["```", "",
          "Render: `self_filter_cal.png` — grey = scene, coloured = "
          "self-returns, dashed = old cuboid, solid = new.", "",
          "## Re-running",
          "",
          "Re-run after any change to the robot's shape (arm re-stow or "
          "a new arm pose, mast/sensor changes, collision damage): "
          "`python -m grunt_analysis.self_filter_cal <new_bag>`. For a "
          "non-stowed arm pose widen `--env-x`/`--env-y` so the extended "
          "arm's component still counts as body, and confirm via the "
          "render.", ""]

    (out_dir / "self_filter_calibration.md").write_text("\n".join(L) + "\n")
    print(f"wrote {out_dir / 'self_filter_calibration.md'}", file=sys.stderr)
    print(f"{label}: cuboid x[{cub_lo[0]:+.2f},{cub_hi[0]:+.2f}] "
          f"y[{cub_lo[1]:+.2f},{cub_hi[1]:+.2f}] "
          f"z[{cub_lo[2]:+.2f},{cub_hi[2]:+.2f}]  "
          f"(fwd reach {OLD_CUBOID['x'][1]:.2f}->{cub_hi[0]:.2f} m)",
          file=sys.stderr)
    return 0


def _render(P, self_pts, old, new, out_dir, label) -> bool:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
    except Exception:
        return False
    # scene context near the robot
    reg = P[(P[:, 0] > -2) & (P[:, 0] < 4) & (np.abs(P[:, 1]) < 2.5)
            & (P[:, 2] > -1) & (P[:, 2] < 3)]
    nlo, nhi = new
    fig, axs = plt.subplots(1, 2, figsize=(15, 6))
    for ax, (h, v, hl, vl) in zip(axs, [(0, 1, "x fwd", "y left"),
                                        (0, 2, "x fwd", "z up")]):
        ax.scatter(reg[:, h], reg[:, v], s=1, c="#bbbbbb")
        ax.scatter(self_pts[:, h], self_pts[:, v], s=2, c="#c0392b")
        for box, style in [(old, dict(ls="--", ec="#888800")),
                           ((nlo, nhi), dict(ls="-", ec="#1f77b4"))]:
            if isinstance(box, dict):
                b0 = (box["xyz"[h]][0], box["xyz"[v]][0])
                w = box["xyz"[h]][1] - box["xyz"[h]][0]
                ht = box["xyz"[v]][1] - box["xyz"[v]][0]
            else:
                b0 = (box[0][h], box[0][v])
                w = box[1][h] - box[0][h]
                ht = box[1][v] - box[0][v]
            ax.add_patch(Rectangle(b0, w, ht, fill=False, lw=2, **style))
        ax.plot(0, 0, "k+", ms=14)
        ax.set_xlabel(hl)
        ax.set_ylabel(vl)
        ax.set_aspect("equal")
        ax.grid(alpha=0.3)
    axs[0].set_title(f"top-down — {label}")
    axs[1].set_title("side  (dashed=old cuboid, solid=new)")
    fig.tight_layout()
    fig.savefig(out_dir / "self_filter_cal.png", dpi=90)
    plt.close(fig)
    return True


if __name__ == "__main__":
    sys.exit(main())
