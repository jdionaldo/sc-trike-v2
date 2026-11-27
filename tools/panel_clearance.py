"""Least distance from a shell panel to the front end's moving parts over the pose set of tools/front2_pose.py, and to the static parts near it.
Reads output/model.step. Run: python -I -B tools/panel_clearance.py [Panel_leaf ...]   (default Dash_panel)
Prints per moving group the smallest distance and the pose where it occurs; static parts within 20 mm at rest."""
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / 'tools'))
import cadquery as cq
import front2_pose as fp

POSES = [(phi, eta, ex, None) for phi in fp.PHIS for eta in fp.ETAS for ex in fp.EXTRAS] + [(0.0, e, 0.0, (a, b)) for a, b, e in fp.CORNER_POSES]


def dist(a, b):
    return a.distance(b)


def main():
    names = sys.argv[1:] or ['Dash_panel']
    parts = fp.load(ROOT / 'output' / 'model.step')
    by = {fp.leaf(k): k for k in parts}
    groups = {}
    for gid, ns in fp.moving_groups().items():
        ks = [by[n.replace(' ', '_')] for n in ns if n.replace(' ', '_') in by]
        if ks:
            groups[gid] = ks
    moving = {k for ks in groups.values() for k in ks}
    for n in names:
        panel = parts[by[n]]
        pb = panel.BoundingBox()
        print('%s: x %.1f..%.1f y %.1f..%.1f z %.1f..%.1f' % (n, pb.xmin, pb.xmax, pb.ymin, pb.ymax, pb.zmin, pb.zmax))
        near = []
        for k, s in parts.items():
            if k == by[n] or k in moving:
                continue
            b = s.BoundingBox()
            if b.xmax < pb.xmin - 20 or b.xmin > pb.xmax + 20 or b.ymax < pb.ymin - 20 or b.ymin > pb.ymax + 20 or b.zmax < pb.zmin - 20 or b.zmin > pb.zmax + 20:
                continue
            near.append((dist(panel, s), fp.short(k)))
        for d, k in sorted(near)[:8]:
            print('  static %-40s %.2f mm' % (k, d))
        best = {}
        for phi, eta, ex, trav in POSES:
            locs, bad, text = fp.pose_locs(phi, eta, ex, trav)
            text = ('lean %+.0f steer %+.0f extra %+.0f: ' % (phi, eta, ex) if trav is None else 'corner %+.1f / %+.1f steer %+.0f: ' % (trav[0], trav[1], eta)) + text
            for gid, ks in groups.items():
                if gid not in locs:
                    continue
                for k in ks:
                    s = parts[k].moved(locs[gid])
                    d = dist(panel, s)
                    if fp.short(k) not in best or d < best[fp.short(k)][0]:
                        best[fp.short(k)] = (d, text)
        for k, (d, text) in sorted(best.items(), key=lambda kv: kv[1][0])[:12]:
            print('  moving %-40s %.2f mm at %s' % (k, d, text))


if __name__ == '__main__':
    main()
