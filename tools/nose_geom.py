"""Approach angle and ground clearance under the nose, from a built STEP (rest pose).

Approach angle: the steepest ramp, tangent to the front tyres, that touches no part ahead of the front axle (tyre radius = axle height above
ground). Nose clearance: lowest point of any part ahead of the front axle line. Tyres and wheels are left out of both.
Run: python -I -B tools/nose_geom.py [path/to/model.step]
"""
import math
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / 'tools'))
import front2_pose as fp
from parts import swingarm2 as sa

GROUND = -15.6                     # mm, world (NOTES: ground z -15.6)
NOSE_X = 50.0                      # mm: the nose between the front wheels (the knuckles and tie rods outboard are the suspension, not the nose)
SKIP = ('mock_tire_', 'wheel_f_', 'Rear_tire', 'wheel_r')


def main():
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / 'output' / 'model.step'
    parts = fp.load(path)
    yw, zc = sa.ARM_TIP_Y if hasattr(sa, 'ARM_TIP_Y') else -216.55, 40.6
    r = zc - GROUND
    best, low, low_n = (90.0, None), (1e9, None), (1e9, None)
    for k, s in parts.items():
        n = fp.leaf(k)
        if n.startswith(SKIP) or s.BoundingBox().ymin > yw:
            continue
        verts, _ = s.tessellate(0.05, 0.2)
        for v in verts:
            if v.y >= yw:
                continue
            if v.z < low[0]:
                low = (v.z, (n, round(v.x, 1), round(v.y, 1), round(v.z, 1)))
            if abs(v.x) <= NOSE_X and v.z < low_n[0]:
                low_n = (v.z, (n, round(v.x, 1), round(v.y, 1), round(v.z, 1)))
            dy, dz = v.y - yw, v.z - zc          # dy < 0: ahead of the axle
            # ramp at angle t rising to the front, tangent to the tyre: point clear if sin(t)*dy + cos(t)*dz >= -r ... find the t where it touches
            lo, hi = 0.0, math.pi / 2
            if math.sin(hi) * dy + math.cos(hi) * dz >= -r:
                continue
            for _ in range(50):
                m = (lo + hi) / 2
                if math.sin(m) * dy + math.cos(m) * dz >= -r:
                    lo = m
                else:
                    hi = m
            t = math.degrees(lo)
            if t < best[0]:
                best = (t, (n, round(v.x, 1), round(v.y, 1), round(v.z, 1)))
    # clearance from the solids, not the mesh vertices (a face's vertices can all lie behind the axle line while the face runs on ahead of it)
    import cadquery as cq
    zone = cq.Solid.makeBox(2 * NOSE_X, 400.0, 400.0, cq.Vector(-NOSE_X, yw - 400.0, GROUND))
    low_n = (1e9, None)
    for k, s in parts.items():
        n = fp.leaf(k)
        if n.startswith(SKIP) or s.BoundingBox().ymin > yw:
            continue
        c = s.intersect(zone)
        if c.Volume() > 1e-6 and c.BoundingBox().zmin < low_n[0]:
            b = c.BoundingBox()
            low_n = (b.zmin, (n, 'x %.1f..%.1f y %.1f..%.1f' % (b.xmin, b.xmax, b.ymin, b.ymax)))
    print('front axle y %.2f, tyre radius %.1f, ground z %.1f' % (yw, r, GROUND))
    print('approach angle %.1f deg, limited by %s' % best)
    print('nose clearance %.1f mm above ground (lowest point ahead of the axle within |x| <= %.0f: %s)' % (low_n[0] - GROUND, NOSE_X, low_n[1]))
    print('lowest point ahead of the axle, any x: %.1f mm (%s)' % (low[0] - GROUND, low[1]))


if __name__ == '__main__':
    main()
