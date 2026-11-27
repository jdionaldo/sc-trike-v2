"""Pod top / flank / side cover seam table (NOTES 62): stations every 10 mm along y, both sides, inside the owner's loop (the pod top and
the flank's upper band, z over 110).

At each station two ray fans scan the section: vertical rays across x and horizontal rays (from outboard) across z. The hits are grouped into
runs: consecutive hits on one part whose normals agree within 2 deg, at least 3 rays (0.6 mm) long. Between two neighbouring runs is a joint
(a fold of one part, or a seam between two parts with whatever the rays saw in its gap). For each joint:
  step = min(|nA . (b - a)|, |nB . (a - b)|), a / b the last / first hit of the two runs, nA / nB their normals: how far one surface stands
         off the other's plane across the joint (a ledge or a tooth shows here; a clean fold reads its gap times the sine of its crease)
  gap  = |b - a| minus the ray spacing (only for seams between two parts)
The worst joint of each station is listed, with the parts and the crease. Reads output/model.step, or STEP=<file>; optional: y0 y1 dy. Run from the project folder: python -I -B tools/pod_seams.py
"""
import sys, os
sys.path.insert(0, '.'); sys.path.insert(0, 'tools')
import numpy as np
import seam_probe as sp
if os.environ.get('STEP'):
    import front2_pose as fp
    sp._CACHE['P'] = {fp.leaf(k): v for k, v in fp.load(os.environ['STEP']).items()}

SP = 0.2
y0, y1, dy = (float(a) for a in sys.argv[1:4]) if len(sys.argv) > 3 else (-144.0, 134.0, 10.0)
names = {n for n in sp._parts()}


def runs(hits):
    out, cur = [], []
    for h in hits:
        if h is None:
            if len(cur) >= 3: out.append(cur)
            cur = []; continue
        if cur and (h[0] != cur[-1][0] or np.dot(h[2], cur[-1][2]) < np.cos(np.radians(2.0))):
            if len(cur) >= 3: out.append(cur)
            cur = []
        cur.append(h)
    if len(cur) >= 3: out.append(cur)
    return out


def scan(meshes, mk, rng, d):
    hits = []
    for t in rng:
        o = mk(t); best = None
        for nm, tri in meshes.items():
            h = sp._hit(o, d, tri)
            if h and (best is None or h[0] < best[1]):
                best = (nm, h[0], h[1])
        hits.append(None if best is None else (best[0], o + d * best[1], best[2]))
    return hits


rows_all = []
for sx, side in ((1, 'right'), (-1, 'left')):
    parts = sorted(n for n in names if ('pod' in n.lower() or 'cover' in n.lower()) and side in n.lower())
    meshes = {n: sp._mesh(n) for n in parts}
    print('side', side, parts)
    y = y0
    while y <= y1 + 1e-6:
        joints = []
        for d, mk, rng in ((np.array([0.0, 0.0, -1.0]), lambda t: np.array([sx * t, y, 300.0]), np.arange(40.0, 100.0, SP)),
                           (np.array([-sx, 0.0, 0.0]), lambda t: np.array([sx * 160.0, y, t]), np.arange(110.0, 190.0, SP))):
            R = runs([h if (h is None or h[1][2] >= 110.0) else None for h in scan(meshes, mk, rng, d)])
            for A, B in zip(R, R[1:]):
                a, b = A[-1], B[0]
                if np.linalg.norm(b[1] - a[1]) > 4.0:
                    continue                    # not neighbours on the surface (an opening or the far side)
                step = min(abs(np.dot(a[2], b[1] - a[1])), abs(np.dot(b[2], a[1] - b[1])))
                crease = float(np.degrees(np.arccos(np.clip(np.dot(a[2], b[2]), -1, 1))))
                gap = max(0.0, np.linalg.norm(b[1] - a[1]) - SP) if a[0] != b[0] else 0.0
                joints.append((step, gap, crease, a[0], b[0], b[1]))
            # a ledge or a tooth: a short run (up to 6 mm) between two runs that are near-parallel (within 35 deg): the offset between those two
            for A, Rr, C in zip(R, R[1:], R[2:]):
                a, c = A[-1], C[0]
                if np.linalg.norm(Rr[-1][1] - Rr[0][1]) > 6.0 or np.linalg.norm(c[1] - a[1]) > 8.0:
                    continue
                ac = float(np.degrees(np.arccos(np.clip(np.dot(a[2], c[2]), -1, 1))))
                if ac > 35.0:
                    continue
                ang = lambda n1, n2: float(np.degrees(np.arccos(np.clip(np.dot(n1, n2), -1, 1))))
                if min(ang(a[2], Rr[0][2]), ang(c[2], Rr[0][2])) < 45.0:
                    continue                    # the middle face of a real ledge stands steep to both surfaces (a grazing ray's facet does not)
                step = max(np.dot(a[2], c[1] - a[1]), np.dot(c[2], a[1] - c[1]), 0.0)   # outward offset only: a convex chamfer reads 0
                joints.append((step, 0.0, ac, a[0] + '|' + Rr[0][0].replace('Side_', ''), c[0], Rr[0][1]))
        if joints:
            w = max(joints, key=lambda j: j[0]); g = max(j[1] for j in joints)
            print('  y %6.1f  joints %2d  worst step %5.2f mm (crease %5.1f) at (%.1f, %.1f, %.1f) %s -> %s   widest seam gap %.2f'
                  % (y, len(joints), w[0], w[2], w[5][0], w[5][1], w[5][2], w[3].replace('Side_', ''), w[4].replace('Side_', ''), g))
            rows_all.append((side, y, w[0], g))
        else:
            print('  y %6.1f  no joints' % y)
        y += dy
print('max step %.2f mm, max gap %.2f mm over %d stations' % (max(r[2] for r in rows_all), max(r[3] for r in rows_all), len(rows_all)))
