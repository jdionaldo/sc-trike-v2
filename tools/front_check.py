"""Interference check of the double-wishbone front end (parts/wishbone.py) against the whole built model, at rest and over lean and steer.

Run from the project folder:  python -I tools/front_check.py [output/model.step]
Reads the STEP only (no model rerun, nothing written). Exits 1 if anything overlaps or a boolean fails.

Rest check: every front suspension and steering part, the battery, the receiver, the front wheels and tyres against every other part.
Pose check: body (head) frame, so the body stays put and only the parts that move against it are placed, with the same Locations as
lean_front.build().pose(): arms by psi about their pivot rods, wheel groups by the ball displacement and the kingpin steer, tie rods,
horn, heave shock. Poses: body roll phi x servo horn eta.
"""
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import math
import cadquery as cq
from soma import import_assembly
from parts import wishbone as wb

V = cq.Vector
MIN_VOL = 0.05          # mm3
GROW = 0.2              # mm, bounding box prefilter
PHIS = (-10.0, 0.0, 10.0)
ETAS = (-30.0, 0.0, 30.0)
import os
BUMP = float(os.environ.get('FRONT_BUMP', '0'))      # extra wheel travel of both wheels, mm (bump +, droop -)
REST_GROUPS = ('Chassis/Front_suspension/', 'Chassis/Steering/')
REST_LEAVES = ('b_battery', 'mock_receiver')
REST_PREFIX = ('wheel_f_', 'mock_tire_')


def leaf(key):
    return key.rsplit('/', 1)[-1]


def short(key):
    return leaf(key).replace('_', ' ')


def load(path):
    asm = import_assembly(str(path))
    parts = {}
    for key, n in asm.objects.items():
        if n.obj is None:
            continue
        sh = (n.obj.val() if isinstance(n.obj, cq.Workplane) else n.obj).moved(n.loc)
        parts[key] = sh
    return parts


def bb(shape):
    b = shape.BoundingBox()
    return (b.xmin, b.xmax, b.ymin, b.ymax, b.zmin, b.zmax)


def bb_hit(a, b):
    return all(a[i] - GROW <= b[i + 1] and b[i] - GROW <= a[i + 1] for i in (0, 2, 4))


def overlap(a, b):
    """(volume, bbox text) of the common solid, or raises."""
    c = a.intersect(b)
    v = c.Volume()
    if v <= MIN_VOL:
        return v, ''
    q = c.BoundingBox()
    return v, 'x %.1f..%.1f y %.1f..%.1f z %.1f..%.1f' % (q.xmin, q.xmax, q.ymin, q.ymax, q.zmin, q.zmax)


def moving_groups():
    """{group id: [part names as in lean_front.py]} of everything that moves against the body."""
    g = {}
    for s in (1, -1):
        g[('wheel', s)] = ['Knuckle %d' % s, 'Axle pin %d' % s, 'Axle nut %d' % s, 'wheel f %d' % s, 'mock tire %d' % s, 'Wheel hex hub %d' % s,
                           'Steer stud knuckle %d' % s, 'Steer stud nut knuckle %d' % s] + \
                          ['Ball stud%s %s %d' % (k, n, s) for k in ('', ' nut') for n in ('upper', 'lower')]
        g[('arm_lo', s)] = ['Lower arm %d' % s, 'Rod end lower %d' % s]
        g[('arm_up', s)] = ['Upper arm %d' % s, 'Rod end upper %d' % s, 'Heave shock pin %d' % s]
        g[('tie', s)] = ['Tie rod %d' % s, 'Tie rod end knuckle %d' % s, 'Tie rod end horn %d' % s]
    g[('horn', 0)] = ['Steer horn', 'Steer servo shaft'] + ['Steer stud%s horn %d' % (k, s) for k in ('', ' nut') for s in (1, -1)]
    g[('shock_b', 0)] = ['Heave shock body']
    g[('shock_s', 0)] = ['Heave shock shaft']
    return g


def pose_locs(phi, eta):
    """{group id: Location} in the body frame, list of kinematic problems, shock eye distance."""
    locs, bad = {}, []
    psi = {s: wb.solve_psi(s, phi) for s in (1, -1)}
    if BUMP:      # both wheels BUMP mm up (+) or down (-) on top of the lean pose (FRONT_BUMP environment variable)
        psi = {s: psi[s] - s * __import__('math').degrees(__import__('math').asin(BUMP / wb.ARM_L)) for s in (1, -1)}
    for s in (1, -1):
        hinge = lambda z: wb._loc_axis((s * wb.XI, 0, z), (0, 1, 0), psi[s])
        locs[('arm_lo', s)] = hinge(wb.ZL)
        locs[('arm_up', s)] = hinge(wb.ZU)
        dl = wb.steer_solution(s, psi[s], eta)
        if math.isnan(dl):
            bad.append('steer_solution NaN side %+d (psi %.2f)' % (s, psi[s]))
            continue                      # wheel group and tie rod of this side are left out of this pose
        dx, dz = wb.arm_ball(s, psi[s])
        locs[('wheel', s)] = cq.Location(V(dx, 0, dz)) * wb._loc_axis((s * wb.KX, wb.KY, 0), (0, 0, 1), dl)
        locs[('tie', s)] = wb.tie_rod_loc(s, psi[s], dl, eta)[0]
    locs[('horn', 0)] = wb._loc_axis((0, wb.SERVO_SHAFT_Y, 0), (0, 0, 1), eta)
    lb, ls, eye = wb.shock_locs(psi[-1], psi[1])
    locs[('shock_b', 0)] = lb
    locs[('shock_s', 0)] = ls
    return locs, bad, eye, psi


def main():
    t0 = time.time()
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / 'output/model.step'
    parts = load(path)
    keys = list(parts)
    box = {k: bb(parts[k]) for k in keys}
    print('front_check: %s, %d parts, loaded in %.0f s' % (path, len(keys), time.time() - t0), flush=True)
    findings, errors = 0, 0

    # ---- rest
    target = [k for k in keys if k.startswith(REST_GROUPS) or leaf(k) in REST_LEAVES or leaf(k).startswith(REST_PREFIX)]
    tset = set(target)
    rest, npair = {}, 0
    for a in target:
        for b in keys:
            if a == b or (b in tset and keys.index(b) < keys.index(a)) or not bb_hit(box[a], box[b]):
                continue
            npair += 1
            try:
                v, where = overlap(parts[a], parts[b])
            except Exception as exc:
                errors += 1
                print('REST ERROR %s <> %s: %s' % (short(a), short(b), exc), flush=True)
                continue
            rest[frozenset((a, b))] = v
            if v > MIN_VOL:
                findings += 1
                print('REST %9.2f mm3  %s <> %s  [%s]' % (v, short(a), short(b), where), flush=True)
    t1 = time.time()
    print('rest: %d of %d parts checked, %d boolean pairs, %d overlaps, %.0f s' % (len(target), len(keys), npair, findings, t1 - t0), flush=True)

    # ---- poses
    by_leaf = {leaf(k): k for k in keys}
    group_of, missing = {}, []
    for gid, names in moving_groups().items():
        for n in names:
            k = by_leaf.get(n.replace(' ', '_'))
            if k is None:
                missing.append(n)
            else:
                group_of[k] = gid
    if missing:
        print('MISSING in the STEP (named in lean_front.py): ' + ', '.join(missing))
    eye0 = wb.shock_locs(0.0, 0.0)[2]
    pose_hits, worst = 0, {}
    for phi in PHIS:
        for eta in ETAS:
            tag = 'phi %+3.0f eta %+3.0f' % (phi, eta)
            locs, bad, eye, psi = pose_locs(phi, eta)
            for msg in bad:
                findings += 1
                print('POSE %s  KINEMATICS %s' % (tag, msg), flush=True)
            print('pose %s: psi %+.2f / %+.2f deg (+x / -x), shock eye distance %.2f mm (rest %.2f, %+.2f)'
                  % (tag, psi[1], psi[-1], eye, eye0, eye - eye0), flush=True)
            shp, bx = dict(parts), dict(box)
            mov = [k for k in keys if group_of.get(k) in locs]
            for k in mov:
                shp[k] = parts[k].moved(locs[group_of[k]])
                bx[k] = bb(shp[k])
            done = set()
            for a in mov:
                for b in keys:
                    pair = frozenset((a, b))
                    if a == b or pair in done or group_of.get(a) == group_of.get(b) or not bb_hit(bx[a], bx[b]):
                        continue
                    if b not in shp:
                        continue
                    done.add(pair)
                    try:
                        v, where = overlap(shp[a], shp[b])
                    except Exception as exc:
                        errors += 1
                        print('POSE %s ERROR %s <> %s: %s' % (tag, short(a), short(b), exc), flush=True)
                        continue
                    r = rest.get(pair, 0.0)
                    if v > MIN_VOL and v > r + max(MIN_VOL, 0.25 * r):      # new, or clearly more than at rest
                        pose_hits += 1
                        print('POSE %s %9.2f mm3 (rest %.2f)  %s <> %s  [%s]' % (tag, v, r, short(a), short(b), where), flush=True)
                        w = worst.get(pair)
                        if w is None or v > w[0]:
                            worst[pair] = (v, tag, short(a), short(b), (w[4] if w else 0) + 1)
                        else:
                            worst[pair] = w[:4] + (w[4] + 1,)
    findings += pose_hits
    print('--- summary')
    for v, tag, a, b, n in sorted(worst.values(), reverse=True):
        print('WORST %9.2f mm3 at %s, in %d of %d poses: %s <> %s' % (v, tag, n, len(PHIS) * len(ETAS), a, b))
    print('rest overlaps %d, pose overlaps %d (%d pairs), boolean errors %d, total %.0f s'
          % (findings - pose_hits, pose_hits, len(worst), errors, time.time() - t0))
    sys.exit(1 if findings or errors else 0)


if __name__ == '__main__':
    main()
