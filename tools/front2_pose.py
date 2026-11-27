"""Interference check of the version 2 swing arm front end (parts/swingarm_front.py, kinematics in parts/swingarm2.py) against the whole built
model, at rest and over lean, steering, bump and droop.

Run from the project folder:  python -I tools/front2_pose.py [output/model.step]
Reads the STEP only (no model rerun, nothing written). Exits 1 if anything overlaps or a boolean fails.

Rest check: EVERY pair of parts of the model (tools/collide.py only looks at pairs with a shell, light, cage or rear bridge part in them).
Clearance check: for every moving part, the least distance to its neighbours over all poses; a pair is reported when it comes closer than CLEAR
and closer than it is at rest (joint partners that touch at rest by design therefore do not show up).
Pose check: body frame, so the body stays put and only the parts that move against it are placed: arms (with lever, ball stud and tip pin) and
upper links about their hinges, carriers riding on the arm tips, knuckles with wheels steered about the kingpin, horns, tie rods, balance beam,
heave shock. Poses: body roll phi x servo horn eta x extra travel of both wheels (0, bump, droop).
"""
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import math
import os
import cadquery as cq
from soma import import_assembly
from parts import swingarm2 as sa
sys.path.insert(0, str(ROOT / 'tools'))
import beam_kin as bk          # beam and carrier solver (CALCULATED, docs/v2/beam_guide.md)
import steer_kin as sk         # steering solver, layout B (CALCULATED, docs/v2/steering.md)

V = cq.Vector
MIN_VOL = 0.01          # mm3
GROW = 0.2              # mm, bounding box prefilter for the overlap check
CLEAR = 1.0             # mm: least distance between a moving part and anything else over the whole travel (design rule)
NEAR = CLEAR + 0.2      # mm, bounding box prefilter for the distance check
PHIS = (-sa.LEAN_DEG, 0.0, sa.LEAN_DEG)
ETAS = (-30.0, 0.0, 30.0)
EXTRAS = (0.0, sa.BUMP, -sa.DROOP)
# one-wheel corners of the reachable region without a per-arm bump stop (tools/arm_stops.py region; travel of +x, -x, horn angle)
CORNERS = [(41.9, -14.7), (18.1, -32.9), (16.4, -32.9), (-14.7, 41.9), (-32.9, 18.1), (-32.9, 16.4)]
CORNER_POSES = [(a, b, 0.0) for a, b in CORNERS] + [(a, b, e) for a, b in CORNERS if max(a, b) > 40 for e in (-30.0, 30.0)]
REST_GROUPS = ('Chassis/Front_suspension/',)
REST_LEAVES = ('mock_receiver', 'b_front_pivot_axle')
REST_PREFIX = ('wheel_f_', 'mock_tire_', 'arm_', 'uplink_', 'carrier_', 'knuckle_', 'tip_shaft_', 'mock_axle_', 'w3_arm_pivot_block_')
# Pairs that touch by design (a pin in its bore, a ball in its cup): named here, never hidden by a volume threshold.
WHITELIST = (('Heave_shock_body', 'Heave_shock_shaft'),        # the shaft slides into the body (telescoping stand-in of the bought shock)
             ('mock_rear_axle', 'mock_rear_hub'),              # mock bought parts, the axle sits in the hub (399.5 mm3, inherited; orchestrator's decision)
             ('arm_1', 'Droop_stop_pad_1'), ('arm_-1', 'Droop_stop_pad_-1'),    # the lever blade lands on the droop stop at -32.9 mm by design (distance 0)
             # the heave stop dowel lands on its TPU saddles at two-wheel +12 / -8 mm, the bar on the yaw pads at 10 deg lean (contact by design)
             ) + tuple(('Heave_stop_dowel', 'Heave_saddle_%s_%d' % (n, s)) for n in ('bump', 'droop') for s in (1, -1)) \
             + tuple(('Balance_beam', 'Yaw_stop_pad_%s_%d' % (n, s)) for n in ('front', 'rear') for s in (1, -1))


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


def _axis(p, d, deg):
    """Rotation by deg (right hand) about the axis through p with direction d."""
    return cq.Location(V(*p)) * cq.Location(V(0, 0, 0), V(*d), deg) * cq.Location(V(*p) * -1.0)


def _seg(a0, b0, a1, b1):
    """Location that carries the segment a0 -> b0 onto a1 -> b1 (least rotation)."""
    a0, b0, a1, b1 = V(*a0), V(*b0), V(*a1), V(*b1)
    u0, u1 = (b0 - a0).normalized(), (b1 - a1).normalized()
    ax = u0.cross(u1)
    ang = math.degrees(math.acos(max(-1.0, min(1.0, u0.dot(u1)))))
    loc = cq.Location(a1 - a0)
    if ax.Length > 1e-9 and ang > 1e-9:
        loc = loc * _axis(a0.toTuple(), ax.toTuple(), ang)
    return loc


def moving_groups():
    """{group id: [part names]} of everything that moves against the body."""
    g = {}
    for s in (1, -1):
        g[('arm', s)] = ['arm %d' % s, 'Lever ball stud %d' % s, 'tip shaft %d' % s]
        g[('uplink', s)] = ['uplink %d' % s]
        g[('carrier', s)] = ['Front carrier %d' % s, 'Uplink pin outer %d' % s, 'Kingpin %d' % s]
        g[('knuckle', s)] = ['Front knuckle %d' % s, 'Front axle bolt %d' % s, 'Front axle nut %d' % s, 'Axle sleeve %d' % s, 'Front wheel bushing %d' % s, 'Knuckle ball stud %d' % s, 'wheel f %d' % s, 'mock tire %d' % s]
        g[('horn', s)] = ['Steer horn %d' % s]
        g[('drag', s)] = ['Drag link %d' % s]
        g[('rocker', s)] = ['Rocker %d' % s, 'Rocker ball stud out %d' % s, 'Rocker ball stud in %d' % s]
        g[('tie', s)] = ['Tie rod %d' % s]
        g[('block', s)] = ['Cup block %d' % s]
    g[('beam', 0)] = ['Balance beam']
    g[('bcar', 0)] = ['Beam carrier', 'Beam yaw pin', 'Beam yaw pin clip', 'Shock pin front', 'Heave stop dowel', 'Yaw pin sleeve', 'Tail sleeve 1', 'Tail sleeve -1'] + ['Yaw stop pad %s %d' % (n, s) for n in ('front', 'rear') for s in (1, -1)]
    g[('shock_b', 0)] = ['Heave shock body']
    g[('shock_s', 0)] = ['Heave shock shaft']
    return g


_KIN = bk.Kin(bk.BEAM_DESIGN)
_MX = lambda p, s: (s * p[0], p[1], p[2])


def pose_locs(phi, eta, extra, travel=None):
    """{group id: Location} in the body frame, list of kinematic problems, a text line with the numbers of the pose.
    Steering from tools/steer_kin.py (layout B, written for the +x side: the -x side is its mirror image with the servo angle reversed),
    beam and carrier from tools/beam_kin.py."""
    locs, bad, t, dl = {}, [], {}, {}
    for s in (1, -1):
        t[s] = sa.side_travel(phi, extra, s) if travel is None else travel[0 if s > 0 else 1]
        a = -math.degrees(sa.arm_swing(t[s]))                 # right hand about +x lowers the tip, so tip up is negative
        sh = sa.wheel_shift(t[s])
        locs[('arm', s)] = _axis((0, sa.PIVOT[0], sa.PIVOT[1]), (1, 0, 0), a)
        locs[('uplink', s)] = _axis((0, sa.UPLINK_IN[1], sa.UPLINK_IN[2]), (1, 0, 0), a)
        locs[('carrier', s)] = cq.Location(V(0, sh, t[s]))
        e = s * eta                                           # servo angle in the +x side's sign
        locs[('horn', s)] = _axis((0, sa.SERVO_SHAFT_B[0], sa.SERVO_SHAFT_B[1]), (1, 0, 0), -e)
        rk = sk.rocker(e)
        dl[s] = s * sk.LAY_B.track(e, t[s])
        if math.isnan(rk) or math.isnan(dl[s]):
            bad.append('steering cannot reach on side %+d (travel %+.1f)' % (s, t[s]))
            continue
        locs[('rocker', s)] = _axis((s * sa.ROCKER_PIN[0], sa.ROCKER_PIN[1], 0), (0, 0, 1), s * rk)
        locs[('knuckle', s)] = cq.Location(V(0, sh, t[s])) * _axis((s * sa.KINGPIN[0], sa.KINGPIN[1], 0), (0, 0, 1), dl[s])
        hb, ri, ro = _MX(tuple(sk.horn_B(e)), s), _MX(tuple(sk.R_P + sk.rot(sk.RIN0 - sk.R_P, sk.R_AX, rk)), s), _MX(tuple(sk.LAY_B.inner(e)), s)
        kb = _MX(tuple(sk.LAY_B.kball(s * dl[s], t[s])), s)
        locs[('drag', s)] = _seg(_MX(sa.HORN_BALL_B, s), _MX(sa.ROCKER_IN_BALL, s), hb, ri)
        locs[('tie', s)] = _seg(_MX(sa.ROD_INNER_BALL, s), _MX(sa.KNUCKLE_BALL_B, s), ro, kb)
    st = _KIN.state(t[1], t[-1])
    g, psi = math.degrees(st['q'][0]), math.degrees(st['q'][1])
    car = _axis(sa.CARRIER_PC, (1, 0, 0), g)
    locs[('bcar', 0)] = car
    locs[('beam', 0)] = car * _axis(sa.BEAM_C, sa.BEAM_N, psi)
    for s in (1, -1):
        a0 = (s * sa.LEVER_BALL_X,) + sa.lever_top(0.0)
        a1 = (s * sa.LEVER_BALL_X,) + sa.lever_top(t[s])
        locs[('block', s)] = cq.Location(V(*a1) - V(*a0)) * _axis(a0, (1, 0, 0), g) * _axis(a0, sa.BEAM_N, psi)
    S = tuple(st['pose']['S'])
    locs[('shock_b', 0)] = _seg(sa.SHOCK_E, sa.SHOCK_S, sa.SHOCK_E, S)
    locs[('shock_s', 0)] = _seg(sa.SHOCK_S, sa.SHOCK_E, S, sa.SHOCK_E)
    text = 'travel %+.1f / %+.1f mm, steer %+.2f / %+.2f deg, shock %+.2f mm, carrier %+.1f deg, beam yaw %+.1f deg' % (t[1], t[-1], dl.get(1, float('nan')), dl.get(-1, float('nan')), st['comp'], g, psi)
    return locs, bad, text


def main():
    t0 = time.time()
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / 'output/model.step'
    parts = load(path)
    keys = list(parts)
    box = {k: bb(parts[k]) for k in keys}
    print('front2_pose: %s, %d parts, loaded in %.0f s' % (path, len(keys), time.time() - t0), flush=True)
    findings, errors = 0, 0
    WL = {frozenset(p) for p in WHITELIST}

    # ---- rest
    target = list(keys)
    tset = set(target)
    rest, npair = {}, 0
    for a in target:
        for b in keys:
            if a == b or (b in tset and keys.index(b) < keys.index(a)) or not bb_hit(box[a], box[b]) or frozenset((leaf(a), leaf(b))) in WL:
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
        print('MISSING in the STEP: ' + ', '.join(missing))
    pose_hits, worst, near = 0, {}, {}
    poses = [(phi, eta, ex, None) for phi in PHIS for eta in ETAS for ex in EXTRAS] + [(0.0, e, 0.0, (a, b)) for a, b, e in CORNER_POSES]
    if os.environ.get('FRONT2_REST_ONLY'):
        poses = []          # rest check over all pairs only (after a change that moves nothing)
    for phi, eta, ex, trav in poses:
        if True:
            tag = ('phi %+3.0f eta %+3.0f extra %+3.0f' % (phi, eta, ex)) if trav is None else ('corner %+5.1f / %+5.1f eta %+3.0f' % (trav[0], trav[1], eta))
            locs, bad, text = pose_locs(phi, eta, ex, trav)
            for msg in bad:
                findings += 1
                print('POSE %s  KINEMATICS %s' % (tag, msg), flush=True)
            print('pose %s: %s' % (tag, text), flush=True)
            shp, bx = dict(parts), dict(box)
            mov = [k for k in keys if group_of.get(k) in locs]
            for k in mov:
                shp[k] = parts[k].moved(locs[group_of[k]])
                bx[k] = bb(shp[k])
            done = set()
            for a in mov:
                for b in keys:
                    pair = frozenset((a, b))
                    if a == b or pair in done or group_of.get(a) == group_of.get(b) or frozenset((leaf(a), leaf(b))) in WL:
                        continue
                    if not bb_hit(bx[a], bx[b]):
                        if b in shp and all(bx[a][i] - NEAR <= bx[b][i + 1] and bx[b][i] - NEAR <= bx[a][i + 1] for i in (0, 2, 4)):
                            done.add(pair)
                            try:
                                d = shp[a].distance(shp[b])
                            except Exception as exc:
                                errors += 1; print('POSE %s DISTANCE ERROR %s <> %s: %s' % (tag, short(a), short(b), exc), flush=True); continue
                            if d < CLEAR and (pair not in near or d < near[pair][0]): near[pair] = (d, tag, a, b)
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
                    if v <= MIN_VOL:
                        try:
                            d = shp[a].distance(shp[b])
                        except Exception as exc:
                            errors += 1; print('POSE %s DISTANCE ERROR %s <> %s: %s' % (tag, short(a), short(b), exc), flush=True); d = CLEAR
                        if d < CLEAR and (pair not in near or d < near[pair][0]): near[pair] = (d, tag, a, b)
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
        print('WORST %9.2f mm3 at %s, in %d of %d poses: %s <> %s' % (v, tag, n, len(poses), a, b))
    tight = 0
    for pair, (d, tag, a, b) in sorted(near.items(), key=lambda kv: kv[1][0]):
        if pair in worst:
            continue                      # already reported as an overlap
        try:
            d0 = parts[a].distance(parts[b])
        except Exception:
            d0 = float('nan')
        if d < d0 - 0.05:                 # closer than at rest: joint partners that touch by design stay out
            tight += 1
            print('CLEAR %6.2f mm (rest %.2f) at %s: %s <> %s' % (d, d0, tag, short(a), short(b)))
    findings += tight
    print('rest overlaps %d, pose overlaps %d (%d pairs), pairs closer than %.1f mm in motion %d, boolean errors %d, total %.0f s'
          % (findings - pose_hits - tight, pose_hits, len(worst), CLEAR, tight, errors, time.time() - t0))
    sys.exit(1 if findings or errors else 0)


if __name__ == '__main__':
    main()
