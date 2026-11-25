"""Version 2 heave spring guide, stops and servo holder: final helper design, round 3 (docs/v2/beam_guide.md, docs/v2/arm_stops.md).

The hard points and the verified envelope solids come from tools/beam_kin.py (BEAM_DESIGN) and tools/arm_stops.py (STOP_DESIGN, carrier_solids,
beam_solids); this module turns them into parts: bores, slots, forks, pads and the cage pieces are added here. World frame, mm.

- Beam carrier (printed): hub on the carrier pin at PC, arm to the yoke over the beam with the boss of the cantilevered D8 yaw pin, the yaw arm
  12 x 20 with four posts for the yaw stop pads, arm to the shock fork at S, tail boss with the D8 x 75 heave stop dowel.
- Balance beam (printed): bar 12 x 22 at the centre tapering to 13 at |x| 40, an end fork on each side (3 mm plates square to M) for the pucks.
- Cage pieces (fused into the cage in model.py): carrier clevis under the spine's front cross tube with plates at |x| 9.4 .. 12.4 and arc slots
  for the dowel, a pad block on each side tied into the cross tube and the clevis plate, the shock's rear clevis under the spine.
- TPU parts: two heave saddles per pad block (bump above the dowel, droop below), four yaw stop pads.
- Servo holder: wedge floor plate under the servo with a tongue, and a clip with a strap, one M3 insert at (25.0, -102.2).
"""
import math
import os
import sys
import cadquery as cq
from parts import swingarm2 as k
from parts.swingarm_front import _ball, _box, _cyl, _fuse, _mir, _local, _lbox, _ncyl, BALL_R, BORE_R, PIN_R, CHEEK, lever_ball_c

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools'))
import numpy as np
import beam_kin as bk
import arm_stops as st

V = cq.Vector
D = bk.BEAM_DESIGN
G = 'Chassis/Front suspension'
PRINT, STEEL, SHOCK_RED, POM, TPU = (0.55, 0.57, 0.60), (0.78, 0.80, 0.84), (0.80, 0.15, 0.12), (0.92, 0.92, 0.88), (0.15, 0.15, 0.16)
_KIN = bk.Kin(D)
_REST = _KIN.state(0.0, 0.0)
PC, C, S, E = (tuple(float(v) for v in D[n]) for n in ('carrier_pivot', 'beam_centre', 'shock_front', 'shock_rear'))
TAIL = tuple(float(v) for v in D['heave_stop_tail'])
N, M = tuple(D['beam_pin_axis']), tuple(D['slot_normal'])
YAW_R = st.CARRIER_SECTIONS['yaw_pin_d'] / 2          # D8 yaw pin
DOWEL_R = D['tail_pin']['d'] / 2                      # D8 heave stop dowel
PLATE_X = (14.4, 17.4)                                # carrier clevis plates |x| 14.4 .. 17.4 (helper: 9.4 .. 12.4). Redesign round: the hub between them is the
                                                      # section every stop moment passes; at 16.8 mm long it read safety 0.57 in the one-wheel case
HUB_LEN2 = 2 * (PLATE_X[0] - 1.0)                     # carrier hub length, 1 mm to each plate (shim washers): 26.8
TAIL_W2 = HUB_LEN2 - 2.0                              # tail web and boss width: 24.8
PIN_BOSS_UP2 = 8.0                                    # yaw pin boss height above the yaw arm (helper: 4.0): pin engagement 20 mm instead of 16
POST_X2 = (25.5, 39.5)                                # yaw pad posts and their cross pieces, 14 wide (helper: 29.5 .. 39.5; FEM 1.43 at the post roots)
PUCK_R, PUCK_H = 4.2, 3.4                             # puck radius (0.8 mm wall round the ball), half height along M (the fork's slot is 7.0 wide)
SLOT_END = 0.8                                        # end wall of the fork's slot along N: the puck slides 8.14 mm at the one-wheel corner (+41.9 / -14.7), 7.6 inside the stops
FORK_OUT = 0.2                                        # fork past its nominal outer end (was 1.2: 0.56 mm to the lever blade at lean + bump)
SLOT_H = 3.5
FORK_M = 7.7                                          # fork half size along M: slot 3.5 + plate 4.2 (helper's envelope 6.5 = 3 mm plates; cantilever bending needs 4.2)
PIN_BOSS_R2 = 9.0                                     # yaw pin boss radius (helper: 7.0; FEM peak 19.2 MPa at the boss / arm top with 3 mm of wall)
CPIN_R, HUB_R2 = 3.0, 8.5                             # carrier pin D6 in a hub r 8.5 (helper: M3 in r 6). FEM with M3: 19.6 MPa at the bore (898 N on 3 x 16.8 mm)
ARM_YOKE_W = HUB_LEN2                                     # carrier arm to the yoke, width along x (helper: 10.8). The yaw stops put 8.6 N m about N through this arm
                                                      # to the hub: FEM 57.7 MPa at 10.8 wide; 16.8 is the hub's length between the clevis plates
FORK_ROOT = 3.5                                       # solid fork root inboard of the nominal fork, full section
BUMP_DEG, DROOP_DEG = 15.46, 9.28                     # carrier angle at the heave stops (two-wheel +12 / -8 mm)
SADDLE_T = D['tail_pin']['tpu_t']
PAD_X = (PLATE_X[1] + 1.0, 35.4)                      # TPU saddles |x| 18.4 .. 35.4 (17 mm: 338 N on D8 x 17 = 2.5 MPa, TPU 5 MPa ASSUMED)
BLOCK_Y = (-50.3, -33.6)                             # front face: 1 mm swept to the carrier's rear post piece at full bump
BLOCK_Z0 = 89.0
SLOT_CLEAR = 0.15                                     # radial clearance of the D8 dowel in the pad blocks' arc slot (slot 8.3 wide, reamed or filed to fit)
HUB_CLEAR = 0.2                                       # radial clearance of the D6 carrier pin in the carrier's hub bore (bore 6.3)
# The yaw stop moment goes to whichever path closes first. Twist the carrier can make before a path bears (CALCULATED):
#   dowel in its slots: SLOT_CLEAR / 26.9 mm (half the base between the pad blocks) = 0.32 deg
#   hub on its pin:     2 x HUB_CLEAR / 26.8 mm (hub length)                         = 0.86 deg
# so the dowel bears first with a factor of 2.7 (required: dowel <= half the hub). PRINT AND ASSEMBLY REQUIREMENT: slot 8.3 +0.1/0, hub bore 6.4 +0.1/0.
LUMP_Y, LUMP_Z1 = 10.0, 100.0                         # strut root lump behind each pad block: depth along y, top z
ASSERT_NAMES = ('Beam carrier', 'Balance beam')


def _v(a):
    return V(*(float(q) for q in a))


def _pc(local_origin=None):
    return _local(C if local_origin is None else local_origin)


def _rot_tail(deg, r=None):
    """Tail (dowel) centre (y, z) with the carrier turned by deg about PC (+ = bump: the tail rises)."""
    a = math.radians(deg)
    dy, dz = TAIL[1] - PC[1], TAIL[2] - PC[2]
    return PC[1] + dy * math.cos(a) - dz * math.sin(a), PC[2] + dy * math.sin(a) + dz * math.cos(a)


def _bar(a, b, w, h, off=0.0):
    """Rectangular bar in the x = 0 plane from a to b ((y, z) each): width w along x, depth h, its section moved `off` along the upward normal."""
    L = math.hypot(b[0] - a[0], b[1] - a[1])
    uy, uz = (b[0] - a[0]) / L, (b[1] - a[1]) / L
    vy, vz = -uz, uy
    if vz < 0:
        vy, vz = -vy, -vz
    pts = [(a[0] + (off + q * h / 2) * vy, a[1] + (off + q * h / 2) * vz) for q in (-1, 1)] + [(b[0] + (off + q * h / 2) * vy, b[1] + (off + q * h / 2) * vz) for q in (1, -1)]
    return cq.Workplane('YZ').workplane(offset=-w / 2).polyline(pts).close().extrude(w).val()


def beam_carrier():
    """Bellcrank carrier, one printed part. Sections and places as in tools/arm_stops.py carrier_solids() and beam_solids() (the verified envelope),
    built here from plain prisms and cylinders: the helper's lofted boxes left zero-length edges, lost the two front posts in the fuse and
    could not be meshed for the FEM."""
    cs = st.CARRIER_SECTIONS
    ya, yp, tp = D['yaw_arm'], D['yaw_post'], D['tail_pin']
    pl = _pc()
    n0, n1 = ya['n_off'] - ya['n_half'], ya['n_off'] + ya['n_half']
    yoke_top = pl.toWorldCoords((0.0, 0.0, n1))
    us = math.hypot(S[1] - PC[1], S[2] - PC[2])
    s_root = (S[1] - 2.0 * (S[1] - PC[1]) / us, S[2] - 2.0 * (S[2] - PC[2]) / us)      # into the fork's cylinder, so the two cheeks beside the eye slot stay on the arm
    ws, db = tp['d'] / 2 + tp['wall_side'], tp['d'] / 2 + tp['wall_bottom']
    drop = os.environ.get('FUGU_GUARD_DEMO') == 'drop_post'      # demonstration of the build guard only: leaves out one yaw pad post
    solids = [_cyl((-HUB_LEN2 / 2, PC[1], PC[2]), (1, 0, 0), HUB_R2, HUB_LEN2),                            # hub on the carrier pin
              _bar((PC[1], PC[2]), (yoke_top.y, yoke_top.z), ARM_YOKE_W, cs['arm_c'][1], cs['arm_c'][2]),            # arm to the yoke
              _bar((PC[1], PC[2]), s_root, cs['arm_s'][0], cs['arm_s'][1]),                                          # arm to the shock fork
              _cyl((-cs['s_fork_half'], S[1], S[2]), (1, 0, 0), cs['s_fork_r'], 2 * cs['s_fork_half']),              # shock fork
              _bar((PC[1], PC[2]), (TAIL[1], TAIL[2]), TAIL_W2, 12.0),                                        # tail web, as wide as the boss: it hands the yaw moment to the dowel
              _cyl((-TAIL_W2 / 2, TAIL[1], TAIL[2]), (1, 0, 0), tp['d'] / 2 + tp['wall_top'], TAIL_W2),              # tail boss round the dowel
              _box(-TAIL_W2 / 2, TAIL_W2 / 2, TAIL[1] - ws, TAIL[1] + tp['d'] / 2, TAIL[2] - db, TAIL[2]),          # and its block under the dowel
              _lbox(pl, -ya['x_half'], ya['x_half'], -ya['m_half'], ya['m_half'], n0, n1),                           # yaw arm 12 x 20
              _ncyl(pl, PIN_BOSS_R2, n1, n1 + PIN_BOSS_UP2)]                                                    # yaw pin boss
    for sx in (1, -1):
        x0, x1 = (POST_X2[0], POST_X2[1]) if sx > 0 else (-POST_X2[1], -POST_X2[0])
        for m0, m1 in (yp['m_front'], yp['m_rear']):
            if drop and sx > 0 and m0 < 0:
                continue
            solids.append(_lbox(pl, x0, x1, m0, m1, yp['n'][0], n0 + 0.5))                                           # yaw pad posts
        solids.append(_lbox(pl, x0, x1, yp['m_front'][0], yp['m_rear'][1], n0, n1))     # cross piece that carries both posts of a side on the yaw arm (mine)
    s = _fuse(*solids)
    s = s.cut(_ncyl(pl, YAW_R, n0 - 0.01, n1 + PIN_BOSS_UP2 + 1.0))                     # D8 yaw pin, pressed into arm and boss (16 mm)
    s = s.cut(_cyl((-20.0, PC[1], PC[2]), (1, 0, 0), CPIN_R + HUB_CLEAR, 40.0))                      # carrier pin, a loose running fit on purpose (HUB_CLEAR)
    s = s.cut(_cyl((-20.0, TAIL[1], TAIL[2]), (1, 0, 0), DOWEL_R, 40.0))                      # heave stop dowel
    w = k.SHOCK_EYE_W / 2 + 0.2
    # fork slot for the shock's front eye: round the eye and open to the rear for the shaft's swing only (a 18 x 18 box first notched the
    # underside of the arm to the yoke beside the hub: FEM 32.7 MPa there)
    s = s.cut(_cyl((-w, S[1], S[2]), (1, 0, 0), 5.2, 2 * w)).cut(_box(-w, w, S[1], S[1] + 9.0, S[2] - 6.5, S[2] + 6.5))
    s = s.cut(_cyl((-20.0, S[1], S[2]), (1, 0, 0), BORE_R, 40.0))
    assert s.isValid() and len(s.Solids()) == 1, 'beam carrier is not one solid (%d)' % len(s.Solids())
    return s


def beam():
    """Balance beam, one printed part: tapered bar on the yaw pin, an end fork on each side with a flat slot for the puck."""
    _, bar, _, _ = st.beam_solids(_REST)
    pl = _pc()
    s = bar
    for sx in (1, -1):
        b = lever_ball_c(); b = (sx * b[0], b[1], b[2])
        pf = _local(b)
        f0, f1 = (-bk.FORK[0], bk.FORK[0] + FORK_OUT) if sx > 0 else (-bk.FORK[0] - FORK_OUT, bk.FORK[0])
        fork = _lbox(pf, f0, f1, -FORK_M, FORK_M, -bk.FORK[2], bk.FORK[2])
        m0, m1 = bk.bar_m(bk.BEAM_INNER_X)
        neck = _lbox(pl, min(sx * (bk.BEAM_INNER_X - 1.0), b[0] - sx * (bk.FORK[0] - 0.5)), max(sx * (bk.BEAM_INNER_X - 1.0), b[0] - sx * (bk.FORK[0] - 0.5)),
                     m0, m1, -bk.BEAM_R, bk.BEAM_R)
        s0, s1 = (-PUCK_R - 0.1, f1 + 1.0) if sx > 0 else (f0 - 1.0, PUCK_R + 0.1)
        slot = _lbox(pf, s0, s1, -SLOT_H, SLOT_H, -bk.FORK[2] + SLOT_END, bk.FORK[2] - SLOT_END)          # open toward the lever: the stud moves +-7.6 mm along N in it
        # FEM attempt 1 (3 mm plates on a root as high as the bar): 18.4 MPa, safety 1.09 at the rear plate's inboard root. Attempt 2: plates 4.2 mm
        # (FORK_M) and the fork's solid end 3.5 mm longer inboard over its whole section (FORK_ROOT), so the plates stand on a full-height root
        r0, r1 = (-bk.FORK[0] - FORK_ROOT, -bk.FORK[0] + 0.3) if sx > 0 else (bk.FORK[0] - 0.3, bk.FORK[0] + FORK_ROOT)
        root = _lbox(pf, r0, r1, -FORK_M, FORK_M, -bk.FORK[2], bk.FORK[2])
        s = s.fuse(neck).fuse(root).fuse(fork).cut(slot)
    s = s.clean().cut(_ncyl(pl, YAW_R + 0.05, -bk.BEAM_R - 2.0, bk.BEAM_R + 2.0))
    assert s.isValid() and len(s.Solids()) == 1, 'balance beam is not one solid (%d)' % len(s.Solids())
    return s


def puck(sx=1):
    """Round puck (POM or brass) on the lever ball, free to spin about M in the fork's slot. Neck opening of 20 deg toward the lever."""
    b = V(*lever_ball_c()); m = V(*M)
    s = cq.Solid.makeCylinder(PUCK_R, 2 * PUCK_H, b - m * PUCK_H, m)
    s = s.cut(_ball(BALL_R + 0.1, b)).cut(_cyl(b.toTuple(), (1, 0, 0), 3.2, PUCK_R + 1.0))
    return _mir(s, sx)


def yaw_pads():
    """Four TPU pads on the carrier's posts, 2 mm, facing the bar's front and rear faces at |x| 34.5."""
    yp = D['yaw_post']
    pl = _pc()
    out = []
    for sx in (1, -1):
        x0, x1 = (yp['x'][0], yp['x'][1]) if sx > 0 else (-yp['x'][1], -yp['x'][0])
        for nm, (m0, m1) in (('front', (yp['m_front'][1], yp['m_front'][1] + yp['tpu'])), ('rear', (yp['m_rear'][0] - yp['tpu'], yp['m_rear'][0]))):
            out.append(('Yaw stop pad %s %d' % (nm, sx), _lbox(pl, x0, x1, m0, m1, yp['n'][0], bk.BEAM_R)))
    return out


def _dowel_path(r, a0=-DROOP_DEG, a1=BUMP_DEG, x0=-60.0, x1=60.0, step=2.5):
    """Space swept by a cylinder of radius r on the dowel's arc between the carrier angles a0 and a1, from x0 to x1."""
    n = max(2, int(math.ceil((a1 - a0) / step)) + 1)
    s = None
    for i in range(n):
        y, z = _rot_tail(a0 + (a1 - a0) * i / (n - 1))
        c = _cyl((x0, y, z), (1, 0, 0), r, x1 - x0)
        s = c if s is None else s.fuse(c)
    return s.clean()


def carrier_clevis(top_z):
    """Clevis of the beam carrier under the spine's front cross tube: plates at |x| 9.4 .. 12.4, a bar under the tube, arc slots for the dowel."""
    y0, y1, z0 = -56.6, -33.6, 89.9
    s = _fuse(_box(PLATE_X[0], PLATE_X[1], y0, y1, z0, top_z), _box(-PLATE_X[1], -PLATE_X[0], y0, y1, z0, top_z),
              _box(-PLATE_X[1], PLATE_X[1], y0, y1, top_z - 3.0, top_z))
    s = s.cut(_cyl((-20.0, PC[1], PC[2]), (1, 0, 0), CPIN_R + 0.05, 40.0))
    return s.cut(_dowel_path(DOWEL_R + 0.6, -DROOP_DEG - 1.5, BUMP_DEG + 1.5))


def pad_block(sx=1, top_z=111.5):
    """Pad block (cage) beside the clevis plate: a window for the dowel's swing with a TPU saddle at each end (bump above, droop below).
    It is tied into the clevis plate (x 12.4) and reaches up into the front cross tube: 338 N per block at the bump stop."""
    s = _box(PLATE_X[1], PAD_X[1] + 1.0, BLOCK_Y[0], BLOCK_Y[1], BLOCK_Z0, top_z)
    # ear from the block's top out to the cage post at x 45, y -48 (FEM attempt 1: 21.3 MPa at the block's outer top corner under the cross tube)
    s = s.fuse(_box(PAD_X[1] + 1.0, 41.0, -52.4, -43.6, 106.8, top_z))          # above the dowel's end (its top reaches z 104.8 at the bump stop)
    # (a second ear under the dowel's sweep, z 89 .. 92, was tried and removed: the short stiff path drew the load and read 28.2 MPa, safety 0.71)
    # root lump behind the dowel window for the stop strut (model.py): the strut must not run through the window (attempt 3 did: 68 mm3 into the dowel)
    # (redesign round, C) the lump is flush with the block's underside and as wide as the block, with a wedge up the block's rear face and one
    # along its top: the corner where a narrower lump met the block was the cage's peak (20.5 MPa)
    s = s.fuse(_box(PLATE_X[1], 40.0, BLOCK_Y[1], BLOCK_Y[1] + LUMP_Y, BLOCK_Z0, LUMP_Z1))
    s = s.fuse(_prism([(BLOCK_Y[1], LUMP_Z1), (BLOCK_Y[1], LUMP_Z1 + 6.0), (BLOCK_Y[1] + 6.0, LUMP_Z1)], PLATE_X[1], PAD_X[1] + 1.0))
    # (redesign round, A1') window = a CLOSE arc slot for the dowel (radial clearance SLOT_CLEAR) plus the two saddle pockets beyond the contact
    # lines: the slot walls guide the dowel across its arc, so the yaw stop moment of the carrier (8.6 N m) leaves through the dowel on the
    # 48.8 mm base of the two pad blocks instead of through the 16.8 mm hub
    s = s.cut(_dowel_path(DOWEL_R + SLOT_CLEAR, x0=0.0, x1=60.0))
    for deg, up in ((BUMP_DEG, 1.0), (-DROOP_DEG, -1.0)):
        s = s.cut(_end_half(deg, up, DOWEL_R + SADDLE_T, 0.0, 60.0))
    return _mir(s, sx)


def _end_half(deg, up, r, x0, x1):
    """Half cylinder of radius r round the dowel at carrier angle deg, on the side beyond the contact (up = +1 bump, -1 droop), from x0 to x1."""
    y, z = _rot_tail(deg)
    a = math.radians(deg)
    ty, tz = -math.sin(a) * up, math.cos(a) * up
    keep = cq.Solid.makeBox(x1 - x0 + 10.0, 20.0, 20.0, V(x0 - 5.0, -10.0, 0.0))
    keep = keep.moved(cq.Location(cq.Plane(origin=(0, y, z), xDir=(1, 0, 0), normal=(0, ty, tz))))
    return _cyl((x0, y, z), (1, 0, 0), r, x1 - x0).intersect(keep)


def saddles(sx=1):
    """TPU saddles in the pad block's pockets: half rings 2 mm thick around the dowel at the bump stop (above it) and at the droop stop (below)."""
    out = []
    for nm, deg, up in (('bump', BUMP_DEG, 1.0), ('droop', -DROOP_DEG, -1.0)):
        y, z = _rot_tail(deg)
        ring = _end_half(deg, up, DOWEL_R + SADDLE_T, PAD_X[0], PAD_X[1]).cut(_cyl((PAD_X[0] - 1.0, y, z), (1, 0, 0), DOWEL_R, PAD_X[1] - PAD_X[0] + 2.0))
        out.append(('Heave saddle %s %d' % (nm, sx), _mir(ring, sx)))
    return out


def rear_clevis(top_z):
    """Clevis under the spine for the shock's rear eye (cheeks start 3.2 mm ahead of the eye: the spring ends 5 mm before it)."""
    g = k.SHOCK_EYE_W / 2 + 0.2
    s = _fuse(_box(g, g + CHEEK, E[1] - 3.2, E[1] + 6.5, E[2] - 6.0, top_z), _box(-g - CHEEK, -g, E[1] - 3.2, E[1] + 6.5, E[2] - 6.0, top_z),
              _box(-g - CHEEK, g + CHEEK, E[1] - 3.2, E[1] + 6.5, top_z - 3.0, top_z))
    return s.cut(_cyl((-20.0, E[1], E[2]), (1, 0, 0), BORE_R, 40.0))


def shock_parts():
    """Heave shock (bought, ASSUMED 1/10 class: 68 mm eye to eye at ride height, spring OD 16.5, eye width 6). The lower spring retainer rides on
    the shaft 14 mm behind the front eye (REQUIREMENT on the bought shock, a spacer if needed); body and spring are anchored at the rear eye."""
    g = st.SHOCK_GEOM
    a, b = V(*S), V(*E)
    u = (b - a).normalized()
    eye = lambda c: _cyl((c.x - g['eye_w'] / 2, c.y, c.z), (1, 0, 0), g['eye_r'], g['eye_w']).cut(_cyl((c.x - 5.0, c.y, c.z), (1, 0, 0), BORE_R, 10.0))
    shaft = _fuse(cq.Solid.makeCylinder(g['shaft_r'], g['cup_from_front_eye'] + 2.0, a + u * (g['eye_r'] - 1.0), u),
                  cq.Solid.makeCylinder(g['spring_r'], 2.0, a + u * g['cup_from_front_eye'], u), eye(a))
    L = (b - a).Length
    body = _fuse(cq.Solid.makeCylinder(g['spring_r'], L - g['cup_from_front_eye'] - 2.0 - 5.0 - 5.0, b - u * 5.0, u * -1.0),
                 cq.Solid.makeCylinder(2.5, 5.0 - g['eye_r'] + 1.0, b - u * 5.0, u), eye(b))
    return body, shaft


def hardware():
    """Steel parts of the guide (bought): carrier pin, yaw pin with its clip, heave stop dowel, shock pins."""
    pl = _pc()
    ya = D['yaw_arm']
    n1 = ya['n_off'] + ya['n_half'] + PIN_BOSS_UP2
    return [('Carrier pin', _cyl((-PLATE_X[1], PC[1], PC[2]), (1, 0, 0), CPIN_R, 2 * PLATE_X[1])),
            ('Beam yaw pin', _ncyl(pl, YAW_R, -bk.BEAM_R - 0.5, n1)),
            ('Beam yaw pin clip', _ncyl(pl, st.CARRIER_SECTIONS['clip_r'] + 2.0, -bk.BEAM_R - 0.5 - st.CARRIER_SECTIONS['clip_h'], -bk.BEAM_R - 0.5)),
            ('Heave stop dowel', _cyl((-D['tail_pin']['len'] / 2, TAIL[1], TAIL[2]), (1, 0, 0), DOWEL_R, D['tail_pin']['len'])),
            ('Shock pin front', _cyl((-st.CARRIER_SECTIONS['s_fork_half'], S[1], S[2]), (1, 0, 0), PIN_R, 2 * st.CARRIER_SECTIONS['s_fork_half'])),
            ('Shock pin rear', _cyl((-k.SHOCK_EYE_W / 2 - 0.2 - CHEEK, E[1], E[2]), (1, 0, 0), PIN_R, k.SHOCK_EYE_W + 0.4 + 2 * CHEEK))]


# ------------------------------------------------------------------------------------------------------------------------------ servo holder
SM = st.STOP_DESIGN['SERVO_MOUNT']
SERVO_Z0 = st.SURROUND['servo'][2]                    # servo underside z 28.2
DECK_TOP = lambda y: 25.06 + 0.0699 * (y + 108.6)     # front deck top under the servo (measured on the build: 3.14 mm under the servo at y -108.6)
INSERT = SM['inserts'][0]
FLOOR_REAR_Y = -62.7                                  # 0.4 mm ahead of the rear deck's front face (y -62.3, MEASURED)


def _prism(pts, x0, x1):
    return cq.Workplane('YZ').workplane(offset=x0).polyline(pts).close().extrude(x1 - x0).val()


def servo_floor(sx=1, sink=0.6):
    """Servo holder part 1 (printed): wedge floor plate under the servo (3.14 down to 0.8 mm), a tongue under the clip's leg and a rim at the
    servo's front face. Built `sink` mm into the deck; model.py cuts it by the deck so it sits exactly on it."""
    yf = st.SURROUND['servo'][1]
    ye = yf + (SERVO_Z0 - SM['floor_tmin'] - DECK_TOP(yf)) / 0.0699
    ye = min(ye, FLOOR_REAR_Y)                 # the floor stays on the front deck (the servo moved 3.3 mm rearward)
    fl = _prism([(yf, DECK_TOP(yf) - sink), (ye, DECK_TOP(ye) - sink), (ye, SERVO_Z0), (yf, SERVO_Z0)], *SM['floor_x'])
    tx0, ty0, tx1, ty1 = SM['tongue']
    tg = _prism([(ty0, DECK_TOP(ty0) - sink), (ty1, DECK_TOP(ty1) - sink), (ty1, SERVO_Z0), (ty0, SERVO_Z0)], tx0, tx1)
    rx0, ry0, rx1, ry1, rh = SM['rim']
    rim = _prism([(ry0, DECK_TOP(ry0) - sink), (ry1, DECK_TOP(ry1) - sink), (ry1, SERVO_Z0 + rh), (ry0, SERVO_Z0 + rh)], rx0, rx1)
    # x datum: the case base rests against the clip leg (x 31); a lower case than the Tekin's 27 mm gets a printed shim there so its top face,
    # and the horn plane, stay at x 58.5 (Savox SC-1251MG+: 1.4 mm)
    s = _fuse(fl, tg, rim).cut(_cyl((INSERT[0], INSERT[1], 10.0), (0, 0, 1), 1.7, 20.0))        # M3 clearance through the tongue
    return _mir(s, sx)


def servo_clip(sx=1):
    """Servo holder part 2 (printed): leg on the tongue, arm over the servo's inboard edge, thin strap across the servo top under the cage rail.
    One M3 heat-set insert from below (pocket D4.0 x 8.5); an M3 x 16 through deck and tongue clamps both parts."""
    lx0, ly0, lx1, ly1, lz1 = SM['leg']
    ax0, ay0, ax1, ay1, az0, az1 = SM['arm']
    sx0, sy0, sx1, sy1, sz0, sz1 = SM['strap']
    s = _fuse(_box(lx0, lx1, ly0, ly1, SERVO_Z0, lz1), _box(ax0, ax1, ay0, ay1, az0, az1), _box(sx0, sx1, sy0, sy1, sz0, sz1))
    s = s.cut(_cyl((INSERT[0], INSERT[1], SERVO_Z0 - 0.01), (0, 0, 1), 2.0, 8.5))
    assert s.isValid() and len(s.Solids()) == 1, 'servo clip is not one solid'
    return _mir(s, sx)


def cage_pieces(top_z):
    """Everything of the guide that is fused into the cage."""
    return [carrier_clevis(top_z), rear_clevis(top_z), pad_block(1, top_z), pad_block(-1, top_z)]


def parts():
    """[(name, shape, group, colour)] of the guide parts registered on their own."""
    body, shaft = shock_parts()
    out = [('Balance beam', beam(), G, PRINT), ('Beam carrier', beam_carrier(), G, PRINT),
           ('Heave shock body', body, G, SHOCK_RED), ('Heave shock shaft', shaft, G, STEEL)]
    out += [(n, s, G, STEEL) for n, s in hardware()]
    out += [(n, s, G, TPU) for n, s in yaw_pads()]
    for sx in (1, -1):
        out.append(('Cup block %d' % sx, puck(sx), G, POM))
        out += [(n, s, G, TPU) for n, s in saddles(sx)]
        out += [('Servo floor %d' % sx, servo_floor(sx), G, PRINT), ('Servo clip %d' % sx, servo_clip(sx), G, PRINT)]
    return out
