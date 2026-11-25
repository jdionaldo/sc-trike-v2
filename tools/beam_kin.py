"""3D kinematics of the version 2 heave spring: lever balls -> slotted beam-end cups -> balance beam on a yaw pin -> carrier rocker on the
cage -> ONE heave shock to an eye under the cage spine. Pure math, no build.
Run: python -I tools/beam_kin.py            (pose table, ratio, stops, 25 x 25 grid, shock check, comparison with sa.beam_pose)

Design (see docs/v2/beam_guide.md):
- CARRIER: a bellcrank that hangs from a double-shear clevis under the spine's front cross tube (pin along x at PC). It carries the beam's
  yaw pin at C and the shock's front eye at S (pin along x, double shear in the carrier fork). It holds the beam sideways and in pitch.
- BEAM: turns on the carrier about the pin axis N (square to the beam, tilted back from vertical). No other freedom: the carrier fixes its
  x position, pitch and roll; the yaw pin lets one end go back while the other goes forward (lean).
- ENDS: a cup block on each 6.8 mm lever ball (x 54) slides in a flat slot of the beam end. The slot faces are square to M = N x X, so each end
  is ONE constraint (ball centre on the slot mid-plane); the block slides along the beam (ball distance grows in lean) and along N (the ball's
  arc and the carrier's arc differ). Carrier angle and beam yaw (2 unknowns) follow from the two wheel travels: fully constrained.
- STOPS: carrier heave stops (bump / droop) and beam yaw stops on the carrier; per-arm stops at the layout travel (+37.9 / -32.9) are needed (tools/arm_stops.py).
Hard points are world mm (Z up, front = -Y), +x side; mirror x for the -x side.
"""
import math, os, sys
import numpy as np

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _swingarm2_snapshot as sa          # frozen copy of the design thread's parts/swingarm2.py (read-only)

# ---------------------------------------------------------------------------------------------------------------------------------------
BEAM_DESIGN = {
    'end_type':        'slot',                  # each beam end: cup block on the lever ball, sliding in a flat slot of the beam end (1 constraint)
    # input from parts/swingarm2.py (7b79b4c): lever ball at rest, 6.8 mm ball on the inboard stud
    'lever_ball':      (54.0, -81.45, 89.63),   # = (LEVER_BALL_X, *lever_top(0))
    'beam_ball':       (54.0, -81.45, 89.63),   # slot centre of the beam end at rest (= lever ball at rest)
    'beam_centre':     (0.0, -81.45, 85.4),     # C: beam yaw pin on the carrier (4.2 mm under the ball line: the beam bows down in the middle)
    'beam_pin_axis':   (0.0, 0.5990, 0.8007),   # N: yaw pin axis at rest, tilted 36.8 deg back from vertical; slots open along N and the beam
    'slot_normal':     (0.0, 0.8007, -0.5990),  # M = N x X: push direction of a beam end (slot faces square to it)
    'carrier_pivot':   (0.0, -50.6, 98.5),      # PC: carrier pin along x, double-shear clevis 15.5 mm under the spine's front cross tube (y -48, z 114)
    'shock_front':     (0.0, -45.6, 81.5),      # S: shock front eye on the carrier (bellcrank arm under PC), pin along x, double shear
    'shock_rear':      None,                    # E: rear eye, pin along x, clevis under the spine (filled below: z shock_rear_z, SHOCK_L from S)
    'shock_rear_z':    97.3,                    # ASSUMED (chosen) E z: swept spring envelope >= 1 mm under the spine underside (ASSUMED 10 mm tube, z 109)
    'shock_rest_len':  sa.SHOCK_L,              # 68.0 eye to eye at rest
    'stop_bump_mm':    sa.BUMP,                 # carrier bump stop at two-wheel +12 (angle printed by main())
    'stop_droop_mm':   sa.DROOP,                # carrier droop stop at two-wheel -8
    'stop_lean_deg':   sa.LEAN_DEG,             # beam yaw stops: 10 deg of lean at the worst heave inside the heave stops
    'heave_stop_tail': (0.0, -42.1, 98.5),      # carrier tail 8.5 mm behind PC (fix round 3: its boss stays clear of the shock's spring cup): bump stop pad above it, droop stop pad below it (clevis plates)
    'yaw_stop_r':      34.5,                    # yaw stops (stop sizing round 1): a PAIR of pads each way at |x| 34.5 on the carrier's upper fork arm
                                                # (inboard of the cage member above the rail at x 40.6..49.4, z up to 70, built model)
    # carrier upper fork arm above the beam bar (carrier frame = world at rest; it turns with the carrier, not with the beam yaw):
    # centre C + n_off * N, along x +-x_half, along M +-m_half (18 mm deep: root bending of the pad force), along N +-n_half
    'yaw_arm':         {'n_off': 13.2, 'x_half': 40.0, 'm_half': 10.0, 'n_half': 6.0},     # 12 x 20 mm (fix round 2: torsion of the pad offset)
    # yaw pad posts hanging from the arm at x 34..44 (both sides), on both faces of the bar: post along M from m0 to m1 (TPU face at m0 - tpu),
    # along N from n0 (bar bottom) to the arm
    'yaw_post':        {'x': (29.5, 39.5), 'm_front': (-28.5, -19.5), 'm_rear': (18.4, 27.4), 'n': (-6.0, 7.2), 'tpu': 2.0},
                                                # TPU faces at M -17.5 / +16.4: bar face at |x| 34.5 + its yaw travel, measured in 3D (bar 2 mm from the post = on the TPU)
    # carrier heave stop: a stock steel dowel D8 x 56 (ISO 8734, hardened, ASSUMED stock) through a 10 mm boss on the carrier tail; its ends pass
    # arc slots in the clevis plates and land on TPU saddles in pad blocks fused under the spine's front cross tube, at |x| 10..28
    'tail_pin':        {'d': 8.0, 'len': 75.0, 'boss_len': 14.8, 'pad_x': (13.4, 35.4), 'tpu_t': 2.0,   # D8 x 75 stock dowel (ASSUMED stock length)
                        # boss walls around the D8 bore (fix round 2): thin on top (spine side, droop load only), thick below (bump tear-out)
                        'wall_top': 2.3, 'wall_bottom': 5.0, 'wall_side': 4.0},     # fix round 3: front side wall 4.0, rear side = the 2 mm round wall
                        # TPU saddle faces square to the tail radius at contact: bump face normal 15.46 deg from vertical, droop 9.28 deg
    'arm_stop_mm':     (-32.9, None),           # per-arm DROOP stop on the lever blade (tools/arm_stops.py); no per-arm bump stop (coordinator, round 2)
}
# ASSUMED shock (a 1/10-class 68-72 mm shock; a real 1/8 shock is 100+ mm and does not fit here)
SHOCK = {'extended': 72.0, 'stroke': 16.0, 'body_od': 12.5, 'spring_od': 16.5, 'eye_width': 6.0, 'eye_bore': 3.0}
BALL_MISALIGN_LIMIT = 25.0      # ASSUMED: allowed tilt of a cup on its ball stud (deg, typical RC ball cup)
KEEPOUT = {'x': (-45.0, 45.0), 'y': (-105.0, -60.0), 'z': (68.0, 105.0)}   # the box the cage keeps free after 7b79b4c (design thread's pose check)
BEAM_INNER_X = 40.0              # beam body checked against the box out to |x| 40 (inboard of the end cups)
FORK = (6.0, 6.5, 13.5)          # half-size of a beam-end fork around its slot centre: along the beam, along M, along N (mm): 3.5 puck + 3 mm plate; closed box fork
CLEVIS_PLATES_X = (9.4, 12.4)   # fix round 3: the carrier clevis plates move out from |x| 6.4..9.4 (built) to 9.4..12.4, so the tail boss can be
                                # 14.8 long (1 mm to each plate) and the plates clear the shock spring (R 8.25) by 1.15 mm
SPINE_UNDER = 108.5             # built spine and clevis underside over the carrier tail (verify-2, built model)
SPINE_Y = (-48.0 - 5.0, 38.0 + 5.0)   # spine between its cross tubes (y -48, y 38), plus the tube radius
CARRIER_R = 4.0                 # ASSUMED half-thickness of the carrier arms (PC-C, PC-S, PC-tail)
PC_HUB_R = 5.0                  # ASSUMED carrier hub around the M3 pivot pin
S_FORK_R = 6.5                  # ASSUMED carrier fork around the shock front eye (y-z radius)
PAD_T = 2.0                     # ASSUMED stop pad thickness above the carrier tail (bump pad on the clevis)
EYE_R = 4.0                     # ASSUMED shock eye OD 8
CAGE_FREE = {'x': (-45.0, 45.0), 'y': (-105.0, -60.0), 'z': (68.0, 105.0)}   # box the cage keeps free after 7b79b4c (thread's pose check)
RAIL_X = 45.0                   # cage lower rail centre line x (NOTES.md, v2 open points); the beam crosses it
BEAM_R = 6.0                    # half-width of the beam bar along N (12 mm, stop sizing round 1)
BAR_HM = 11.0                   # half-depth of the beam bar along M (the push direction) at C: 22 mm deep, tapering to
BAR_HM_END = 6.5                # 13 mm at |x| BEAM_INNER_X (the bending moment of the slot forces falls linearly to the ball)
BAR_OFF_C = -4.0                # section centre moved forward (-M) at C so the bar's rear-bottom edge clears the motor (built b_motor top z 73.2)
bar_hm = lambda xx: BAR_HM + (BAR_HM_END - BAR_HM) * min(1.0, abs(xx) / BEAM_INNER_X)
bar_off = lambda xx: BAR_OFF_C * (1.0 - min(1.0, abs(xx) / BEAM_INNER_X))
bar_m = lambda xx: (bar_off(xx) - bar_hm(xx), bar_off(xx) + bar_hm(xx))     # front (-M) and rear (+M) face of the bar section at x
# b_motor top surface, measured with vertical rays on the built output/model.step (tools/arm_stops.py ray_z): z_top(x, y), front face y -69.53
MOTOR = {'x': (-19.5, 16.5), 'y': (-69.53, -7.21),
         'gx': [-19.0, -15.0, -10.0, -5.0, 0.0, 5.0, 10.0, 15.0, 16.0],
         'gy': [-68.0, -60.0, -50.0, -35.0, -15.0],
         'gz': [[55.32, 63.04, 67.01, 68.80, 69.08, 67.93, 64.98, 58.31, 55.32],
                [55.87, 63.59, 67.56, 69.35, 69.63, 68.48, 65.53, 58.86, 55.87],
                [56.56, 64.27, 68.24, 70.04, 70.32, 69.16, 66.22, 59.55, 56.56],
                [57.59, 65.30, 69.27, 71.07, 71.35, 70.20, 67.25, 60.58, 57.59],
                [58.97, 66.68, 70.65, 72.44, 72.72, 71.57, 68.62, 61.96, 58.97]]}


def motor_top(x, y):
    """Motor top z at (x, y), bilinear in the measured grid, held flat beyond it (x clamped, y clamped to the grid)."""
    xs, ys, zz = MOTOR['gx'], MOTOR['gy'], np.array(MOTOR['gz'])
    x = min(max(x, xs[0]), xs[-1]); y = min(max(y, ys[0]), ys[-1])
    row = [np.interp(x, xs, zz[i]) for i in range(len(ys))]
    return float(np.interp(y, ys, row))
T_MAX = 41.94                   # one-wheel bump reach without a per-arm bump stop (tools/arm_stops.py region, coordinator round 2)
RAIL_TOP_BUILT = 61.4           # cage lower rail top in the built model (CAGE_Z 57; coordinator)
CUP_R = 4.5                     # ASSUMED: outer radius of a cup on a 6.8 mm ball (envelope used against the keep-out walls)
# ---------------------------------------------------------------------------------------------------------------------------------------

V = lambda *a: np.array(a, float)
X = V(1, 0, 0)


def rot(axis, ang):
    """Rotation matrix about the unit vector axis by ang (rad), Rodrigues."""
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K


def _fill(d):
    """Derived hard point: rear eye on z = shock_rear_z, shock_rest_len behind S."""
    S = V(*d['shock_front'])
    dz = d['shock_rear_z'] - S[2]
    d['shock_rear'] = (0.0, round(float(S[1] + math.sqrt(d['shock_rest_len'] ** 2 - dz ** 2)), 2), d['shock_rear_z'])
    return d


_fill(BEAM_DESIGN)


class Kin:
    def __init__(self, d):
        self.d = d
        self.PC, self.C0, self.S0, self.E = (V(*d[k]) for k in ('carrier_pivot', 'beam_centre', 'shock_front', 'shock_rear'))
        self.N0 = V(*d['beam_pin_axis'])
        self.N0 /= np.linalg.norm(self.N0)
        bb = V(*d['beam_ball'])
        self.Bp0 = bb - self.C0                                     # +x beam ball relative to C (beam frame at rest)
        self.Bm0 = self.Bp0 * V(-1, 1, 1)
        self.slot = d.get('end_type', 'slot') == 'slot'
        self.L = d.get('link_len', 0.0)
        self.M0 = np.cross(self.N0, X); self.M0 /= np.linalg.norm(self.M0)   # slot normal = push direction of a beam end (beam frame)
        self._g = np.zeros(2)

    def lever_ball(self, t, s):
        y, z = sa.lever_top(t)
        return V(s * sa.LEVER_BALL_X, y, z)

    def pose(self, q):
        """Carrier angle g (about +x through PC) and beam yaw psi (about N): world points."""
        g, psi = q
        Rg, Rb = rot(X, g), rot(self.N0, psi)
        C = self.PC + Rg @ (self.C0 - self.PC)
        return {'C': C, 'Bp': C + Rg @ Rb @ self.Bp0, 'Bm': C + Rg @ Rb @ self.Bm0, 'S': self.PC + Rg @ (self.S0 - self.PC),
                'N': Rg @ self.N0, 'beam_dir': Rg @ Rb @ X, 'M': Rg @ Rb @ self.M0, 'Rg': Rg}

    def resid(self, q, Ap, Am):
        p = self.pose(q)
        if self.slot:       # ball centre on the mid-plane of the beam-end slot
            return np.array([(Ap - p['Bp']) @ p['M'], (Am - p['Bm']) @ p['M']])
        return np.array([np.linalg.norm(p['Bp'] - Ap) - self.L, np.linalg.norm(p['Bm'] - Am) - self.L])

    def solve(self, tp, tm, q0=None):
        """Newton solve for (g, psi); returns q, pose, Jacobian condition. Raises if the links cannot reach."""
        Ap, Am = self.lever_ball(tp, 1), self.lever_ball(tm, -1)
        q = np.array(self._g if q0 is None else q0, float)
        for _ in range(60):
            r = self.resid(q, Ap, Am)
            J = np.empty((2, 2))
            for i in range(2):
                dq = np.zeros(2); dq[i] = 1e-7
                J[:, i] = (self.resid(q + dq, Ap, Am) - r) / 1e-7
            step = np.linalg.solve(J, -r)
            q = q + np.clip(step, -0.2, 0.2)
            if np.max(np.abs(r)) < 1e-10:
                break
        if np.max(np.abs(self.resid(q, Ap, Am))) > 1e-6:
            raise RuntimeError('no solution at t+ %.1f t- %.1f' % (tp, tm))
        sv = np.linalg.svd(J, compute_uv=False)
        return q, self.pose(q), Ap, Am, sv[-1] / sv[0], sv[-1]

    def state(self, tp, tm, q0=None, from_t=(0.0, 0.0)):
        """Pose at the wheel travels (tp, tm). Without q0 it marches from rest in <= 1 mm steps, so it stays on the assembled branch."""
        if q0 is None:
            n = max(1, int(math.ceil(max(abs(tp - from_t[0]), abs(tm - from_t[1])))))
            q0 = np.zeros(2)
            for i in range(1, n):
                q0 = self.solve(from_t[0] + (tp - from_t[0]) * i / n, from_t[1] + (tm - from_t[1]) * i / n, q0)[0]
        q, p, Ap, Am, cond, smin = self.solve(tp, tm, q0)
        D = math.degrees
        shock = np.linalg.norm(p['S'] - self.E)
        sh = (self.E - p['S']) / shock
        out = {'tp': tp, 'tm': tm, 'q': q, 'g': D(q[0]), 'psi': D(q[1]), 'shock': shock, 'comp': self.d['shock_rest_len'] - shock,
               'shock_pitch': D(math.atan2(sh[2], sh[1])), 'cond': cond, 'smin': smin, 'pose': p}
        # transmission angles (deg, 90 = ideal, 0/180 = dead centre)
        a_sw = sa.arm_swing(tp), sa.arm_swing(tm)
        tr, mis = [], []
        slide = []
        for s, A, B, aw in ((1, Ap, p['Bp'], a_sw[0]), (-1, Am, p['Bm'], a_sw[1])):
            link = p['M'] if self.slot else (B - A) / self.L                 # direction of the force between lever ball and beam end
            a = math.radians(sa.LEVER_TILT) + aw
            lever_dir = V(0, math.sin(a), math.cos(a))              # pivot -> lever top
            tr.append(D(math.acos(np.clip(link @ lever_dir, -1, 1))))
            if self.slot:
                # cup block tilt on the ball: lever stud (x) against the block axis (beam axis); slide of the block in the slot
                mis.append(D(math.acos(min(1, abs(p['beam_dir'] @ X)))))
                slide.append(((A - B) @ p['beam_dir'] * s, (A - B) @ p['N']))
            else:
                mis.append(max(D(math.asin(min(1, abs(link @ X)))), D(math.asin(min(1, abs(link @ p['beam_dir']))))))
        out['tr_lever'] = tr
        out['mis'] = mis
        out['slide'] = slide
        # link vs carrier: angle between mean link force and the carrier arm PC->C
        lk = p['M'] if self.slot else ((p['Bp'] - Ap) + (p['Bm'] - Am)) / (2 * self.L)
        arm = p['C'] - self.PC
        out['tr_carrier'] = D(math.acos(np.clip(lk @ arm / np.linalg.norm(lk) / np.linalg.norm(arm), -1, 1)))
        sa_ = p['S'] - self.PC
        out['tr_shock'] = D(math.acos(np.clip(sh @ sa_ / np.linalg.norm(sa_), -1, 1)))
        out['ball_dist'] = np.linalg.norm(Ap - Am)
        inner = [p['C'] + xx * p['beam_dir'] + mm * p['M'] + c * BEAM_R * p['N'] for xx in np.linspace(-BEAM_INNER_X, BEAM_INNER_X, 9)
                 for mm in bar_m(xx) for c in (-1, 1)]
        out['keepout'] = self.keepout_margin(inner)
        # the beam ends and links are the connection to the levers (exempt from the box, like the lever balls themselves):
        # report how far their cups reach instead (cup bottom above the lowered rail top, rearmost / frontmost cup edge)
        cups = [Ap, Am, p['Bp'], p['Bm']]
        out['beam_over_rail'] = min((p['C'] + s_ * p['beam_dir'] * RAIL_X + b * BAR_HM_END * p['M'] + c * BEAM_R * p['N'])[2]
                                    for s_ in (1, -1) for b in (-1, 1) for c in (-1, 1)) - RAIL_TOP_BUILT
        out['cup_y'] = (min(P[1] for P in cups) - CUP_R, max(P[1] for P in cups) + CUP_R)
        out['cup_z'] = (min(P[2] for P in cups) - CUP_R, max(P[2] for P in cups) + CUP_R)
        # swept envelope points: fork corners at both beam ends, beam bar (|x| <= BEAM_INNER_X, +-BEAM_R along M and N), carrier PC-C-S
        env_beam = []
        for B in (p['Bp'], p['Bm']):
            for a in (-1, 1):
                for b in (-1, 1):
                    for c in (-1, 1):
                        env_beam.append(B + a * FORK[0] * p['beam_dir'] + b * FORK[1] * p['M'] + c * FORK[2] * p['N'])
        for xx in np.linspace(-BEAM_INNER_X, BEAM_INNER_X, 9):
            for b in (-1, 1):
                for c in (-1, 1):
                    env_beam.append(p['C'] + xx * p['beam_dir'] + bar_m(xx)[(b + 1) // 2] * p['M'] + c * BEAM_R * p['N'])
        out['env_beam'] = np.array(env_beam)
        out['env_bar'] = np.array(env_beam[16:])
        fk = out['env_beam'][:16]
        out['fork_low'] = fk[np.abs(fk[:, 0]) <= RAIL_X + 5.0][:, 2].min() - RAIL_TOP_BUILT
        out['env_carrier'] = np.array([self.PC, p['C'], p['S']])
        # carrier and shock solids as (point, radius) samples for the spine clearance
        tail = self.PC + p['Rg'] @ (V(*self.d['heave_stop_tail']) - self.PC)
        tpd = self.d['tail_pin']
        car = [(self.PC, PC_HUB_R), (p['S'], S_FORK_R), (tail, tpd['d'] / 2 + tpd['wall_top'])]
        for Q in (p['C'], p['S'], tail):
            car += [(self.PC + f * (Q - self.PC), CARRIER_R) for f in np.linspace(0, 1, 11)]
        out['car_top'] = max(P[2] + r for P, r in car if SPINE_Y[0] <= P[1] <= SPINE_Y[1]) if any(
            SPINE_Y[0] <= P[1] <= SPINE_Y[1] for P, r in car) else -1e9
        out['car_ytop'] = max(P[2] + r for P, r in car)
        # motor keep-out: carrier samples and the beam bar's lower corners that lie over the motor's plan footprint
        mpts = [(P, r_) for P, r_ in car] + [(q, 0.0) for q in inner]
        over = [(P[2] - r_) - max(motor_top(P[0] + dx, P[1] + dy) for dx in (-r_, 0.0, r_) for dy in (0.0, r_)) for P, r_ in mpts
                if MOTOR['x'][0] - r_ <= P[0] <= MOTOR['x'][1] + r_ and P[1] + r_ >= MOTOR['y'][0]]
        out['motor_margin'] = min(over) if over else 1e9
        out['tail_z'] = tail[2]
        shk = []
        for f in np.linspace(0, 1, 69):
            P = p['S'] + f * (self.E - p['S'])
            d_end = min(f, 1 - f) * shock
            shk.append((P, EYE_R if d_end < EYE_R + 1.0 else SHOCK['spring_od'] / 2))
        out['shock_top'] = max(P[2] + r for P, r in shk if SPINE_Y[0] <= P[1] <= SPINE_Y[1])
        # shock eyes: both pins along x and the carrier turns about x only, so the shock stays in the x = 0 plane
        out['eye_mis'] = math.degrees(math.asin(min(1, abs(sh @ X))))
        return out

    @staticmethod
    def keepout_margin(pts):
        """Smallest distance of the points to the walls of the keep-out box (negative = outside)."""
        m = 1e9
        for P in pts:
            for i, k in enumerate('xyz'):
                lo, hi = KEEPOUT[k]
                m = min(m, P[i] - lo, hi - P[i])
        return m


def lean_pair(phi, heave=0.0):
    return sa.travel_for_lean(phi, 1) + heave, sa.travel_for_lean(phi, -1) + heave


def main():
    d = BEAM_DESIGN
    k = Kin(d)
    D = math.degrees
    r0 = k.state(0, 0)
    rb = k.state(sa.BUMP, sa.BUMP)
    rd = k.state(-sa.DROOP, -sa.DROOP)
    print('BEAM GUIDE (tools/beam_kin.py): carrier rocker on the cage + beam on a yaw pin + slotted end cups (%s); shock S %s -> E %s' %
          (d['end_type'], d['shock_front'], d['shock_rear']))
    print('rest: shock %.2f mm, pitch %+.1f deg (rear end up), carrier arm PC->C %.1f mm, PC->S %.1f mm' %
          (r0['shock'], r0['shock_pitch'], np.linalg.norm(k.C0 - k.PC), np.linalg.norm(k.S0 - k.PC)))
    poses = [('rest', 0, 0), ('bump +12', sa.BUMP, sa.BUMP), ('droop -8', -sa.DROOP, -sa.DROOP),
             ('lean +10', *lean_pair(10)), ('lean -10', *lean_pair(-10)),
             ('lean +10 + bump', *lean_pair(10, sa.BUMP)), ('lean -10 + bump', *lean_pair(-10, sa.BUMP)),
             ('lean +10 + droop', *lean_pair(10, -sa.DROOP)), ('lean -10 + droop', *lean_pair(-10, -sa.DROOP))]
    hdr = ('%-17s %6s %6s | %6s %6s %6s %6s | %6s %6s %6s | %5s %5s %5s %5s | %5s %5s | %5s %5s | %6s %6s' %
           ('pose', 't+x', 't-x', 'shock', 'comp', 'ratio', 'pitch', 'carr', 'yaw', 'balls', 'lev+', 'lev-', 'carr', 'shk',
            'mis+', 'mis-', 'slX', 'slN', 'cond', 'box'))
    print(hdr)
    print('-' * len(hdr))
    res = {}
    for name, tp, tm in poses:
        r = k.state(tp, tm)
        res[name] = r
        mean = (tp + tm) / 2
        ratio = r['comp'] / mean if abs(mean) > 1e-6 else float('nan')
        print('%-17s %+6.1f %+6.1f | %6.2f %+6.2f %6.3f %+6.1f | %+6.2f %+6.2f %6.1f | %5.1f %5.1f %5.1f %5.1f | %5.1f %5.1f | %+5.2f %+5.2f | %6.3f %+6.1f' %
              (name, tp, tm, r['shock'], r['comp'], ratio, r['shock_pitch'], r['g'], r['psi'], r['ball_dist'], r['tr_lever'][0],
               r['tr_lever'][1], r['tr_carrier'], r['tr_shock'], r['mis'][0], r['mis'][1], r['slide'][0][0], r['slide'][0][1], r['cond'], r['keepout']))
    print('columns: comp = shock compression (mm, + = shorter); ratio = comp / mean wheel travel; pitch = shock line above horizontal toward the'
          ' rear (deg); carr = carrier angle (deg about +x); yaw = beam yaw on its pin (deg); balls = lever ball to lever ball (mm); lev = link'
          ' vs lever (90 ideal); carr / shk = link force vs carrier arm, shock vs carrier arm PC->S (90 ideal); mis = cup block tilt on its ball, lever stud vs beam axis (deg); slX / slN = +x cup block slide in the slot along the beam (+ = outboard) / along N (mm);'
          ' cond = Jacobian sigma_min/sigma_max (0 = singular); box = smallest margin of the beam body (C and |x| <= BEAM_INNER_X, half-depth BEAM_R) to the keep-out walls (mm)')
    lr = [res['lean +10']['comp'] - r0['comp'], res['lean -10']['comp'] - r0['comp']]
    print('lean residual (change of shock compression, + = shorter, 10 deg, rest heave): %+.3f / %+.3f mm; with bump %+.3f / %+.3f, with droop %+.3f / %+.3f mm' %
          (lr[0], lr[1], res['lean +10 + bump']['comp'] - rb['comp'], res['lean -10 + bump']['comp'] - rb['comp'],
           res['lean +10 + droop']['comp'] - rd['comp'], res['lean -10 + droop']['comp'] - rd['comp']))

    # ---- ratio vs two-wheel travel ----
    print('two-wheel travel -> shock compression and local ratio d(comp)/dt:')
    line = []
    for t in (-8, -6, -4, -2, 0, 2, 4, 6, 8, 10, 12):
        c1, c2 = k.state(t - 0.05, t - 0.05)['comp'], k.state(t + 0.05, t + 0.05)['comp']
        line.append('%+d: %+.2f (%.3f)' % (t, k.state(t, t)['comp'], (c2 - c1) / 0.1))
    print('  ' + ', '.join(line))

    # ---- stops ----
    g_b, g_d = rb['g'], rd['g']
    psi_max = max(abs(res[n]['psi']) for n in res if n.startswith('lean'))
    print('STOPS: carrier bump stop at %+.2f deg (two-wheel +12), droop stop at %+.2f deg (two-wheel -8); beam yaw stops at +-%.2f deg'
          ' (largest yaw of the 10 deg lean poses inside the heave stops)' % (g_b, g_d, psi_max))
    tail = V(*d['heave_stop_tail']) - k.PC
    for nm, rr in (('bump', rb), ('droop', rd)):
        P = k.PC + rot(X, rr['q'][0]) @ tail
        print('  %-5s stop: carrier tail moves %+.2f mm in z (pad gap at rest, tail point %s)' % (nm, P[2] - k.PC[2] - tail[2],
                                                                                             tuple(round(float(v), 1) for v in V(*d['heave_stop_tail']))))
    print('  yaw stops: beam face at |x| %.0f moves %.2f mm along M at the stop (pad gap at rest, both pads both sides)' %
          (d['yaw_stop_r'], d['yaw_stop_r'] * math.sin(math.radians(psi_max))))
    print('  shock at the stops: bump %.2f mm, droop %.2f mm -> stroke used %.2f mm' % (rb['shock'], rd['shock'], rd['shock'] - rb['shock']))

    # ---- full grid ----
    ts = np.round(np.linspace(-32.9, T_MAX, 25), 3)
    grid = {}
    q_row = np.zeros(2)
    worst = {'cond': (1e9, None), 'tr': (90, None), 'mis': (0, None), 'box_in': (1e9, None), 'box_all': (1e9, None), 'fail': []}
    for tp in ts:
        q = q_row.copy()
        for j, tm in enumerate(ts):
            try:
                r = k.state(tp, tm)
            except RuntimeError:
                worst['fail'].append((tp, tm)); continue
            q = r['q']
            if j == 0:
                q_row = q.copy()
            inside = g_d - 1e-6 <= r['g'] <= g_b + 1e-6 if g_b >= g_d else g_b - 1e-6 <= r['g'] <= g_d + 1e-6
            inside = inside and abs(r['psi']) <= psi_max + 1e-6
            grid[(tp, tm)] = (r, inside)
            if r['cond'] < worst['cond'][0]:
                worst['cond'] = (r['cond'], (tp, tm))
            dev = max(abs(90 - a) for a in r['tr_lever'] + [r['tr_carrier'], r['tr_shock']])
            if inside and (dev > abs(90 - worst['tr'][0]) or worst['tr'][1] is None):
                worst['tr'] = (90 + dev, (tp, tm))
            if inside and max(r['mis']) > worst['mis'][0]:
                worst['mis'] = (max(r['mis']), (tp, tm))
            if r['keepout'] < worst['box_all'][0]:
                worst['box_all'] = (r['keepout'], (tp, tm))
            if inside and r['keepout'] < worst['box_in'][0]:
                worst['box_in'] = (r['keepout'], (tp, tm))
    n_in = sum(1 for v in grid.values() if v[1])
    sl = [sd for r, ins in grid.values() if ins for sd in r['slide']]
    print('  cup block slide inside the stops: along the beam %+.2f .. %+.2f mm, along N %+.2f .. %+.2f mm (slot length = ball + both)' %
          (min(a[0] for a in sl), max(a[0] for a in sl), min(a[1] for a in sl), max(a[1] for a in sl)))
    print('  beam over the lowered rail (x %.0f, top z %.1f), inside the stops: min %.1f mm' %
          (RAIL_X, RAIL_TOP_BUILT, min(r['beam_over_rail'] for r, ins in grid.values() if ins)))
    cy = [r['cup_y'] for r, ins in grid.values() if ins]; cz = [r['cup_z'] for r, ins in grid.values() if ins]
    print('  cup envelope (lever balls and beam ends, radius %.1f) inside the stops: y %.1f .. %.1f, z %.1f .. %.1f' %
          (CUP_R, min(a[0] for a in cy), max(a[1] for a in cy), min(a[0] for a in cz), max(a[1] for a in cz)))
    ins = [r for r, i in grid.values() if i]
    eb = np.vstack([r['env_beam'] for r in ins]); ec = np.vstack([r['env_carrier'] for r in ins]); bar = np.vstack([r['env_bar'] for r in ins])
    fmt = lambda a: 'x %.1f .. %.1f, y %.1f .. %.1f, z %.1f .. %.1f' % (a[:, 0].min(), a[:, 0].max(), a[:, 1].min(), a[:, 1].max(), a[:, 2].min(), a[:, 2].max())
    print('SWEPT ENVELOPE inside the stops (box the design needs): beam with end forks (FORK %s ASSUMED) %s' % (FORK, fmt(eb)))
    print('  beam bar |x| <= %.0f (%.0f wide along N, %.0f deep along M at C tapering to %.0f): %s; carrier hard points PC, C, S: %s' % (BEAM_INNER_X, 2 * BEAM_R, 2 * BAR_HM, 2 * BAR_HM_END, fmt(bar), fmt(ec)))
    m_bar = min(min(bar[:, i].min() - CAGE_FREE[k_][0], CAGE_FREE[k_][1] - bar[:, i].max()) for i, k_ in ((1, 'y'), (2, 'z')))
    print('  beam bar against the cage free box after 7b79b4c (x +-45, y -105..-60, z 68..105): min margin %.1f mm (y, z walls);'
          ' the end forks reach |x| %.1f, outboard of x 45, where only the lever blades and the rail (top z %.1f) are'
          % (m_bar, eb[:, 0].max(), RAIL_TOP_BUILT))
    inb = eb[np.abs(eb[:, 0]) <= RAIL_X + 5.0]
    print('  fork corners inboard of x %.0f (over the rail band): lowest z %.1f (rail top %.1f) -> %s' % (
        RAIL_X + 5.0, inb[:, 2].min(), RAIL_TOP_BUILT, 'clear by %.1f mm' % (inb[:, 2].min() - RAIL_TOP_BUILT) if inb[:, 2].min() - 1.0 >= RAIL_TOP_BUILT
        else 'CLASH: the rail top must be at z <= %.1f under the forks (y %.0f .. %.0f) for 1 mm, or the fork\'s inboard lower corner chamfered' % (
            inb[:, 2].min() - 1.0, inb[:, 1].min(), inb[:, 1].max())))
    ct = max(r['car_top'] for r in ins); st = max(r['shock_top'] for r in ins)
    pad = max(r['tail_z'] for r in ins) + d['tail_pin']['d'] / 2 + d['tail_pin']['wall_top']
    print('SPINE CLEARANCE (built underside z %.1f, y %.0f..%.0f) swept inside the stops: carrier (hub, arms, fork, tail boss top %.1f over the pin axis) top z %.2f -> %.2f mm;'
          ' tail boss top at the bump stop z %.2f -> %.2f mm; shock (eyes OD %.0f, spring OD %.1f over the rest) top z %.2f -> %.2f mm (owner rule >= 1.0)' %
          (SPINE_UNDER, SPINE_Y[0], SPINE_Y[1], d['tail_pin']['d'] / 2 + d['tail_pin']['wall_top'], ct, SPINE_UNDER - ct, pad, SPINE_UNDER - pad, 2 * EYE_R, SHOCK['spring_od'], st,
           SPINE_UNDER - st))
    print('  carrier top anywhere (also ahead of the cross tube): z %.2f' % max(r['car_ytop'] for r in ins))
    print('MOTOR KEEP-OUT (built b_motor top, measured grid, x %.1f..%.1f, y >= %.2f): the carrier and beam-bar sample points over it stay %.2f mm above'
          ' it inside the stops (rule >= 1.0; the solids are checked in 3D in tools/arm_stops.py)' % (*MOTOR['x'], MOTOR['y'][0],
                                                                                                     min(r['motor_margin'] for r in ins)))
    print('  misalignment cones over the whole grid: lever ball cup %.1f deg (stud vs cup block axis), shock eyes %.2f deg (plain pins along x)'
          % (max(max(r['mis']) for r, i in grid.values()), max(r['eye_mis'] for r, i in grid.values())))
    print('GRID t+x, t-x in -32.9..%+.2f (25 x 25 = %d poses): solved %d, no solution %d %s' %
          (T_MAX, len(ts) ** 2, len(grid), len(worst['fail']), [tuple(map(float, f)) for f in worst['fail'][:4]]))
    print('  worst Jacobian condition %.3f at %s (0 = singular); worst transmission deviation from 90 inside the stops: %.1f deg at %s' %
          (worst['cond'][0], worst['cond'][1], worst['tr'][0] - 90, worst['tr'][1]))
    print('  largest cup block tilt inside the stops %.1f deg at %s (ASSUMED limit %.0f)' % (worst['mis'][0], worst['mis'][1], BALL_MISALIGN_LIMIT))
    print('  inside the stops: %d poses; keep-out margin min %.1f mm at %s; whole grid (beyond the stops) %.1f mm at %s' %
          (n_in, worst['box_in'][0], worst['box_in'][1], worst['box_all'][0], worst['box_all'][1]))
    tin = [(tp, tm) for (tp, tm), (r, ins) in grid.items() if ins]
    print('  wheel travel reachable inside the carrier and yaw stops on the grid: %+.1f .. %+.1f mm (grid resolution %.1f mm; per-arm droop stop %+.1f)' %
          (min(min(a) for a in tin), max(max(a) for a in tin), ts[1] - ts[0], d['arm_stop_mm'][0]))
    comp = [r['comp'] for r, ins in grid.values() if ins]
    print('  shock compression inside the stops: %+.2f .. %+.2f mm; shock pitch %.1f .. %.1f deg' %
          (min(comp), max(comp), min(r['shock_pitch'] for r, ins in grid.values() if ins),
           max(r['shock_pitch'] for r, ins in grid.values() if ins)))
    # one-wheel reach with only the carrier and yaw stops (no per-arm stops): corners g = stop, psi = +-stop
    reach = []
    for gt in (g_b, g_d):
        for pt in (psi_max, -psi_max):
            t = np.array([10.0, -10.0]) if gt == g_b else np.array([-10.0, -20.0])
            t = t if pt > 0 else t[::-1]
            for _ in range(30):
                r = k.state(*t); f = np.array([r['g'] - gt, r['psi'] - pt])
                if np.max(np.abs(f)) < 1e-6: break
                J = np.empty((2, 2))
                for i in range(2):
                    dt = np.zeros(2); dt[i] = 0.05
                    r2 = k.state(*(t + dt)); J[:, i] = (np.array([r2['g'] - gt, r2['psi'] - pt]) - f) / 0.05
                t = t + np.clip(np.linalg.solve(J, -f), -5, 5)
            reach.append(t)
    print('  one-wheel reach with the carrier and yaw stops only (corners): %s -> one wheel %+.1f .. %+.1f mm; there is no per-arm bump stop'
          ' (coordinator), the per-arm droop stop at %+.1f is needed (tools/arm_stops.py)' %
          (', '.join('(%+.1f, %+.1f)' % tuple(a) for a in reach), min(a.min() for a in reach), max(a.max() for a in reach), d['arm_stop_mm'][0]))
    # yaw-limited lean at the heave extremes
    for name, h in (('bump', sa.BUMP), ('rest', 0.0), ('droop', -sa.DROOP)):
        lo, hi = 0.0, 20.0
        for _ in range(40):
            m = (lo + hi) / 2
            try:
                ok = abs(k.state(*lean_pair(m, h))['psi']) <= psi_max
            except RuntimeError:
                ok = False
            lo, hi = (m, hi) if ok else (lo, m)
        print('  lean allowed by the yaw stops at %-5s heave: %.1f deg' % (name, lo))

    # ---- shock check ----
    ext, st = SHOCK['extended'], SHOCK['stroke']
    closed = ext - st
    print('SHOCK (ASSUMED %.0f mm extended, %.0f mm stroke, %.0f mm closed, body OD %.1f, spring OD %.1f, eye width %.1f, M%.0f):' %
          (ext, st, closed, SHOCK['body_od'], SHOCK['spring_od'], SHOCK['eye_width'], SHOCK['eye_bore']))
    print('  at rest %.1f, at the bump stop %.2f (%.2f mm before closed), at the droop stop %.2f (%.2f mm before full extension)' %
          (r0['shock'], rb['shock'], rb['shock'] - closed, rd['shock'], ext - rd['shock']))
    smax = max(r['shock'] for r, ins in grid.values() if ins)
    smin = min(r['shock'] for r, ins in grid.values() if ins)
    print('  over every pose inside the stops: %.2f .. %.2f mm -> %s' %
          (smin, smax, 'stops end the travel, not the shock' if smin > closed and smax < ext else 'SHOCK BOTTOMS OR TOPS OUT'))
    tp, tm = lean_pair(10)
    bp, bm, stretch, eye, rear, dl = sa.beam_pose(tp, tm)
    r = res['lean +10']
    print('COMPARISON at lean +10 (t %+.1f / %+.1f): thread sa.beam_pose: ball stretch %.2f mm, shock length change %+.2f mm (beam centred between'
          ' the balls, shock level); this guide: ball stretch %.2f mm (taken by the slots: %+.2f / %+.2f mm per end), shock length change %+.3f mm' %
          (tp, tm, stretch, -dl, r['ball_dist'] - 2 * sa.LEVER_BALL_X, r['slide'][0][0], r['slide'][1][0], -(r['comp'] - r0['comp'])))
    b12 = sa.beam_pose(sa.BUMP, sa.BUMP)[5]
    print('  two-wheel +12: thread %.2f mm of shock compression (ratio %.3f), this guide %.2f mm (ratio %.3f)' % (b12, b12 / 12, rb['comp'], rb['comp'] / 12))
    print('HARD POINTS (+x side, world mm): ' + ', '.join('%s %s' % (kk, tuple(round(float(v), 2) for v in vv))
                                                         for kk, vv in (('PC', k.PC), ('C', k.C0), ('N', k.N0), ('beam ball', k.Bp0 + k.C0),
                                                                        ('lever ball', k.lever_ball(0, 1)), ('S', k.S0), ('E', k.E))))


if __name__ == '__main__':
    main()
