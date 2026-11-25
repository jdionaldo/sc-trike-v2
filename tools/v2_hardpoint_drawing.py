"""Hard-point drawings of the V2 swing-arm front end: side, front and top view as PNG in docs/v2/.

Grey: outline of the built body (Print_shell), the cage (Cage_2) and the decks, taken from the built model of commit 7b79b4c (MODEL_STEP).
Colour: hard points and links of parts/swingarm2.py as they are now (rest, +37.9 / -32.9 mm travel ghosted).
Run:  python -I tools/v2_hardpoint_drawing.py
"""
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tools'))

import numpy as np
import cadquery as cq
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.patches import Circle, Rectangle, Polygon
from matplotlib.lines import Line2D
from soma import import_assembly
import _swingarm2_snapshot as S      # frozen copy of the hard points (coordinator, Nov 23); same functions as parts/swingarm2.py

COMMIT = "7b79b4c"
MODEL_STEP = "/Users/johndionaldo/GitHub/sc-trike-v2/output/model.step"     # built model of commit 7b79b4c, read only
OUT = os.path.join(ROOT, "docs", "v2")
os.makedirs(OUT, exist_ok=True)
G = S.GROUND_Z
T_UP, T_DN = S.travel_for_lean(S.LEAN_DEG, 1) + S.BUMP, S.travel_for_lean(S.LEAN_DEG, -1) - S.DROOP   # 37.9 / -32.9
T_LEAN = (S.travel_for_lean(S.LEAN_DEG, 1), S.travel_for_lean(S.LEAN_DEG, -1))
ARM_C = "#d62728"; LINK_C = "#1f77b4"; LEV_C = "#2ca02c"; STEER_C = "#9467bd"; SHOCK_C = "#ff7f0e"; KO_C = "#e377c2"; TYRE_C = "#111111"

# ---------------------------------------------------------------- body outlines from the built model
def leaves(n, loc, path):
    l = loc * n.loc
    if n.obj is not None:
        yield path + [n.name], n.obj, l
    for c in n.children:
        yield from leaves(c, l, path + [n.name])


def mesh(obj, loc):
    s = obj.val() if hasattr(obj, "val") else obj
    vs, tr = s.moved(loc).tessellate(0.4, 0.3)
    return np.array([[v.x, v.y, v.z] for v in vs]), np.array(tr)


asm = import_assembly(MODEL_STEP)
SHELL, CAGE, DECK = [], [], []
deck_front_v = None
for p, o, l in leaves(asm, cq.Location(), []):
    if len(p) < 2:
        continue
    v, t = mesh(o, l)
    if p[1] == "Print_shell":
        SHELL.append((v, t))
    elif p[1] == "Cage_2":
        CAGE.append((v, t))
    elif "Frame_and_decks" in p and p[-1].startswith("w3_deck"):
        DECK.append((v, t))
        if p[-1] == "w3_deck_front":
            deck_front_v = v

# ground clearance of the front deck: upright and at 10 deg lean (low side)
cl_up = deck_front_v[:, 2].min() - G
cl_lean = np.array([S.clearance_at_lean(x, z) for x, z in deck_front_v[:, [0, 2]]])
i_lean = int(cl_lean.argmin())
i_up = int(deck_front_v[:, 2].argmin())
cl_lean_v = cl_lean[i_lean]
P_UP, P_LEAN = deck_front_v[i_up], deck_front_v[i_lean]
print("deck front clearance upright %.1f mm at (%.0f, %.0f, %.0f); at %g deg lean %.1f mm at (%.0f, %.0f, %.0f)" % (
    cl_up, *P_UP, S.LEAN_DEG, cl_lean_v, *P_LEAN))

AX = {"side": (1, 2), "front": (0, 2), "top": (0, 1)}


def draw_body(ax, view):
    a, b = AX[view]
    clip = Rectangle((-1000, -1000), 1030, 3000, transform=ax.transData) if view == "side" else None
    for groups, col, z in ((SHELL, "#ececec", 1), (DECK, "#a9a9a9", 3), (CAGE, "#c4c4c4", 2)):
        for v, t in groups:
            pc = PolyCollection(v[t][:, :, [a, b]], facecolors=col, edgecolors=col, linewidths=0.3, zorder=z)
            ax.add_collection(pc)
            if clip is not None:
                pc.set_clip_path(clip)
    # thin outline of the shell silhouette
    for v, t in SHELL:
        pc = PolyCollection(v[t][:, :, [a, b]], facecolors="none", edgecolors="#d0d0d0", linewidths=0.15, zorder=1.5)
        ax.add_collection(pc)
        if clip is not None:
            pc.set_clip_path(clip)


LABS = []


def lab(ax, text, xy, off, color, ha="left", fs=11, weight="bold"):
    """Queue a label; flush_labels() stacks them in a column left or right of the drawing with leader lines."""
    LABS.append((text, xy, color, off[0] < 0 if abs(off[0]) > 0 else False))


def flush_labels(ax, xl, xr, mid, fig, force_side=None):
    """Place the queued labels in two columns (right edge at xl, left edge at xr), spread so that they do not overlap."""
    ax.apply_aspect()
    bb = ax.get_position(); ylo, yhi = ax.get_ylim(); xlo, xhi = ax.get_xlim()
    in_per_y = bb.height * fig.get_figheight() / (yhi - ylo)
    for left in (True, False):
        items = [l for l in LABS if (l[1][0] < mid) == left]
        items.sort(key=lambda l: l[1][1])
        hs = [(l[0].count("\n") + 1) * 0.215 / in_per_y + 3.0 for l in items]
        pos = []
        for k, l in enumerate(items):
            y = l[1][1]
            if k and y - pos[-1] < (hs[k] + hs[k - 1]) / 2:
                y = pos[-1] + (hs[k] + hs[k - 1]) / 2
            pos.append(y)
        top = yhi - 2
        for k in range(len(pos) - 1, -1, -1):          # pull back down if the stack ran out of the top
            lim = top - hs[k] / 2
            if pos[k] > lim:
                pos[k] = lim
            top = pos[k] - hs[k] / 2
        for l, y in zip(items, pos):
            x = xl if left else xr
            ax.annotate(l[0], xy=l[1], xytext=(x, y), textcoords="data", color=l[2], fontsize=11, fontweight="bold",
                        ha="right" if left else "left", va="center", zorder=20, annotation_clip=False,
                        arrowprops=dict(arrowstyle="-", color=l[2], lw=1.0, shrinkA=0, shrinkB=3),
                        bbox=dict(boxstyle="round,pad=0.2", fc="white", ec=l[2], lw=0.8))
    LABS.clear()


def dot(ax, xy, color, s=60, marker="o", zorder=15, **kw):
    ax.scatter([xy[0]], [xy[1]], s=s, c=color, marker=marker, zorder=zorder, edgecolors="white", linewidths=0.8, **kw)


def seg(ax, p, q, color, lw=2.5, ls="-", zorder=10, alpha=1.0):
    ax.plot([p[0], q[0]], [p[1], q[1]], color=color, lw=lw, ls=ls, zorder=zorder, alpha=alpha, solid_capstyle="round")


def finish(ax, title, xl, yl, xlim, ylim, legend_loc="upper left"):
    ax.set_xlim(*xlim); ax.set_ylim(*ylim); ax.set_aspect("equal")
    ax.set_xlabel(xl, fontsize=13); ax.set_ylabel(yl, fontsize=13)
    ax.set_title(title, fontsize=17, fontweight="bold", loc="left")
    ax.grid(True, color="#e8e8e8", lw=0.6, zorder=0); ax.set_axisbelow(True)
    ax.tick_params(labelsize=11)


def legend(ax, extra=(), loc="upper left", **kw):
    h = [Line2D([], [], color="#ececec", lw=9, label="body shell (Print_shell)"), Line2D([], [], color="#c4c4c4", lw=9, label="cage (Cage_2)"),
         Line2D([], [], color="#a9a9a9", lw=9, label="decks"),
         Line2D([], [], color=ARM_C, lw=3, label="swing arm (pivot to wheel centre)"), Line2D([], [], color=LINK_C, lw=3, label="upper link"),
         Line2D([], [], color=LEV_C, lw=3, label="spring lever + balance beam"), Line2D([], [], color=SHOCK_C, lw=3, label="heave shock 68 mm"),
         Line2D([], [], color=STEER_C, lw=3, label="servo horn + tie rod"), Line2D([], [], color=KO_C, lw=2, ls="--", label="keep-out box"),
         Line2D([], [], color=TYRE_C, lw=2, label="tyre R 56.2 x 62: rest"), Line2D([], [], color="#777777", lw=2, ls="--", label="tyre ghost: +37.9 / -32.9 mm")]
    ax.figure.legend(handles=h + list(extra), loc="lower center", ncol=4, fontsize=12, frameon=True)


def c(p):
    return "(%.1f, %.1f)" % p


# ---------------------------------------------------------------- geometry shared by the views
PY, PZ = S.PIVOT
TY, TZ = S.TIP
ws = {0.0: S.wheel_shift(0.0), T_UP: S.wheel_shift(T_UP), T_DN: S.wheel_shift(T_DN)}
wheel = {t: (TY + ws[t], TZ + t) for t in ws}
levtop = {t: S.lever_top(t) for t in ws}
BEAM_C = (levtop[0.0][0], levtop[0.0][1])        # beam centre at rest: cups sit directly on the balls at the lever tops
BX = S.LEVER_BALL_X
FE = (BEAM_C[0] + S.BEAM_EYE_BACK, BEAM_C[1])        # shock front eye, 3 mm behind the ball line
EYE = (FE[0] + S.SHOCK_L, FE[1])                      # rear eye (computed as in beam_pose of the snapshot: front eye + 68 mm along y)
SPINE_Y = (-48.0, 38.0)
KB = S.KNUCKLE_BALL_YZ
HB = S.HORN_BALL
KO = dict(x=64.0, y=(-110.0, -60.0), z=(74.0, 106.0))
GREY = "#777777"

# ================================================================ SIDE VIEW (from +x, front on the left)
fig, ax = plt.subplots(figsize=(22, 11.5), dpi=100)
draw_body(ax, "side")
ax.axhline(G, color="k", lw=2, zorder=5); ax.text(-60, G - 5.5, "ground  z = %.1f" % G, fontsize=12, va="top")
ax.add_patch(Rectangle((KO["y"][0], KO["z"][0]), KO["y"][1] - KO["y"][0], KO["z"][1] - KO["z"][0], fill=False, ec=KO_C, lw=2, ls="--", zorder=9))
lab(ax, "keep-out box |x|<=64\ny -110..-60, z 74..106", (-85, KO["z"][1]), (0, 62), KO_C, ha="center")
for t, ls, col in ((0.0, "-", TYRE_C), (T_UP, "--", GREY), (T_DN, "--", GREY)):
    ax.add_patch(Circle(wheel[t], S.TYRE_R, fill=False, ec=col, lw=2.2 if t == 0 else 1.8, ls=ls, zorder=8))
    seg(ax, (PY, PZ), wheel[t], ARM_C, lw=4 if t == 0 else 2, ls="-" if t == 0 else "--", alpha=1 if t == 0 else 0.55)
    dot(ax, wheel[t], ARM_C, s=70 if t == 0 else 35, alpha=1 if t == 0 else 0.6)
    out = (S.UPLINK_OUT[1] + ws[t], S.UPLINK_OUT[2] + t)
    seg(ax, (S.UPLINK_IN[1], S.UPLINK_IN[2]), out, LINK_C, lw=3.5 if t == 0 else 1.8, ls="-" if t == 0 else "--", alpha=1 if t == 0 else 0.55)
    dot(ax, out, LINK_C, s=55 if t == 0 else 28, alpha=1 if t == 0 else 0.6)
    seg(ax, (PY, PZ), levtop[t], LEV_C, lw=4 if t == 0 else 2, ls="-" if t == 0 else "--", alpha=1 if t == 0 else 0.55)
    dot(ax, levtop[t], LEV_C, s=55 if t == 0 else 28, alpha=1 if t == 0 else 0.6)
# lever-top arc
tt = np.linspace(T_DN, T_UP, 60); arc = np.array([S.lever_top(t) for t in tt])
ax.plot(arc[:, 0], arc[:, 1], color=LEV_C, lw=2, ls=":", zorder=11)
# beam end, shock
be = levtop[0.0]
dot(ax, be, LEV_C, s=50, marker="s")
seg(ax, FE, EYE, SHOCK_C, lw=5); dot(ax, EYE, SHOCK_C, s=80, marker="D")
# servo, horn, tie rod (parallel to the arm)
dot(ax, (S.SERVO_SHAFT[0], S.SERVO_SHAFT[1]), STEER_C, s=100, marker="s")
seg(ax, (S.SERVO_SHAFT[0], S.SERVO_SHAFT[1]), (HB[1], HB[2]), STEER_C, lw=4); dot(ax, (HB[1], HB[2]), STEER_C, s=70)
seg(ax, (HB[1], HB[2]), KB, STEER_C, lw=3); dot(ax, KB, STEER_C, s=70)
dot(ax, (S.UPLINK_IN[1], S.UPLINK_IN[2]), LINK_C, s=70, marker="s")
dot(ax, (PY, PZ), ARM_C, s=120, marker="s")
ax.axhline(S.CAGE_RAIL_TOP, color='#555555', lw=1.2, ls=(0,(6,3)), zorder=6); ax.text(-60, S.CAGE_RAIL_TOP + 2, 'cage lower rail top z 64.4 (lowered)', fontsize=11, color='#333333')
seg(ax, (SPINE_Y[0], S.SPINE_Z), (SPINE_Y[1], S.SPINE_Z), '#555555', lw=3, ls='--')
for yy in SPINE_Y: dot(ax, (yy, S.SPINE_Z), '#555555', s=40, marker='s')
ax.text(-47, S.SPINE_Z + 4, 'cage spine z 114', fontsize=11, color='#333333')
# labels
lab(ax, "arm pivot (y, z) = %s" % c(S.PIVOT), S.PIVOT, (-40, -110), ARM_C)
lab(ax, "arm tip = wheel centre, rest\n%s" % c((TY, TZ)), wheel[0.0], (-110, 100), ARM_C, ha="center")
lab(ax, "wheel centre +37.9 mm\n(%.1f, %.1f)" % wheel[T_UP], wheel[T_UP], (-60, 150), GREY, ha="center")
lab(ax, "wheel centre -32.9 mm\n(%.1f, %.1f)" % wheel[T_DN], wheel[T_DN], (-60, -120), GREY, ha="center")
lab(ax, "upper link, inner hinge\n(y, z) = (%.1f, %.1f)  x %.1f" % (S.UPLINK_IN[1], S.UPLINK_IN[2], S.UPLINK_IN[0]), (S.UPLINK_IN[1], S.UPLINK_IN[2]), (20, 85), LINK_C)
lab(ax, "upper link, outer hinge, rest\n(%.1f, %.1f)  x %.1f" % (S.UPLINK_OUT[1], S.UPLINK_OUT[2], S.UPLINK_OUT[0]), (S.UPLINK_OUT[1], S.UPLINK_OUT[2]), (-120, 60), LINK_C, ha="center")
lab(ax, "lever top, rest (%.1f, %.1f)\nlever R 55, tilt 40 deg" % levtop[0.0], levtop[0.0], (115, -40), LEV_C)
lab(ax, "lever top +37.9 (%.1f, %.1f)" % levtop[T_UP], levtop[T_UP], (-5, -95), LEV_C, ha="center")
lab(ax, "lever top -32.9 (%.1f, %.1f)" % levtop[T_DN], levtop[T_DN], (-60, 70), LEV_C, ha="center")
lab(ax, "balance beam end, rest\n(%.1f, %.1f)" % be, be, (15, 100), LEV_C)
lab(ax, "shock rear eye (computed)\n(y, z) = (%.1f, %.1f)" % EYE, EYE, (60, -70), SHOCK_C)
lab(ax, "shock 68 mm, eye to eye", ((FE[0] + EYE[0]) / 2, (FE[1] + EYE[1]) / 2), (30, -85), SHOCK_C)
lab(ax, "servo shaft (y, z) = %s" % c(S.SERVO_SHAFT), S.SERVO_SHAFT, (70, -75), STEER_C)
lab(ax, "horn ball (y, z) = %s" % c((HB[1], HB[2])), (HB[1], HB[2]), (-95, 85), STEER_C, ha="center")
lab(ax, "tie rod knuckle ball (y, z) = %s\nx %.1f; rod parallel to the arm" % (c(KB), S.KNUCKLE_BALL[0]), KB, (-30, 105), STEER_C, ha="center")
# ground clearance
dot(ax, (P_UP[1], P_UP[2]), "k", s=50, marker="v")
ax.annotate("", xy=(P_UP[1] - 28, P_UP[2]), xytext=(P_UP[1] - 28, G), arrowprops=dict(arrowstyle="<->", color="k", lw=1.6), zorder=12)
lab(ax, "deck front %.1f mm upright\n(%.1f at 10 deg lean)\n(measured, build 7b79b4c)" % (cl_up, cl_lean_v), (P_UP[1] - 28, (P_UP[2] + G) / 2), (-1, 0), "k")
fig.text(0.5, 0.105, "SIDE VIEW from +x   (front = left, -y).  Grey = built model of commit 7b79b4c; ground clearance measured on it, shock rear eye computed.  View cropped to the front half.", fontsize=13, ha="center")
finish(ax, "V2 hard points - side view (from +x) - build " + COMMIT, "y  (mm)   front <-  ->  rear", "z  (mm)", (-450, 200), (-32, 235))
legend(ax)
fig.subplots_adjust(left=0.05, right=0.99, top=0.95, bottom=0.17)
flush_labels(ax, -312, 36, -150, fig); fig.savefig(os.path.join(OUT, "hardpoints_side.png")); plt.close(fig)

# ================================================================ FRONT VIEW (from -y, +x right)
fig, ax = plt.subplots(figsize=(22, 11.5), dpi=100)
draw_body(ax, "front")
XT = S.HALF_TRACK
ax.axhline(G, color="k", lw=2, zorder=5); ax.text(220, G - 5.5, "ground  z = %.1f" % G, fontsize=12, va="top")
ax.add_patch(Rectangle((-KO["x"], KO["z"][0]), 2 * KO["x"], KO["z"][1] - KO["z"][0], fill=False, ec=KO_C, lw=2, ls="--", zorder=9))
lab(ax, "keep-out box |x|<=64, z 74..106", (-KO["x"], KO["z"][1]), (-70, 55), KO_C, ha="center")
# tyres (rest both sides, ghosts on +x) as rectangles of the tyre section
for sgn in (1, -1):
    ax.add_patch(Rectangle((sgn * XT - S.TYRE_W / 2, TZ - S.TYRE_R), S.TYRE_W, 2 * S.TYRE_R, fill=False, ec=TYRE_C, lw=2.2, zorder=8))
    dot(ax, (sgn * XT, TZ), ARM_C, s=70)
for t in (T_UP, T_DN):
    ax.add_patch(Rectangle((XT - S.TYRE_W / 2, TZ + t - S.TYRE_R), S.TYRE_W, 2 * S.TYRE_R, fill=False, ec=GREY, lw=1.8, ls="--", zorder=8))
    dot(ax, (XT, TZ + t), ARM_C, s=30, alpha=0.6)
    seg(ax, (S.ARM_PLATES_X[1], PZ), (XT, TZ + t), ARM_C, lw=1.6, ls="--", alpha=0.55)
for sgn in (1, -1):
    ax.plot([sgn * S.PIVOT_BLOCK_X[0], sgn * S.ARM_PLATES_X[1]], [PZ, PZ], color=ARM_C, lw=5, zorder=11, solid_capstyle="round")   # pivot axle
    for xp in S.ARM_PLATES_X:
        seg(ax, (sgn * xp, PZ), (sgn * XT, TZ), ARM_C, lw=2.2)
    dot(ax, (sgn * S.UPLINK_IN[0], S.UPLINK_IN[2]), LINK_C, s=70, marker="s")
    seg(ax, (sgn * S.UPLINK_IN[0], S.UPLINK_IN[2]), (sgn * S.UPLINK_OUT[0], S.UPLINK_OUT[2]), LINK_C, lw=3.5)
    dot(ax, (sgn * S.UPLINK_OUT[0], S.UPLINK_OUT[2]), LINK_C, s=55)
    seg(ax, (sgn * S.LEVER_X, PZ), (sgn * S.LEVER_X, levtop[0.0][1]), LEV_C, lw=4)
    seg(ax, (sgn * S.LEVER_X, levtop[0.0][1]), (sgn * BX, levtop[0.0][1]), LEV_C, lw=3, ls=':')
    dot(ax, (sgn * BX, levtop[0.0][1]), LEV_C, s=55, marker="s")
    dot(ax, (sgn * S.KNUCKLE_BALL[0], S.KNUCKLE_BALL[2]), STEER_C, s=70, marker='^')
    dot(ax, (sgn * S.HORN_X, HB[2]), STEER_C, s=70)
    seg(ax, (sgn * S.HORN_X, S.SERVO_SHAFT[1]), (sgn * S.HORN_X, HB[2]), STEER_C, lw=4)
    ax.plot([sgn * S.SERVO_X[0], sgn * S.SERVO_X[1]], [S.SERVO_SHAFT[1]] * 2, color=STEER_C, lw=6, zorder=11, solid_capstyle="round", alpha=0.7)
seg(ax, (-BX, levtop[0.0][1]), (BX, levtop[0.0][1]), LEV_C, lw=5)     # balance beam
ax.axhline(S.CAGE_RAIL_TOP, color='#555555', lw=1.2, ls=(0,(6,3)), zorder=6)
ax.text(-196, S.CAGE_RAIL_TOP + 2, 'cage lower rail top z 64.4 (lowered)', fontsize=11, color='#333333')
ax.axhline(S.SPINE_Z, color='#555555', lw=1.2, ls=(0,(2,3)), zorder=6)
ax.text(-196, S.SPINE_Z + 2, 'cage spine z 114', fontsize=11, color='#333333')
dot(ax, (0, EYE[1]), SHOCK_C, s=90, marker="D"); dot(ax, (0, BEAM_C[1]), SHOCK_C, s=60)
# lean pose: ground at 10 deg (body frame) through the ground contact line and the two lean-pose wheel centres
a = math.radians(S.LEAN_DEG)
gx = np.array([-190, 190]); ax.plot(gx, G + (-gx) * math.tan(a) * 0 + np.array([0, 0]), alpha=0)
rot = lambda p: (p[0] * math.cos(a) - (p[1] - G) * math.sin(a) * -1 * 0, 0)
# ground plane seen from the leaned body (body leans +a: +x side down), drawn through the x=0 ground point
lx = np.array([-190.0, 190.0]); lz = G + lx * math.tan(a)
ax.plot(lx, lz, color="#555555", lw=1.8, ls="-.", zorder=5)
ax.text(-335, -26, "dash-dot line: ground seen from the body at 10 deg lean (+x side down)", fontsize=11, color="#555555", va="center")
for sgn, t in ((1, T_LEAN[0]), (-1, T_LEAN[1])):
    dot(ax, (sgn * XT, TZ + t), "#555555", s=40, marker="x")
# labels
lab(ax, "arm pivot axle z = %.1f\nx %.1f .. %.1f (blocks x 12..30, plates 66.2 / 89.5)" % (PZ, S.PIVOT_BLOCK_X[0], S.ARM_PLATES_X[1]), (60, PZ), (130, -95), ARM_C, ha="center")
lab(ax, "wheel centre, rest\n(x, z) = (144, %.1f)\ntyre 62 wide, R 56.2" % TZ, (XT, TZ), (95, -70), ARM_C, ha="center")
lab(ax, "wheel centre +37.9\n(144, %.1f)" % (TZ + T_UP), (XT, TZ + T_UP), (80, 25), GREY, ha="center")
lab(ax, "wheel centre -32.9\n(144, %.1f)" % (TZ + T_DN), (XT, TZ + T_DN), (95, -40), GREY, ha="center")
lab(ax, "upper link inner hinge\n(%.1f, %.1f)" % (S.UPLINK_IN[0], S.UPLINK_IN[2]), (S.UPLINK_IN[0], S.UPLINK_IN[2]), (40, 100), LINK_C, ha="center")
lab(ax, "upper link outer hinge\n(%.1f, %.1f)" % (S.UPLINK_OUT[0], S.UPLINK_OUT[2]), (S.UPLINK_OUT[0], S.UPLINK_OUT[2]), (75, 75), LINK_C, ha="center")
lab(ax, "lever strut x +/-66.2; ball on stud x +/-54, z %.1f" % levtop[0.0][1], (BX, levtop[0.0][1]), (130, 60), LEV_C)
lab(ax, "balance beam (z %.1f at rest)\ncups on balls x +/-54, shock on the centre line" % levtop[0.0][1], (-BX / 2, levtop[0.0][1]), (-150, 70), LEV_C, ha="center")
lab(ax, "shock rear eye x 0, z %.1f\n(computed)" % EYE[1], (0, EYE[1]), (-30, 60), SHOCK_C, ha="center")
lab(ax, "knuckle ball (%.1f, %.1f)" % (S.KNUCKLE_BALL[0], S.KNUCKLE_BALL[2]), (S.KNUCKLE_BALL[0], S.KNUCKLE_BALL[2]), (-70, -30), STEER_C, ha="center")
lab(ax, "servo shaft z %.1f\nx 32..58.5" % S.SERVO_SHAFT[1], (-45, S.SERVO_SHAFT[1]), (-140, -75), STEER_C, ha="center")
lab(ax, "horn ball (%.1f, %.1f)" % (-S.HORN_X, HB[2]), (-S.HORN_X, HB[2]), (-125, -22), STEER_C, ha="center")
lab(ax, "horn ball (%.1f, %.1f)" % (S.HORN_X, HB[2]), (S.HORN_X, HB[2]), (140, 10), STEER_C, ha="center")
xd = P_UP[0]
ax.annotate("", xy=(xd, P_UP[2]), xytext=(xd, G), arrowprops=dict(arrowstyle="<->", color="k", lw=1.6), zorder=12)
lab(ax, "deck front %.1f mm upright\n%.1f mm at 10 deg lean (star)\n(measured, build 7b79b4c)" % (cl_up, cl_lean_v), (xd, (P_UP[2] + G) / 2), (1, 0), "k")
dot(ax, (P_LEAN[0], P_LEAN[2]), "k", s=60, marker="*")
fig.text(0.5, 0.105, "FRONT VIEW from -y   (x to the right).  Ghost tyres on +x side only; x marks mark the wheel centres in the 10 deg lean pose (+%.1f / %.1f mm)." % T_LEAN, fontsize=13, ha="center")
finish(ax, "V2 hard points - front view (from -y) - build " + COMMIT, "x  (mm)", "z  (mm)", (-345, 345), (-32, 245))
legend(ax)
fig.subplots_adjust(left=0.05, right=0.99, top=0.95, bottom=0.17)
flush_labels(ax, -202, 202, 0, fig); fig.savefig(os.path.join(OUT, "hardpoints_front.png")); plt.close(fig)

# ================================================================ TOP VIEW (from +z, +x right, front at the bottom)
fig, ax = plt.subplots(figsize=(22, 11.5), dpi=100)
draw_body(ax, "top")
ax.add_patch(Rectangle((-KO["x"], KO["y"][0]), 2 * KO["x"], KO["y"][1] - KO["y"][0], fill=False, ec=KO_C, lw=2, ls="--", zorder=9))
lab(ax, "keep-out box |x|<=64, y -110..-60", (-KO["x"], KO["y"][0]), (-95, 15), KO_C, ha="center")
for sgn in (1, -1):
    for t, ls, col in ((0.0, "-", TYRE_C), (T_UP, "--", GREY), (T_DN, "--", GREY)):
        if t != 0.0 and sgn < 0:
            continue
        w = wheel[t]
        ax.add_patch(Rectangle((sgn * XT - S.TYRE_W / 2, w[0] - S.TYRE_R), S.TYRE_W, 2 * S.TYRE_R, fill=False, ec=col, lw=2.2 if t == 0 else 1.6, ls=ls, zorder=8))
        dot(ax, (sgn * XT, w[0]), ARM_C, s=60 if t == 0 else 28, alpha=1 if t == 0 else 0.6)
    for xp in S.ARM_PLATES_X:
        seg(ax, (sgn * xp, PY), (sgn * xp, TY), ARM_C, lw=3)
    seg(ax, (sgn * S.ARM_PLATES_X[1], TY), (sgn * XT, TY), ARM_C, lw=2.5, ls=":")
    ax.plot([sgn * S.PIVOT_BLOCK_X[0], sgn * S.ARM_PLATES_X[1]], [PY, PY], color=ARM_C, lw=6, zorder=11, solid_capstyle="round")
    seg(ax, (sgn * S.UPLINK_IN[0], S.UPLINK_IN[1]), (sgn * S.UPLINK_OUT[0], S.UPLINK_OUT[1]), LINK_C, lw=3.5)
    dot(ax, (sgn * S.UPLINK_IN[0], S.UPLINK_IN[1]), LINK_C, s=70, marker="s"); dot(ax, (sgn * S.UPLINK_OUT[0], S.UPLINK_OUT[1]), LINK_C, s=55)
    xs = sgn * S.LEVER_X
    ax.plot([xs] * 2, [levtop[T_DN][0], levtop[T_UP][0]], color=LEV_C, lw=3, ls=":", zorder=11)        # lever-top travel along y
    dot(ax, (xs, levtop[0.0][0]), LEV_C, s=60); dot(ax, (xs, be[0]), LEV_C, s=55, marker="s")
    seg(ax, (xs, levtop[0.0][0]), (xs, be[0]), LEV_C, lw=3)
    dot(ax, (sgn * S.HORN_X, HB[1]), STEER_C, s=70)
    seg(ax, (sgn * S.HORN_X, S.SERVO_SHAFT[0]), (sgn * S.HORN_X, HB[1]), STEER_C, lw=3)
    ax.plot([sgn * S.SERVO_X[0], sgn * S.SERVO_X[1]], [S.SERVO_SHAFT[0]] * 2, color=STEER_C, lw=6, zorder=11, solid_capstyle="round", alpha=0.7)
seg(ax, (-BX, be[0]), (BX, be[0]), LEV_C, lw=5)                              # beam at rest
# beam in the 10 deg lean pose (+x lever top at T_LEAN[0], -x at T_LEAN[1]), ghost
bl = ((-BX, S.lever_top(T_LEAN[1])[0]), (BX, S.lever_top(T_LEAN[0])[0]))
for yy in SPINE_Y:
    seg(ax, (-60, yy), (60, yy), '#555555', lw=2, ls='--')
    ax.text(62, yy, 'cage cross tube y %g (z 114)' % yy, fontsize=11, color='#333333', va='center')
seg(ax, (0, SPINE_Y[0]), (0, SPINE_Y[1]), '#555555', lw=2, ls='--')
dot(ax, (S.KNUCKLE_BALL[0], S.KNUCKLE_BALL[1]), STEER_C, s=70, marker='^')
seg(ax, (S.HORN_X, HB[1]), (S.KNUCKLE_BALL[0], S.KNUCKLE_BALL[1]), STEER_C, lw=3)
seg(ax, bl[0], bl[1], LEV_C, lw=2.5, ls="--", alpha=0.7)
ctr = ((bl[0][0] + bl[1][0]) / 2, (bl[0][1] + bl[1][1]) / 2)
seg(ax, (0, EYE[0]), (0, FE[0]), SHOCK_C, lw=5); dot(ax, (0, EYE[0]), SHOCK_C, s=90, marker="D"); dot(ax, (0, FE[0]), SHOCK_C, s=60)
seg(ax, (0, EYE[0]), (ctr[0], ctr[1] + S.BEAM_EYE_BACK), SHOCK_C, lw=2, ls="--", alpha=0.7)
lab(ax, "arm pivot axle y %.1f\nx %.1f .. %.1f" % (PY, S.PIVOT_BLOCK_X[0], S.ARM_PLATES_X[1]), (60, PY), (110, 85), ARM_C, ha="center")
lab(ax, "wheel centre rest\n(x, y) = (144, %.1f)" % TY, (XT, TY), (95, 40), ARM_C, ha="center")
lab(ax, "wheel +37.9\n(144, %.1f)" % wheel[T_UP][0], (XT, wheel[T_UP][0]), (95, 105), GREY, ha="center")
lab(ax, "wheel -32.9\n(144, %.1f)" % wheel[T_DN][0], (XT, wheel[T_DN][0]), (110, 70), GREY, ha="center")
lab(ax, "upper link inner hinge\n(%.1f, %.1f)" % (S.UPLINK_IN[0], S.UPLINK_IN[1]), (S.UPLINK_IN[0], S.UPLINK_IN[1]), (-95, 70), LINK_C, ha="center")
lab(ax, "upper link outer hinge\n(%.1f, %.1f)" % (S.UPLINK_OUT[0], S.UPLINK_OUT[1]), (S.UPLINK_OUT[0], S.UPLINK_OUT[1]), (-120, -50), LINK_C, ha="center")
lab(ax, "lever top rest (x 66.2, y %.1f)\nmoves y %.1f .. %.1f over travel" % (levtop[0.0][0], levtop[T_DN][0], levtop[T_UP][0]), (S.LEVER_X, levtop[0.0][0]), (-100, 50), LEV_C, ha="center")
lab(ax, "beam end = ball on lever top\n(54.0, %.1f) rest" % be[0], (BX, be[0]), (80, 25), LEV_C, ha="center")
lab(ax, "balance beam, rest (solid)\nand 10 deg lean pose (dashed)", (-BX, be[0]), (-110, 5), LEV_C, ha="center")
lab(ax, "shock rear eye (0, %.1f)\n(computed), z %.1f" % (EYE[0], EYE[1]), (0, EYE[0]), (90, -10), SHOCK_C)
lab(ax, "heave shock on the centre line", (0, (be[0] + EYE[0]) / 2), (-130, -10), SHOCK_C, ha="center")
lab(ax, "servo shaft y %.1f, x 32..58.5" % S.SERVO_SHAFT[0], (45, S.SERVO_SHAFT[0]), (115, -45), STEER_C, ha="center")
lab(ax, "horn ball (69.0, %.1f)" % HB[1], (S.HORN_X, HB[1]), (-110, -30), STEER_C, ha="center")
lab(ax, "knuckle ball (%.1f, %.1f)\ntie rod to horn ball" % (S.KNUCKLE_BALL[0], S.KNUCKLE_BALL[1]), (S.KNUCKLE_BALL[0], S.KNUCKLE_BALL[1]), (70, -50), STEER_C, ha="center")
fig.text(0.5, 0.105, "TOP VIEW from +z   (x right, FRONT AT THE BOTTOM).  Tyre footprints shown; ghost footprints on the +x side only.", fontsize=13, ha="center")
finish(ax, "V2 hard points - top view (from +z) - build " + COMMIT, "x  (mm)", "y  (mm)   front (bottom) -> rear (top)", (-345, 345), (-322, 52))
legend(ax)
fig.subplots_adjust(left=0.05, right=0.99, top=0.95, bottom=0.17)
flush_labels(ax, -202, 202, 0, fig); fig.savefig(os.path.join(OUT, "hardpoints_top.png")); plt.close(fig)
print("written", OUT)
