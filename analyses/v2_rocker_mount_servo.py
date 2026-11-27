"""Version 2 rocker mount (+x side, PETG) at the servo limit, the pin load from the steering kinematics instead of the tie rod force alone.

Servo torque 0.47 N m (Tekin 40 %, ASSUMED linear), wheel blocked. The drag link carries T / (horn lever square to the rod), the tie rod what
balances the rocker about its pin; the pin carries the rest: R = -(F_drag + F_tie). CALCULATED with tools/steer_kin.py over horn -30 .. +30 deg
(table in NOTES 38): the largest is at horn +30 deg, drag 48.2 N, tie 60.5 N, R = (32.2, -10.3, 4.3) N, |R| 34.1 N; the other torque direction
gives the opposite sign. The two rods pull the rocker's arms in opposite directions, so on the pin they mostly cancel; analyses/v2_rocker_mount_limit.py
puts the whole tie rod force (64.4 N) on the pin along y and stays as the envelope case.
SIGN: +1 or -1 (environment ROCKER_SIGN, default +1). Mesh: FEM_MESH (default 1.0; read at two meshes)."""
import os
from soma.fem import Study
MESH = float(os.environ.get("FEM_MESH", "1.0"))
SIGN = float(os.environ.get("ROCKER_SIGN", "1"))
HORN = os.environ.get("ROCKER_HORN", "+30")
# pin moved to x 80.0 (NOTES 42): pin loads recomputed (CALCULATED, tools/steer_kin.py, 0.47 N m): horn +30 deg the largest, horn -30 deg the most fore-and-aft
R = {"+30": (38.3, -1.3, 5.1), "-30": (14.8, 9.9, 8.4)}[HORN]
study = Study("Rocker mount, servo 0.47 N m at horn %s deg: pin load %+.0f x %s N (CALCULATED)" % (HORN, SIGN, R), material="PETG", safety=2, mesh=MESH, parts=["Rocker pin 1"])
study.set_material("Rocker pin 1", "Steel S235")
study.fix("Rocker bracket 1", near=(78.0, -116.8, 44.95))       # axle bore, a point on its wall
study.force("Rocker pin 1", N=tuple(SIGN * c for c in R), near=(82.0, -116.8, 26.0))   # on the pin where the rocker's hub (z 19 .. 33) bears (pin at x 80.0)
