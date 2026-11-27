"""Version 2 front arm, the lever load with its real 12.2 mm offset as an equivalent couple (orchestrator, after NOTES 56; NOTES 58).
The direct case on the ball (analyses/v2_front_arm_ball.py) is refused by the FEM tool. Here the steel stud is taken as rigid and its load goes
into the arm where it really bears: on the wall of its bore. FUGU_FEM_BORE_SEG=1 splits that wall at the blade's mid plane (x 66.2) into a mouth
half (x 62.2 .. 66.2, centre 64.2) and a far half (66.2 .. 70.2, centre 68.2); the build never sets it.
Lever reaction F = (0, -94.5, 70.7) N (118.1 N along -M) at the ball centre, x 54.0. Bearing pair, statically equal to F at the ball:
  R2 = F (54.0 - 64.2) / (68.2 - 64.2) = -2.55 F on the far half, R1 = F - R2 = 3.55 F on the mouth half: the couple 12.2 x 118.1 = 1441 N mm in all.
(The first case analyses/v2_front_arm.py put F at a point 3.2 mm from both the blade's whole inner face and the bore: that point is no boss face,
the stud boss lies inside the blade.) Same supports and wheel loads as analyses/v2_front_arm.py. Mesh: FEM_MESH (default 2.6)."""
import os
from soma.fem import Study
assert os.environ.get('FUGU_FEM_BORE_SEG') == '1', 'run with FUGU_FEM_BORE_SEG=1'
F = (0.0, -94.5, 70.7)
k2 = (54.0 - 64.2) / (68.2 - 64.2)
R2 = tuple(k2 * c for c in F); R1 = tuple((1 - k2) * c for c in F)
study = Study("Front arm v2, 65 N bump + 30 N side at the tip, 118 N lever load at the ball (12.2 mm offset) as the stud's bearing pair",
              material="PETG", safety=2, mesh=float(os.environ.get("FEM_MESH", "2.6")))
study.fix("arm 1", near=(66.2, -116.8, 47.5), axes="yz")
study.fix("arm 1", near=(89.5, -116.8, 47.5), axes="yz")
study.fix("arm 1", near=(93.25, -116.8, 54.5), axes="x")
study.force("arm 1", N=(-15.0, 0, 32.5), near=(66.2, -216.6, 40.6))
study.force("arm 1", N=(-15.0, 0, 32.5), near=(89.5, -216.6, 40.6))
study.force("arm 1", N=R1, near=(64.2, -81.45, 89.63 + 1.55))      # bore wall, mouth half (a point on it, the top)
study.force("arm 1", N=R2, near=(68.2, -81.45, 89.63 + 1.55))      # bore wall, far half
