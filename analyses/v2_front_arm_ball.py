"""Version 2 front arm, the lever load on the ball itself (orchestrator, NOTES 56): analyses/v2_front_arm.py with the 118.1 N lever reaction applied
on the steel ball stud's ball, 12.2 mm inboard of the blade's mid plane (the first case put it on the stud boss's end face, 6.8 mm inboard, and
understated the blade's torsion). The stud is bonded in its bore (FUGU_FEM_STUD_BOND=1 makes the shank touch the bore; the build never sets it).
Same supports and wheel loads as the first case. Mesh: FEM_MESH (default 2.6)."""
import os
from soma.fem import Study
assert os.environ.get('FUGU_FEM_STUD_BOND') == '1', 'run with FUGU_FEM_STUD_BOND=1'
study = Study("Front arm v2, 65 N bump + 30 N side at the tip, 118 N on the lever ball", material="PETG", safety=2,
              mesh=float(os.environ.get("FEM_MESH", "2.6")), parts=["Lever ball stud 1"])
study.set_material("Lever ball stud 1", "Steel S235")
study.fix("arm 1", near=(66.2, -116.8, 47.5), axes="yz")
study.fix("arm 1", near=(89.5, -116.8, 47.5), axes="yz")
study.fix("arm 1", near=(93.25, -116.8, 54.5), axes="x")
study.force("arm 1", N=(-15.0, 0, 32.5), near=(66.2, -216.6, 40.6))
study.force("arm 1", N=(-15.0, 0, 32.5), near=(89.5, -216.6, 40.6))
study.force("Lever ball stud 1", N=(0.0, -94.5, 70.7), near=(53.83, -81.45, 92.63))     # on the ball (its upper half; a point 0.4 mm inside it, so the cup block's cavity 0.1 mm outside is not nearer): the fork pushes it along -M
