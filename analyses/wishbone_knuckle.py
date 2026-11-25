"""Front knuckle (+x side, parts/wishbone.py), PETG, checked against 20 MPa across the layers.
Load case: landing on one front wheel, 65 N up plus 30 N sideways toward the vehicle at the tyre contact (x 144, z -15.6), as in analyses/front_arm.py.

The wheel passes the load into the 6 mm axle pin, the pin into the axle bore (x 112.5 to 142, z 40.6, centre x 127.25). Moved to the bore centre the
contact force (-30, 0, 65) N has the lever (16.75, -56.2) mm: moment about y = -56.2 x (-30) - 16.75 x 65 = 1686 - 1089 = +597 N mm. The pin's couple
is put on as two vertical forces 14.75 mm apart: +105.5 N on the bore (x 127.25) and -40.5 N on the boss end face (x 142): together 65 N up and
597 N mm. The 30 N side force goes on the bore.

Reactions expected (free body, ball studs at x 106, z 70 and 42): upper stud (+26.5, 0, -65) N on the knuckle (the upper arm carries the wheel load),
lower stud +3.5 N in x only (the lower arm is a two-force member). The support forces of the run should come out close to these.

Tie rod: none in this load case. The contact point lies at the kingpin's y (no trail) and both forces lie in the xz plane, so they have no moment
about the kingpin. A fore-aft force F at the contact would turn the knuckle with 38 mm x F and load the tie rod (lever 66.55 mm from the kingpin to
the steering ball at y -150) with 0.57 x F.

In the study: held at the two M3 thread bores of the ball studs in the upright plate (x 106, y -208 to -203): the upper one in x, y, z, the lower one
in x only. The studs' balls sit 8.5 mm ahead of the plate face; that offset is not modelled (the bores are held where they are)."""
from soma.fem import Study
study = Study("Knuckle, one-wheel landing: 65 N up + 30 N in at the tyre contact, through the axle bore", material="PETG", safety=2)
study.fix("Knuckle 1", near=(106.0, -203.0, 71.5))
study.fix("Knuckle 1", near=(106.0, -203.0, 43.5), axes="x")
study.force("Knuckle 1", N=(-30.0, 0, 105.5), near=(127.0, -216.55, 43.65))
study.force("Knuckle 1", N=(0, 0, -40.5), near=(142.0, -216.55, 46.1))
