"""Version 2 PETG knuckle (parts/v2_knuckle.py, +x side): one-wheel landing, 65 N up + 30 N in at the tyre contact (x 144, z -15.6), the same
case as the knuckle study (sc-trike-v2_knuckle docs/v2/knuckle_options.md, option B: safety 2.80 with a bonded sleeve). Path: tyre -> wheel hub
on the M4 axle bolt (two 3 mm bands at x 151 and 157, inside the hub, as in the study) -> bolt and 6 x 1 sleeve (bonded: press fit, clamped nut)
-> knuckle; the side force on the seat ring face. Held at both kingpin bores (x, y) and the lower thrust seat (z).
Bearing forces on the bolt (CALCULATED in the study for these hard points): +421.8 N z at x 151, -356.8 N z at x 157."""
from soma.fem import Study
Y0, Z0 = -216.55, 40.6
study = Study("PETG knuckle, one-wheel landing: 65 N up + 30 N in at the tyre contact", material="PETG", safety=2, parts=["Front axle bolt 1", "Axle sleeve 1"])
study.set_material("Front axle bolt 1", dict(name="steel 12.9", E=210000, nu=0.3, strength=1100, density=7.85))
study.set_material("Axle sleeve 1", "Steel S235")
study.fix("Front knuckle 1", near=(125.05, Y0, 25.0), axes="xy")      # lower kingpin bore
study.fix("Front knuckle 1", near=(125.05, Y0, 55.0), axes="xy")      # upper kingpin bore
study.fix("Front knuckle 1", near=(127.0, Y0, 30.5), axes="z")         # lower thrust seat (the carrier block presses down through the washer)
study.force("Front knuckle 1", N=(-30.0, 0.0, 0.0), near=(148.0, Y0 + 4.0, Z0))   # side force on the seat ring face
study.force("Front axle bolt 1", N=(0.0, 0.0, 421.8), near=(151.0, Y0, Z0 + 2.0))
study.force("Front axle bolt 1", N=(0.0, 0.0, -356.8), near=(157.0, Y0, Z0 + 2.0))
