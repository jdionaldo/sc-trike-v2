"""Version 2 PETG knuckle: steering stall at the servo's torque limit with the wheel blocked. Limit: Tekin T-180 HotWire Torque 40 %, Torque
Delay 0 s, at 6.0 V: 0.47 N m if the percentage is of the rated 11.9 kg-cm (ASSUMED linear; teamtekin.com servoguide, read 2026-11-24; bench check
in the kit manual). Tie rod force: 274 N at the ASSUMED 2.0 N m stall (docs/v2/steering.md) x 0.47 / 2.0 = 64.4 N, along the tie rod (knuckle
ball (107.31, -222.45, 14.6) to the rocker ball (69.81, -122.70, 21.5)). It acts through the M4 ball stud on the insert pocket wall. The wheel
is blocked: the axle bolt is held where the hub sits; the knuckle also at the kingpin bores (x, y) and the thrust seat (z).
Safety at 0.367 N m (the steering need) and at 0.6 N m: this result x 0.47 / 0.367 and x 0.47 / 0.6 (linear)."""
from soma.fem import Study
Y0, Z0 = -216.55, 40.6
F = 64.4; d = (-0.3512, 0.9341, 0.0646)
study = Study("PETG knuckle, steering stall at the 0.47 N m limit: 64.4 N along the tie rod, wheel blocked", material="PETG", safety=2,
              parts=["Front axle bolt 1", "Axle sleeve 1"])
study.set_material("Front axle bolt 1", dict(name="steel 12.9", E=210000, nu=0.3, strength=1100, density=7.85))
study.set_material("Axle sleeve 1", "Steel S235")
study.fix("Front knuckle 1", near=(125.05, Y0, 25.0), axes="xy")
study.fix("Front knuckle 1", near=(125.05, Y0, 55.0), axes="xy")
study.fix("Front knuckle 1", near=(127.0, Y0, 30.5), axes="z")
study.fix("Front axle bolt 1", near=(154.0, Y0, Z0 + 2.0))           # blocked wheel: the hub holds the bolt
study.force("Front knuckle 1", N=(F * d[0], F * d[1], F * d[2]), near=(107.31 + 2.8 * d[0], -222.45 + 2.8 * d[1], 25.5))
