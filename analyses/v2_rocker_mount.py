"""Version 2 rocker mount (+x side, PETG): clamp on the fixed pivot axle, foot over the rocker, 4 mm steel pin hanging down.
Load: steering stall with the wheel blocked. ASSUMED servo stall 2.0 N m (not from a data sheet): 274 N in the tie rod (docs/v2/steering.md) and
167 N in the drag link (2.0 N m / 12 mm horn); both act on the rocker and so on its pin, roughly along the rods (fore and aft). Taken here as
274 N rearward on the pin's shank (the larger of the two; they partly cancel, so this is on the safe side). With the servo limited to 0.6 N m
(orchestrator's proposal) every stress scales by 0.3. The clamp is held at the axle bore (the grub screw and clamp screw are not modelled).
Redesign round, B1: the pin is now held at both ends (foot above the rocker, shoe under it)."""
from soma.fem import Study
study = Study("Rocker mount, steering stall 274 N on the rocker pin (ASSUMED 2.0 N m servo)", material="PETG", safety=2, parts=["Rocker pin 1"])
study.set_material("Rocker pin 1", "Steel S235")
study.fix("Rocker bracket 1", near=(78.0, -116.8, 44.95))       # axle bore, a point on its wall
study.force("Rocker pin 1", N=(0.0, 274.0, 0.0), near=(87.5, -116.8, 26.0))
