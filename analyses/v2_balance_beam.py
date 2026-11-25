"""Version 2 balance beam (PETG), TWO-WHEEL LANDING at the bump stop: each lever ball pushes its puck against the rear plate of the beam's end fork
with 131 N along +M (docs/v2/arm_stops.md, CALCULATED from 65 N per wheel). The beam is held at its yaw pin bore."""
from soma.fem import Study
study = Study("Balance beam, two-wheel landing: 131 N on each fork", material="PETG", safety=2)
study.fix("Balance beam", near=(0.0, -81.45, 85.4))
study.force("Balance beam", N=(0.0, 104.9, -78.5), near=(54.0, -78.65, 87.53))
study.force("Balance beam", N=(0.0, 104.9, -78.5), near=(-54.0, -78.65, 87.53))
