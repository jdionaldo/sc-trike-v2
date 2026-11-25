# Mass, centre of gravity and tip-over (model.step, wishbone front end)

Tool: `tools/cg_tip.py output/model.step [--json out.json] [--verbose] [--battery-g G] [--battery-xyz X Y Z] [--roof-drop 25]`.
Runs in about 6 s (exact OCC volume properties, no tessellation). Reusable on later builds: parts are matched by name;
a part that matches nothing is counted as steel by volume and listed ("UNMATCHED"). Units mm and g, ground z = -15.62 (lowest tyre point),
tyre contacts: front (+-144, -216.4), rear (0, 236.4).

## Result (current build, 84 parts)

"Sliced" = printed volume x density x 0.88 (calibration below), "solid" = 100 % fill upper bound. Battery included unless stated.

| case | mass g | CG x, y, z mm | CG height mm | lateral tip deg |
|---|---|---|---|---|
| with body, sliced | 2576 | 4.5, -37.5, 79.4 | 95.0 | 39.6 |
| with body, solid | 2805 | 4.6, -37.1, 80.4 | 96.0 | 39.3 |
| no body, sliced | 2247 | 5.2, -34.5, 73.5 | 89.1 | 40.9 |
| no body, solid | 2432 | 5.3, -33.9, 74.3 | 89.9 | 40.5 |
| with body, sliced, no battery | 2456 | 4.7, -34.7, 81.3 | 97.0 | 38.7 |
| no body, sliced, no battery | 2127 | 5.5, -31.1, 75.3 | 91.0 | 39.8 |
| with body, sliced, roof -25 mm (ESTIMATE) | 2576 | 4.5, -37.5, 72.3 | 87.9 | 41.8 |
| no body, sliced, roof -25 mm (ESTIMATE) | 2247 | 5.2, -34.5, 68.0 | 83.6 | 42.7 |

Battery line (ASSUMED, see below): 120 g, CG 56.4 mm above ground. Adding it lowers the full-car CG from 97.0 to 95.0 mm and
raises the lateral tip angle from 38.7 to 39.6 deg. Forward (braking) tip about the front axle: about 62 deg.
Lateral tip is about the line front tyre contact to rear tyre contact, the smaller of the two sides (the +X side, because the CG sits 4.5 mm to +X).
Static, rigid: suspension, lean and tyre compliance are not included. Height = CG z minus ground z.

## What is counted

- Body = group Print_shell (Front_shell, Tail_light_housing, Grille_insert) plus group Lights (two light strips). Everything else, including Cage_2
  (Cage, Roof_plate, light contact block), is chassis. The canopy is part of the single Front_shell solid in this build.
- Printed (PETG 1.27 g/cm3; body and strips PLA 1.24): w3_* parts, Cage, Roof_plate, contact block, Lean_head, upper/lower arms, knuckles, Steer_horn,
  Steer_servo_mount, pulleys, printed wheel rims. Mass = exact solid volume x density.
- Infill: BOM files give per-part infill (10 to 100 %) but no per-part grams. Calibration: Bambu slice check total 1678 g (`kit/print/bambu/slice_check.json`,
  includes supports) divided by the sum of solid volume x density of the same files (`kit/print/parts.json`, 1908 g) gives 0.88. That factor is applied to every printed
  part ("sliced"), so the printed total equals 1678 g. Per-part infill is not used, so shell (10 to 15 % infill, thin walls) is probably overstated relative to the
  60 % to 100 % parts. 100 % solid is shown as the upper bound.

## Bought parts: ALL masses ASSUMED (no mass appears in BOM_COST.md/.csv, kit/bom.md or NOTES.md)

| part | g | note |
|---|---|---|
| Motor + ESC (Hobbywing Fusion SE 2-in-1) | 160 | typical 150 to 170 g |
| Steering servo | 55 | same value `tools/cg_fit.py` used |
| Receiver | 12 | FS-iA6B class |
| Front tyres (2) | 90 | 45 each, 1.9 in with foam. Tyre is modelled as a solid cylinder at its CAD centroid |
| Rear tyre | 60 | 2.2 in with foam |
| Rear shock 100 mm | 30 | |
| Front heave shock body + shaft | 18 + 5 | |
| Rear ring gear | 30 | steel |
| Spur gear, pinion, rear hex hub | 10, 5, 10 | |
| GT2 belt | 4 | |
| Steel rods, pins, studs, nuts, tie rods, rod ends, axle | 254 | CAD volume x 7.85. Probably 30 to 40 % too high (tie rods 15 g each, rear axle 39 g are solid in CAD) |
| Allowance for bearings, inserts, screws, wire, magnets, LEDs | 35 | placed at the car's own CG, so it does not move it |
| Battery, 3S 1300 mAh LiPo (BOM B8) | 120 | NOT in the model (removed Nov 22, no position designed). Placed crosswise on the deck just ahead of the motor: (0, -93, 40.8), on top of the deck. That spot overlaps the steering servo in y, so the position is a stand-in. Override with `--battery-xyz` and `--battery-g` |

Sensitivity of the battery position: 10 mm of battery height moves the full-car CG by about 0.5 mm; a y move of 25 mm moves CG y by about 1 mm and does not change the lateral angle.

## Roof 25 mm lower (ESTIMATE)

Every part that reaches above z = 100 (16 parts: cage, roof plate, front shell, tail light housing, rear shock, pins, heave shock and others) is cut at z = 100 and the
portion above is moved down 25 mm with its mass unchanged. Real shorter pillars would also lose a little mass at the top, which lowers the CG slightly further. Effect with body: CG height 95.0
to 87.9 mm (-7.1 mm), lateral tip 39.6 to 41.8 deg (+2.2). Without body: 89.1 to 83.6 mm (-5.5 mm), 40.9 to 42.7 deg.

## Reading the numbers

Mass is dominated by printed parts (1678 g sliced) plus about 745 g of bought parts. The CG is 4.5 mm off the centre line toward +X because the rear swing arm, pulleys and
belt sit at +X, so the +X side of the tip line is the one that limits. Weighing the real car and the real battery replaces every ASSUMED value above.
