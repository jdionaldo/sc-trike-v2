# Print or buy: every part of the Version 2 suspension + drivetrain kit

Owner (November 24): "make sure i can either print this whole kit of use a steel part that i can buy off the shelf".
Listings were opened on 2026-11-24 (sources: `../sourcing_v2_2026-11-24.md`, `../../docs/v2/bought_part_datasheets_2026-11-24.md`, Version 1's
`kit/sourcing_2026-11-24.md` for the rear and drive rows). Strength figures are FEM results from NOTES.md (PETG across layers 20 MPa, target safety 2).
Interface for a machined replacement: `../../docs/v2/knuckle_rocker_interface.md` (knuckle and rocker mount); for every other printed part the
machinable "drawing" is its STEP (`soma export "<part>" --format step`): a metal copy of the same shape keeps every hole and face.

Status: PRINTED = the kit's default; BUY = bought only (reason given); EITHER = both work with no other change; OPTION = an alternative that does
not reach safety 2, listed so the owner can choose.

## Chamfers (owner: "apply chamfers to them. all except gears")

Every printed moving part has 0.5 mm chamfers on its convex corners (0.4 / 0.3 where the CAD kernel refused 0.5), with fit diameters unchanged
(NOTES decision 30 has the per-part figures). Not chamfered: the rear bearing carriers, pillow blocks and ring hub (traced facet meshes without a
source profile), the rocker ST1 (its chamfered solid meshes open; break its edges by hand) and the pulleys' teeth. A metal replacement made from a part's STEP carries the same chamfers.

## Front suspension

| Part | Printed version | Off-the-shelf steel / metal version | Status |
|---|---|---|---|
| Front swing arm (SU1) | yes, PETG; FEM bore edge 1.49 (OPEN below 2) | no stock part fits this leading-arm geometry; machinable from its STEP (aluminium) | PRINTED (strength OPEN) |
| Upper link (SU2) | yes, PETG | no stock part; machinable from its STEP | PRINTED |
| Front carrier (SU3) | yes, PETG | no stock part; machinable from its STEP | PRINTED |
| Knuckle (SU4) | yes, PETG: landing 2.71, steering stall at the 0.47 N m limit 2.43 | no stock part; machined to `knuckle_rocker_interface.md` (steel or 7075) | PRINTED (owner) / machined alternative |
| Arm pivot blocks (SU5) | yes, PETG | no stock part; machinable from its STEP | PRINTED |
| Balance beam (SU6) | yes, PETG: 2.13 | no stock part; machinable from its STEP | PRINTED |
| Beam carrier (SU7) | yes, PETG: 1.30 two-wheel, 1.06 one-wheel (OPEN) | no stock part; machinable from its STEP | PRINTED (strength OPEN) |
| Heave saddles, yaw pads, droop pads (SU8..SU12) | yes, TPU 95A | no stock part (any 2 mm rubber or polyurethane cut to the outline works) | PRINTED |
| Cup blocks (pucks on the lever balls) | not advised: a printed socket on a steel ball wears fast; a PETG puck is possible as a stopgap | turned from POM rod D10 (no stock puck) | BUY rod, turn |
| Kingpin D4 x 40 | not printable: a 4 mm pin carries the knuckle's bearing loads in shear | McMaster 91595A185 ISO 8734 D4 x 40 hardened (pack 25; page opened, no price) | BUY |
| Arm tip shafts D4 x 55 | not printable (shear) | McMaster 91595A192 D4 x 60, cut to 54 (ISO 8734; 55 not stocked) | BUY |
| Uplink and shock pins D3 x 12 | not printable (shear) | McMaster 91595A112 ISO 8734 D3 x 12 (pack 50) | BUY |
| Carrier pin D6 x 35 | not printable (shear) | McMaster 91595A153 D6 x 34 | BUY |
| Heave stop dowel D8 x 75 | not printable (bending under the yaw stop) | McMaster 91595A591 D8 x 80, cut to 75 | BUY |
| Yaw pin D8 x 30 + ring | not printable | McMaster 91595A567 D8 x 30 + DIN 471 8 mm ring | BUY |
| Arm pivot axle D5 x 172 | not printable (shaft) | Ooznest HGLS-CP-5-CTS 5 mm hardened, cut to size | BUY |
| Front axle bolt M4 x 35 12.9 + nut | not printable (bending: 12.9 needed) | ISO 4762 12.9 / ISO 4032 (any fastener shop) | BUY |
| Axle sleeve 6 x 1 | not printable (press fit, bore bearing) | BES 13086 316L tube 6 x 1, cut to 13.6 | BUY |
| Front wheel bushings 4 x 8 x 6, flange 12 x 2 (owner: "plain bushings") | not printable (a PETG bore on a steel shank wears; the owner chose bronze) | AL04-08-06 sintered oil-filled bronze, USD 1.85 (123bearing, opened 2026-11-25); alternative RS PRO 529-013 (ISO 2795, bag of 8) | BUY |
| Heave shock | not printable (damper) | RC4WD RC4Z-D0004 70 mm (Version 1 choice) | BUY |
| Ball studs 6.8 mm (M3 and M4) | not printable (ball wear, thread) | HPI 66247 (M3, thread not stated), Tekno TKR5052A (M4, page not opened) | BUY |
| Heat-set inserts, screws, shims | not printable | 3DJAKE / Ruthex inserts, ISO screws, DIN 988 shims | BUY |

## Steering

| Part | Printed version | Off-the-shelf steel / metal version | Status |
|---|---|---|---|
| Rocker (ST1) | yes, PETG | no stock part; machinable from its STEP | PRINTED |
| Rocker bracket (ST2) | yes, PETG: safety 0.75 at the 0.47 N m servo limit (about 0.96 at the 0.367 N m the steering needs): BELOW 2 | no stock part; machined to `knuckle_rocker_interface.md` (aluminium 6082 about 2.3, steel about 2.2 at 2.0 N m) | OPTION (owner chose printed) / machined alternative |
| Servo floor and clip (ST3, ST4) | yes, PETG | no stock part (a standard servo mount does not fit the flat-lying layout) | PRINTED |
| Steering servo | not printable | Tekin T-180 TEKTT1501 (torque limit 40 %); alternative Savox SC-1251MG+ (9 kg-cm at 6 V, no torque setting: only with the machined rocker bracket) | BUY |
| Servo horn 25T | not printable (25T spline in PETG strips under servo torque) | Tekno RC TKR5253B aluminium (hole radius not published: MEASURE ON ARRIVAL) | BUY |
| Tie rods (M4 rod ends + turnbuckle) | not provided: a printed link would need a buckling and socket check that has not been done | Tekno RC TKR5188B rod ends + TKR9423 M4 turnbuckle; slim knuckle end RC4WD RC4ZS1378 | BUY |
| Drag links (M3 rod ends + stud) | not provided (as above) | RC4WD RC4ZS1378 on an M3 x 10 stud (swivel not stated); candidate Gmade GMA20216 M3 rod end with its own 6.8 mm steel ball, 10 per pack, USD 12.00, out of stock (page opened 2026-11-25: M3 and 6.8 mm in the title, swivel NOT stated, so the 25 deg need stays ASSUMED) https://www.hobbyrecreationproducts.com/collections/gmade-komodo-kit/products/gma20216-m3-rod-end-w-6-8mm-steel-ball | BUY |
| Rocker pin D4 x 20 | not printable (390 MPa in the pin at the assumed stall) | McMaster 91595A167 ISO 8734 D4 x 20 | BUY |

## Rear and drivetrain (Version 1's choices; its orchestrator writes the same table)

| Part | Printed version | Off-the-shelf steel / metal version | Status |
|---|---|---|---|
| Rear swing arm (DT11) | yes, PETG | no stock part; machinable from its STEP | PRINTED |
| Pillow block, bearing carriers, ring hub (DT5..DT10) | yes, PETG | no stock parts for this layout | PRINTED |
| Motor plate (CH5) | yes, PETG (heat) | no stock part; aluminium from its STEP | PRINTED |
| Rear pivot pulley 26T (DT14) | yes, PETG (teeth are not chamfered) | none: no 26T GT2 aluminium pulley with a 5 mm bore was found; only 20T and 36T (Adafruit, discontinued). Version 1, a604a25 | PRINTED only (unless the ratio changes) |
| Rear axle pulley 30T (DT15) | yes, PETG (teeth are not chamfered) | none: ServoCity 30T GT2 (VERIFIED by Version 1) has an 8 mm REX (rounded hex) bore and does not fit the round 8 mm axle; a REX-bore axle would allow it. Version 1, a604a25 | PRINTED only (unless the axle gets a REX section) |
| Wheels 1.9 / 2.2 in (DT16, DT17) | yes, PETG | stock 1.9 / 2.2 crawler wheels with 12 mm hex exist; none checked against this hub | PRINTED |
| Bearings 5x11x4, 8x16x5 | not printable | MR115-2RS / 688-2RS (Hobby ETC, Version 1) | BUY |
| Shafts 5 x 112, 5 x 88, axle 8 x 100 | not printable (turning shafts in bearings) | ServoCity 5 mm x 300 mm stainless, cut both (VERIFIED by Version 1; 83 HRB, NOT hardened) https://www.servocity.com/5mm-x-300mm-stainless-steel-precision-shafting; VXB 8 x 100 HRC60 (VERIFIED, Version 1) | BUY |
| Belt 2GT-300-6 | not printable | generic 2GT-300-6 (Version 1: listing not opened) | BUY |
| Gears (pinion, spur, ring and pinion) | not printable (gears: excluded by the owner) | ServoCity 15T 32P, Yeah Racing / Axial 40T, Axial AXI232003 (Version 1) | BUY |
| Motor + ESC | not printable | Hobbywing QuicRun Fusion SE HWI30404317 (Version 1) | BUY |
| Rear shock | not printable | Integy C26651RED (Version 1: page not opened) | BUY |
| Rear hub 12 mm hex, 8 mm bore | not printable | none found with an 8 mm bore (Version 1 open item) | BUY (not found) |
| Tyres | not printable | Pro-Line PRO1021114 front, RC4WD RC4ZT0122 rear (Version 1) | BUY |
| Screws, retaining rings, grub screws | not printable | generic ISO / DIN | BUY |

## Printed fallbacks for low-load parts only

Shims and spacers (the 0.5 mm thrust shims, servo shims 1.4 / 1.16 mm for the Savox) can be printed in PETG if the steel ones are not at hand; the
D3 .. D8 pins, bolts, balls and the sleeve cannot.
