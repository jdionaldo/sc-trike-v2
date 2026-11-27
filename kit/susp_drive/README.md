# Fugu V2 suspension + drivetrain kit (leading swing arms)

Front swing arms, upper links, carriers, knuckle, balance beam and carrier, heave shock and stops, steering (rockers, servos, horns, drag links, tie rods), rear swing arm, belt drive, motor plate, wheels and tyres. 42 printed files on 6 plates, 25.2 h and 724 g by the Bambu Studio slice check (0 problems), the option plate included. Bought parts as listed on 2026-11-24 (16 rows VERIFIED, 34 ASSUMED, 6 UNKNOWN). Nothing bought.

## Stated openly

- Strength below the target safety of 2 (PETG across layers 20 MPa; NOTES.md): rocker mount (printed: 1.02 at the steering need, 0.19 at the ASSUMED 2.0 N m stall), beam carrier (1.30 two-wheel, 1.06 one-wheel landing), front arm (1.49 at the outer bore edge), cage at the stops (0.73 at the upper ear). Only the balance beam passes (2.13).
- No FEM of the cage with the desert shell's exo frame and the raised hoop corner: every cage FEM ran on the cage before it.
- Owner's choice (NOTES 26): printed knuckle and rocker mount with torque-limited servos. FEM at the servo limit (0.47 N m, ASSUMED from the Tekin 40 % setting): knuckle landing 2.71, knuckle steering stall 2.43 (about 1.9 at 0.6 N m); rocker mount 0.75 (about 0.96 at the steering need 0.367 N m): the rocker mount is BELOW 2.
- Rocker mount, in plain words: the printed bracket is at about 0.96 at the 31 % servo setting (0.75 at 40 %) against the target of 2. The owner chose it knowing that (NOTES 32). The machined aluminium bracket is the same-interface alternative.
- Steering at horn +-30 deg: +16.35 / -19.28 deg (verified layout B: +16.28 / -19.50) after the servo moved 3.3 mm rearward for its ears; the 20 deg figure elsewhere is the shell cut-out allowance, not a lock.
- Nothing in this kit has been printed, fitted or driven. Every fit comes from the model.
- Front wheel bushings: the housing tolerance is not on the bushing's page; the printed seat D7.90 (0.10 mm press) is ASSUMED. Print the fit coupon first.
- Heave shock: the RC4WD 70 mm (Version 1's choice) has no stated spring rate (about 30 N/mm assumed in the design) and is 2 mm longer than the model's 68 mm.
- Rear and drivetrain: the open items of Version 1 apply unchanged (ring size against the carrier cavity, no 8 mm-bore 12 mm hex hub, 42T spur not found).
- TPU parts are sliced with the PETG profile of the saved project: their time and grams are ASSUMED; switch the filament to TPU 95A in Studio.
- Not chamfered, needs a redraw: the rocker ST1 (its chamfered solid meshes open; break its edges by hand), the rear bearing carriers DT7/DT8, the pillow blocks DT5/DT6 and the ring hub DT10 (traced facet meshes without a source profile). Everything else that moves has 0.5 mm chamfers (NOTES 30).

## 0. The owner's choice and the alternatives (same interface: docs/v2/knuckle_rocker_interface.md)

| Decision | Option | What it means | Parts it adds | Price USD |
|---|---|---|---|---|
| Rocker mount | R-PETG (owner chose) | printed bracket ST2 (FEM at 0.47 N m: safety 0.75, below 2) | the torque-limited servo S-A2 | 0.00 |
| Rocker mount | R-AL | aluminium 6082 bracket, the same shape machined (safety about 2.3 at 2.0 N m), any servo | 2 machined brackets | 70.00 |
| Rocker mount | R-STEEL | steel bracket, the same shape machined (safety about 2.2), any servo | 2 machined brackets | 80.00 |
| Knuckle | K-PETG (owner chose) | printed knuckle SU4 (FEM landing 2.71, stall at 0.47 N m 2.43) | sleeve, M4 inserts, the torque-limited servo S-A2 | 88.74 |
| Knuckle | K-STEEL | the same knuckle machined in steel or 7075 (D4 bore, tapped M4), same interface | 2 machined knuckles | 90.00 |

Totals of the bought parts per combination (servo class follows: any printed option needs the torque-limited servo):

| Rocker mount | Knuckle | Servo | Total USD |
|---|---|---|---|
| R-PETG | K-PETG | S-A2 | 1046.53 |
| R-PETG | K-STEEL | S-A2 | 1047.79 |
| R-AL | K-PETG | S-A2 | 1116.53 |
| R-AL | K-STEEL | S-A1 | 900.58 |
| R-STEEL | K-PETG | S-A2 | 1126.53 |
| R-STEEL | K-STEEL | S-A1 | 910.58 |

## 1. Bought parts

Quantities from the model (`Where` names the stand-in parts). Rows with an option ID are bought only with that option. Total = packs x pack price, all rows (each option once). Sourcing notes: `../sourcing_v2_2026-11-24.md` (Version 2 front end) and Version 1's `kit/sourcing_2026-11-24.md` (rear, drive, tyres, bearings, heave shock: the same choices).

| Item | Option | Qty | Where (model) | Part / retailer | Unit USD (pack) | Packs | Total USD | Status | Note |
|---|---|---|---|---|---|---|---|---|---|
| Heave shock 70 mm (one, between the beam carrier and the cage) | - | 1 | Heave shock body/shaft (model 68.0 mm eye to eye, D3 eye pins) | RC4WD Ultimate Scale Shock 70 mm RC4Z-D0004, Hobby ETC https://store.hobbyetc.com/parts/view/29506 (Version 1 choice) | 20.99 (2) | 1 | 20.99 | VERIFIED | V1 sourcing: 72.5 ext / 55.1 comp, stroke 17.4, eye bore 3.0 (fits the D3 pins); DISCONTINUED; no spring rate stated (about 30 N/mm assumed) |
| Plain sleeve, oil-filled sintered bronze, 8 x 12 x 12 (SKF PSM 8-12-12 A51) | - | 1 | Yaw pin sleeve: pressed into the top 12 mm of the beam carrier's yaw pin bore (printed bore 11.9), the D8 yaw pin pressed into it (NOTES 57) | SKF PSM 8-12-12 A51, 123bearing.com https://www.123bearing.com/accessories/bushes/bronze/psm8-12-12-a51-skf (opened 2026-11-25) | 7.67 (1) | 1 | 7.67 | VERIFIED | ID 8, OD 12, length 12, 5 g; 2 in stock on 2026-11-25; press fit in PETG ASSUMED (ream / coupon) |
| Hardened dowel pin D3 x 12 (ISO 8734 m6) | - | 6 | Uplink pin inner/outer x 2 sides (4), Shock pin front/rear (2) | McMaster 91595A112 (pack 50) https://www.mcmaster.com/products/dowel-pins/specifications-met~iso-8734/ | 9.00 (50) | 1 | 9.00 | ASSUMED | no price on the page: est.; C52 |
| Hardened dowel pin D4 x 55 (arm tip shaft) | - | 2 | tip shaft x 2 (model D4 x 54) | McMaster 91595A188 D4 x 50 (25) or 91595A192 D4 x 60 (10) | 12.00 (10) | 1 | 12.00 | ASSUMED | 55 is not in the series: 60 mm cut and deburred to 54; no price on the page: est. |
| Hardened dowel pin D4 x 40 (kingpin) | - | 2 | Kingpin x 2 (model D4 x 39) | McMaster 91595A185 (25) | 11.00 (25) | 1 | 11.00 | ASSUMED | no price: est. |
| Axle bolt ISO 4762 M4 x 35, class 12.9 (partly threaded, b 20) + ISO 4032 M4 nut | - | 2 | Front axle bolt / nut x 2 through the knuckle sleeve and the wheel hub | generic ISO 4762 12.9 / ISO 4032 (dimensions: docs/v2/bought_part_datasheets_2026-11-24.md 6b, 6c) | 0.80 (1) | 2 | 1.60 | ASSUMED | est.; 12.9 required (8.8 safety 1.89 at the seat); thread must stay inboard of x 140 |
| Steel tube 6 x 1 (ID 4), cut to 13.6 mm (knuckle axle sleeve, pressed in) | K-PETG | 2 | Axle sleeve x 2 | BES 13086 316L seamless 6 x 1, 3 m https://www.bes.co.uk/ (docs/v2/bought_part_datasheets_2026-11-24.md 7) | 40.90 (1) | 2 | 81.80 | ASSUMED | GBP 35.52 ex VAT at an ASSUMED 1.15; no tolerance on the page: MEASURE ON ARRIVAL (OD 6.0 press fit) |
| Shim ring DIN 988 4 x 8 x 0.5 (knuckle thrust seats) | - | 4 | between each knuckle ear seat and the carrier kingpin block | Aspen Fasteners DIN 988 4 x 8 x 0.5 A2 (docs/v2/bought_part_datasheets_2026-11-24.md 6d) | 0.40 (1) | 4 | 1.60 | ASSUMED | est. price |
| Ball stud 6.8 mm, M4 (knuckle, tie rod) | - | 2 | Knuckle ball stud x 2, screwed into the M4 insert from below | Tekno RC TKR5052A 6.8 mm aluminium pivot balls (4) | 7.99 (4) | 1 | 7.99 | UNKNOWN | page 403: dimensions not read: MEASURE ON ARRIVAL (thread length into the 9 mm insert pocket) |
| M4 heat-set insert Ruthex M4 x 8.1 (hole 5.6) | K-PETG | 2 | knuckle ball boss, pressed in from the top | Ruthex M4 x 8.1, 50 pcs, 3DJAKE https://www.3djake.com/ruthex/threaded-inserts-m4-50-pieces | 6.94 (50) | 1 | 6.94 | VERIFIED | hole D3 5.6 read from the table (labels not in text: likely) |
| Hardened shaft D5, cut to 172 mm (arm pivot axle) | - | 1 | b front pivot axle (D5 x 172) through the pivot blocks, both arm roots and the rocker bracket clamps | Ooznest HGLS-CP-5-CTS 5 mm chrome hardened ground shaft, cut to size https://ooznest.co.uk/5mm-chrome-plated-hardened-ground-bearing-linear-shaft-cut-to-size | 8.90 (1) | 1 | 8.90 | ASSUMED | GBP 7.74 incl. VAT; whether it is per any length is unclear on the page; UK shop |
| Hardened dowel pin D6 x 35 (carrier pin) | - | 1 | Carrier pin (model D6 x 35) in the cage clevis, beam carrier hub bore 6.4 | McMaster 91595A153 D6 x 34 (1) | 1.80 (1) | 1 | 1.80 | ASSUMED | no price: est.; 34 mm instead of 35 fits the clevis span (check) |
| Hardened dowel pin D8 x 75 (heave stop dowel) | - | 1 | Heave stop dowel (model D8 x 75) through the carrier tail and the pad block slots | McMaster 91595A591 D8 x 80 (5), cut to 75 | 14.00 (5) | 1 | 14.00 | ASSUMED | 75 not in the series; no price: est. |
| Hardened dowel pin D8 x 30 (yaw pin) + 8 mm retaining ring | - | 1 | Beam yaw pin + Beam yaw pin clip | McMaster 91595A567 (10) + DIN 471 ring from an assortment (98875A150) | 18.00 (1) | 1 | 18.00 | ASSUMED | no prices: est.; the pin needs a ring groove (DIN 471 8 mm groove 7.6 x 0.9): grind it or use DIN 6799 on a turned groove |
| Ball stud 6.8 mm, M3 (lever balls) | - | 2 | Lever ball stud x 2 in the arm roots | HPI 66247 6.8 mm ball stud https://www.tradeinn.com/techinn/en/hpi-racing-6.8mm-ball-stud/143442104/p | 17.49 (10) | 1 | 17.49 | ASSUMED | thread and pack count not on the page (pack 10 ASSUMED); shared with the steering studs below |
| POM (acetal) rod D10 for the two cup blocks (turned D9 x 9, ball socket 6.8) | - | 1 | Cup block x 2 (POM puck on the lever ball, slides in the beam fork) | any POM rod D10 x 50+ (RS 0514512 seen, page refused) | 5.00 (1) | 1 | 5.00 | UNKNOWN | no 10 mm page opened; est.; brass is the alternative named in the model |
| Flanged plain bushing, sintered oil-impregnated bronze, 4 x 8 x 6, flange 12 x 2 (front wheels) | - | 2 | Front wheel bushing x 2: pressed into the wheel hub from the inner face, flange flush in the D12.4 counterbore, runs on the plain D4 shank of the M4 x 35 axle bolt; the flange runs on the knuckle's seat ring (r 6) | AL04-08-06 oil filled flanged bronze bush (generic, BFMG) https://www.123bearing.com/accessories/bushes/collar/al04-08-06 | 1.85 (1) | 2 | 3.70 | VERIFIED | dimensions read 2026-11-25, 50+ in stock; no tolerances on the page: housing printed D7.90 for a 0.10 mm press (ASSUMED, check with the coupon); alternative RS PRO 529-013 (bag of 8, length and flange not on its page) |
| Shim washers DIN 988 4 x 8, 0.1 .. 1.0 (front wheel end float) | - | 1 | between the wheel hub's outer face and the axle bolt head: fill the 1.4 mm until the wheel turns without end play | DIN 988 4 x 8 shim assortment (generic) | 4.00 (1) | 1 | 4.00 | ASSUMED | est.; no page opened |
| M3 heat-set insert OD 4.6 x 5.7 | - | 4 | servo clips 2, rocker bracket clamps 2 (printed option) or deck | 3DJAKE threaded inserts M3 x 5.7, OD 4.6, 50 pcs https://www.3djake.com/3djake/threaded-inserts-50-piece-set | 3.18 (50) | 1 | 3.18 | VERIFIED | in stock |
| Socket and countersunk screws M3 (pivot blocks, servo clips, clamps), assortment | - | 1 | pivot blocks 2 x 2, servo clips 2 x M3 x 16, bracket clamps 2 | generic ISO 4762 / 10642 M3 assortment | 9.00 (1) | 1 | 9.00 | ASSUMED | est. |
| Knuckle, steel, made to the model's stand-in (parts/swingarm_front.py knuckle: two 4 mm ears, 4 x 12 spine, axle boss, steering arm) | K-STEEL | 2 | Front knuckle x 2 | machined or laser-cut and welded steel part, from a local shop or an online CNC service | 45.00 (1) | 2 | 90.00 | ASSUMED | est.; not quoted; the drawing is the model |
| Knuckle, printed PETG (SU4, owner's choice) | K-PETG | 2 | Front knuckle x 2 | printed | 0.00 (1) | 2 | 0.00 | ASSUMED | FEM landing 2.71, stall at 0.47 N m 2.43 (target 2) |
| Low-profile steering servo, 25T, full torque (with a metal rocker mount and the steel knuckle) | S-A1 | 2 | Steer servo x 2 (model 41 x 20 x 27) | AGF B44BLS FSB44BLS, hobbyfactory.fi https://hobbyfactory.fi/en/p74937/agf-b44bls-digital-brushless-high-speed-servo-14-5kg-0-062s | 68.88 (1) | 2 | 137.76 | VERIFIED | EUR 59.90 incl. VAT (USD at an ASSUMED 1.15); 40.8 x 20.2 x 26.1, 25T, 13.5 kg-cm at 7.4 V; out of stock, earliest 2026-11-28; Version 1's B44BLS V2 page showed contradictory prices |
| Low-profile programmable servo, torque limit about 31 % (owner: printed bracket) | S-A2 | 2 | Steer servo x 2 | Tekin T-180 TEKTT1501, AMain https://amainhobbies.com/tekin-t180-low-profile-digital-hitorque-programmable-servo-high-voltage-tektt1501/p707231 | 159.99 (1) | 2 | 319.98 | VERIFIED | 40 x 20 x 27, 25T, 11.9 kg-cm at 6 V; HotWire 'Torque 0-100 %'; back order; 31 % = 0.367 N m is ASSUMED (the basis of the percentage is not stated): bench check in the manual |
| Tekin HotWire V2.0 interface TT1451 (to set Torque about 31 %, Torque Delay 0 s) | S-A2 | 1 | - | Tekin store https://store.teamtekin.com/ (docs/v2/bought_part_datasheets_2026-11-24.md 1) | 34.99 (1) | 1 | 34.99 | ASSUMED | the page does not say whether the T-180 needs V2.0 (TT1451, 34.99) or V3.0 Bluetooth (TT1452, 64.99) |
| Rocker clamp screw ISO 4762 M3 x 12 + M3 insert (3DJAKE M3 x 5.7, in the row above) | - | 2 | Rocker bracket clamp x 2 | generic ISO 4762 | 0.10 (1) | 2 | 0.20 | ASSUMED | est. |
| Metal servo horn 25T, ball hole at 12 mm | - | 2 | Steer horn x 2 (model: ball at 12 mm radius, on a 7 mm stud) | Tekno RC TKR5253B aluminium 25T horn, Horizon https://assets.horizonhobby.com/product/aluminum-servo-horn-25t-spline-m3-clamp-2-hole-arm/TKR5253B.html | 15.99 (1) | 2 | 31.98 | ASSUMED | hole distances not on the page: 12 mm unconfirmed |
| Ball stud 6.8 mm, M3, with nylock | - | 8 | Knuckle ball stud x 2, Rocker ball stud in/out x 4, horn ball x 2 | HPI 66247 (see the lever row) | 17.49 (10) | 1 | 17.49 | ASSUMED | pack 10 ASSUMED; 10 studs in all |
| M3 nylock DIN 985 | - | 10 | all ball studs | generic DIN 985 A2 (100) | 12.65 (100) | 1 | 12.65 | UNKNOWN | no page opened (Zoro snippet) |
| M3 rod end for a 6.8 mm ball, swivel 25 deg or more (drag link, two per link on a short M3 stud) | - | 4 | Drag link x 2 (16.7 mm ball to ball, NOTES 42) | RC4WD RC4ZS1378 plastic short rod end (body OD 7, ball 6.92) https://amainhobbies.com/rc4wd-m3-m4-plastic-short-rod-ends-2-rc4zs1378/p425162 | 8.99 (2) | 2 | 17.98 | ASSUMED | swivel not stated; pack size contradictory on the page; back order; no 6.8 mm M3 end with a stated 25 deg swivel was found |
| M3 stud 10 mm (drag link body) | - | 2 | Drag link x 2 | generic M3 set screw 10 mm | 0.20 (1) | 2 | 0.40 | ASSUMED | est. |
| M4 rod end for a 6.8 mm ball (tie rods; the knuckle end slim, OD 8.2 at most) | - | 4 | Tie rod x 2, both ends | Tekno RC TKR5188B hard 6.8 mm M4 rod ends (8) https://store.hobbyetc.com/parts/view/132356; slim knuckle end: RC4WD RC4ZS1378 (OD 7) | 11.99 (8) | 1 | 11.99 | ASSUMED | housing OD of TKR5188B not given: use RC4ZS1378 (OD 7) at the knuckle end |
| M4 turnbuckle about 100 mm (tie rod body, about 109 mm ball to ball: 108.84, NOTES 42) | - | 2 | Tie rod x 2 | Tekno RC TKR9423 M4 turnbuckle 100 mm (2) https://store.hobbyetc.com/parts/view/148994 | 16.99 (2) | 1 | 16.99 | ASSUMED | ball-to-ball length with the chosen ends not checked |
| Hardened dowel pin D4 x 24 (rocker pin) | - | 2 | Rocker pin x 2 (model D4 x 24), from the shoe up 7 mm into the hanger (NOTES 42) | ISO 8734 D4 m6 x 24 (McMaster 91595A series; the x 24 part number not looked up) | 9.00 (50) | 1 | 9.00 | ASSUMED | no price: est.; at the ASSUMED stall the pin sees 390 MPa: hardened pin required |
| Rocker bracket, aluminium 6082-T6, the printed bracket's shape machined (same interface) | R-AL | 2 | Rocker bracket x 2 | machined part from the model (parts/swingarm_front.py rocker_bracket), online CNC service | 35.00 (1) | 2 | 70.00 | ASSUMED | est., not quoted; safety about 2.3 at the ASSUMED 2.0 N m stall, about 17 g |
| Rocker bracket B2, steel S235 or 1.4301, same shape | R-STEEL | 2 | Rocker bracket x 2 | machined or welded steel part | 40.00 (1) | 2 | 80.00 | ASSUMED | est., not quoted; safety about 2.2 (S235) / 2.0 (1.4301), about 50 g each |
| Rocker bracket printed PETG (ST2 on plate 01, front suspension and steering) | R-PETG | 2 | Rocker bracket x 2 | printed | 0.00 (1) | 2 | 0.00 | ASSUMED | owner's choice; FEM at 0.47 N m: safety 0.75 (0.96 at 0.367 N m): BELOW 2; needs the torque-limited servo S-A2 |
| Motor + ESC 2-in-1, 540, 1800KV | - | 1 | b motor on the motor plate CH5 | Hobbywing QuicRun Fusion SE HWI30404317, Horizon Hobby https://www.horizonhobby.com/product/quicrun-fusion-se-for-crawler-540spec-1800kv/HWI30404317.html | 89.99 (1) | 1 | 89.99 | VERIFIED | in stock on the listing |
| Motor pinion 32P 15T, 1/8 in (3.175 mm) bore, steel | - | 1 | b motor pinion | ServoCity 15T 1/8 in bore 32P steel https://www.servocity.com/15t-0-125-1-8-bore-32p-shaft-mount-pinion-gear-steel | 5.99 (1) | 1 | 5.99 | VERIFIED | set screw not listed |
| Spur gear 32P 40T, 5 mm bore | - | 1 | b spur gear on the front end of the rear shaft (model D33) | Yeah Racing AXSC-034 40T hardened steel (5 mm shaft), alt. Axial AXI-3860 40T | 14.99 (1) | 1 | 14.99 | ASSUMED | both DISCONTINUED; 15T/40T meshes at 21.8 mm: the motor plate slots (MESH_SLOT +-1 mm) cover 21.5..23.5; a 42T 5 mm-bore spur was not found |
| Ring and pinion, steel (90 deg drive, rear shaft to cross shaft) | - | 1 | b rear ring on the ring hub DT10, pinion on the rear shaft end | Axial AXI232003 30T / 8T, https://www.axialadventure.com/product/AXI232003.html | 19.99 (1) | 1 | 19.99 | ASSUMED | DISCONTINUED, bores not listed; MISMATCH: the model's ring stand-in is 25.6 mm OD, a 30T SCX10 ring is larger: the carrier DT7/DT8 cavity must be checked against the real ring |
| Rear drive shaft, 5 mm stainless (NOT hardened), 112 mm | - | 1 | b rear shaft front + b rear shaft (y -2.5..109.4), in 3 bearings | ServoCity 5 mm x 300 mm stainless precision shafting, cut (both 5 mm shafts from one rod) https://www.servocity.com/5mm-x-300mm-stainless-steel-precision-shafting | 4.79 (1) | 1 | 4.79 | VERIFIED | Version 1's choice (steel_parts_2026-11-24.md): 83 HRB, not hardened; ground to slip-fit a 5 mm bearing |
| Cross (pivot) shaft, 5 mm stainless (NOT hardened), 88 mm | - | ? | x -10..78 through the carriers DT7/DT8, ring hub DT10, pivot pulley DT14 and the 3 pivot plates of DT11 | cut from the same ServoCity 5 x 300 rod (row above) | 0.00 (1) | 0 | 0.00 | VERIFIED | Version 1's choice; 83 HRB, not hardened |
| Rear axle, 8 mm hardened steel, 100 mm | - | 1 | mock rear axle (x -24..76) in DT11, axle pulley DT15, rear hub | VXB 8 x 100 mm HRC60, 5 pcs https://vxb.com/products/8mm-100mm-hardened-linear-motion-shafts | 49.93 (5) | 1 | 49.93 | VERIFIED | price label ambiguous on the page (49.93 / 80.00) |
| Bearing 5x11x4 (MR115) | - | 8 | rear shaft: pillow block DT5/DT6 1, carrier DT7/DT8 2; cross shaft: carrier 2, swing arm pivot plates 3 | MR115-2RS 10 pack, Hobby ETC https://store.hobbyetc.com/parts/view/130884 | 11.49 (10) | 1 | 11.49 | VERIFIED | seat D11.1 |
| Bearing 8x16x5 (688) | - | 2 | rear axle in the swing arm DT11 (seats D16.1 x 5) | 688-2RS 10 pack, Hobby ETC https://store.hobbyetc.com/parts/view/131894 | 10.99 (10) | 1 | 10.99 | VERIFIED |  |
| Retaining ring DIN 471, 5 mm and 8 mm | - | 6 | 2 on the cross shaft, 2 on the rear shaft, 2 on the rear axle | generic assortment | 6.00 (100) | 1 | 6.00 | UNKNOWN | est.; no listing opened |
| Rear hub, 12 mm hex, 8 mm bore | - | 1 | mock rear hub: axle to the rear wheel DT17 | Axial AXI232046 12 mm hex hub (bore not listed) https://store.hobbyetc.com/parts/view/159353 | 11.99 (1) | 1 | 11.99 | UNKNOWN | no 8 mm-bore 12 mm hex hub found: bore a 12 mm hex hub out to 8 mm (2 mm wall) with a grub on an axle flat, or redesign the hub |
| GT2 closed-loop belt 2GT-300, 6 mm wide (150 teeth) | - | 1 | b gt2 belt r: pivot pulley DT14 26T to axle pulley DT15 30T | generic 2GT-300-6 | 7.50 (1) | 1 | 7.50 | ASSUMED | est.; listing 404; centre distance 122.1 gives 300.2 mm: no slack |
| Grub screw M3 x 4 | - | 2 | one in each printed pulley hub DT14, DT15 | generic ISO 4029 | 0.10 (1) | 2 | 0.20 | ASSUMED | est. |
| Rear shock 100 mm class, oil | - | 1 | Rear shock between the cage clevis and the arm bridge | Integy C26651RED (pair) https://integy.shop/products/rock-crawlers-hop-up-parts-c26651red | 36.99 (2) | 1 | 36.99 | ASSUMED | page refused (429): price from search; eye-to-eye, stroke and eye bore not read; model eye bore M6 |
| Shock bolt M6 + M6 nylock: M6 x 30 (lower), M6 x 35 (upper) | - | 2 | M6 shock pin (lower), M6 shock pin | generic ISO 4762 + DIN 985 | 0.50 (1) | 2 | 1.00 | ASSUMED | est.; lengths from the model clevis spans 20.8 / 24.0 mm |
| Socket screw M3 x 6 (motor face) and M3 x 10 (plate to deck) | - | 4 | 2 into the motor on a 25 mm pattern, 2 through the plate gussets into the deck | generic | 0.10 (1) | 4 | 0.40 | ASSUMED | est. |
| Screws for the pillow block and bearing carrier halves | - | ? | DT5/DT6 and DT7/DT8 to the decks | generic M3 | 0.10 (1) | 0 | 0.00 | UNKNOWN | count not taken from the traced carrier meshes |
| Rear tyre 2.2 in, about 140 mm OD | - | 1 | Rear tire on the printed wheel DT17 (bead seat 55.9) | RC4WD Mud Slinger 2 XL 2.2 RC4ZT0122 (pair, with foams) https://www.amainhobbies.com/rc4wd-mud-slinger-2-xl-2.2-scale-crawler-tires-2-rc4zt0122/p-qtqrtarqbc2xactz | 29.99 (2) | 1 | 29.99 | VERIFIED | 139.7 x 53.5 mm, ID 55.5 (0.4 mm tight on the bead seat); backorder; the model's tyre envelope is 62 mm wide |
| Front tyre 1.9 in, about 112 mm OD (Version 2 front) | - | 2 | mock tire x 2 on the printed wheels DT16 | Pro-Line Toyo Open Country R/T G8 1.9 PRO1021114 (pair) https://www.axialadventure.com/product/1-10-toyo-open-country-r-t-g8-f-r-1.9-rock-crawling-tires-2/PRO1021114.html | 24.99 (2) | 1 | 24.99 | VERIFIED | V1 sourcing: 113 x 38 mm; DISCONTINUED; alt. RC4WD Scrambler RC4ZT0144 |
| **All rows (every option once)** | | | | | | | **1424.29** | | |

## 2. Printed parts

Every file is oriented and packed: do not auto-orient in Bambu Studio. L/R parts have their own IDs (L = -x side, the left seen from behind).

| ID | Part | Material | Walls | Infill | Support | Face on the bed | Size mm | Plate |
|---|---|---|---|---|---|---|---|---|
| SU6 | Balance beam | PETG | 6 | 60% | auto | model +z down | 120 x 29 x 30 | Front suspension and steering (PETG) |
| SU7 | Beam carrier (bellcrank) | PETG | 6 | 70% | auto | model +x down | 53 x 72 x 80 | Front suspension and steering (PETG) |
| SU12L | Droop stop pad (L) | TPU 95A | 3 | 100% | off | model +z down | 9 x 7 x 10 | Stops and pads (TPU 95A) |
| SU12R | Droop stop pad (R) | TPU 95A | 3 | 100% | off | model +z down | 9 x 7 x 10 | Stops and pads (TPU 95A) |
| SU3L | Front carrier (L) | PETG | 6 | 70% | auto | model -y down | 65 x 36 x 16 | Front suspension and steering (PETG) |
| SU3R | Front carrier (R) | PETG | 6 | 70% | auto | model -y down | 65 x 36 x 16 | Front suspension and steering (PETG) |
| SU4L | Knuckle (PETG) (L) | PETG | 8 | 100% | auto | model +y down | 39 x 47 x 21 | Front suspension and steering (PETG) |
| SU4R | Knuckle (PETG) (R) | PETG | 8 | 100% | auto | model +y down | 39 x 47 x 21 | Front suspension and steering (PETG) |
| SU8L | Heave saddle, bump (L) | TPU 95A | 3 | 100% | off | model +x down | 8 x 12 x 17 | Stops and pads (TPU 95A) |
| SU8R | Heave saddle, bump (R) | TPU 95A | 3 | 100% | off | model +x down | 8 x 12 x 17 | Stops and pads (TPU 95A) |
| SU9L | Heave saddle, droop (L) | TPU 95A | 3 | 100% | off | model +x down | 7 x 12 x 17 | Stops and pads (TPU 95A) |
| SU9R | Heave saddle, droop (R) | TPU 95A | 3 | 100% | off | model +x down | 7 x 12 x 17 | Stops and pads (TPU 95A) |
| ST1L | Rocker (L) | PETG | 6 | 100% | auto | model +z down | 24 x 20 x 14 | Front suspension and steering (PETG) |
| ST1R | Rocker (R) | PETG | 6 | 100% | auto | model +z down | 24 x 20 x 14 | Front suspension and steering (PETG) |
| ST2L | Rocker bracket (PETG) (L) | PETG | 8 | 100% | auto | model +y down | 37 x 17 x 17 | Front suspension and steering (PETG) |
| ST2R | Rocker bracket (PETG) (R) | PETG | 8 | 100% | auto | model +y down | 37 x 17 x 17 | Front suspension and steering (PETG) |
| ST4L | Servo clip (L) | PETG | 4 | 40% | auto | model +y down | 25 x 38 x 12 | Front suspension and steering (PETG) |
| ST4R | Servo clip (R) | PETG | 4 | 40% | auto | model +y down | 25 x 38 x 12 | Front suspension and steering (PETG) |
| ST3L | Servo floor (L) | PETG | 4 | 40% | auto | model -z down | 36 x 32 x 4 | Front suspension and steering (PETG) |
| ST3R | Servo floor (R) | PETG | 4 | 40% | auto | model -z down | 36 x 32 x 4 | Front suspension and steering (PETG) |
| SU10L | Yaw stop pad, front (L) | TPU 95A | 3 | 100% | off | model +x down | 10 x 8 x 10 | Stops and pads (TPU 95A) |
| SU10R | Yaw stop pad, front (R) | TPU 95A | 3 | 100% | off | model +x down | 10 x 8 x 10 | Stops and pads (TPU 95A) |
| SU11L | Yaw stop pad, rear (L) | TPU 95A | 3 | 100% | off | model +x down | 10 x 8 x 10 | Stops and pads (TPU 95A) |
| SU11R | Yaw stop pad, rear (R) | TPU 95A | 3 | 100% | off | model +x down | 10 x 8 x 10 | Stops and pads (TPU 95A) |
| SU1L | Front swing arm with spring lever (L) | PETG | 6 | 60% | auto | model -x down | 68 x 148 x 34 | Front suspension and steering (PETG) |
| SU1R | Front swing arm with spring lever (R) | PETG | 6 | 60% | auto | model +x down | 68 x 148 x 34 | Front suspension and steering (PETG) |
| DT15 | Rear axle pulley 30T GT2 | PETG | 6 | 100% | off | model -x down | 21 x 21 x 13 | Drivetrain and wheels (PETG) |
| DT14 | Rear pivot pulley 26T GT2 | PETG | 6 | 100% | off | model -x down | 18 x 18 x 13 | Drivetrain and wheels (PETG) |
| SU2L | Upper link (L) | PETG | 5 | 50% | auto | model +z down | 42 x 109 x 16 | Front suspension and steering (PETG) |
| SU2R | Upper link (R) | PETG | 5 | 50% | auto | model +z down | 42 x 109 x 16 | Front suspension and steering (PETG) |
| SU5L | Arm pivot block (L) | PETG | 5 | 60% | auto | model -y down | 30 x 18 x 15 | Front suspension and steering (PETG) |
| SU5R | Arm pivot block (R) | PETG | 5 | 60% | auto | model -y down | 30 x 18 x 15 | Front suspension and steering (PETG) |
| CH5 | Motor plate | PETG | 4 | 40% | auto | model +x down | 46 x 21 x 60 | Drivetrain and wheels (PETG) |
| DT5 | Pillow block lower | PETG | 4 | 60% | auto | model -x down | 22 x 15 x 30 | Drivetrain and wheels (PETG) |
| DT6 | Pillow block upper | PETG | 4 | 60% | auto | model -x down | 22 x 15 x 30 | Drivetrain and wheels (PETG) |
| DT7 | Rear bearing carrier lower | PETG | 4 | 50% | auto | model -x down | 25 x 66 x 34 | Drivetrain and wheels (PETG) |
| DT8 | Rear bearing carrier upper | PETG | 4 | 50% | auto | model -x down | 25 x 66 x 34 | Drivetrain and wheels (PETG) |
| DT11 | Rear swing arm | PETG | 6 | 60% | auto | model +x down | 80 x 150 x 90 | Drivetrain and wheels (PETG) |
| DT10 | Rear ring hub | PETG | 4 | 80% | auto | model +x down | 22 x 22 x 9 | Drivetrain and wheels (PETG) |
| DT16L | Front wheel 1.9 in (L) | PETG | 6 | 60% | off | model +x down | 53 x 53 x 34 | Drivetrain and wheels (PETG) |
| DT16R | Front wheel 1.9 in (R) | PETG | 6 | 60% | off | model -x down | 53 x 53 x 34 | Drivetrain and wheels (PETG) |
| DT17 | Rear wheel 2.2 in | PETG | 6 | 60% | off | model -x down | 61 x 61 x 46 | Drivetrain and wheels (PETG) |

## 3. Plates (Bambu Studio slice check, P2S profile of the saved project, 0.4 mm nozzle)

Open `bambu/all_plates.3mf` or one plate file. PETG in AMS slot 4; the TPU plate needs TPU 95A (external spool). The printed knuckle and rocker bracket are on the front plates now. `bambu/plate_04_Options.3mf` belongs to the FIRST kit (superseded): do NOT print it; the knuckle and rocker bracket are on the front plates.

| Plate file | Time | Grams | Slice check |
|---|---|---|---|
| plate_01_Front_suspension_and_steering_1.3mf | 9.7 h | 246 | OK |
| plate_01_Front_suspension_and_steering_2.3mf | 3.1 h | 98 | OK |
| plate_01_Front_suspension_and_steering_3.3mf | 0.4 h | 13 | OK |
| plate_02_Drivetrain_and_wheels_1.3mf | 10.4 h | 324 | OK |
| plate_02_Drivetrain_and_wheels_2.3mf | 1.3 h | 39 | OK |
| plate_03_Stops_and_pads.3mf | 0.2 h | 4 | OK |
| **Total** | **25.2 h** | **724** | 0 problems |

## 3b. Two paths: print, or buy / machine in metal

`print_or_buy.md` lists every part with its printed version and its off-the-shelf steel or metal version (part numbers, listings opened 2026-11-24). In short: every structural part of the kit prints (default); pins, dowels, shafts, bolts, the sleeve, ball studs, rod ends, turnbuckles, horns, bearings, the belt, gears, shocks, servos and the motor are bought (no printed version carries their loads). No stock metal arm, carrier, knuckle, rocker or beam fits this car; any of them can be machined from its STEP (`soma export "<part>" --format step`), and the knuckle and the rocker mount have a machinist's interface table (`docs/v2/knuckle_rocker_interface.md`). A machined part goes in exactly where the printed one did, with the same pins and screws: the knuckle takes a D4 bore instead of the sleeve and a tapped M4 instead of the insert; the rocker bracket takes a tapped M3 instead of the clamp insert. With a machined rocker bracket the servo's torque limit may be raised (Savox SC-1251MG+ needs no programmer).

## 4. Tools

- Hand drills 3 / 4 / 5 / 6 / 8 mm for reaming, a needle file (arc slots)
- Soldering iron with an insert tip (heat-set inserts)
- Bench vise or arbor press and sockets 11 / 16 mm (rear bearings, outer race only)
- Hex keys 1.5 / 2 / 2.5 / 3 / 4 mm, 5.5 / 7 / 10 mm nut drivers
- Circlip pliers (8 mm ring on the yaw pin, DIN 471 5 and 8 mm), hacksaw and file (D5 axle to 172 mm, shafts)
- A small lathe or drill-press setup for the two POM pucks
- Blue thread locker (metal threads only), CA glue for TPU

## 5. Fits and torque

- REQUIREMENT: front wheel bushing seat D7.90 x 6 in the wheel hub for the 8.0 OD sintered bronze bushing (0.10 mm press, ASSUMED: no housing tolerance on the listing). Press it in square; a sintered bush closes up when pressed, so check it spins free on a D4 pin afterwards and ream to 4.0 only if it binds (reaming smears the pores: re-oil it).
- REQUIREMENT: pad block arc slot 8.3 mm for the D8 heave stop dowel (radial clearance 0.15): ream or file; the dowel must slide along the slot without play across it.
- REQUIREMENT: beam carrier hub bore 6.4 mm on the D6 carrier pin (radial clearance 0.2, loose on purpose): the carrier twists 0.86 deg before the hub bears, the dowel bears at 0.32 deg, so the yaw stop moment goes into the dowel (NOTES decision 16).
- Pin bores: D3.1 for the 3 mm pins, D4.1 for the 4 mm pins, D5.1 for the pivot axle: printed bores come out small, ream by hand to a sliding fit.
- Rear bearing seats D11.1 (5x11x4) and D16.1 (8x16x5): a light press fit in PETG, press on the outer race only.
- Ball studs: M3 through the printed plate with a nylock on top, tightened until the stud stops turning; rod ends must swivel 25 deg (drag link) without binding.
- Torque: printed parts take snug plus a quarter turn, no torque value. Blue thread locker on metal threads only. CA glue for the TPU parts.
- MEASURE ON ARRIVAL (no maker number published; docs/v2/bought_part_datasheets_2026-11-24.md): servo shaft distance from the case's front end (the model has 10.2 mm: a shim at the floor's front rim sets the shaft at y -95.1, it sets the drag link geometry); servo ear overhang and thickness (the mount is designed for up to 8.0 x 2.5 mm and does not use the ears); horn hole radius (12 mm in the model: it sets the steering ratio); M4 ball stud thread length (at most 9 mm into the knuckle insert); rod end housing OD at the knuckle (8.2 mm at most); 6 x 1 tube OD (press fit in the knuckle's D6.0 bore).

## 6. Assembly (in this order)

### 0. Test fit (do this first)

Print the carrier SU3R and the rocker ST1R alone. Ream the pin bores by hand (D3.1 bores with a 3 mm drill, D4.1 with a 4 mm drill) until the hardened pin slides through without play; check one D4 pin in SU3R's kingpin block and one D3 pin in its link eye. Push a 6.8 mm ball stud through ST1R and snap an M3 rod end on: it must swivel 25 deg.

Hardware: 1 x D4 pin, 1 x D3 pin, 1 ball stud + nylock, 1 M3 rod end.

Check: pins slide, do not wobble; if a bore is tight after reaming, the printer over-extrudes: fix the flow before printing the rest.

### 1. Arm pivot

Screw the pivot blocks SU5 L/R to the front deck. Slide the D5 pivot axle through the left block, the left arm SU1L root, the left rocker bracket clamp (option R: see 7), the right ones and the right block. Centre it; the rocker bracket clamps (one M3 each across the slit) hold it.

Hardware: 1 x hardened D5 shaft cut to 172 mm, 2 x M3 x 10 into the deck.

Check: each arm swings 40 mm at the tip by its own weight, no side play.

### 2. Upper links and carriers

Pin each carrier SU3 to its arm tip with a D4 x 55 (tip shaft). Pin the upper link SU2 to the carrier and to the cage rail lug with D3 x 12 pins.

Hardware: 2 x D4 x 55, 4 x D3 x 12.

Check: the carrier stays upright while the arm moves through its whole travel (the parallel link).

### 3. Knuckles and front wheels

Knuckle SU4 (printed PETG, the owner's choice; a machined one with the same interface fits the same way). Press the 6 x 1 steel sleeve (13.6 mm) into the axle bore from the outboard end, flush with the seat ring. Press an M4 heat-set insert (Ruthex M4 x 8.1) into the ball boss from the top, 250 C, straight. Put a 0.5 mm DIN 988 shim on each thrust seat, slide the knuckle over the carrier block and drop the D4 x 40 kingpin through both ears. Screw the 6.8 mm M4 ball stud into the insert from below. Drop an M4 nut into the hex pocket, press a 4 x 8 x 6 flanged bronze bushing into each wheel DT16 from the inner face (flange flush in its counterbore; a vise and two flat plates, square), then push the M4 x 35 12.9 bolt from outboard through the DIN 988 4 x 8 shims, the wheel's bushing and the sleeve into the nut. Shim until the wheel has no end play and still spins free; snug the bolt.

Hardware: per side: knuckle, sleeve, M4 insert, 2 x DIN 988 4 x 8 x 0.5, D4 x 40 kingpin, M4 ball stud, M4 x 35 12.9 + M4 nut, flanged bronze bushing 4 x 8 x 6, DIN 988 4 x 8 shims, front tyre.

Check: the knuckle turns 20 deg each way without binding; the wheel spins on its bushing without wobble (owner: 'plain bushings').

### 4. Spring levers and cup blocks

Screw a 6.8 mm ball stud into each lever (arm root, x 54). Turn two pucks of D9 x 9 from the POM rod with the 6.8 mm ball socket and the 20 deg neck (drawing: parts/swingarm_front.py cup_block) and press each onto its ball.

Hardware: 2 x ball stud 6.8, POM rod D10.

Check: puck turns on the ball without play.

### 5. Beam carrier, heave shock and stops

Glue the TPU saddles SU8/SU9 into the cage pad blocks and the yaw pads SU10/SU11 onto the carrier SU7 posts (CA for TPU). Ream the pad block arc slots to 8.3 mm (REQUIREMENT: radial clearance 0.15 on the D8 dowel) and the carrier hub bore to 6.4 mm (REQUIREMENT: loose on purpose, so the dowel takes the yaw moment first). Pin the carrier into the cage clevis with the D6 x 35. Push the D8 x 75 heave stop dowel through the carrier tail and both pad block slots. Mount the heave shock between the carrier fork and the cage rear clevis with the two D3 pins.

Hardware: D6 x 35, D8 x 75, heave shock, 2 x D3 x 12.

Check: push both wheels up 12 mm: the dowel lands on the bump saddles; let them drop 8 mm: on the droop saddles.

### 6. Balance beam

Set the beam SU6 on the D8 yaw pin in the carrier, clip it. Guide the cup blocks into the beam's end forks as the beam goes down.

Hardware: D8 x 30 yaw pin + 8 mm retaining ring.

Check: push one wheel up: the other goes down by the same amount (lean); lean 10 deg: the beam lands on the yaw pads.

### 7. Steering

Rocker bracket ST2 (printed PETG, the owner's choice; a machined one with the same interface fits the same way): press an M3 insert into the hanger under the clamp screw hole, slide the bracket onto the pivot axle (step 1) and tighten the M3 x 12 clamp screw from the top. Drop the D4 x 20 rocker pin through the foot and the rocker ST1 into the blind shoe bore (the arm boss above keeps it in). Seat the servo floor ST3 on the deck, lay the servo in, fit the servo clip ST4 (one M3 x 16 through the deck and the tongue into the insert). SERVO (Tekin T-180, before fitting): with the HotWire interface set Torque about 31 % (the steering need, owner's choice NOTES 32), Torque Delay 0 s, and limit Travel so the horn swings no more than about +-28 deg (the wheels reach the 18.5 deg design lock). The 31 % is ASSUMED to give 0.367 N m at 6.0 V (the maker does not state the base of the percentage): BENCH CHECK it: block the horn and pull at the ball (12 mm from the shaft) with a spring scale while commanding full throw: it must hold at least 31 N (0.367 N m, what the steering needs). If it does not, raise the percentage until it does and WRITE DOWN the % used: the printed rocker mount's safety falls from about 0.96 (31 %) toward 0.75 (40 %). A servo lower than the Tekin's 27 mm gets a printed shim between its base and the clip leg so its top face stays at x 58.5 (Savox SC-1251MG+: 1.4 mm). Centre the servo (1500 us), fit the metal horn. Drag link: two M3 rod ends on a short M3 stud, horn ball to the rocker input ball. Tie rod: M4 rod ends (the knuckle end the slim one) from the rocker output ball to the knuckle ball. SERVICE: a steering servo comes out only with the beam assembly off (step 6 backwards).

Hardware: 2 servos, 2 metal horns, 2 x D4 x 20, 4 x M3 rod end, 4 x M4 rod end, 2 x M4 rod, 6 x ball stud 6.8 + nylock, 2 x M3 x 16 + insert.

Check: at horn +-30 deg the wheels steer 16.3 / 19.5 deg; bump steer is about 0.02 deg (move a wheel up and down at centre: the toe must not move).

### 8. Droop pads

Glue the TPU droop pads SU12 onto the upper link rail lugs (CA).

Hardware: -.

Check: lever blade lands flat on the pad at full droop.

### 9. Rear drive shaft and carriers (as Version 1)

Press 5x11x4 bearings into the pillow block DT5 (1) and the rear bearing carrier DT7 (2 on the rear shaft, 2 on the cross shaft). Spur on the front end of the rear shaft, pinion on its rear end, ring on the ring hub DT10, close DT6 and DT8.

Hardware: 5 x 5x11x4, shafts, spur, ring and pinion, 2 x DIN 471 5 mm.

Check: turning the spur turns the cross shaft smoothly over a full ring turn.

### 10. Motor

Motor on the motor plate CH5 with the 15T pinion, set the mesh with a paper strip, plate to the deck.

Hardware: motor/ESC, 15T pinion, 4 x M3.

Check: a tooth print on the paper, not torn.

### 11. Rear swing arm

Three 5x11x4 in the pivot plates of DT11, two 8x16x5 in the axle plates. Pivot pulley DT14 on the cross shaft, belt on, axle pulley DT15 and the rear hub on the 8 mm axle, rear wheel DT17 with its tyre.

Hardware: 3 x 5x11x4, 2 x 8x16x5, axle 8 x 100, belt 2GT-300-6, 2 x M3 x 4 grub, 4 x DIN 471, rear hub, rear tyre.

Check: the arm swings freely with the belt on; belt deflects about 3 mm under a finger.

### 12. Rear shock

Bolt the shock between the cage clevis (M6 x 35) and the arm bridge (M6 x 30), nylocks snug.

Hardware: rear shock, M6 x 30, M6 x 35, 2 x M6 nylock.

Check: the tyre does not touch the tail at full bump.
