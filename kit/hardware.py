"""Bought parts of the Fugu Version 2 suspension + drivetrain kit, one row per item: the single source for kit/susp_drive/README.md (make_susp_drive.py).

Fields: grp (SU front suspension, ST steering, DT drivetrain), item, qty (counted from the model, see `where`), where, part (brand + part number,
retailer URL), unit (USD per pack as listed), pack (pieces per pack), status, note, opt (None = always; else the option ID it belongs to).
Status: VERIFIED = listing opened and its spec read on SRC_DATE; ASSUMED = listing opened but a spec or the price is missing, or a generic part
whose price is an estimate ("est."); UNKNOWN = no listing opened for a qualifying part. Packs to buy = ceil(qty / pack).
Sourcing: kit/sourcing_v2_2026-11-24.md (front end, this run) and Version 1's kit/sourcing_2026-11-24.md (drivetrain, tyres, bearings, heave shock:
the same choices as Version 1, rows copied unchanged). Options: OPTIONS below; the owner chooses, nothing is bought.
"""
import math

SRC_DATE = "2026-11-24"
EUR = 1.15            # USD per EUR, ASSUMED rate for the one EUR listing
H = lambda **k: dict(dict(opt=None), **k)
SV2 = "kit/sourcing_v2_2026-11-24.md"
HW = [
    # --- front suspension -------------------------------------------------------------------------------------------------------------------------
    H(grp="SU", item="Heave shock 70 mm (one, between the beam carrier and the cage)", qty=1, where="Heave shock body/shaft (model 68.0 mm eye to eye, D3 eye pins)",
      part="RC4WD Ultimate Scale Shock 70 mm RC4Z-D0004, Hobby ETC https://store.hobbyetc.com/parts/view/29506 (Version 1 choice)", unit=20.99, pack=2,
      status="VERIFIED", note="V1 sourcing: 72.5 ext / 55.1 comp, stroke 17.4, eye bore 3.0 (fits the D3 pins); DISCONTINUED; internal spring "super soft", body D8.8, shaft D3.0, no external retainer (NOTES 58, 60); no rate stated: 3 N/mm ASSUMED (tools/arm_stops.py), measure on arrival"),
    H(grp="SU", item="Plain sleeve, oil-filled sintered bronze, 8 x 12 x 12 (SKF PSM 8-12-12 A51)", qty=3, where="Two in the beam carrier's tail boss, one pressed in from each end (printed bore 11.9), the D8 heave stop dowel running in them (NOTES 60). Yaw pin sleeve: pressed into the top 12 mm of the beam carrier's yaw pin bore (printed bore 11.9), the D8 yaw pin pressed into it (NOTES 57)",
      part="SKF PSM 8-12-12 A51, 123bearing.com https://www.123bearing.com/accessories/bushes/bronze/psm8-12-12-a51-skf (opened 2026-11-25)", unit=7.67, pack=1,
      status="VERIFIED", note="ID 8, OD 12, length 12, 5 g; 2 in stock on 2026-11-25, 3 needed: a second seller or a restock to be found; press fit in PETG ASSUMED (ream / coupon)"),
    H(grp="SU", item="Hardened dowel pin D3 x 12 (ISO 8734 m6)", qty=6, where="Uplink pin inner/outer x 2 sides (4), Shock pin front/rear (2)",
      part="McMaster 91595A112 (pack 50) https://www.mcmaster.com/products/dowel-pins/specifications-met~iso-8734/", unit=9.00, pack=50, status="ASSUMED", note="no price on the page: est.; C52"),
    H(grp="SU", item="Hardened dowel pin D4 x 55 (arm tip shaft)", qty=2, where="tip shaft x 2 (model D4 x 54)", part="McMaster 91595A188 D4 x 50 (25) or 91595A192 D4 x 60 (10)", unit=12.00, pack=10,
      status="ASSUMED", note="55 is not in the series: 60 mm cut and deburred to 54; no price on the page: est."),
    H(grp="SU", item="Hardened dowel pin D4 x 40 (kingpin)", qty=2, where="Kingpin x 2 (model D4 x 39)", part="McMaster 91595A185 (25)", unit=11.00, pack=25, status="ASSUMED", note="no price: est."),
    H(grp="SU", item="Axle bolt ISO 4762 M4 x 35, class 12.9 (partly threaded, b 20) + ISO 4032 M4 nut", qty=2, where="Front axle bolt / nut x 2 through the knuckle sleeve and the wheel hub",
      part="generic ISO 4762 12.9 / ISO 4032 (dimensions: docs/v2/bought_part_datasheets_2026-11-24.md 6b, 6c)", unit=0.80, pack=1, status="ASSUMED", note="est.; 12.9 required (8.8 safety 1.89 at the seat); thread must stay inboard of x 140"),
    H(grp="SU", item="Steel tube 6 x 1 (ID 4), cut to 13.6 mm (knuckle axle sleeve, pressed in)", qty=2, where="Axle sleeve x 2",
      part="BES 13086 316L seamless 6 x 1, 3 m https://www.bes.co.uk/ (docs/v2/bought_part_datasheets_2026-11-24.md 7)", unit=40.90, pack=1, status="ASSUMED", note="GBP 35.52 ex VAT at an ASSUMED 1.15; no tolerance on the page: MEASURE ON ARRIVAL (OD 6.0 press fit)", opt="K-PETG"),
    H(grp="SU", item="Shim ring DIN 988 4 x 8 x 0.5 (knuckle thrust seats)", qty=4, where="between each knuckle ear seat and the carrier kingpin block",
      part="Aspen Fasteners DIN 988 4 x 8 x 0.5 A2 (docs/v2/bought_part_datasheets_2026-11-24.md 6d)", unit=0.40, pack=1, status="ASSUMED", note="est. price"),
    H(grp="SU", item="Ball stud 6.8 mm, M4 (knuckle, tie rod)", qty=2, where="Knuckle ball stud x 2, screwed into the M4 insert from below",
      part="Tekno RC TKR5052A 6.8 mm aluminium pivot balls (4)", unit=7.99, pack=4, status="UNKNOWN", note="page 403: dimensions not read: MEASURE ON ARRIVAL (thread length into the 9 mm insert pocket)"),
    H(grp="SU", item="M4 heat-set insert Ruthex M4 x 8.1 (hole 5.6)", qty=2, where="knuckle ball boss, pressed in from the top",
      part="Ruthex M4 x 8.1, 50 pcs, 3DJAKE https://www.3djake.com/ruthex/threaded-inserts-m4-50-pieces", unit=6.94, pack=50, status="VERIFIED", note="hole D3 5.6 read from the table (labels not in text: likely)", opt="K-PETG"),
    H(grp="SU", item="Hardened shaft D5, cut to 172 mm (arm pivot axle)", qty=1, where="b front pivot axle (D5 x 172) through the pivot blocks, both arm roots and the rocker bracket clamps",
      part="Ooznest HGLS-CP-5-CTS 5 mm chrome hardened ground shaft, cut to size https://ooznest.co.uk/5mm-chrome-plated-hardened-ground-bearing-linear-shaft-cut-to-size", unit=8.90, pack=1,
      status="ASSUMED", note="GBP 7.74 incl. VAT; whether it is per any length is unclear on the page; UK shop"),
    H(grp="SU", item="Hardened dowel pin D6 x 35 (carrier pin)", qty=1, where="Carrier pin (model D6 x 35) in the cage clevis, beam carrier hub bore 6.4", part="McMaster 91595A153 D6 x 34 (1)", unit=1.80, pack=1, status="ASSUMED", note="no price: est.; 34 mm instead of 35 fits the clevis span (check)"),
    H(grp="SU", item="Hardened dowel pin D8 x 75 (heave stop dowel)", qty=1, where="Heave stop dowel (model D8 x 75) through the carrier tail and the pad block slots", part="McMaster 91595A591 D8 x 80 (5), cut to 75", unit=14.00, pack=5, status="ASSUMED", note="75 not in the series; no price: est."),
    H(grp="SU", item="Hardened dowel pin D8 x 30 (yaw pin) + 8 mm retaining ring", qty=1, where="Beam yaw pin + Beam yaw pin clip", part="McMaster 91595A567 (10) + DIN 471 ring from an assortment (98875A150)", unit=18.00, pack=1, status="ASSUMED", note="no prices: est.; the pin needs a ring groove (DIN 471 8 mm groove 7.6 x 0.9): grind it or use DIN 6799 on a turned groove"),
    H(grp="SU", item="Ball stud 6.8 mm, M3 (lever balls)", qty=2, where="Lever ball stud x 2 in the arm roots", part="HPI 66247 6.8 mm ball stud https://www.tradeinn.com/techinn/en/hpi-racing-6.8mm-ball-stud/143442104/p", unit=17.49, pack=10, status="ASSUMED", note="thread and pack count not on the page (pack 10 ASSUMED); shared with the steering studs below"),
    H(grp="SU", item="POM (acetal) rod D10 for the two cup blocks (turned D9 x 9, ball socket 6.8)", qty=1, where="Cup block x 2 (POM puck on the lever ball, slides in the beam fork)", part="any POM rod D10 x 50+ (RS 0514512 seen, page refused)", unit=5.00, pack=1, status="UNKNOWN", note="no 10 mm page opened; est.; brass is the alternative named in the model"),
    H(grp="SU", item="Flanged plain bushing, sintered oil-impregnated bronze, 4 x 8 x 6, flange 12 x 2 (front wheels)", qty=2, where="Front wheel bushing x 2: pressed into the wheel hub from the inner face, flange flush in the D12.4 counterbore, runs on the plain D4 shank of the M4 x 35 axle bolt; the flange runs on the knuckle's seat ring (r 6)", part="AL04-08-06 oil filled flanged bronze bush (generic, BFMG) https://www.123bearing.com/accessories/bushes/collar/al04-08-06", unit=1.85, pack=1, status="VERIFIED", note="dimensions read 2026-11-25, 50+ in stock; no tolerances on the page: housing printed D7.90 for a 0.10 mm press (ASSUMED, check with the coupon); alternative RS PRO 529-013 (bag of 8, length and flange not on its page)"),
    H(grp="SU", item="Shim washers DIN 988 4 x 8, 0.1 .. 1.0 (front wheel end float)", qty=1, where="between the wheel hub's outer face and the axle bolt head: fill the 1.4 mm until the wheel turns without end play", part="DIN 988 4 x 8 shim assortment (generic)", unit=4.00, pack=1, status="ASSUMED", note="est.; no page opened"),
    H(grp="SU", item="M3 heat-set insert OD 4.6 x 5.7", qty=4, where="servo clips 2, rocker bracket clamps 2 (printed option) or deck", part="3DJAKE threaded inserts M3 x 5.7, OD 4.6, 50 pcs https://www.3djake.com/3djake/threaded-inserts-50-piece-set", unit=3.18, pack=50, status="VERIFIED", note="in stock"),
    H(grp="SU", item="Socket and countersunk screws M3 (pivot blocks, servo clips, clamps), assortment", qty=1, where="pivot blocks 2 x 2, servo clips 2 x M3 x 16, bracket clamps 2", part="generic ISO 4762 / 10642 M3 assortment", unit=9.00, pack=1, status="ASSUMED", note="est."),
    # --- knuckle option (owner's decision) ---------------------------------------------------------------------------------------------------------
    H(grp="SU", item="Knuckle, steel, made to the model's stand-in (parts/swingarm_front.py knuckle: two 4 mm ears, 4 x 12 spine, axle boss, steering arm)", qty=2, where="Front knuckle x 2",
      part="machined or laser-cut and welded steel part, from a local shop or an online CNC service", unit=45.00, pack=1, status="ASSUMED", note="est.; not quoted; the drawing is the model", opt="K-STEEL"),
    H(grp="SU", item="Knuckle, printed PETG (SU4, owner's choice)", qty=2, where="Front knuckle x 2", part="printed", unit=0.00, pack=1, status="ASSUMED",
      note="FEM landing 2.71, stall at 0.47 N m 2.43 (target 2)", opt="K-PETG"),
    # --- steering -----------------------------------------------------------------------------------------------------------------------------------
    H(grp="ST", item="Low-profile steering servo, 25T, full torque (with a metal rocker mount and the steel knuckle)", qty=2, where="Steer servo x 2 (model 41 x 20 x 27)",
      part="AGF B44BLS FSB44BLS, hobbyfactory.fi https://hobbyfactory.fi/en/p74937/agf-b44bls-digital-brushless-high-speed-servo-14-5kg-0-062s", unit=round(59.90 * EUR, 2), pack=1,
      status="VERIFIED", note="EUR 59.90 incl. VAT (USD at an ASSUMED 1.15); 40.8 x 20.2 x 26.1, 25T, 13.5 kg-cm at 7.4 V; out of stock, earliest 2026-11-28; Version 1's B44BLS V2 page showed contradictory prices", opt="S-A1"),
    H(grp="ST", item="Low-profile programmable servo, torque limit about 31 % (owner: printed bracket)", qty=2, where="Steer servo x 2",
      part="Tekin T-180 TEKTT1501, AMain https://amainhobbies.com/tekin-t180-low-profile-digital-hitorque-programmable-servo-high-voltage-tektt1501/p707231", unit=159.99, pack=1,
      status="VERIFIED", note="40 x 20 x 27, 25T, 11.9 kg-cm at 6 V; HotWire 'Torque 0-100 %'; back order; 31 % = 0.367 N m is ASSUMED (the basis of the percentage is not stated): bench check in the manual", opt="S-A2"),
    H(grp="ST", item="Tekin HotWire V2.0 interface TT1451 (to set Torque about 31 %, Torque Delay 0 s)", qty=1, where="-", part="Tekin store https://store.teamtekin.com/ (docs/v2/bought_part_datasheets_2026-11-24.md 1)", unit=34.99, pack=1, status="ASSUMED",
      note="the page does not say whether the T-180 needs V2.0 (TT1451, 34.99) or V3.0 Bluetooth (TT1452, 64.99)", opt="S-A2"),
    H(grp="ST", item="Rocker clamp screw ISO 4762 M3 x 12 + M3 insert (3DJAKE M3 x 5.7, in the row above)", qty=2, where="Rocker bracket clamp x 2", part="generic ISO 4762", unit=0.10, pack=1, status="ASSUMED", note="est."),
    H(grp="ST", item="Metal servo horn 25T, ball hole at 12 mm", qty=2, where="Steer horn x 2 (model: ball at 12 mm radius, on a 7 mm stud)", part="Tekno RC TKR5253B aluminium 25T horn, Horizon https://assets.horizonhobby.com/product/aluminum-servo-horn-25t-spline-m3-clamp-2-hole-arm/TKR5253B.html", unit=15.99, pack=1, status="ASSUMED", note="hole distances not on the page: 12 mm unconfirmed"),
    H(grp="ST", item="Ball stud 6.8 mm, M3, with nylock", qty=8, where="Knuckle ball stud x 2, Rocker ball stud in/out x 4, horn ball x 2", part="HPI 66247 (see the lever row)", unit=17.49, pack=10, status="ASSUMED", note="pack 10 ASSUMED; 10 studs in all"),
    H(grp="ST", item="M3 nylock DIN 985", qty=10, where="all ball studs", part="generic DIN 985 A2 (100)", unit=12.65, pack=100, status="UNKNOWN", note="no page opened (Zoro snippet)"),
    H(grp="ST", item="M3 rod end for a 6.8 mm ball, swivel 25 deg or more (drag link, two per link on a short M3 stud)", qty=4, where="Drag link x 2 (16.7 mm ball to ball, NOTES 42)", part="RC4WD RC4ZS1378 plastic short rod end (body OD 7, ball 6.92) https://amainhobbies.com/rc4wd-m3-m4-plastic-short-rod-ends-2-rc4zs1378/p425162", unit=8.99, pack=2,
      status="ASSUMED", note="swivel not stated; pack size contradictory on the page; back order; no 6.8 mm M3 end with a stated 25 deg swivel was found"),
    H(grp="ST", item="M3 stud 10 mm (drag link body)", qty=2, where="Drag link x 2", part="generic M3 set screw 10 mm", unit=0.20, pack=1, status="ASSUMED", note="est."),
    H(grp="ST", item="M4 rod end for a 6.8 mm ball (tie rods; the knuckle end slim, OD 8.2 at most)", qty=4, where="Tie rod x 2, both ends", part="Tekno RC TKR5188B hard 6.8 mm M4 rod ends (8) https://store.hobbyetc.com/parts/view/132356; slim knuckle end: RC4WD RC4ZS1378 (OD 7)", unit=11.99, pack=8,
      status="ASSUMED", note="housing OD of TKR5188B not given: use RC4ZS1378 (OD 7) at the knuckle end"),
    H(grp="ST", item="M4 turnbuckle about 100 mm (tie rod body, about 109 mm ball to ball: 108.84, NOTES 42)", qty=2, where="Tie rod x 2", part="Tekno RC TKR9423 M4 turnbuckle 100 mm (2) https://store.hobbyetc.com/parts/view/148994", unit=16.99, pack=2, status="ASSUMED", note="ball-to-ball length with the chosen ends not checked"),
    H(grp="ST", item="Hardened dowel pin D4 x 24 (rocker pin)", qty=2, where="Rocker pin x 2 (model D4 x 24), from the shoe up 7 mm into the hanger (NOTES 42)", part="ISO 8734 D4 m6 x 24 (McMaster 91595A series; the x 24 part number not looked up)", unit=9.00, pack=50, status="ASSUMED", note="no price: est.; at the ASSUMED stall the pin sees 390 MPa: hardened pin required"),
    # --- rocker mount option (owner's decision) ------------------------------------------------------------------------------------------------------
    H(grp="ST", item="Rocker bracket, aluminium 6082-T6, the printed bracket's shape machined (same interface)", qty=2, where="Rocker bracket x 2",
      part="machined part from the model (parts/swingarm_front.py rocker_bracket), online CNC service", unit=35.00, pack=1, status="ASSUMED", note="est., not quoted; safety about 2.3 at the ASSUMED 2.0 N m stall, about 17 g", opt="R-AL"),
    H(grp="ST", item="Rocker bracket B2, steel S235 or 1.4301, same shape", qty=2, where="Rocker bracket x 2", part="machined or welded steel part", unit=40.00, pack=1, status="ASSUMED",
      note="est., not quoted; safety about 2.2 (S235) / 2.0 (1.4301), about 50 g each", opt="R-STEEL"),
    H(grp="ST", item="Rocker bracket printed PETG (ST2 on plate 01, front suspension and steering)", qty=2, where="Rocker bracket x 2", part="printed", unit=0.00, pack=1, status="ASSUMED",
      note="owner's choice; FEM at 0.47 N m: safety 0.75 (0.96 at 0.367 N m): BELOW 2; needs the torque-limited servo S-A2", opt="R-PETG"),
    # --- drivetrain: Version 1 rows, the same choices (V1 sourcing 2026-11-24); counts checked against the V2 model (same rear end) -------------------------------------------------------------------------------------------------------------------------------------
    H(grp="DT", item="Motor + ESC 2-in-1, 540, 1800KV", qty=1, where="b motor on the motor plate CH5", part="Hobbywing QuicRun Fusion SE HWI30404317, Horizon Hobby https://www.horizonhobby.com/product/quicrun-fusion-se-for-crawler-540spec-1800kv/HWI30404317.html", unit=89.99, pack=1, status="VERIFIED", note="in stock on the listing"),
    H(grp="DT", item="Motor pinion 32P 15T, 1/8 in (3.175 mm) bore, steel", qty=1, where="b motor pinion", part="ServoCity 15T 1/8 in bore 32P steel https://www.servocity.com/15t-0-125-1-8-bore-32p-shaft-mount-pinion-gear-steel", unit=5.99, pack=1, status="VERIFIED", note="set screw not listed"),
    H(grp="DT", item="Spur gear 32P 40T, 5 mm bore", qty=1, where="b spur gear on the front end of the rear shaft (model D33)", part="Yeah Racing AXSC-034 40T hardened steel (5 mm shaft), alt. Axial AXI-3860 40T", unit=14.99, pack=1,
      status="ASSUMED", note="both DISCONTINUED; 15T/40T meshes at 21.8 mm: the motor plate slots (MESH_SLOT +-1 mm) cover 21.5..23.5; a 42T 5 mm-bore spur was not found"),
    H(grp="DT", item="Ring and pinion, steel (90 deg drive, rear shaft to cross shaft)", qty=1, where="b rear ring on the ring hub DT10, pinion on the rear shaft end", part="Axial AXI232003 30T / 8T, https://www.axialadventure.com/product/AXI232003.html", unit=19.99, pack=1,
      status="ASSUMED", note="DISCONTINUED, bores not listed; MISMATCH: the model's ring stand-in is 25.6 mm OD, a 30T SCX10 ring is larger: the carrier DT7/DT8 cavity must be checked against the real ring"),
    H(grp="DT", item="Rear drive shaft, 5 mm stainless (NOT hardened), 112 mm", qty=1, where="b rear shaft front + b rear shaft (y -2.5..109.4), in 3 bearings", part="ServoCity 5 mm x 300 mm stainless precision shafting, cut (both 5 mm shafts from one rod) https://www.servocity.com/5mm-x-300mm-stainless-steel-precision-shafting", unit=4.79, pack=1, status="VERIFIED", note="Version 1's choice (steel_parts_2026-11-24.md): 83 HRB, not hardened; ground to slip-fit a 5 mm bearing"),
    H(grp="DT", item="Cross (pivot) shaft, 5 mm stainless (NOT hardened), 88 mm", qty=0, where="x -10..78 through the carriers DT7/DT8, ring hub DT10, pivot pulley DT14 and the 3 pivot plates of DT11", part="cut from the same ServoCity 5 x 300 rod (row above)", unit=0.0, pack=1, status="VERIFIED", note="Version 1's choice; 83 HRB, not hardened"),
    H(grp="DT", item="Rear axle, 8 mm hardened steel, 100 mm", qty=1, where="mock rear axle (x -24..76) in DT11, axle pulley DT15, rear hub", part="VXB 8 x 100 mm HRC60, 5 pcs https://vxb.com/products/8mm-100mm-hardened-linear-motion-shafts", unit=49.93, pack=5, status="VERIFIED", note="price label ambiguous on the page (49.93 / 80.00)"),
    H(grp="DT", item="Bearing 5x11x4 (MR115)", qty=8, where="rear shaft: pillow block DT5/DT6 1, carrier DT7/DT8 2; cross shaft: carrier 2, swing arm pivot plates 3", part="MR115-2RS 10 pack, Hobby ETC https://store.hobbyetc.com/parts/view/130884", unit=11.49, pack=10, status="VERIFIED", note="seat D11.1"),
    H(grp="DT", item="Bearing 8x16x5 (688)", qty=2, where="rear axle in the swing arm DT11 (seats D16.1 x 5)", part="688-2RS 10 pack, Hobby ETC https://store.hobbyetc.com/parts/view/131894", unit=10.99, pack=10, status="VERIFIED", note=""),
    H(grp="DT", item="Retaining ring DIN 471, 5 mm and 8 mm", qty=6, where="2 on the cross shaft, 2 on the rear shaft, 2 on the rear axle", part="generic assortment", unit=6.00, pack=100, status="UNKNOWN", note="est.; no listing opened"),
    H(grp="DT", item="Rear hub, 12 mm hex, 8 mm bore", qty=1, where="mock rear hub: axle to the rear wheel DT17", part="Axial AXI232046 12 mm hex hub (bore not listed) https://store.hobbyetc.com/parts/view/159353", unit=11.99, pack=1,
      status="UNKNOWN", note="no 8 mm-bore 12 mm hex hub found: bore a 12 mm hex hub out to 8 mm (2 mm wall) with a grub on an axle flat, or redesign the hub"),
    H(grp="DT", item="GT2 closed-loop belt 2GT-300, 6 mm wide (150 teeth)", qty=1, where="b gt2 belt r: pivot pulley DT14 26T to axle pulley DT15 30T", part="generic 2GT-300-6", unit=7.50, pack=1, status="ASSUMED", note="est.; listing 404; centre distance 122.1 gives 300.2 mm: no slack"),
    H(grp="DT", item="Grub screw M3 x 4", qty=2, where="one in each printed pulley hub DT14, DT15", part="generic ISO 4029", unit=0.10, pack=1, status="ASSUMED", note="est."),
    H(grp="DT", item="Rear shock 100 mm class, oil", qty=1, where="Rear shock between the cage clevis and the arm bridge", part="Integy C26651RED (pair) https://integy.shop/products/rock-crawlers-hop-up-parts-c26651red", unit=36.99, pack=2, status="ASSUMED", note="page refused (429): price from search; eye-to-eye, stroke and eye bore not read; model eye bore M6"),
    H(grp="DT", item="Shock bolt M6 + M6 nylock: M6 x 30 (lower), M6 x 35 (upper)", qty=2, where="M6 shock pin (lower), M6 shock pin", part="generic ISO 4762 + DIN 985", unit=0.50, pack=1, status="ASSUMED", note="est.; lengths from the model clevis spans 20.8 / 24.0 mm"),
    H(grp="DT", item="Socket screw M3 x 6 (motor face) and M3 x 10 (plate to deck)", qty=4, where="2 into the motor on a 25 mm pattern, 2 through the plate gussets into the deck", part="generic", unit=0.10, pack=1, status="ASSUMED", note="est."),
    H(grp="DT", item="Screws for the pillow block and bearing carrier halves", qty=0, where="DT5/DT6 and DT7/DT8 to the decks", part="generic M3", unit=0.10, pack=1, status="UNKNOWN", note="count not taken from the traced carrier meshes"),
    H(grp="DT", item="Rear tyre 2.2 in, about 140 mm OD", qty=1, where="Rear tire on the printed wheel DT17 (bead seat 55.9)", part="RC4WD Mud Slinger 2 XL 2.2 RC4ZT0122 (pair, with foams) https://www.amainhobbies.com/rc4wd-mud-slinger-2-xl-2.2-scale-crawler-tires-2-rc4zt0122/p-qtqrtarqbc2xactz", unit=29.99, pack=2,
      status="VERIFIED", note="139.7 x 53.5 mm, ID 55.5 (0.4 mm tight on the bead seat); backorder; the model's tyre envelope is 62 mm wide"),
    H(grp="DT", item="Front tyre 1.9 in, about 112 mm OD (Version 2 front)", qty=2, where="mock tire x 2 on the printed wheels DT16", part="Pro-Line Toyo Open Country R/T G8 1.9 PRO1021114 (pair) https://www.axialadventure.com/product/1-10-toyo-open-country-r-t-g8-f-r-1.9-rock-crawling-tires-2/PRO1021114.html", unit=24.99, pack=2,
      status="VERIFIED", note="V1 sourcing: 113 x 38 mm; DISCONTINUED; alt. RC4WD Scrambler RC4ZT0144"),
]

OPTIONS = {     # the owner's choice first (NOTES 26: "Printed + servo"), then the alternatives with the same interface (docs/v2/knuckle_rocker_interface.md)
    "Rocker mount": [dict(id="R-PETG", rec=True, what="printed bracket ST2 (FEM at 0.47 N m: safety 0.75, below 2)", adds="the torque-limited servo S-A2"),
                     dict(id="R-AL", what="aluminium 6082 bracket, the same shape machined (safety about 2.3 at 2.0 N m), any servo", adds="2 machined brackets"),
                     dict(id="R-STEEL", what="steel bracket, the same shape machined (safety about 2.2), any servo", adds="2 machined brackets")],
    "Knuckle": [dict(id="K-PETG", rec=True, what="printed knuckle SU4 (FEM landing 2.71, stall at 0.47 N m 2.43)", adds="sleeve, M4 inserts, the torque-limited servo S-A2"),
                dict(id="K-STEEL", what="the same knuckle machined in steel or 7075 (D4 bore, tapped M4), same interface", adds="2 machined knuckles")],
}
DEFAULT = ("R-PETG", "K-PETG")

for _o in [o for v in OPTIONS.values() for o in v]:      # price of an option = its own rows (the servo difference shows in the totals)
    _o["price"] = None


def cost(r):
    """(packs, line total) for one row."""
    packs = math.ceil(r["qty"] / r["pack"]) if r["qty"] else 0
    return packs, round(packs * r["unit"], 2)


for _o in [o for v in OPTIONS.values() for o in v]:
    _o["price"] = sum(cost(r)[1] for r in HW if r["opt"] == _o["id"])


def option_totals():
    base = sum(cost(r)[1] for r in HW if r["opt"] is None)
    out = []
    for R in ("R-PETG", "R-AL", "R-STEEL"):
        for K in ("K-PETG", "K-STEEL"):
            S = "S-A2" if "PETG" in R or "PETG" in K else "S-A1"
            t = base + sum(cost(r)[1] for r in HW if r["opt"] in (R, K, S))
            out.append(dict(R=R, K=K, servo=S, total=round(t, 2)))
    return out


def table(rows):
    L = ["| Item | Option | Qty | Where (model) | Part / retailer | Unit USD (pack) | Packs | Total USD | Status | Note |", "|---|---|---|---|---|---|---|---|---|---|"]
    tot = 0.0
    for r in rows:
        n, t = cost(r); tot += t
        L.append(f"| {r['item']} | {r['opt'] or '-'} | {r['qty'] or '?'} | {r['where']} | {r['part']} | {r['unit']:.2f} ({r['pack']}) | {n} | {t:.2f} | {r['status']} | {r['note']} |")
    L.append(f"| **All rows (every option once)** | | | | | | | **{tot:.2f}** | | |")
    return L, round(tot, 2)
