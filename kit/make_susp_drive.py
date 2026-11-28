"""Fugu Version 2 suspension + drivetrain kit: plates and README for the printed parts in kit/susp_drive (build_kit.py writes them there).

  python kit/build_kit.py renders/kit_subset --out kit/susp_drive --no-polish   (subset: the kit's stems out of `soma export --all`)
  python kit/make_susp_drive.py            -> kit/susp_drive/kit_manifest.json
  cd kit/susp_drive && python ../bambu_kit.py kit_manifest.json --check
  python kit/make_susp_drive.py --docs     -> kit/susp_drive/README.md (reads bambu/slice_check.json and kit/hardware.py)
"""
import json, os, sys
from registry import PARTS
from hardware import HW, table as hw_table, SRC_DATE, OPTIONS, option_totals

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = HERE + "/susp_drive"
SLOT = {"PETG": 4, "TPU 95A": 4}
FACE = ["+x", "-x", "+y", "-y", "+z", "-z"]        # build_kit orient_face: the model face that lies on the bed
P = json.load(open(OUT + "/parts.json"))["parts"]
D = {d["id"]: d for d in PARTS}
opt = lambda p: D[p["design"]].get("option")


def manifest():
    plates = []
    groups = (("Front suspension and steering (PETG)", lambda p: p["sub"] in ("SU", "ST") and p["material"] == "PETG" and not opt(p)),
              ("Drivetrain and wheels (PETG)", lambda p: (p["sub"] in ("DT", "CH")) and p["material"] == "PETG"),
              ("Stops and pads (TPU 95A)", lambda p: p["material"] == "TPU 95A"),
              ("Options: print only the chosen one (PETG)", lambda p: bool(opt(p))))
    for n, (title, sel) in enumerate(groups, 1):
        ps = [p for p in P if sel(p)]
        if ps:
            plates.append(dict(name=f"{n:02d}_" + title.split(" (")[0].split(":")[0].replace(" ", "_").replace(",", ""), title=title,
                               parts=[dict(stl=f"stl/{p['id']}.stl", name=p["id"], walls=p["walls"], infill=p["infill"], support=p["support"], slot=SLOT[p["material"]]) for p in ps]))
    man = dict(title="Fugu V2 suspension + drivetrain kit", profile="../p2s_profile.config", out="bambu", bed=[256.0, 256.0], margin=5, gap=22, filament_slot=4,
               supports="easy_peel", plates=plates)
    json.dump(man, open(OUT + "/kit_manifest.json", "w"), indent=1)
    print(len(P), "printed files,", len(plates), "plate groups")


STEPS = [
    ("0. Test fit (do this first)",
     "Print the carrier SU3R and the rocker ST1R alone. Ream the pin bores by hand (D3.1 bores with a 3 mm drill, D4.1 with a 4 mm drill) until the hardened pin "
     "slides through without play; check one D4 pin in SU3R's kingpin block and one D3 pin in its link eye. Push a 6.8 mm ball stud through ST1R and snap an M3 rod end on: it must swivel 25 deg.",
     "1 x D4 pin, 1 x D3 pin, 1 ball stud + nylock, 1 M3 rod end", "Check: pins slide, do not wobble; if a bore is tight after reaming, the printer over-extrudes: fix the flow before printing the rest."),
    ("1. Arm pivot", "Screw the pivot blocks SU5 L/R to the front deck. Slide the D5 pivot axle through the left block, the left arm SU1L root, the left rocker bracket clamp "
     "(option R: see 7), the right ones and the right block. Centre it; the rocker bracket clamps (one M3 each across the slit) hold it.",
     "1 x hardened D5 shaft cut to 172 mm, 2 x M3 x 10 into the deck", "Check: each arm swings 40 mm at the tip by its own weight, no side play."),
    ("2. Upper links and carriers", "Pin each carrier SU3 to its arm tip with a D4 x 55 (tip shaft). Pin the upper link SU2 to the carrier and to the cage rail lug with D3 x 12 pins.",
     "2 x D4 x 55, 4 x D3 x 12", "Check: the carrier stays upright while the arm moves through its whole travel (the parallel link)."),
    ("3. Knuckles and front wheels", "Knuckle SU4 (printed PETG, the owner's choice; a machined one with the same interface fits the same way). Press the 6 x 1 steel "
     "sleeve (13.6 mm) into the axle bore from the outboard end, flush with the seat ring. Press an M4 heat-set insert (Ruthex M4 x 8.1) into the ball boss from the "
     "top, 250 C, straight. Put a 0.5 mm DIN 988 shim on each thrust seat, slide the knuckle over the carrier block and drop the D4 x 40 kingpin through both ears. "
     "Screw the 6.8 mm M4 ball stud into the insert from below. Drop an M4 nut into the hex pocket, press a 4 x 8 x 6 flanged bronze bushing into each "
     "wheel DT16 from the inner face (flange flush in its counterbore; a vise and two flat plates, square), then push the M4 x 35 12.9 bolt from outboard through "
     "the DIN 988 4 x 8 shims, the wheel's bushing and the sleeve into the nut. Shim until the wheel has no end play and still spins free; snug the bolt.",
     "per side: knuckle, sleeve, M4 insert, 2 x DIN 988 4 x 8 x 0.5, D4 x 40 kingpin, M4 ball stud, M4 x 35 12.9 + M4 nut, flanged bronze bushing 4 x 8 x 6, DIN 988 4 x 8 shims, front tyre",
     "Check: the knuckle turns 20 deg each way without binding; the wheel spins on its bushing without wobble (owner: 'plain bushings')."),
    ("4. Spring levers and cup blocks", "Screw a 6.8 mm ball stud into each lever (arm root, x 54). Turn two pucks of D9 x 9 from the POM rod with the 6.8 mm ball socket and the "
     "20 deg neck (drawing: parts/swingarm_front.py cup_block) and press each onto its ball.", "2 x ball stud 6.8, POM rod D10", "Check: puck turns on the ball without play."),
    ("5. Beam carrier, heave shock and stops", "The TPU saddles SU8/SU9 are REQUIRED, not optional: the 4 mm bump saddle SU8 (TPU 95A) is what keeps the landing load at 123 N per front wheel from a 0.20 m drop (164 N without it; NOTES 61, 64); the droop saddle SU9 is 2 mm. Glue the TPU saddles SU8/SU9 into the cage pad blocks and the yaw pads SU10/SU11 onto the carrier SU7 posts (CA for TPU). Ream the pad block arc slots "
     "to 8.3 mm (REQUIREMENT: radial clearance 0.15 on the D8 dowel) and the carrier hub bore to 6.4 mm (REQUIREMENT: loose on purpose, so the dowel takes the yaw moment first). "
     "Pin the carrier into the cage clevis with the D6 x 35. Push the D8 x 75 heave stop dowel through the carrier tail and both pad block slots. Mount the heave shock between the "
     "carrier fork and the cage rear clevis with the two D3 pins.", "D6 x 35, D8 x 75, heave shock, 2 x D3 x 12", "Check: push both wheels up 12 mm: the dowel lands on the bump saddles; let them drop 8 mm: on the droop saddles."),
    ("6. Balance beam", "Set the beam SU6 on the D8 yaw pin in the carrier, clip it. Guide the cup blocks into the beam's end forks as the beam goes down.",
     "D8 x 30 yaw pin + 8 mm retaining ring", "Check: push one wheel up: the other goes down by the same amount (lean); lean 10 deg: the beam lands on the yaw pads."),
    ("7. Steering", "Rocker bracket ST2 (printed PETG, the owner's choice; a machined one with the same interface fits the same way): press an M3 insert into the "
     "hanger under the clamp screw hole, slide the bracket onto the pivot axle (step 1) and tighten the M3 x 12 clamp screw from the top. Drop the D4 x 20 rocker pin "
     "through the foot and the rocker ST1 into the blind shoe bore (the arm boss above keeps it in). Seat the servo floor ST3 on the deck, lay the servo in, fit the servo clip ST4 (one M3 x 16 through the deck and the tongue into the insert). "
     "SERVO (Tekin T-180, before fitting): with the HotWire interface set Torque about 31 % (the steering need, owner's choice NOTES 32), Torque Delay 0 s, and limit Travel so the horn swings no more than "
     "about +-28 deg (the wheels reach the 18.5 deg design lock). The 31 % is ASSUMED to give 0.367 N m at 6.0 V (the maker does not state the base of the "
     "percentage): BENCH CHECK it: block the horn and pull at the ball (12 mm from the shaft) with a spring scale while commanding full throw: it must hold "
     "at least 31 N (0.367 N m, what the steering needs). If it does not, raise the percentage until it does and WRITE DOWN the % used: the printed rocker "
     "mount's safety falls from about 0.96 (31 %) toward 0.75 (40 %). A servo lower than the Tekin's 27 mm gets a "
     "printed shim between its base and the clip leg so its top face stays at x 58.5 (Savox SC-1251MG+: 1.4 mm). Centre the servo (1500 us), fit the metal horn. Drag link: two M3 rod ends on a short M3 stud, horn ball to the rocker input ball. Tie rod: M4 rod ends (the knuckle end the slim one) "
     "from the rocker output ball to the knuckle ball. SERVICE: a steering servo comes out only with the beam assembly off (step 6 backwards).",
     "2 servos, 2 metal horns, 2 x D4 x 20, 4 x M3 rod end, 4 x M4 rod end, 2 x M4 rod, 6 x ball stud 6.8 + nylock, 2 x M3 x 16 + insert",
     "Check: at horn +-30 deg the wheels steer 16.3 / 19.5 deg; bump steer is about 0.02 deg (move a wheel up and down at centre: the toe must not move)."),
    ("8. Droop pads", "Glue the TPU droop pads SU12 onto the upper link rail lugs (CA).", "-", "Check: lever blade lands flat on the pad at full droop."),
    ("9. Rear drive shaft and carriers (as Version 1)", "Press 5x11x4 bearings into the pillow block DT5 (1) and the rear bearing carrier DT7 (2 on the rear shaft, 2 on the cross shaft). "
     "Spur on the front end of the rear shaft, pinion on its rear end, ring on the ring hub DT10, close DT6 and DT8.", "5 x 5x11x4, shafts, spur, ring and pinion, 2 x DIN 471 5 mm",
     "Check: turning the spur turns the cross shaft smoothly over a full ring turn."),
    ("10. Motor", "Motor on the motor plate CH5 with the 15T pinion, set the mesh with a paper strip, plate to the deck.", "motor/ESC, 15T pinion, 4 x M3", "Check: a tooth print on the paper, not torn."),
    ("11. Rear swing arm", "Three 5x11x4 in the pivot plates of DT11, two 8x16x5 in the axle plates. Pivot pulley DT14 on the cross shaft, belt on, axle pulley DT15 and the rear hub on the 8 mm axle, "
     "rear wheel DT17 with its tyre.", "3 x 5x11x4, 2 x 8x16x5, axle 8 x 100, belt 2GT-300-6, 2 x M3 x 4 grub, 4 x DIN 471, rear hub, rear tyre",
     "Check: the arm swings freely with the belt on; belt deflects about 3 mm under a finger."),
    ("12. Rear shock", "Bolt the shock between the cage clevis (M6 x 35) and the arm bridge (M6 x 30), nylocks snug.", "rear shock, M6 x 30, M6 x 35, 2 x M6 nylock", "Check: the tyre does not touch the tail at full bump."),
]
FITS = [
    "REQUIREMENT: front wheel bushing seat D7.90 x 6 in the wheel hub for the 8.0 OD sintered bronze bushing (0.10 mm press, ASSUMED: no housing tolerance on the listing). Press it in square; a sintered bush closes up when pressed, so check it spins free on a D4 pin afterwards and ream to 4.0 only if it binds (reaming smears the pores: re-oil it).",
    "REQUIREMENT: pad block arc slot 8.3 mm for the D8 heave stop dowel (radial clearance 0.15): ream or file; the dowel must slide along the slot without play across it.",
    "REQUIREMENT: beam carrier hub bore 6.4 mm on the D6 carrier pin (radial clearance 0.2, loose on purpose): the carrier twists 0.86 deg before the hub bears, the dowel bears at 0.32 deg, so the yaw stop moment goes into the dowel (NOTES decision 16).",
    "Pin bores: D3.1 for the 3 mm pins, D4.1 for the 4 mm pins, D5.1 for the pivot axle: printed bores come out small, ream by hand to a sliding fit.",
    "Rear bearing seats D11.1 (5x11x4) and D16.1 (8x16x5): a light press fit in PETG, press on the outer race only.",
    "Ball studs: M3 through the printed plate with a nylock on top, tightened until the stud stops turning; rod ends must swivel 25 deg (drag link) without binding.",
    "Torque: printed parts take snug plus a quarter turn, no torque value. Blue thread locker on metal threads only. CA glue for the TPU parts.",
    "MEASURE ON ARRIVAL (no maker number published; docs/v2/bought_part_datasheets_2026-11-24.md): servo shaft distance from the case's front end (the model has 10.2 mm: "
    "a shim at the floor's front rim sets the shaft at y -95.1, it sets the drag link geometry); servo ear overhang and thickness (the mount is designed for up to 8.0 x 2.5 mm "
    "and does not use the ears); horn hole radius (12 mm in the model: it sets the steering ratio); M4 ball stud thread length (at most 9 mm into the knuckle insert); "
    "rod end housing OD at the knuckle (8.2 mm at most); 6 x 1 tube OD (press fit in the knuckle's D6.0 bore).",
]
OPEN = [
    "Front arm (owner's choice, 2026-11-26, 'Keep printed'; NOTES 59, 61): the printed front arm is the first part expected to fail in a drop above about 0.15 .. 0.20 m. It is a replaceable print: reprint SU1L / SU1R (kit/susp_drive/stl/SU1L.stl, SU1R.stl, on plate 01 Front suspension and steering).",
    "Strength below the target safety of 2 (PETG across layers 20 MPa; NOTES.md): rocker mount (printed: 1.02 at the steering need, 0.19 at the ASSUMED 2.0 N m stall), "
    "beam carrier (1.30 two-wheel, 1.06 one-wheel landing), front arm (1.49 at the outer bore edge), cage at the stops (0.73 at the upper ear). Only the balance beam passes (2.13).",
    "No FEM of the cage with the desert shell's exo frame and the raised hoop corner: every cage FEM ran on the cage before it.",
    "Owner's choice (NOTES 26): printed knuckle and rocker mount with torque-limited servos. FEM at the servo limit (0.47 N m, ASSUMED from the Tekin 40 % setting): "
    "knuckle landing 2.71, knuckle steering stall 2.43 (about 1.9 at 0.6 N m); rocker mount 0.75 (about 0.96 at the steering need 0.367 N m): the rocker mount is BELOW 2.",
    "Rocker mount, in plain words: the printed bracket is at about 0.96 at the 31 % servo setting (0.75 at 40 %) against the target of 2. The owner chose it knowing that (NOTES 32). The machined aluminium bracket is the same-interface alternative.",
    "Steering at horn +-30 deg: +16.35 / -19.28 deg (verified layout B: +16.28 / -19.50) after the servo moved 3.3 mm rearward for its ears; the 20 deg figure elsewhere is the shell cut-out allowance, not a lock.",
    "Nothing in this kit has been printed, fitted or driven. Every fit comes from the model.",
    "Front wheel bushings: the housing tolerance is not on the bushing's page; the printed seat D7.90 (0.10 mm press) is ASSUMED. Print the fit coupon first.",
    "Heave shock: the RC4WD 70 mm (Version 1's choice) has no stated spring rate (about 30 N/mm assumed in the design) and is 2 mm longer than the model's 68 mm.",
    "Rear and drivetrain: the open items of Version 1 apply unchanged (ring size against the carrier cavity, no 8 mm-bore 12 mm hex hub, 42T spur not found).",
    "TPU parts are sliced with the PETG profile of the saved project: their time and grams are ASSUMED; switch the filament to TPU 95A in Studio.",
    "Not chamfered, needs a redraw: the rocker ST1 (its chamfered solid meshes open; break its edges by hand), the rear bearing carriers DT7/DT8, the "
    "pillow blocks DT5/DT6 and the ring hub DT10 (traced facet meshes without a source profile). Everything else that moves has 0.5 mm chamfers (NOTES 30).",
]


def docs():
    sc = json.load(open(OUT + "/bambu/slice_check.json"))
    t, tot = hw_table(HW)
    n = {s: sum(1 for r in HW if r["status"] == s) for s in ("VERIFIED", "ASSUMED", "UNKNOWN")}
    mins, grams = sum(v["minutes"] for v in sc.values()), sum(v["grams"] for v in sc.values())
    probs = sum(len(v["problems"]) for v in sc.values())
    combos = option_totals()
    L = ["# Fugu V2 suspension + drivetrain kit (leading swing arms)", "",
         f"Front swing arms, upper links, carriers, knuckle, balance beam and carrier, heave shock and stops, steering (rockers, servos, horns, drag links, tie rods), "
         f"rear swing arm, belt drive, motor plate, wheels and tyres. {len(P)} printed files on {len(sc)} plates, {mins / 60:.1f} h and {grams:.0f} g by the Bambu Studio "
         f"slice check ({probs} problems), the option plate included. Bought parts as listed on {SRC_DATE} ({n['VERIFIED']} rows VERIFIED, {n['ASSUMED']} ASSUMED, "
         f"{n['UNKNOWN']} UNKNOWN). Nothing bought.", "",
         "## Stated openly", ""] + [f"- {o}" for o in OPEN] + ["",
         "## 0. The owner's choice and the alternatives (same interface: docs/v2/knuckle_rocker_interface.md)", "",
         "| Decision | Option | What it means | Parts it adds | Price USD |", "|---|---|---|---|---|"]
    for key, opts in OPTIONS.items():
        for o in opts:
            L.append(f"| {key} | {o['id']}{' (owner chose)' if o.get('rec') else ''} | {o['what']} | {o['adds']} | {o['price']:.2f} |")
    L += ["", "Totals of the bought parts per combination (servo class follows: any printed option needs the torque-limited servo):", "",
          "| Rocker mount | Knuckle | Servo | Total USD |", "|---|---|---|---|"]
    for c in combos:
        L.append(f"| {c['R']} | {c['K']} | {c['servo']} | {c['total']:.2f} |")
    L += ["", "## 1. Bought parts", "", "Quantities from the model (`Where` names the stand-in parts). Rows with an option ID are bought only with that option. Total = packs x pack price, "
          "all rows (each option once). Sourcing notes: `../sourcing_v2_2026-11-24.md` (Version 2 front end) and Version 1's `kit/sourcing_2026-11-24.md` (rear, drive, tyres, bearings, heave shock: the same choices).", ""] + t + ["",
          "## 2. Printed parts", "", "Every file is oriented and packed: do not auto-orient in Bambu Studio. L/R parts have their own IDs (L = -x side, the left seen from behind).", "",
          "Infill rule (owner 2026-11-27, NOTES 67): every mechanical and moving printed part prints SOLID (100 % infill) - every load class other than "
          "'none' (kit/registry.py print_fill). The FEM safeties in NOTES assume solid sections; at the old 30 .. 70 % the printed part would not match them. "
          "Shell and light pieces keep their infill with 5 walls, so their screw bosses and tabs print solid. TPU pads: 100 %, 3 walls. Layer 0.2 mm, 5 top / 3 bottom layers.", "",
          "| ID | Part | Material | Walls | Infill | Support | Face on the bed | Size mm | Plate |", "|---|---|---|---|---|---|---|---|---|"]
    man = json.load(open(OUT + "/kit_manifest.json"))
    plate_of = {pp["name"]: pl["title"] for pl in man["plates"] for pp in pl["parts"]}
    for p in P:
        L.append(f"| {p['id']} | {p['name']} | {p['material']} | {p['walls']} | {p['infill']} | {p['support']} | model {FACE[p['orient_face']]} down | {' x '.join(str(round(e)) for e in p['extents'])} | {plate_of.get(p['id'], '?')} |")
    L += ["", "## 3. Plates (Bambu Studio slice check, P2S profile of the saved project, 0.4 mm nozzle)", "",
          "Open `bambu/all_plates.3mf` or one plate file. PETG in AMS slot 4; the TPU plate needs TPU 95A (external spool). The printed knuckle and rocker bracket are on the front plates now. `bambu/plate_04_Options.3mf` belongs to the FIRST kit (superseded): do NOT print it; the knuckle and rocker bracket are on the front plates.", "",
          "| Plate file | Time | Grams | Slice check |", "|---|---|---|---|"]
    for k, v in sorted(sc.items()):
        L.append(f"| {k} | {v['minutes'] / 60:.1f} h | {v['grams']:.0f} | {'; '.join(v['problems']) or 'OK'} |")
    L += [f"| **Total** | **{mins / 60:.1f} h** | **{grams:.0f}** | {probs} problems |", "",
          "## 3b. Two paths: print, or buy / machine in metal", "",
          "`print_or_buy.md` lists every part with its printed version and its off-the-shelf steel or metal version (part numbers, listings opened 2026-11-24). "
          "In short: every structural part of the kit prints (default); pins, dowels, shafts, bolts, the sleeve, ball studs, rod ends, turnbuckles, horns, "
          "bearings, the belt, gears, shocks, servos and the motor are bought (no printed version carries their loads). No stock metal arm, carrier, knuckle, "
          "rocker or beam fits this car; any of them can be machined from its STEP (`soma export \"<part>\" --format step`), and the knuckle and the rocker "
          "mount have a machinist's interface table (`docs/v2/knuckle_rocker_interface.md`). A machined part goes in exactly where the printed one did, "
          "with the same pins and screws: the knuckle takes a D4 bore instead of the sleeve and a tapped M4 instead of the insert; the rocker bracket takes a "
          "tapped M3 instead of the clamp insert. With a machined rocker bracket the servo's torque limit may be raised (Savox SC-1251MG+ needs no programmer).", "",
          "## 4. Tools", "", "- Hand drills 3 / 4 / 5 / 6 / 8 mm for reaming, a needle file (arc slots)", "- Soldering iron with an insert tip (heat-set inserts)",
          "- Bench vise or arbor press and sockets 11 / 16 mm (rear bearings, outer race only)", "- Hex keys 1.5 / 2 / 2.5 / 3 / 4 mm, 5.5 / 7 / 10 mm nut drivers",
          "- Circlip pliers (8 mm ring on the yaw pin, DIN 471 5 and 8 mm), hacksaw and file (D5 axle to 172 mm, shafts)", "- A small lathe or drill-press setup for the two POM pucks",
          "- Blue thread locker (metal threads only), CA glue for TPU", "", "## 5. Fits and torque", ""] + [f"- {f}" for f in FITS] + ["", "## 6. Assembly (in this order)"]
    for title, body, hw, chk in STEPS:
        L += ["", f"### {title}", "", body, "", f"Hardware: {hw}.", "", chk]
    open(OUT + "/README.md", "w").write("\n".join(L) + "\n")
    print("README written:", len(P), "files,", len(sc), "plates,", f"{mins / 60:.1f} h, {grams:.0f} g, all rows {tot:.2f} USD", [(c['R'], c['K'], c['total']) for c in combos])


if __name__ == "__main__":
    docs() if "--docs" in sys.argv else manifest()
