"""Generate bom.md, hardware list and assembly_manual.md from the registry, parts.json and the Bambu slice check."""
import json, os, collections
from registry import PARTS, SUBS, SUB_ORDER
HERE = os.path.dirname(os.path.abspath(__file__))
P = json.load(open(HERE + "/print/parts.json"))["parts"]
try:
    SC = json.load(open(HERE + "/print/bambu/slice_check.json"))
except Exception:
    SC = {}
DENS = {"PLA": 1.24, "PLA-translucent": 1.24, "PETG": 1.27}

# Hardware: (item, qty, where, source/part number, status)
HW = [
    ("Heat-set insert M3 x 4 mm (OD 4.6)", 8, "6 cage feet (inserts in the feet, screws from under the decks), 2 tail-housing stubs on the cage", "generic, e.g. Voron-style M3x5x4 brass inserts", "ASSUMED size, pocket 4.2 x 5 deep"),
    ("Countersunk screw M3 x 10", 6, "cage feet, from under the decks into the inserts in the feet", "generic ISO 10642", "ASSUMED length"),
    ("Countersunk screw M3 x 8 (ISO 10642 / DIN 7991)", 2, "tail housing to cage", "generic", "length from the model: 9.7 mm from head top to insert bottom"),
    ("Self-tapping screw M2 x 6 (pilot 1.8)", 6, "shell to cage bosses", "generic", "ASSUMED length, buy an assortment 4-10 mm"),
    ("Neodymium disc magnet 6 x 3 mm N52", 8, "light contacts: 2 per block and 2 per cage pad, front and rear", "generic", "ASSUMED, alternate polarity"),
    ("LED 3 mm red (with resistor for 2S-3S supply)", 4, "2 tail (x +-8), 2 nose (x +-22)", "generic 3 mm LED", "ASSUMED, wire and resistor not designed"),
    ("Steel drive axle 8 mm, extended inboard to x -9 (86 mm total span) + 2 x 8 mm external retaining ring", 1, "rear swing-arm pivot: the arm's inner plate (x -9..1) rides on the extended axle; outer plates (x 42..77) as before", "generic", "ASSUMED: the axle passes through the ring-gear hub and both bearing-block halves (checked: the axis crosses only those parts and the pulley); the 10 mm stub from the block face to the inner plate carries at most 93 N (FEA, bump + side): bending about 19 MPa in 8 mm steel, no extra bearing needed, but the block's inboard bearing must be through-bored for the axle"),
    ("M6 bolt 28 mm + nyloc nut (shock pins)", 2, "rear shock upper and lower pin", "generic", "ASSUMED length"),
    ("Shock 100 mm class, oil, threaded body", 1, "rear shock", "Integy C26651RED (100 mm piggyback, listed for SCX10 III); alt. Injora BZ-05BS", "part number VERIFIED as listed, eye-to-eye length ASSUMED"),
    ("Low-profile servo, 40.8 x 20.2 x 26.5 mm (AGFRC B44BLS class, steering)", 1, "steering: stands in the printed mount ST2 on the front deck behind the lean head, output shaft up, two screws through each ear into the mount's pillars", "AGFRC B44BLS V2 (Amazon B07L83M8SM) or similar low-profile servo", "one servo steers both wheels through the printed T horn ST1 (tall hub, ball studs hang down); body 20.2 x 40.8 x 26.5 mm with ears; see kit/sourcing_amazon.md"),
    ("Servo horn screw and 25T spline insert (comes with the servo)", 1, "printed steering horn ST1 on the servo shaft", "supplied with the servo", "ASSUMED: the printed horn takes the servo's own spline disc or is drilled for it; shaft bore 5 mm in the model"),
    ("Ball stud 6.8 mm, M3 thread (1/8 scale RC), with ball cup M3", 8, "4 in the knuckles SU4 for the wishbones (stud along the car, cup on the arm tip), 2 in the knuckles' steering arms and 2 in the horn ST1 (stud vertical) for the tie rods", "generic 6.8 mm ball stud and ball cup set", "ASSUMED sizes: ball 6.8, neck 3.2 x 3.5, flange 7 x 2, cup OD 10.5 x 6; cups must allow 20 deg of tilt"),
    ("Tie rod / turnbuckle M3, about 72 mm between the cups", 2, "steering: horn ST1 to the knuckle steering arms", "generic M3 turnbuckle or 3 mm threaded rod in a 7 mm tube", "length from the model: 93 mm ball to ball"),
    ("M3 threaded rod or set screw, about 25 mm, for the arm tip cups", 4, "one in the apex boss of each wishbone SU2 / SU3 (4.1 mm bore: heat-set insert M3 or glue)", "generic", "ASSUMED"),
    ("Shoulder screw D6 x 48 mm (M5 thread) or 6 mm steel rod with retaining rings", 4, "wishbone pivot rods through the lean head SU1 (head behind the rear post)", "generic ISO 7379 shoulder screw", "ASSUMED thread and fixing: the model has a plain 6 mm rod with a low head, no thread in the front post"),
    ("Shock 68 mm eye to eye, oil, about 10 mm stroke (front heave shock)", 1, "between the two shock posts of the upper wishbones SU2", "generic 1/10 or 1/12 scale shock, 65-70 mm class", "ASSUMED: eye 3.3 mm wide, M4 pin; needs about 250 N at full bump (spring rate about 30 N/mm): not selected"),
    ("M4 x 20 pin or screw + nyloc nut (heave shock pins)", 2, "shock eyes in the forks of the upper wishbones", "generic", "ASSUMED length"),
    ("Steel axle pin D6 x 47 mm, M6 nut, 12 mm wheel hex hub", 2, "front stub axles in the knuckles SU4", "generic 6 mm axle / stub axle with 12 mm hex", "ASSUMED: fixed in the knuckle boss (6.1 mm bore), wheel runs on its hex hub; no wheel bearings modelled"),
    ("Countersunk screw M3 x 10 + heat-set insert M3", 6, "lean head SU1 base tubes to the front deck (4, from below) and skid plate to the deck (2)", "generic", "positions from the model: head (+-16, y -195.3) and (+-28, y -159.0), skid (+-26, y -206.4)"),
    ("2S-3S LiPo", 1, "NOT PLACED: the battery tray and the battery stand-in were removed from the front deck (owner, November 22)", "any 2S-3S pack", "OPEN: no battery position or hold-down is designed in this version"),
    ("Zip tie", 2, "bridge on the swing-arm tube", "generic", ""),
    ("Hobbywing QuicRun Fusion SE 1800KV 2-in-1 (ESC + 540 motor)", 1, "motor plate, deck", "HWI30404317 (1200KV = HWI30404318)", "VERIFIED part numbers (retailer listings); body 36 x 58 mm"),
    ("1.9 in beadlock wheels, 12 mm hex (front)", 2, "front", "Axial AXI43005 (MW19, pair)", "VERIFIED wheel; tyre 112 x 62 mm NOT chosen: pick a 1.9 tyre of about 112 mm OD"),
    ("Rear wheel and tyre, about 140 mm OD x 41-54 mm, 12 mm hex", 1, "rear swing arm", "2.2 in crawler tyre/wheel, see NOTES 'Rear wheel candidates'", "UNKNOWN part, the model uses 140 mm OD x 62 mm: choose a real tyre and measure"),
    ("Bearings 5 x 11 x 4 mm", 0, "drivetrain", "count not derivable from the stand-ins", "ASSUMED, count from the chassis build"),
    ("GT2 closed-loop belt, 2 mm pitch, 6 mm wide, 300 mm (2GT-300, 150 teeth)", 1, "rear arm: pivot 26T to axle 30T pulley", "generic GT2 closed loop", "centre distance 122.1 mm gives 300.2 mm; the belt sits 0.1 mm under size: no slack, fit with the pulley grub screws loose first"),
    ("Grub screw M3 x 4", 6, "one in each printed GT2 pulley hub (DT12-DT15)", "generic", "ASSUMED"),
    ("Drive shafts, diff gears, rings (SCX10 III-class)", 0, "drivetrain", "Axial SCX10 III donor parts; AXI238000 is discontinued, buy single parts", "UNKNOWN part numbers: list from the user's chassis BOM"),
]
TOOLS = ["Soldering iron (heat-set inserts, LED and magnet wiring)", "Hex drivers 1.5 / 2.0 / 2.5, small Phillips", "CA glue + activator (shell seams), PETG-safe", "Side cutters and deburring knife (support removal)", "Fine sandpaper 220-400 and filler primer if painting"]


def plate_rows():
    rows = []
    for name, v in sorted(SC.items()):
        rows.append((name, v))
    return rows


def main():
    byd = collections.OrderedDict()
    for p in P: byd.setdefault(p["design"], []).append(p)
    L = ["# BOM and print list (generated)", "", "Mass = volume x density (PLA 1.24, PETG 1.27 g/cm3) at 100 % fill: an upper bound, ASSUMED. The Bambu slice numbers in the manual replace it.", "",
         "| ID | Part | Qty | Material | Walls | Infill | Support | Extents mm (oriented) | Volume cm3 | Mass g (max) |", "|---|---|---|---|---|---|---|---|---|---|"]
    tot = collections.Counter()
    for p in P:
        m = p["volume_cm3"] * DENS[p["material"]]; tot[p["material"]] += m
        L.append(f"| {p['id']} | {p['name']} | 1 | {p['material']} | {p['walls']} | {p['infill']} | {p['support']} | {' x '.join(str(int(e)) for e in p['extents'])} | {p['volume_cm3']} | {m:.0f} |")
    L += ["", "Upper-bound filament: " + ", ".join(f"{k} {v / 1000:.2f} kg" for k, v in tot.items()), ""]
    L += ["## Hardware list", "", "| Item | Qty | Where | Source / part number | Status |", "|---|---|---|---|---|"]
    L += [f"| {a} | {b if b else '?'} | {c} | {d} | {e} |" for a, b, c, d, e in HW]
    L += ["", "Status words: VERIFIED = found in a retailer or maker listing on 2026-11-14; ASSUMED = from the model or notes, not measured; UNKNOWN = no purchasable part chosen yet.", ""]
    open(HERE + "/bom.md", "w").write("\n".join(L))

    M = ["# Fugu reverse trike: assembly manual", "", "Print-only kit. Target printer: Bambu Lab P2S (256 x 256 x 256 mm), 0.4 mm nozzle, PLA for the shell and lights, PETG for the structure. "
         "Everything also fits the larger H2C-class bed. **Do not auto-orient or auto-arrange in Bambu Studio**: every file is already oriented and packed.", "",
         "## 1. Print plates", "", "Open `kit/print/bambu/all_plates.3mf` (every plate) or one plate file. Slot 3 = PLA, slot 4 = PETG (put PETG in AMS slot 4 or reassign in Studio).", "",
         "| Plate file | Filament | Time | Grams (Bambu slice) | Notes |", "|---|---|---|---|---|"]
    for name, v in plate_rows():
        fil = 'PETG' if 'PETG' in name else 'PLA'
        M.append(f"| {name} | {fil} | {v.get('minutes', 0):.0f} min | {v.get('grams', 0):.0f} | {'; '.join(v.get('problems', [])) or 'sliced clean'} |")
    M += ["", "## 2. Printed parts", "", "| ID | Part | Plate group | Material | Joins |", "|---|---|---|---|---|"]
    for d in PARTS:
        pcs = ", ".join(p["id"] for p in byd.get(d["id"], []))
        M.append(f"| {pcs} | {d['name']} | {SUBS[d['sub']]} | {d['material']} | {d['mates']} |")
    M += ["", "Parts longer than the bed are printed in pieces (suffix a, b, c): the Front shell (SH1a/b), the Cage (CG1a/b/c) and, only with the closed cockpit, the Canopy (SH2a/b). Pin joints: pins on piece `a` go into the holes of piece `b`. Glue with CA, hold flat until set.", "",
          "## 3. Hardware to buy", "", "See `bom.md` for counts, part numbers and which are only ASSUMED.", "", "## 4. Tools", ""] + [f"- {t}" for t in TOOLS]
    steps = [
        ("Step 0: test fit", "Print the steering horn ST1 and one knuckle SU4 first. Press an M3 insert into a scrap, screw a 6.8 mm ball stud into the knuckle plate and snap a ball cup on. Test one 1.6 mm pin in a 1.75 mm hole on a Cage joint. Stop and tell the designer if a stud does not thread or a pin does not enter."),
        ("Step 1: chassis frame (CH1-CH5)", "Join decks CH1/CH2/CH3 to the donor chassis hardware, screw the skid CH4 under the front deck's front edge (2 x M3) and fit the motor plate CH5. Check: decks sit flat, no rocking."),
        ("Step 2: lean head and wishbones (SU1-SU4)", "Press M3 inserts into the four base tube bores of the lean head SU1 and screw it to the front deck from below, right behind the skid. Push the four 6 mm pivot rods in from the rear through the head and the pivot barrels of the upper wishbones SU2 (with the shock posts) and lower wishbones SU3. Screw the ball studs into the knuckle plates SU4, thread a ball cup into each wishbone tip and snap the cups onto the studs. Hang the heave shock between the two posts of the upper wishbones (M4 pins). Check: both wheels move up and down together against the shock, the body leans freely about 10 degrees each way with the shock length unchanged, nothing touches the deck or the head."),
        ("Step 3: steering (ST1-ST2)", "Screw the servo mount ST2 to the front deck behind the head (4 x M3 from below into inserts in its pillars), stand the low-profile servo between the pillars with the shaft up and screw its ears down (2 screws per ear), fit the printed T horn ST1 on the shaft and hang a ball stud from each end of it, and connect each horn ball to the ball stud on the knuckle's steering arm with a tie rod. Set both tie rods to the same length (93 mm ball to ball). Check: full lock both ways at full lean without rubbing; the tie rods pass 1.5 mm behind the pivot rod heads."),
        ("Step 4: drivetrain (DT5-DT17)", "Rear wheel drive only: assemble the bearing blocks DT5-DT8 and the ring hub DT10 on the rear shaft, fit the motor with its pinion to the motor plate against the spur gear, the single-sided rear arm DT11 with its pulleys DT14/DT15 and the 300 mm belt, the rear wheel DT17 and the rear shock. Fit the front wheels DT16 on the stub axles. Check: wheels turn by hand, belt tight, rear arm swings with no contact."),
        ("Step 5: cage (CG1a/b/c, CG3)", "Glue the three cage pieces on their pins (a to b to c), let cure 30 min. Press M3 inserts into the six feet, drop the feet into their deck pockets (front pair at y -127, behind the tie rods) and screw them from under the decks. Lay the roof plate CG3 on the roof frame. Fit the shock top bolt through the cage clevis. Check: full steering lock and full lean with the cage on: tie rods and shock posts clear the front tubes."),
        ("Step 6: shell (SH1a/b, SH4)", "Glue SH1a to SH1b on their pins, then the grille SH4 from inside the nose. Dry-fit over the cage first. Screw M2 into the cage bosses. Check: both front wheels at full lean and full lock clear the arches."),
        ("Step 7: lights (LT1, LT2, SH3, CG2)", "Press magnets (alternate polarity) into CG2 and the cage pad, fit LEDs in the pegs, push the strips into their channels, screw SH3 to the cage with two countersunk M3 x 8. Check: contacts meet with a 0.3 mm gap, strips sit flush."),
    ]
    for t, b in steps: M += ["", f"### {t}", "", b]
    M += ["", "## Known risks (not verified on a printed kit)", "",
          "- Nothing in this kit has been printed or assembled yet. Fits (ball studs, pivot rods, inserts) are from the model only.",
          "- Front end strength from FEM (target safety 2): upper wishbone 2.65, lower wishbone 2.30, lean head 1.90, knuckle 1.54 at the upper ball stud bore. Cage 1.39 at a tail tube joint, rear arm 1.66 at one inner corner. See NOTES.md.",
          "- The heave shock, ball studs, tie rods and pivot rods are not selected parts: sizes in bom.md are taken from the model and marked ASSUMED.",
          "- No battery position or hold-down is designed in this version (tray and battery were removed from the front deck).",
          "- Pin joints in the 3 mm shell walls are 1.6-2.0 mm pins: glue joint strength is unmeasured.",
          "- Chassis parts marked in the model as stand-ins (decks, plates) may not match the real printable parts. Check CH1-CH5, DT5-DT10 against your real chassis files.",
          "- Printer: NOTES.md says H3C (H2C-class, 330 x 320 mm); the saved projects are P2S. The kit is sized for P2S.",
          "- Rear tyre and wheel are not chosen (140 mm OD in the model).", ""]
    open(HERE + "/assembly_manual.md", "w").write("\n".join(M))
    print("docs written")


if __name__ == "__main__":
    main()
