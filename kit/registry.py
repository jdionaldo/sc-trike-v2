"""Part registry of the Fugu Version 2 kit (leading swing arms): one entry per printed DESIGN. Every printed STL stem must resolve through key_of().

Fields: id, sub (build-order group), name, stems (exported instance names, mirror pairs list both), material, function, mates, load,
walls, infill, support (auto|on|off), brim, note, mirrored (L/R IDs: L = -x side), option (printed only with that BOM option),
optional (not part of the suspension + drivetrain kit: built later with the full kit; the outputs of the rest of the car are on hold).
Bought parts: kit/hardware.py. The Version 1 (wishbone) registry is in the git history (commit before this one).
Sub-assemblies follow build order: SU front suspension (swing arms, balance beam, stops), ST steering, DT drivetrain and wheels, CH chassis,
CG cage, SH shell, LT lights.
"""
SUBS = {"SU": "Front suspension (swing arms, balance beam, stops)", "ST": "Steering", "DT": "Drivetrain, rear arm and wheels",
        "CH": "Chassis frame and decks", "CG": "Cage", "SH": "Shell", "LT": "Lights"}
SUB_ORDER = ["SU", "ST", "DT", "CH", "CG", "SH", "LT"]
KIT_SUBS = ("SU", "ST", "DT")          # the suspension + drivetrain kit; CH5 (motor plate) joins it

D = lambda **k: k
PETG_S = dict(material="PETG", load="structural", support="auto")
TPU = dict(material="TPU 95A", load="impact", walls=3, infill="100%", support="off")
PARTS = [
    # --- front suspension -------------------------------------------------------------------------------------------------------------------------
    D(id="SU1", sub="SU", name="Front swing arm with spring lever", stems=["arm_-1", "arm_1"], mirrored=True, walls=6, infill="60%", **PETG_S,
      function="Leading truss arm (100 mm) on the D5 pivot axle; its root carries the spring lever blade and the lever ball stud",
      mates="SU5 pivot blocks (D5 axle), SU3 carrier (D4 tip pin), SU2 upper link (D3 pin), lever ball stud + cup block in SU6",
      note="FEM: safety 1.49 at the outer bore edge with pinned bores (target 2): OPEN"),
    D(id="SU2", sub="SU", name="Upper link", stems=["uplink_-1", "uplink_1"], mirrored=True, walls=5, infill="50%", **PETG_S,
      function="Parallel link above the arm: keeps the carrier upright (wheels lean with the body)", mates="cage rail lug and SU3 (D3 x 12 pins)"),
    D(id="SU3", sub="SU", name="Front carrier", stems=["Front_carrier_-1", "Front_carrier_1"], mirrored=True, walls=6, infill="70%", **PETG_S,
      function="Translating carrier at the arm tip: kingpin block for the knuckle", mates="SU1 tip (D4 x 55), SU2 (D3), knuckle (D4 kingpin)"),
    D(id="SU4", sub="SU", name="Knuckle (PETG)", stems=["Front_knuckle_-1", "Front_knuckle_1"], mirrored=True, walls=8, infill="100%", **PETG_S,
      function="Knuckle on the D4 kingpin: 8 mm ears, deep spine in the rim, 6 x 1 steel sleeve, M4 12.9 axle bolt, M4 insert for the ball stud",
      mates="SU3 (D4 x 40 kingpin, 0.5 mm shims), M4 axle bolt + nut, M4 ball stud (insert from the top)",
      note="owner's choice (NOTES 26). FEM landing 2.71, steering stall at the 0.47 N m limit 2.43. Interface: docs/v2/knuckle_rocker_interface.md"),
    D(id="SU5", sub="SU", name="Arm pivot block", stems=["w3_arm_pivot_block_-1", "w3_arm_pivot_block_1"], mirrored=True, walls=5, infill="60%", **PETG_S,
      function="Holds the D5 arm pivot axle on the front deck", mates="front deck (M3), D5 pivot axle, SU1"),
    D(id="SU6", sub="SU", name="Balance beam", stems=["Balance_beam"], walls=6, infill="60%", **PETG_S,
      function="Beam on the D8 yaw pin: its end forks take the cup blocks on the lever balls; heave goes to the shock through the carrier",
      mates="SU7 (D8 yaw pin + clip), 2 cup blocks", note="FEM two-wheel landing: safety 2.13"),
    D(id="SU7", sub="SU", name="Beam carrier (bellcrank)", stems=["Beam_carrier"], walls=6, infill="70%", **PETG_S,
      function="Bellcrank on the D6 carrier pin under the cage spine: carries the yaw pin, the heave shock and the heave stop dowel",
      mates="cage clevis (D6 x 35), SU6 (D8 yaw pin), heave shock (D3 pin), heave stop dowel D8 x 75 in the pad blocks",
      note="FEM: safety 1.30 two-wheel, 1.06 one-wheel (target 2): OPEN. Hub bore 6.4 on the D6 pin is a REQUIREMENT (loose on purpose)"),
    D(id="SU8", sub="SU", name="Heave saddle, bump", stems=["Heave_saddle_bump_-1", "Heave_saddle_bump_1"], mirrored=True, **TPU,
      function="TPU seat of the heave stop dowel at +12 mm (two-wheel bump)", mates="cage pad block (glued)"),
    D(id="SU9", sub="SU", name="Heave saddle, droop", stems=["Heave_saddle_droop_-1", "Heave_saddle_droop_1"], mirrored=True, **TPU,
      function="TPU seat of the heave stop dowel at -8 mm (two-wheel droop)", mates="cage pad block (glued)"),
    D(id="SU10", sub="SU", name="Yaw stop pad, front", stems=["Yaw_stop_pad_front_-1", "Yaw_stop_pad_front_1"], mirrored=True, **TPU,
      function="Pad on the carrier post: the beam lands on it at 10 deg lean", mates="SU7 post (glued)"),
    D(id="SU11", sub="SU", name="Yaw stop pad, rear", stems=["Yaw_stop_pad_rear_-1", "Yaw_stop_pad_rear_1"], mirrored=True, **TPU,
      function="Pad on the carrier post: the beam lands on it at 10 deg lean", mates="SU7 post (glued)"),
    D(id="SU12", sub="SU", name="Droop stop pad", stems=["Droop_stop_pad_-1", "Droop_stop_pad_1"], mirrored=True, **TPU,
      function="Wedge on the upper link's rail lug: the lever blade lands on it at -32.9 mm", mates="cage rail lug (glued)"),
    # --- steering ---------------------------------------------------------------------------------------------------------------------------------
    D(id="ST1", sub="ST", name="Rocker", stems=["Rocker_-1", "Rocker_1"], mirrored=True, walls=6, infill="100%", **PETG_S,
      function="Steering rocker on the vertical D4 pin: drag link in, tie rod out", mates="ST2 (D4 x 20 pin), 2 ball studs"),
    D(id="ST2", sub="ST", name="Rocker bracket (PETG)", stems=["Rocker_bracket_-1", "Rocker_bracket_1"], mirrored=True, walls=8, infill="100%", **PETG_S,
      function="Clamp on the arm pivot axle with the strap, shoe and rear wall that hold the rocker pin in double shear",
      mates="D5 pivot axle, D4 rocker pin, front deck",
      note="owner's choice (NOTES 26). FEM at the servo limit 0.47 N m: safety 0.75 (about 0.96 at the steering need 0.367 N m): BELOW 2"),
    D(id="ST3", sub="ST", name="Servo floor", stems=["Servo_floor_-1", "Servo_floor_1"], mirrored=True, walls=4, infill="40%", **PETG_S,
      function="Wedge floor that seats the flat-lying steering servo on the deck", mates="front deck, servo, ST4"),
    D(id="ST4", sub="ST", name="Servo clip", stems=["Servo_clip_-1", "Servo_clip_1"], mirrored=True, walls=4, infill="40%", **PETG_S,
      function="Leg, arm and strap over the servo: one M3 x 16 through deck and tongue into an insert", mates="ST3, servo, front deck"),
    # --- drivetrain, rear arm and wheels (as Version 1) ---------------------------------------------------------------------------------------------
    D(id="DT5", sub="DT", name="Pillow block lower", stems=["w3_pillow_lower"], walls=4, infill="60%", **PETG_S, function="Bearing block, lower half", mates="DT6, 5x11x4"),
    D(id="DT6", sub="DT", name="Pillow block upper", stems=["w3_pillow_upper"], walls=4, infill="60%", **PETG_S, function="Bearing block, upper half", mates="DT5"),
    D(id="DT7", sub="DT", name="Rear bearing carrier lower", stems=["w3_rbc_lower"], walls=4, infill="50%", **PETG_S, function="Rear bearing carrier, lower half", mates="tail deck, DT8"),
    D(id="DT8", sub="DT", name="Rear bearing carrier upper", stems=["w3_rbc_upper"], walls=4, infill="50%", **PETG_S, function="Rear bearing carrier, upper half", mates="DT7"),
    D(id="DT10", sub="DT", name="Rear ring hub", stems=["w3_rring_hub"], walls=4, infill="80%", **PETG_S, function="Hub carrying the rear ring gear", mates="DT7/DT8"),
    D(id="DT11", sub="DT", name="Rear swing arm", stems=["w3_rear_arm"], walls=6, infill="60%", **PETG_S,
      function="Single-sided rear swing arm, shock bridge over the 140 mm tyre; three 5x11x4 on the 5 mm cross shaft, two 8x16x5 on the 8 mm axle",
      mates="5 mm cross shaft, rear shock (M6), 8 mm axle"),
    D(id="DT14", sub="DT", name="Rear pivot pulley 26T GT2", stems=["pulley_r_pivot"], walls=6, infill="100%", material="PETG", load="drive", support="off",
      function="26T GT2 pulley on the pivot shaft, D5 bore, M3 grub", mates="2GT-300-6 belt"),
    D(id="DT15", sub="DT", name="Rear axle pulley 30T GT2", stems=["pulley_r_axle"], walls=6, infill="100%", material="PETG", load="drive", support="off",
      function="30T GT2 pulley on the 8 mm axle, M3 grub", mates="2GT-300-6 belt, 8 mm axle"),
    D(id="DT16", sub="DT", name="Front wheel 1.9 in", stems=["wheel_f_-1", "wheel_f_1"], mirrored=True, walls=6, infill="60%", material="PETG", load="structural", support="off",
      function="6-spoke wheel, bead seat 48.3, seat D7.90 x 6 + counterbore D12.4 x 2 for a flanged bronze bushing 4 x 8 x 6, D4.4 clearance bore", mates="flanged bronze bushing on the M4 axle bolt's D4 shank, front tyre",
      note="owner: plain bushings (NOTES decision 35); press the bushing in square, check the seat with the fit coupon"),
    D(id="DT17", sub="DT", name="Rear wheel 2.2 in", stems=["wheel_r"], walls=6, infill="60%", material="PETG", load="structural", support="off",
      function="6-spoke wheel, bead seat 55.9, 12 mm hex drive pocket", mates="12 mm hex hub, rear tyre"),
    D(id="CH5", sub="CH", name="Motor plate", stems=["w3_motor_plate"], walls=4, infill="40%", material="PETG", load="structural+heat", support="auto",
      function="Holds the 540 motor (heat: PETG)", mates="rear deck, motor"),
    # --- the rest of the car (not in this kit; full kit outputs are on hold) --------------------------------------------------------------------------
    D(id="CH1", sub="CH", name="Front deck", stems=["w3_deck_front"], optional=True, walls=4, infill="35%", **PETG_S, function="Flat front deck", mates=""),
    D(id="CH2", sub="CH", name="Rear deck", stems=["w3_deck_rear"], optional=True, walls=4, infill="35%", **PETG_S, function="Centre deck with the pod brackets", mates=""),
    D(id="CH3", sub="CH", name="Tail deck", stems=["w3_deck_tail"], optional=True, walls=4, infill="35%", **PETG_S, function="Rear deck", mates=""),
    D(id="CH4", sub="CH", name="Skid plate", stems=["w3_skid"], optional=True, walls=4, infill="30%", **PETG_S, function="Skid, front edge at the cage V bar", mates=""),
    D(id="CG1", sub="CG", name="Cage", stems=["Cage"], optional=True, walls=4, infill="30%", **PETG_S, function="Cage with the exo frame", mates=""),
]
SH = [("SH1", "Nose tub", "Nose_tub"), ("SH2", "Hood deck", "Hood_deck"), ("SH3", "Side pod", "Side_pod_left Side_pod_right"),
      ("SH4", "Side cover", "Side_cover_left Side_cover_right"), ("SH5", "Hex grille", "Hex_grille"), ("SH6", "Roof panel", "Roof_panel"),
      ("SH7", "Tail light housing", "Tail_light_housing"), ("LT1", "Nose light bar", "Nose_light_bar"), ("LT2", "Tail light strip", "Tail_light_strip")]
for _id, _n, _s in SH:
    PARTS.append(D(id=_id, sub=_id[:2], name=_n, stems=_s.split(), optional=True, walls=3, infill="15%", material="PETG", load="none", support="auto", function="", mates=""))

# bought stand-ins in the model -> hardware (not printed)
HARDWARE_STEMS_PREFIX = ("b_", "mock_", "tip_shaft", "M6_shock_pin", "Rear_shock", "Rear_tire", "Steer_servo", "Steer_horn", "Drag_link", "Tie_rod",
                         "Kingpin", "Front_axle_pin", "Front_axle_bolt", "Front_axle_nut", "Axle_sleeve", "Front_wheel_bushing", "Knuckle_ball_stud", "Rocker_pin", "Rocker_ball_stud", "Lever_ball_stud",
                         "Heave_shock", "Carrier_pin", "Beam_yaw_pin", "Heave_stop_dowel", "Shock_pin", "Uplink_pin", "Cup_block")


def key_of(stem):
    for p in PARTS:
        if stem in p["stems"]:
            return p["id"]
    return None


def is_hardware(stem):
    return stem.startswith(HARDWARE_STEMS_PREFIX)
