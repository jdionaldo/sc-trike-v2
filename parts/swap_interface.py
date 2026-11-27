"""Interfaces of the knuckle and the rocker mount (owner, November 24: "printed + servo based on actual production off the shelf parts and
measurements JIC i want to swap it out later"; NOTES decisions 26, 27).

ONE place for every feature a metal replacement must copy: hole positions, pin and stud sizes, screw patterns, mounting faces and the outer
envelope. The printed parts (parts/v2_knuckle.py, parts/swingarm_front.py rocker_bracket) are built from these constants; the metal versions
are the same shapes machined (aluminium 6082 / 7075 or steel), so they bolt in without changing anything else. Table for a machinist or a
buyer: docs/v2/knuckle_rocker_interface.md. Bought-part dimensions and their sources: docs/v2/bought_part_datasheets_2026-11-24.md.
World mm, +x side (the -x side is the mirror image).
"""
from parts import swingarm2 as k

# ---------------------------------------------------------------------------------------------------------------------------- knuckle
KN_KINGPIN_AXIS = k.KINGPIN                  # (x, y) of the vertical kingpin axis
KN_KINGPIN = (4.0, 40.0)                     # ISO 8734 hardened dowel pin D4 m6 x 40 (bought): diameter, length
KN_KINGPIN_BORE = 4.1                        # knuckle ears and carrier block: running fit on the D4 m6 pin (ream by hand in the printed part)
KN_EAR_LO = (22.0, 30.0)                     # lower ear, z bottom face (bearing span) .. top face
KN_EAR_HI = (51.0, 59.0)                     # upper ear
KN_THRUST = (5.5, 0.5)                       # raised thrust seat on each ear's inner face: radius, height; a 0.5 mm M4 shim washer on it
KN_BLOCK_GAP = 1.0                           # ear face to the carrier's kingpin block (seat 0.5 + washer 0.5)
KN_BLOCK_Z = (KN_EAR_LO[1] + KN_BLOCK_GAP, KN_EAR_HI[0] - KN_BLOCK_GAP)    # carrier kingpin block and web z 31 .. 50
KN_AXLE_AXIS = k.TIP                         # (y, z) of the horizontal axle (along x)
KN_AXLE_BOLT = ("ISO 4762 M4 x 35, class 12.9", 4.0, 35.0)   # from outboard through the wheel hub into the nut; the thread must stay inboard of x 140
KN_AXLE_BORE = 4.2                           # through the spine
KN_NUT_POCKET = (7.2, 3.4, 131.0)            # M4 nut (ISO 4032: 7.0 AF, 3.2 high): pocket across flats, depth, inboard face x
KN_SLEEVE = (6.0, 4.0, 134.4, 148.0)         # steel tube 6 x 1, pressed in: OD, ID, x from .. to (the bolt runs inside it)
KN_HUB_FACE_X = 148.0                        # seat ring face on the wheel hub (running face)
KN_SEAT_R = 6.0
KN_BALL_AXIS = k.KNUCKLE_BALL_B[:2]          # (x, y) of the tie rod ball stud, vertical
KN_BALL_Z = k.KNUCKLE_BALL_B[2]              # ball centre z (stud screwed in from below)
KN_STUD = ("6.8 mm ball stud, M4", 4.0)      # thread
KN_STUD_INSERT = (5.6, 9.0)                  # M4 heat-set insert pocket from the top of the ball boss (printed); tapped M4 in metal
KN_ARM_Z = (21.0, 30.0)                      # steering arm plate z
KN_ENVELOPE = dict(spine_x=(131.0, 143.0), spine_r_max=k.WHEEL_BORE_R - 1.1, x_min=101.3, carrier_gap=1.0, wheel_gap=1.0)

# ---------------------------------------------------------------------------------------------------------------------------- rocker mount
RM_AXLE = (5.0, 5.1)                         # arm pivot axle (hardened D5 shaft): diameter, clamp bore
RM_AXLE_AXIS = k.PIVOT                       # (y, z) of the pivot axle, along x
RM_CLAMP_X = k.ROCKER_MOUNT_X[:2]            # clamp length along the axle, x from .. to (between the arm's plates)
RM_SLIT = (0.8, 13.0)                        # clamp slit: width, length rearward from the axle centre in the plane z = axle z (C opens to the rear)
RM_CLAMP_SCREW = ("ISO 4762 M3 x 12", (78.0, -110.0))   # vertical, from the top through the upper jaw into the hanger: (x, y) of its axis
RM_CLAMP_HOLE = 3.4                          # clearance in the upper jaw
RM_CLAMP_INSERT = (4.0, 5.7)                 # M3 heat-set insert OD 4.6 x 5.7 in the printed hanger: pocket diameter, depth; tapped M3 in metal
RM_PIN_AXIS = k.ROCKER_PIN                   # (x, y) of the vertical rocker pin
RM_PIN = (4.0, 24.0)                         # ISO 8734 D4 m6 x 24 (bought; was x 20 when the pin's top sat in the foot, NOTES 42)
RM_PIN_BORE = 4.0                            # press fit for the pin in foot and shoe (the rocker turns on it with D4.1)
RM_SHOE_FLOOR = 0.5                          # the shoe's pin bore is blind: 0.5 mm floor under the pin
RM_ENVELOPE = dict(foot_z=(34.0, 37.5), shoe_z=(16.5, 18.7), strap_x=(85.5, 88.5), strap_y=(-120.8, -103.8), mount_y=(-120.8, -112.8),
                   rear_wall_y=(-106.8, -103.8), top_gap_to_arm_boss=1.0)

# ------------------------------------------------------------------------------------------------------------------ beam carrier tail
# (NOTES 60, item 1(b)) the beam carrier's tail and the cage's pad blocks it swings in. A machined carrier (aluminium, the written
# alternative to the printed one) copies these; parts/swingarm_guide.py asserts its values against them. Literals here, because the
# carrier geometry comes from tools/beam_kin.py, which imports this package's swingarm2 (no import back).
CT_PIVOT_AXIS = (-50.6, 98.5)                # (y, z) of the carrier pin axis PC, along x
CT_TAIL_AXIS = (-40.25, 97.5)                # (y, z) of the heave stop dowel at rest: r 10.4 from PC, 5.5 deg below its horizontal
CT_DOWEL = ("D8 x 75 stock dowel", 8.0, 75.0)                       # name, diameter, length (ASSUMED stock length)
CT_SLEEVE = ("SKF PSM 8-12-12 A51", 8.0, 12.0, 12.0)               # two, pressed in from each end of the tail boss: ID, OD, length
CT_SLEEVE_BORE = (11.9, 2.2)                 # printed bore for the sleeves (ASSUMED press fit), D8 locating bore between them (length)
CT_BOSS_X = 13.1                             # tail boss and web end faces at |x|
CT_BOSS_WALL_TOP = 1.2                       # boss wall over the 12 mm bore on the spine side (the spine bridge sets it)
CT_ARC = (-9.28, 15.46)                      # carrier turn about PC at the droop and bump stops, deg (+ = bump)
CT_SLOT_CLEAR = 0.15                         # radial clearance of the dowel in the arc slots of the pad blocks and clevis plates
CT_PLATE_X = (14.4, 17.4)                    # clevis plates |x| (cage); the pad blocks from 17.4 outward
CT_SADDLE_X = (18.4, 35.4)                   # TPU saddles |x|
CT_PAD_BLOCK_Y = (-50.3, -31.7)              # pad block y front .. rear face (rear face moved back 1.9 mm in 1(b))
CT_BRIDGE_GAP = 1.00                         # least gap carrier to cage (clevis plate inner face, x 14.4) at rest and at both stops
