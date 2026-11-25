# Fugu reverse trike: assembly manual

Print-only kit. Target printer: Bambu Lab P2S (256 x 256 x 256 mm), 0.4 mm nozzle, PLA for the shell and lights, PETG for the structure. Everything also fits the larger H2C-class bed. **Do not auto-orient or auto-arrange in Bambu Studio**: every file is already oriented and packed.

## 1. Print plates

Open `kit/print/bambu/all_plates.3mf` (every plate) or one plate file. Slot 3 = PLA, slot 4 = PETG (put PETG in AMS slot 4 or reassign in Studio).

| Plate file | Filament | Time | Grams (Bambu slice) | Notes |
|---|---|---|---|---|
| plate_01_CH_PETG.3mf | PETG | 447 min | 284 | sliced clean |
| plate_02_SU_PETG.3mf | PETG | 384 min | 212 | sliced clean |
| plate_03_ST_PETG.3mf | PETG | 33 min | 15 | sliced clean |
| plate_04_DT_PETG_1.3mf | PETG | 625 min | 321 | sliced clean |
| plate_04_DT_PETG_2.3mf | PETG | 41 min | 21 | sliced clean |
| plate_05_CG_PETG_1.3mf | PETG | 464 min | 212 | sliced clean |
| plate_05_CG_PETG_2.3mf | PETG | 682 min | 278 | sliced clean |
| plate_06_SH_PLA_1.3mf | PLA | 288 min | 169 | sliced clean |
| plate_06_SH_PLA_2.3mf | PLA | 247 min | 158 | sliced clean |
| plate_07_LT_PLA_translucent.3mf | PLA | 26 min | 7 | sliced clean |

## 2. Printed parts

| ID | Part | Plate group | Material | Joins |
|---|---|---|---|---|
| CH1 | Front deck | Chassis frame and decks | PETG | CH2, CH3, cage feet (M3 inserts) |
| CH2 | Rear deck | Chassis frame and decks | PETG | CH1, CH3, CH4, cage feet |
| CH3 | Tail deck | Chassis frame and decks | PETG | CH2, DT rear carriers |
| CH4 | Skid plate | Chassis frame and decks | PETG | CH1 |
| CH5 | Motor plate | Chassis frame and decks | PETG | CH2, motor |
| SU1 | Lean head | Front suspension | PETG | CH1 (4 x M3 from below into inserts), SU2, SU3 (6 mm pivot rods) |
| SU2L, SU2R | Upper wishbone | Front suspension | PETG | SU1 pivot rod, SU4 (6.8 mm ball cup), heave shock (M4 pin) |
| SU3L, SU3R | Lower wishbone | Front suspension | PETG | SU1 pivot rod, SU4 (6.8 mm ball cup) |
| SU4L, SU4R | Knuckle | Front suspension | PETG | SU2, SU3 (ball studs M3), 6 mm axle pin, tie rod ball stud |
| ST1 | Steering horn | Steering | PETG | steering servo, tie rod ball studs |
| ST2 | Steering servo mount | Steering | PETG | CH1, steering servo |
| DT5 | Pillow block lower | Drivetrain housings and rear arm | PETG | DT6 |
| DT6 | Pillow block upper | Drivetrain housings and rear arm | PETG | DT5 |
| DT7 | Rear bearing carrier lower | Drivetrain housings and rear arm | PETG | CH3, DT8 |
| DT8 | Rear bearing carrier upper | Drivetrain housings and rear arm | PETG | DT7 |
| DT10 | Rear ring hub | Drivetrain housings and rear arm | PETG | DT7/DT8 |
| DT11 | Rear swing arm | Drivetrain housings and rear arm | PETG | CH3, rear shock, rear hub |
| DT14 | Rear pivot pulley 26T GT2 (6 mm belt) | Drivetrain housings and rear arm | PETG | 2GT-300 belt |
| DT15 | Rear axle pulley 30T GT2 (6 mm belt) | Drivetrain housings and rear arm | PETG | 2GT-300 belt, 8 mm axle |
| DT16L, DT16R | Front wheel 1.9 in, 6-spoke | Drivetrain housings and rear arm | PETG | 12 mm hex hub, front tyre |
| DT17 | Rear wheel 2.2 in, 6-spoke | Drivetrain housings and rear arm | PETG | 12 mm hex hub, rear tyre |
| CG1a, CG1b, CG1c | Cage | Cage | PETG | CH1/CH2 (6 x M3 into inserts), SH1, SH2, SH3, rear shock pin |
| CG2 | Front light contact block | Cage | PETG | CG1 front bridge |
| CG3 | Roof plate | Cage | PETG | CG1 roof rails |
| SH1a, SH1b1, SH1b2, SH1c1, SH1c2 | Front shell | Shell | PLA | SH2 (lap flange), SH4, LT1, CG1 bosses |
|  | Canopy | Shell | PLA | SH1 lap flange, CG1 |
| SH3 | Tail light housing | Shell | PLA | CG1 (2 x M3 countersunk), LT2 |
| SH4 | Grille insert | Shell | PLA | SH1 nose |
| LT1 | Nose light strip | Lights | PLA-translucent | SH1 groove, pegs through backing |
| LT2 | Tail light strip | Lights | PLA-translucent | SH3 seat |

Parts longer than the bed are printed in pieces (suffix a, b, c): the Front shell (SH1a/b), the Cage (CG1a/b/c) and, only with the closed cockpit, the Canopy (SH2a/b). Pin joints: pins on piece `a` go into the holes of piece `b`. Glue with CA, hold flat until set.

## 3. Hardware to buy

See `bom.md` for counts, part numbers and which are only ASSUMED.

## 4. Tools

- Soldering iron (heat-set inserts, LED and magnet wiring)
- Hex drivers 1.5 / 2.0 / 2.5, small Phillips
- CA glue + activator (shell seams), PETG-safe
- Side cutters and deburring knife (support removal)
- Fine sandpaper 220-400 and filler primer if painting

### Step 0: test fit

Print the steering horn ST1 and one knuckle SU4 first. Press an M3 insert into a scrap, screw a 6.8 mm ball stud into the knuckle plate and snap a ball cup on. Test one 1.6 mm pin in a 1.75 mm hole on a Cage joint. Stop and tell the designer if a stud does not thread or a pin does not enter.

### Step 1: chassis frame (CH1-CH5)

Join decks CH1/CH2/CH3 to the donor chassis hardware, screw the skid CH4 under the front deck's front edge (2 x M3) and fit the motor plate CH5. Check: decks sit flat, no rocking.

### Step 2: lean head and wishbones (SU1-SU4)

Press M3 inserts into the four base tube bores of the lean head SU1 and screw it to the front deck from below, right behind the skid. Push the four 6 mm pivot rods in from the rear through the head and the pivot barrels of the upper wishbones SU2 (with the shock posts) and lower wishbones SU3. Screw the ball studs into the knuckle plates SU4, thread a ball cup into each wishbone tip and snap the cups onto the studs. Hang the heave shock between the two posts of the upper wishbones (M4 pins). Check: both wheels move up and down together against the shock, the body leans freely about 10 degrees each way with the shock length unchanged, nothing touches the deck or the head.

### Step 3: steering (ST1-ST2)

Screw the servo mount ST2 to the front deck behind the head (4 x M3 from below into inserts in its pillars), stand the low-profile servo between the pillars with the shaft up and screw its ears down (2 screws per ear), fit the printed T horn ST1 on the shaft and hang a ball stud from each end of it, and connect each horn ball to the ball stud on the knuckle's steering arm with a tie rod. Set both tie rods to the same length (93 mm ball to ball). Check: full lock both ways at full lean without rubbing; the tie rods pass 1.5 mm behind the pivot rod heads.

### Step 4: drivetrain (DT5-DT17)

Rear wheel drive only: assemble the bearing blocks DT5-DT8 and the ring hub DT10 on the rear shaft, fit the motor with its pinion to the motor plate against the spur gear, the single-sided rear arm DT11 with its pulleys DT14/DT15 and the 300 mm belt, the rear wheel DT17 and the rear shock. Fit the front wheels DT16 on the stub axles. Check: wheels turn by hand, belt tight, rear arm swings with no contact.

### Step 5: cage (CG1a/b/c, CG3)

Glue the three cage pieces on their pins (a to b to c), let cure 30 min. Press M3 inserts into the six feet, drop the feet into their deck pockets (front pair at y -127, behind the tie rods) and screw them from under the decks. Lay the roof plate CG3 on the roof frame. Fit the shock top bolt through the cage clevis. Check: full steering lock and full lean with the cage on: tie rods and shock posts clear the front tubes.

### Step 6: shell (SH1a/b, SH4)

Glue SH1a to SH1b on their pins, then the grille SH4 from inside the nose. Dry-fit over the cage first. Screw M2 into the cage bosses. Check: both front wheels at full lean and full lock clear the arches.

### Step 7: lights (LT1, LT2, SH3, CG2)

Press magnets (alternate polarity) into CG2 and the cage pad, fit LEDs in the pegs, push the strips into their channels, screw SH3 to the cage with two countersunk M3 x 8. Check: contacts meet with a 0.3 mm gap, strips sit flush.

## Known risks (not verified on a printed kit)

- Nothing in this kit has been printed or assembled yet. Fits (ball studs, pivot rods, inserts) are from the model only.
- Front end strength from FEM (target safety 2): upper wishbone 2.65, lower wishbone 2.30, lean head 1.90, knuckle 1.54 at the upper ball stud bore. Cage 1.39 at a tail tube joint, rear arm 1.66 at one inner corner. See NOTES.md.
- The heave shock, ball studs, tie rods and pivot rods are not selected parts: sizes in bom.md are taken from the model and marked ASSUMED.
- No battery position or hold-down is designed in this version (tray and battery were removed from the front deck).
- Pin joints in the 3 mm shell walls are 1.6-2.0 mm pins: glue joint strength is unmeasured.
- Chassis parts marked in the model as stand-ins (decks, plates) may not match the real printable parts. Check CH1-CH5, DT5-DT10 against your real chassis files.
- Printer: NOTES.md says H3C (H2C-class, 330 x 320 mm); the saved projects are P2S. The kit is sized for P2S.
- Rear tyre and wheel are not chosen (140 mm OD in the model).
