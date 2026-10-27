# BIOBUZZ (FTC 2026–2027): HIVE geometry, field coordinates, game pieces, and launching rules for a shoot-on-the-move robot

Source notes. Primary source: the official Competition Manual PDF, Version TU05 (Oct 15, 2026; 175 pages), downloaded from https://ftc-resources.firstinspires.org/ftc/game/manual. The PDF was text-extracted and its figure pages were rendered and read directly. Section, figure, and rule numbers below refer to that TU05 PDF. Per-section links follow the pattern https://ftc-resources.firstinspires.org/ftc/game/manual-NN (for example https://ftc-resources.firstinspires.org/ftc/game/manual-09). Other primary sources: the Event Field Setup Guide V1.0 (Sep 12, 2026), the combined Team Updates TU00–TU05, the FTC SDK 12.0 source (`AprilTagGameDatabase.java` in Vision-12.0.0-sources.jar), and the SDK 12.0 README.

## 1. Official name, release date, and where the official resources are

### Takeaway
"BioBuzz" is the real game: the official name is **BIOBUZZ™ presented by RTX** (stylized in all caps), the FIRST Tech Challenge game for 2026–2027 within the **FIRST® CANOPY™** season. It was released at Kickoff on **September 12, 2026**. The current manual is **TU05 (Oct 15, 2026)**. All official field resources (manual, field CAD, setup guide, AprilTag art, animation, field tour) are posted, and the links are below.

### Cited Findings
- Official name: "In BIOBUZZ™ presented by RTX, 2 competing ALLIANCES of 2 teams each collect POLLEN and NECTAR, LAUNCHING them into their HIVE and placing them into FLOWERS." (Section 8 Game Overview) — [Competition Manual TU05](https://ftc-resources.firstinspires.org/ftc/game/manual)
- The manual's page header reads "FIRST® CANOPY™" (the season brand). The cover reads "2026-2027 FIRST® Tech Challenge Competition Manual, BIOBUZZ TM Presented by RTX, Version: TU05" — [Competition Manual TU05](https://ftc-resources.firstinspires.org/ftc/game/manual)
- Kickoff and reveal date was September 12, 2026. A search snippet of the FIRST page said the reveal was at 12:00 pm ET and streamed on the FTC YouTube channel; I did not open that page — [FIRST info page](https://info.firstinspires.org/ftc-05-07-26); [FIRST press release](https://www.firstinspires.org/about/press-room/first-tech-challenge-teams-to-draw-inspiration-from-the-natural-world-during-biobuzz-presented-by-rtx)
- The Event Field Setup Guide revision history shows "1.0 9/12/2026 Kickoff Release" — [Event Field Setup Guide V1.0](https://ftc-resources.firstinspires.org/ftc/archive/2027/field/eventfieldguide)
- The Brazilian press article (dated Sep 14, 2026) confirms the Sep 12 launch and describes yellow POLLEN, red/blue NECTAR, and the HIVE TIP mechanic — [Portal da Indústria](https://noticias.portaldaindustria.com.br/noticias/robotica/conheca-o-desafio-da-biobuzz-nova-temporada-da-first-tech-challenge/)
- The FTC SDK 12.0 README says: "This repository contains the public FTC SDK for the BIOBUZZ (2026-2027) competition season". The SDK 12.0 build is dated 20260907-090034. In code the game is spelled "BioBuzz", e.g. `AprilTagGameDatabase.getBioBuzzTagLibrary()` — [FtcRobotController GitHub](https://github.com/FIRST-Tech-Challenge/FtcRobotController) (local copy: /home/andrei0204/Coding/Robo/FtcRobotController-12.0/README.md)
- The Game Materials page lists:
  - Competition Manual PDF `/ftc/game/manual` and HTML `/ftc/game/cm-html`, both Version TU05 (Oct 15, 2026)
  - Team Updates TU00 (Sep 12, v2 Sep 17), TU01 (Sep 17), TU02 (Sep 24), TU03 (Oct 1), TU04 (Oct 8), TU05 (Oct 15), plus a combined update at `/ftc/game/tu-combined`
  - Team Q&A at https://ftc.game/qa
  - Game Animation at https://youtu.be/sUH3z5a5S9I
  - Competition Manual AI chatbot at https://ftc-cmchatbot.firstinspires.org/

  Source: [FTC Game Materials page](https://ftc-resources.firstinspires.org/ftc/game)
- Status of individual manual sections: Section 9 ARENA is at TU04 (Oct 8, 2026). Sections 10 Game Details, 11 Game Rules, and 12 Robot Construction are at TU05 (Oct 15, 2026) — [FTC Game Materials page](https://ftc-resources.firstinspires.org/ftc/game)
- The Playing Field Resources page (https://ftc-resources.firstinspires.org/ftc/archive/2027/field) lists:
  - Initial Field Element Assembly Guide (V1.0, Sep 12): `/ftc/archive/2027/field/initialfieldguide`
  - Event Field Setup Guide (V1.0, Sep 12): `/ftc/archive/2027/field/eventfieldguide`
  - Field Acceptance Checklist (26-27.1, Oct 16): `/ftc/archive/2027/field/field-check`
  - Field Mitigation Guide (26-27.1, Oct 16): `/ftc/archive/2027/field/field-mitigation-guide`
  - Onshape Field CAD (Version 1, Sep 12): https://cad.onshape.com/documents/a355e772e3d24813de7852ee/w/f106353168f1f92100b81259/e/95d1e1e442b4138cccaf2d73
  - FLOWER Scoring Volume CAD (Onshape): https://cad.onshape.com/documents/a355e772e3d24813de7852ee/w/f106353168f1f92100b81259/e/8f89b946e295be9c543faa08
  - Field CAD STEP zip (v26-27.2, Sep 15): `/ftc/archive/2027/field/field-cad-step`
  - AprilTag Production Art (V1, Sep 18): `/ftc/archive/2027/field/apriltag-art`
  - AprilTag printables, US Letter and A4 (V1, Sep 21): `/ftc/archive/2027/field/apriltag-us` and `/ftc/archive/2027/field/apriltag-a4`
  - Field Tour video: https://youtu.be/47X9sYnPijw
  - AndyMark store: https://andymark.com/collections/first-tech-challenge-2026-2027

  Source: [Playing Field Resources](https://ftc-resources.firstinspires.org/ftc/archive/2027/field)
- Section 9.1: "The 3D CAD model is the official representation of the BIOBUZZ FIELD... Measurements may be taken from this model with a general tolerance of +/- 1 in. (+/- 2.5 cm)." Illustration dimensions "are nominal" and carry ±1 in (±2.5 cm) unless noted otherwise — [Competition Manual TU05 §9.1](https://ftc-resources.firstinspires.org/ftc/game/manual)
- Official events use the full BIOBUZZ field sold by AndyMark (am-5850_Full), with Soft Tiles am-2499 and Perimeter Kit am-0481 (§9.2) — [Competition Manual TU05](https://ftc-resources.firstinspires.org/ftc/game/manual)
- The pre-season Game Preview (May 3, 2026) first revealed POLLEN as "plastic balls approximately 3 in. in diameter" with "similar characteristics to the Artifacts used in the 2025-2026" game DECODE — [FTC Community: Game Preview Field Elements](https://community.firstinspires.org/game-preview-field-elements)

### Inferences
- "BioBuzz" matches the official game. The correct spelling is "BIOBUZZ" (all caps, ™). The SDK code uses "BioBuzz".
- The Section 9 figures are the most authoritative 2D drawings. For anything not dimensioned there, such as the exact XY location of the cell opening, the Onshape CAD is the official reference and should be measured directly before final tuning.

### Gaps
- I did not open the Onshape CAD or the STEP file, so CAD-measured coordinates of the cell openings are not included. Section 2 gives values derived from the manual's dimensioned figures.
- I could not read the Field Acceptance Checklist (Oct 16), which holds the controlled tolerances of HIVE dimensions.

## 2. The HIVES: count, location, opening size/shape/height, orientation, obstructions

### Takeaway
There are **2 HIVES** (1 red, 1 blue). Both hang from **one central HIVE Structure** at the center of the field. Each HIVE is a bi-stable seesaw with **2 CELLS** (targets): one toward the audience, one toward the rear ("Scoring") wall. Only the **upward-facing CELL** is a valid target.

The opening is a **house-shaped pentagon, 20 in wide × 14 in tall, 12 in deep**. Its lower edge is at **53.5 in** and its top at **65.6 in** above the tiles. The pivot is at **43.95 in** and the arm is tilted **30°**.

The opening sits at the **outer end** of the arm and faces **away from field center**, toward the audience wall or the rear wall, tilted about 30° upward. The red and blue HIVE centerlines are **25.5 in apart**. The up cell flips sides after every TIP, so the target moves during a match.

### Cited Findings
**Location and frame**
- "The HIVE Structure is located in the center of the FIELD. A frame holds a red HIVE and a blue HIVE. Each HIVE consists of 2 CELLS, one on either end of each HIVE." (§9.6) — [Competition Manual TU05](https://ftc-resources.firstinspires.org/ftc/game/manual)
- "Each HIVE is on a pivot and can tip so that one of the CELLS is facing upwards at any given time. Each HIVE is bi-stable and will hold its position until enough POLLEN or NECTAR are LAUNCHED into the upwards-facing CELL. On the bottom face of each CELL is a unique AprilTag Cluster containing 4 distinct AprilTags." (§9.6) — [Competition Manual TU05](https://ftc-resources.firstinspires.org/ftc/game/manual)
- Frame (§9.6.1, Fig 9-8): "two triangular metal structures that attach to mounting strips under the TILES which are connected at their apex by a crossbar." It is **49.46 in (125.65 cm) wide** and **38.95 in (98.95 cm) deep** at its base, which is also its widest point. The **pivot axis is 43.95 in (111.65 cm) above the TILES**. A BIOBUZZ logo panel is attached to each side of the frame — [Competition Manual TU05](https://ftc-resources.firstinspires.org/ftc/game/manual)
- Setup guide §9.1: the under-tile strips go below the **4 center tiles**, and staff "adjust the placement later so the Frame Assembly is centered on the field." Wing nuts clamp the frame foot bars into the tile foam (§9.2) — [Event Field Setup Guide V1.0](https://ftc-resources.firstinspires.org/ftc/archive/2027/field/eventfieldguide)
- In Fig 9-4/9-5 (tile grid) and Fig 9-17 (AprilTag locations), the HIVE Structure occupies tiles C3, C4, D3, D4. The red HIVE is in column C (red half) and the blue HIVE in column D (blue half). Each HIVE's long axis runs audience↔rear, with one cell in row 3 (audience) and one in row 4 (rear) — [Competition Manual TU05 Figs 9-4, 9-5, 9-17](https://ftc-resources.firstinspires.org/ftc/game/manual)

**HIVE and CELL dimensions**
- §9.6.2 text: "Each HIVE includes two CELLS (either both red or both blue) approximately 18.8 in. (47.8 cm) apart... Each HIVE rotates around a pivot located at the top of the frame and has two stable positions: each one with one CELL facing upwards." — [Competition Manual TU05](https://ftc-resources.firstinspires.org/ftc/game/manual)
- §9.6.2 text: "The opening of the CELL is approximately 20 in. (50.8 cm) wide by 14 in (35.6 cm) tall and 12 in. (30.5 cm) deep" — [Competition Manual TU05](https://ftc-resources.firstinspires.org/ftc/game/manual)
- Fig 9-9 (HIVE Details), read from the figure:
  - Gap between the two cells: 18.84 in (47.85 cm)
  - Each cell's length along the arm: 12.04 in (30.58 cm)
  - Overall HIVE length: 42.91 in (109.0 cm)
  - Labeled parts: "Pivot" (centered between the cells) and "Damper"

  Source: [Competition Manual TU05 Fig 9-9](https://ftc-resources.firstinspires.org/ftc/game/manual)
- Fig 9-10 (HIVE and CELL Key Details), read from the figure:
  - The HIVE rests tilted **30°** from horizontal
  - "Top of HIVE Opening above TILES 65.6 in. (166.60 cm)"
  - "Bottom of HIVE Opening above TILES 53.5 in. (135.90 cm)"
  - "Bottom of HIVE above TILES 30.6 in. (77.70 cm)"
  - "HIVE Center to Center 25.5 in. (64.75 cm)" (red HIVE plane to blue HIVE plane)

  Source: [Competition Manual TU05 Fig 9-10](https://ftc-resources.firstinspires.org/ftc/game/manual)
- TU01 (Sep 17, 2026) corrected "Bottom of HIVE above TILES" in Fig 9-10 to 30.6 in (77.70 cm) — [Combined Team Updates](https://ftc-resources.firstinspires.org/ftc/game/tu-combined)
- Fig 9-11 (CELL), read from the figure: the opening is a pentagon ("house" shape), 20 in (50.8 cm) wide. Its straight side walls are 7.61 in (19.35 cm) tall and its peak is 14 in (35.55 cm) above the flat base edge. The flat base edge has a square notch where the HIVE's square tube runs under the cells — [Competition Manual TU05 Fig 9-11](https://ftc-resources.firstinspires.org/ftc/game/manual)
- Fig 9-15 (AprilTag cluster), read from the figure:
  - "Front of Cell" to "Centerline of Reference Holes" = 9.938 in (25.25 cm)
  - Tag centers are 2.75 in (7.0 cm) above the reference-hole line
  - This places the tag centers 7.19 in back from the front (opening) of the cell

  Source: [Competition Manual TU05 Fig 9-15](https://ftc-resources.firstinspires.org/ftc/game/manual)

**Initial state and staging**
- Setup guide §11.1: "At the start of each match, the Hives should be tipped with the Cell of the Red Hive on the 'Audience' side of the field tilted up, and the Cell of the Blue Hive on the 'Scoring' side of the field tilted up." The "Scoring" side is the side opposite the audience (labeled so in the guide's figure). "Each upward tilted Cell has [3] Nectar in it. These Nectar start adjacent to the Hive Back Skin, approximately adjacent to the Cell Wall closest to the Field Perimeter" — [Event Field Setup Guide V1.0](https://ftc-resources.firstinspires.org/ftc/archive/2027/field/eventfieldguide)
- Manual §10.3.1: "Each HIVE is tilted such that one CELL is pointed down, and the other CELL is pointed up... the CELL which 'points at' a FLOWER should be the one tilted down." NECTAR in the CELL is placed "contacting the back wall of the CELL and in a line against the side closest to the ALLIANCE AREA of corresponding color." — [Competition Manual TU05](https://ftc-resources.firstinspires.org/ftc/game/manual)

**Tip thresholds and ballast**
- Setup guide §12 (Hive Calibration): "Each Hive should be calibrated to tip when [2] combinations of scoring elements are placed in an upward facing Cell: [8] Pollen + [0] Nectar, and [3] Pollen + [3] Nectar." The required test results (§12.3) are:

  | Cell contents | Next POLLEN | Required result |
  |---|---|---|
  | 3 NECTAR + 1 POLLEN | 2nd POLLEN "Tossed In" | No Tip |
  | 3 NECTAR + 2 POLLEN | 3rd POLLEN "Tossed In" | Tip |
  | 0 NECTAR + 6 POLLEN | 7th POLLEN "Tossed In" | No Tip |
  | 0 NECTAR + 7 POLLEN | 8th POLLEN "Tossed In" | Tip |

  Source: [Event Field Setup Guide V1.0](https://ftc-resources.firstinspires.org/ftc/archive/2027/field/eventfieldguide)
- Setup guide: ballast washers sit on "the 10-32 x 2.0 in. Long Screw at the end of each Cell nearest its opening." The guide also states "[3] Pollen + [3] Nectar has less mass than [8] Pollen" and "[6] Pollen has less mass than [3] Nectar + [2] Pollen" — [Event Field Setup Guide V1.0](https://ftc-resources.firstinspires.org/ftc/archive/2027/field/eventfieldguide)

**What counts as a TIP**
- §10.5.1: a TIP is when the HIVE "moves from one stable state to the other... and subsequently, the damper on the HIVE that was previously not contacting the frame begins to contact the frame." — [Competition Manual TU05](https://ftc-resources.firstinspires.org/ftc/game/manual)
- §10.5.1 also warns: "LAUNCHING at the downward-facing CELL while a HIVE is tipping may disrupt its movement and may result in the TIP not being achieved... teams in this situation may need to pause their LAUNCHING" — [Competition Manual TU05](https://ftc-resources.firstinspires.org/ftc/game/manual)

**Other field elements**
- FLOWERS (possible obstacles and secondary targets, §9.7): 4 FLOWERS on the perimeter wall. The top opening is about 4 in (10.15 cm) in diameter and about 21.5 in (54.6 cm) above the TILES, with a 1.25 in backstop — [Competition Manual TU05](https://ftc-resources.firstinspires.org/ftc/game/manual)

### Inferences
**Which way the opening faces.** These derivations are mine, built from the official figures. Several facts agree:
- Fig 9-10 gives the opening a vertical extent of 65.6 − 53.5 = 12.1 in. That equals 14 in × sin 60°, so the pentagonal opening plane is tilted 60° from horizontal, perpendicular to the 30°-tilted arm.
- The ballast is "at the end of each Cell nearest its opening".
- Staged NECTAR rests against the "back skin", where the up cell's 30° floor slopes down toward the pivot.
- The ftc-docs AprilTag tip says a robot "on the AUDIENCE half" sees the scorable RED AUDIENCE cell right side up (Section 5).

So the **opening is at the outer end of each cell and faces away from field center**: toward the audience wall for the audience-side cell, toward the rear wall for the scoring-side cell. Its outward normal points about **30° above horizontal**.

**Where to shoot from.** Shots should come from the half of the field the up cell faces, travelling toward field center. A ball enters only if its velocity has a component into the opening plane. With horizontal speed h toward center and vertical speed v_z, that means v_z < 1.73·h: rising at less than 60°, or descending at any angle. The opening looks largest to a ball descending at about 30° below horizontal toward center. Its effective narrow dimension is about 14·cos(γ − 30°) in, where γ is the descent angle; a flat shot sees about 12.1 in. Subtract the ball diameter (2.8 in POLLEN, 3.6 in NECTAR) for clearance.

**Pentagon shape matters for aim.** The pentagon gives the full 14 in of height only at its horizontal center. At the side walls it is only 7.61 in. Aim at the lateral center of the 20-in width.

**Derived target coordinates.** These are geometry estimates from the nominal figures, not official coordinates.
- Pivot at Z = 43.95 in, arm at 30°, cell opening plane about 21.5–22.9 in from the pivot along the arm.
- That bracket comes from two fits: (A) half the 42.91 in overall length, fitted to the 53.5 in opening bottom; (B) fitted to both the 53.5 in opening bottom and the 30.6 in "bottom of HIVE".
- Center of the up cell's opening, i.e. the AprilTag cluster origin (Section 5):
  - **Z ≈ 58.4 in (≈1483 mm)** above the tiles
  - **horizontal distance from field center along the audience↔rear axis ≈ 16.5–18.1 in (≈419–460 mm)**, on the side the up cell faces
- The opening spans Z 53.5–65.6 in and runs horizontally from about 12.3–13.9 in (peak) to about 19.3–20.9 in (lower edge) from center.
- Laterally, along the red↔blue axis, the red HIVE centerline is 12.75 in (≈324 mm) to the red side of center and the blue centerline 12.75 in to the blue side. This assumes the structure is centered, which the setup guide requires. Each cell spans 2.75–22.75 in from center on its alliance's side.
- All of these carry the manual's ±1 in tolerance. Measure them in the Onshape CAD before final use.

**Clearance and obstructions.**
- The down cell's lowest point is about 30.6 in above the tiles. Robots may be at most 29 in tall (R105), leaving about 1.6 in nominal clearance under a down cell, within the ±1 in tolerance. Do not plan on driving a 29-in robot under a down cell.
- The up cell's outer end is the target. The down cell and the frame legs (footprint about 49.5 × 39 in centered on the field) are obstacles near the hive.
- The neighboring alliance's cells are only about 5.5 in away laterally (25.5 − 20 in). A lateral miss of more than about 10 in can hit the opponent HIVE, which risks G417.D (Section 4).

**How many balls trigger a TIP.**
- First TIP of the match: the up cell already holds 3 NECTAR, so about 3 POLLEN tips it.
- Later TIPS: the newly up cell starts roughly empty, so about 8 POLLEN are needed (or mixes involving NECTAR).
- Mass ratio from the guide's two inequalities: 4·m_POLLEN < 3·m_NECTAR < 5·m_POLLEN, so **1.33 < m_NECTAR / m_POLLEN < 1.67**.

**Consequence for shoot-on-the-move.** The target is two-state. A shooter must track the HIVE state (which cell is up) and switch the aim point from +X to −X (or the reverse) after each TIP. It should also hold fire briefly during the tip transient.

### Gaps
- No official numeric XY coordinate of the cell openings exists in the manual or the SDK; the SDK sets cluster field positions to (0,0,0). The coordinates above are derived. The Onshape CAD should be measured to confirm them.
- I did not find the exact internal geometry of the cell interior: back-skin angle, side-wall material, any lip or retaining features. These determine bounce-out behavior.
- Tip transient duration and damper behavior are not specified officially.

## 3. Game pieces and how they score in the HIVE

### Takeaway
**POLLEN**: yellow, about **2.8 in (71 mm)**, Gopher ResisDent™ polyethylene ball; 40 per match. **NECTAR**: red or blue, about **3.6 in (91 mm)**, same material; 8 per alliance. The balls are not perfectly spherical. Robots launch POLLEN and their own-color NECTAR into their own HIVE's **upward-facing CELL**. Each **TIP is worth 20 points**, in AUTO or TELEOP. Each element **left in an up-facing CELL at match end is worth 2 points**. TIPS also unlock NECTAR and count toward the POLLINATOR ranking points. No color-pattern requirement applies to the HIVE.

### Cited Findings
**The balls**
- §9.8: "POLLEN are approximately 2.8 in. (7.1 cm) Gopher ResisDent™ polyethylene balls in yellow (am-5851_yellow). NECTAR are approximately 3.6 in. (9.1 cm) Gopher ResisDent™ polyethylene balls in red (am-5852_red) and blue (am-5852_blue). There are 40 POLLEN, 8 red NECTAR, and 8 blue NECTAR total in a BIOBUZZ MATCH. POLLEN and NECTAR are not perfectly spherical and may vary in size." — [Competition Manual TU05](https://ftc-resources.firstinspires.org/ftc/game/manual)

**Staging (§10.3.1)**
- 40 POLLEN: 4 in each of the 4 FLOWERS, 4 in the red GARDEN, 4 in the blue GARDEN, and 4 pre-loaded in each ROBOT.
- NECTAR: 3 in each upward-facing CELL of matching color, and 5 in each ALLIANCE AREA.

  Source: [Competition Manual TU05](https://ftc-resources.firstinspires.org/ftc/game/manual)
- Robots must start "contacting 4 pre-loaded POLLEN" (§10.3.4; G304.G, revised in TU03 on Oct 1, 2026) — [Competition Manual TU05](https://ftc-resources.firstinspires.org/ftc/game/manual); [Combined Team Updates](https://ftc-resources.firstinspires.org/ftc/game/tu-combined)

**Point values (Table 10-2)**

| Achievement | AUTO | TELEOP |
|---|---|---|
| LEAVE | 3 | — |
| PARK | 5 | 5 |
| HIVE TIP | 20 | 20 |
| POLLEN and/or NECTAR remaining in CELL | — | 2 |
| Bottom NECTAR Bonus | — | 5 |
| POLLEN and/or NECTAR in an owned FLOWER | — | 2 |
| POLLEN and/or NECTAR in GARDEN | — | 1 |

Fouls: MINOR = 5 points and MAJOR = 20 points credited to the opponent (Glossary).

Source: [Competition Manual TU05](https://ftc-resources.firstinspires.org/ftc/game/manual)

**Ranking points (Table 10-3)** at regular events (Regional Championship and Championship thresholds are TBA; Premier Events set their own):
- SWARM RP: 16 combined LEAVE + PARK points
- POLLINATOR 1 RP: 4 TIPS
- POLLINATOR 2 RP: 7 TIPS
- WIN: 3 RP; TIE: 1 RP

Source: [Competition Manual TU05](https://ftc-resources.firstinspires.org/ftc/game/manual)

**Scoring rules**
- §10.5:
  - "Assessment of HIVE TIPS occurs throughout the MATCH and continues until all SCORING ELEMENTS and ROBOTS have come to rest"
  - "HIVE TIPS that are complete prior to the start of TELEOP are assessed as part of AUTO"
  - Elements remaining in the CELL are assessed after everything comes to rest at match end

  Source: [Competition Manual TU05](https://ftc-resources.firstinspires.org/ftc/game/manual)
- §10.5.1: "LAUNCHING into the upward-facing CELL is the only allowed way to earn a HIVE TIP." At match end, "any POLLEN and/or NECTAR left in an upward-facing CELL will earn points for that ALLIANCE." — [Competition Manual TU05](https://ftc-resources.firstinspires.org/ftc/game/manual)
- G426: each time an alliance's HIVE is TIPPED, one more of its 5 staged NECTAR may be entered via its LOADING ZONE. With 60 s or less left, all remaining NECTAR may be entered — [Competition Manual TU05](https://ftc-resources.firstinspires.org/ftc/game/manual)
- G408: "A ROBOT may not CONTROL the opponent's NECTAR." TU05 added that repeated control is likely STRATEGIC even with auto-discard sorting — [Competition Manual TU05](https://ftc-resources.firstinspires.org/ftc/game/manual); [Combined Team Updates](https://ftc-resources.firstinspires.org/ftc/game/tu-combined)
- Game Preview (May 3, 2026): POLLEN have "similar characteristics to the Artifacts used in the 2025-2026" DECODE game — [FTC Community](https://community.firstinspires.org/game-preview-field-elements)

### Inferences
- **What to launch.** POLLEN and own-alliance NECTAR, into the own-alliance HIVE's upward-facing CELL. Opponent NECTAR cannot be legally controlled (G408), so it can never be launched.
- **NECTAR vs POLLEN trade-off.** NECTAR is larger (3.6 in vs 2.8 in, so less clearance in the opening) and about 1.33–1.67× heavier (from the calibration inequalities in Section 2). It counts more toward a tip per ball, but needs different launcher settings.
- **Model two projectile types.** A shoot-on-the-move model should handle two diameters and two masses, plus non-sphericity and wear.
- **Value of a TIP.** Pieces in a cell that tips spill out; only pieces still in an *upward-facing* cell at the end score 2 each. A TIP (20 points, and it feeds the RPs) is worth far more than resting pieces.

### Gaps
- **Ball mass is not stated** in the manual, the setup guide, or the Game Preview. Only the ratio bounds above can be derived. AndyMark product pages (am-5851, am-5852) did not return specs through the fetch tool. Weigh real balls; I found no source for coefficient of restitution or surface hardness either.

## 4. Launch zones and restrictions on where and when a robot may launch

### Takeaway
**The TU05 manual has no launch zone and no "no-launch" area.** A robot may LAUNCH from anywhere on the field, including while moving; no rule addresses moving while launching. The constraints that matter are listed below.

| Rule | Constraint |
|---|---|
| G417 | Only launch into the upward-facing CELL; no hitting the HIVE or impeding the opponent's TIP |
| G407 | Control at most 4 SCORING ELEMENTS |
| G408 | Never control opponent NECTAR |
| G402 | In AUTO, stay on your own half (columns A–C red, D–F blue) |
| G403 | No powered movement during the 8-s AUTO→TELEOP transition |
| G404 | No powered movement after TELEOP ends |
| G405 | Don't deliberately eject elements from the field |
| G409 | Don't catch elements falling from a TIPPED HIVE |
| G412 | Don't endanger people outside the field |
| R105 / G416 | Robot envelope is 18 × 24 × 29 in |
| T405 | No launching during field calibration time |

### Cited Findings
**Definitions**
- Glossary, LAUNCH/LAUNCHING: "an action by a ROBOT in which the SCORING ELEMENT is shot into the air, propelled across the floor to a desired location or in a preferred direction, or thrown in a forceful way" — [Competition Manual TU05 §16](https://ftc-resources.firstinspires.org/ftc/game/manual)
- Glossary, CONTROL: the element "is fully supported by or stuck in, on, or under the ROBOT or it intentionally pushes a SCORING ELEMENT to a desired location..." Per G407.F, "SCORING ELEMENTS that have been LAUNCHED by a ROBOT that are no longer in contact with the ROBOT" are not controlled — [Competition Manual TU05](https://ftc-resources.firstinspires.org/ftc/game/manual)
- Zones defined in §9.3: only ALLIANCE AREA, LOADING ZONE (23 × 11 in), and GARDEN (23 × 2 in). None is a launch zone. A full-text search of the TU05 manual for "LAUNCH" found no positional launch restriction — [Competition Manual TU05](https://ftc-resources.firstinspires.org/ftc/game/manual)

**G417 — ROBOTS may not meddle with the HIVE Structure**
- "ROBOTS may not manipulate the motion of the HIVE in any way other than by LAUNCHING SCORING ELEMENTS into an upward-facing CELL." Violation: VERBAL WARNING, or MAJOR FOUL + YELLOW CARD per MATCH if STRATEGIC.
- Likely STRATEGIC: (A/B) ramming the frame; (C) "deliberately LAUNCHING NECTAR or POLLEN into the external bottom, sides, or top faces of a CELL"; (D) "impeding the TIP of an opponent's HIVE by LAUNCHING NECTAR or POLLEN at it"; (E) "contacting a HIVE directly or transitively through a CONTROLLED SCORING ELEMENT"; (F) repeats after a warning.
- Not STRATEGIC: (H) "A ROBOT attempting to LAUNCH POLLEN or NECTAR into the upward-facing CELL and missing in a way that hits the bottom, sides, or top of the CELL."

  Source: [Competition Manual TU05 §11.4.4](https://ftc-resources.firstinspires.org/ftc/game/manual)

**G407 — No more than 4 at a time**
- "A ROBOT may not simultaneously CONTROL more than 4 SCORING ELEMENTS." Violation: VERBAL WARNING; MAJOR FOUL + YELLOW CARD if STRATEGIC. The rule gives examples: 6+ controlled is likely STRATEGIC; momentary 5 that is quickly reversed is not.
- TU05 (Oct 15, 2026) added: "REPEATED instances of a ROBOT CONTROLLING 5 or more SCORING ELEMENTS are likely to be perceived as STRATEGIC, even if the ROBOT has a 'keep 4 and discard the rest' sorting mechanism."

  Source: [Competition Manual TU05](https://ftc-resources.firstinspires.org/ftc/game/manual); [Combined Team Updates](https://ftc-resources.firstinspires.org/ftc/game/tu-combined)

**G402 — No AUTO opponent interference**
- "During AUTO, FIELD columns A, B, C constitute the red side of the FIELD, and columns D, E, F constitute the blue side... Navigating into the opposing ALLIANCE'S side of the FIELD during AUTO is a risky gameplay strategy that may be seen as STRATEGIC." Violation: MAJOR FOUL per MATCH; adds a YELLOW CARD if STRATEGIC — [Competition Manual TU05 §11.4.1](https://ftc-resources.firstinspires.org/ftc/game/manual)

**G403 / G404 — Motionless periods**
- G403: "Any powered movement of the ROBOT or any of its MECHANISMS is not allowed during the transition period between AUTO and TELEOP" (8 s).
- G404: no powered movement after the end of TELEOP.
- §10.5: "Achievements scored before the MATCH starts, during the AUTO-to-TELEOP transition period, and after the MATCH ends at 0:00 may be subject to penalties."

  Source: [Competition Manual TU05](https://ftc-resources.firstinspires.org/ftc/game/manual)

**G405 — Keep SCORING ELEMENTS in bounds**
- "A ROBOT may not deliberately eject a SCORING ELEMENT from the FIELD (either directly or by bouncing it off a FIELD element or another ROBOT)." Violation: MAJOR FOUL per element.
- "SCORING ELEMENTS that leave the FIELD during scoring attempts... are not considered deliberate ejections."

  Source: [Competition Manual TU05](https://ftc-resources.firstinspires.org/ftc/game/manual)

**G409 — Don't catch SCORING ELEMENTS**
- A ROBOT may not catch or deflect an element "released by a TIPPED HIVE unless and until that SCORING ELEMENT contacts anything else besides that ROBOT."
- STRATEGIC examples: repeatedly waiting under the HIVE, and wide-open top catchers.

  Source: [Competition Manual TU05](https://ftc-resources.firstinspires.org/ftc/game/manual)

**G412 — ROBOTS must be under control**
- Violated if "the ROBOT or anything it CONTROLS, i.e., a SCORING ELEMENT, disrupts anything outside the FIELD or contacts a human that is outside the FIELD, or... the ROBOT operation is dangerous." Violation: DISABLED + VERBAL WARNING — [Competition Manual TU05](https://ftc-resources.firstinspires.org/ftc/game/manual)

**R105 / G416 — Size limits**
- In STARTING CONFIGURATION the robot fits an 18-inch cube (R102).
- After the start, the robot must "remain within a 18 in. (45.70 cm) by 24 in. (61.0 cm) by 29 in. (73.65 cm) tall sizing volume," enforced mechanically, not by software. The 29 in is "always the vertical height above the FIELD surface."
- G416 enforces this during matches: VERBAL WARNING; MAJOR FOUL per instance if STRATEGIC.

  Source: [Competition Manual TU05 §12.1](https://ftc-resources.firstinspires.org/ftc/game/manual)

**Other robot rules relevant to launchers**
- Per TU00 (Sep 12, 2026):
  - R503: at most 8 motors and 8 servos (servo maximum reduced to 8)
  - R501: adds the WATTOS Stingray motor
  - R801: "No Pneumatic Actuators, High-Speed Blowers, or Vacuums"

  Source: [Combined Team Updates](https://ftc-resources.firstinspires.org/ftc/game/tu-combined)

**T405 — Field calibration time**
- During field measurement/calibration time, robots may run OpModes but not drive. The manual lists "J. ROBOT may not LAUNCH SCORING ELEMENTS" — [Competition Manual TU05 §13.4](https://ftc-resources.firstinspires.org/ftc/game/manual)

**Match timing and audio cues (§10.4, Table 9-1)**
- AUTO: 30 s
- Transition: 8 s
- TELEOP: 2:00
- 1:00: "FLOWER Ownership Unlocked" (NECTAR into FLOWERS allowed only in the last 60 s, G410)
- 0:20: "Train Whistle" cue for the final 20 seconds

Source: [Competition Manual TU05](https://ftc-resources.firstinspires.org/ftc/game/manual)

### Inferences
- **Launching while moving is legal.** No rule restricts robot motion while launching. The shooter must still respect the G403/G404 motionless windows and must not fire into the 8-s transition. AUTO launches must be complete (TIP achieved) before TELEOP starts to count as AUTO TIPs.
- **The aim point must always be own alliance's up cell.** Firing at the opponent HIVE, or deliberately at a down cell or an outer face, risks G417.C/D. A software guard should inhibit firing when the predicted impact point is not inside one's own up-cell opening.
- **The launcher must lift balls about 25–37 in.** The exit point is at most about 29 in above the tiles (R105). The opening spans 53.5–65.6 in, center about 58.4 in. Shot speed must cover that rise plus the horizontal range.
- **Feeding logic.** G407 caps a robot at 4 held pieces. A first TIP needs about 3 POLLEN; later TIPs need about 8. A shoot-on-the-move cycle is therefore "collect ≤4 → fire ≤4", needing 2+ cycles per TIP after the first.
- **Hold fire during a tip.** The §10.5.1 note implies the shooter should briefly stop firing while a HIVE is mid-tip, so the TIP is unambiguous to referees.

### Gaps
- The FTC Q&A system (https://ftc.game/qa → https://game-qa.firstinspires.org/boards/2027/FTC/QA) needs login/JavaScript; the fetch returned no posts. **No official Q&A rulings on launching were retrieved.** Search it directly for "LAUNCH", "G417", "CELL" before relying on edge cases, such as launching from under the HIVE structure or from the opponent half in TELEOP.

## 5. Field coordinate system, AprilTags near the HIVES, and red/blue symmetry

### Takeaway
The official FTC field coordinate system:
- **Origin**: field center, on the tile top surface.
- **+Y**: away from the Red Wall (toward blue).
- **+X**: to the right as seen from the Red Wall.
- **+Z**: up.
- **Rotation**: counter-clockwise positive.
- **Units**: any, consistent across axes.

BIOBUZZ puts red on the audience's left, which makes it the "Square field, inverted" case (as in DECODE): **+X points toward the audience**. This is my application of the general rule; ftc-docs has no BIOBUZZ-specific diagram yet.

**AprilTags**: the only tags are four 4-tag **clusters (36h11, 3.25 in)**, one under each CELL:
- IDs 30–33: red, rear ("Scoring") cell
- IDs 34–37: red, audience cell
- IDs 38–41: blue, audience cell
- IDs 42–45: blue, rear ("Scoring") cell

In SDK 12.0 **each cluster's origin is the center of the CELL opening**, which makes the cluster pose a direct aiming target. The tags move with the HIVE, so the SDK gives them no field pose and FIRST says they are **not suitable for absolute localization**.

**Symmetry**: the layout has **180° rotational (point) symmetry, not mirror symmetry**.

### Cited Findings
**Coordinate system**
- ftc-docs, Field Coordinate System:
  - "The 0,0,0 origin of the FIRST Tech Challenge coordinate system is the point in the center of the field" (on the top surface of the floor mat)
  - Viewed from the center of the Red Wall: "The X axis values increase to the right"; "Increasing Y values run out (away) from the Red Wall"; "Increasing Z values extend upwards"
  - Positive rotations are counter-clockwise, looking down an axis from its positive end
  - Any unit "as long as the same measure is used for all three axes"
  - "Square field, inverted": Red Wall on the audience's left, X pointing toward the audience (DECODE example)
  - The page does not mention BIOBUZZ

  Source: [ftc-docs Field Coordinate System](https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html)
- §9.5: "The FIELD is oriented such that the red ALLIANCE AREA is located on the left from the primary audience viewing direction." — [Competition Manual TU05](https://ftc-resources.firstinspires.org/ftc/game/manual)
- §9.2: the field is about 144 × 144 in (365.75 × 365.75 cm) inside the walls, made of 36 tiles of about 24 × 24 × 0.59 in — [Competition Manual TU05](https://ftc-resources.firstinspires.org/ftc/game/manual)

**Tile grid and zone locations**
- Tile grid (§9.4, Figs 9-4/9-5): columns A–F run left (red) to right (blue) as seen from the audience; rows 1–6 run from the audience wall (row 1) to the rear wall (row 6). Tile-seam lines are V–Z and 1–5 — [Competition Manual TU05](https://ftc-resources.firstinspires.org/ftc/game/manual)
- Setup guide §8.3–8.4:
  - Red LOADING ZONE on tile **A5**; blue LOADING ZONE on tile **F2**
  - Red GARDEN on tile **A1**; blue GARDEN on tile **F6**

  Source: [Event Field Setup Guide V1.0](https://ftc-resources.firstinspires.org/ftc/archive/2027/field/eventfieldguide)

**AprilTags (§9.9)**
- "AprilTags for BIOBUZZ are 3.25 in. (8.25 cm) square targets from the 36h11 tag family... arranged in AprilTag Clusters, consisting of four AprilTags laid out on a single sticker." — [Competition Manual TU05](https://ftc-resources.firstinspires.org/ftc/game/manual)
- "Each AprilTag Cluster is placed on the bottom of a CELL facing downward towards the TILES with its bottom edge oriented towards the center of the FIELD (Figure 9-16)." — [Competition Manual TU05](https://ftc-resources.firstinspires.org/ftc/game/manual)
- Tag IDs (§9.9): 30, 31, 32, 33 on the red CELL opposite the audience; 34–37 on the red CELL on the audience side; 38–41 on the blue CELL on the audience side; 42–45 on the blue CELL opposite the audience. Fig 9-17 labels these "Red Scoring Tags", "Red Audience Tags", "Blue Audience Tags", and "Blue Scoring Tags". In top view it orders them 33-32-31-30 and 45-44-43-42 left to right for the rear cells, and 34-35-36-37 and 38-39-40-41 for the audience cells. (The PDF text layer drops some digits; the IDs were read from the rendered page.) — [Competition Manual TU05](https://ftc-resources.firstinspires.org/ftc/game/manual)
- Fig 9-15 cluster layout, read from the figure:
  - Tag centers are 2.75 in (7.0 cm) and 6.5 in (16.50 cm) either side of the cluster centerline
  - The cluster centerline is 7.0 in (17.80 cm) from each 0.5 in reference hole
  - Tag centers sit 2.75 in above the reference-hole line
  - The reference-hole line is 9.938 in (25.25 cm) from the "Front of Cell"

  Source: [Competition Manual TU05](https://ftc-resources.firstinspires.org/ftc/game/manual)

**SDK 12.0**
- `AprilTagGameDatabase.getBioBuzzTagLibrary()` defines four clusters with first IDs 30 ("RED SCORING"), 34 ("RED AUDIENCE"), 38 ("BLUE AUDIENCE"), and 42 ("BLUE SCORING"). Each has `fieldPosition = new VectorF(0,0,0)` and `Quaternion.identityQuaternion()`, so **no field pose is given**.
- Member tags (size 3.25 in) are at cluster-frame positions (−6.50, 7.1874, −5.622), (−2.75, 7.1874, −5.622), (+2.75, 7.1874, −5.622), (+6.50, 7.1874, −5.622) inches, for IDs first, first+1, first+2, first+3.
- `getCurrentGameTagLibrary()` now includes the BioBuzz library.

  Source: FTC SDK 12.0 Vision-12.0.0-sources.jar, `org/firstinspires/ftc/vision/apriltag/AprilTagGameDatabase.java` (local: ~/.gradle/caches/modules-2/files-2.1/org.firstinspires.ftc/Vision/12.0.0/…/Vision-12.0.0-sources.jar); [FtcRobotController GitHub](https://github.com/FIRST-Tech-Challenge/FtcRobotController)
- SDK 12.0 README (Version 12.0, 20260907-090034):
  - "In BIOBUZZ, the Origin of each cluster is located in the center of the Cell opening for easy aiming."
  - "Unfortunately, since BIOBUZZ AprilTags move, they are not suitable for absolute Field Localization."
  - "Clusters are resilient to partial occlusion. Full 6DOF pose can be estimated from a cluster even if only a single member tag is visible."
  - Breaking change: detections must be checked for `AprilTagSingleDetection` vs `AprilTagClusterDetection`.

  Source: [FtcRobotController README](https://github.com/FIRST-Tech-Challenge/FtcRobotController) (local copy: /home/andrei0204/Coding/Robo/FtcRobotController-12.0/README.md)
- ftc-docs "AprilTag Clusters" tech tip: the cluster "references a point that roughly correlates to the center of the opening of the CELL", "as if there was an AprilTag sitting in the middle of the opening." "Scorable CELLS will have AprilTag Clusters that are mostly right-side up," judged by |roll| < 90°. Its example: RED SCORING roll 159.3° (upside down, not scorable) and RED AUDIENCE roll −19.7° (scorable), "therefore... the robot is on the AUDIENCE half of the field, pointed towards the RED AUDIENCE CELL." The page does not define cluster X/Y/Z axis conventions — [ftc-docs AprilTag Clusters](https://ftc-docs.firstinspires.org/en/latest/tech_tips/tech-tips/tech-tip-apriltag-clusters/tech-tip-apriltag-clusters.html) (short link https://ftc-docs.firstinspires.org/apriltag-clusters)

**Symmetry evidence**
- Initial HIVE states are point-symmetric: red audience cell up, blue scoring cell up. LOADING ZONES are at A5 and F2; GARDENS at A1 and F6. In Fig 9-17 the FLOWERS sit on all four walls in a point-symmetric pattern — [Event Field Setup Guide V1.0](https://ftc-resources.firstinspires.org/ftc/archive/2027/field/eventfieldguide); [Competition Manual TU05 Figs 9-2, 9-17](https://ftc-resources.firstinspires.org/ftc/game/manual)

### Inferences
**Axes in BIOBUZZ terms.** Applying the general ftc-docs rule to BIOBUZZ (red on the audience's left):
- **+X points toward the audience**
- **+Y points from the red wall toward the blue wall**
- **+Z points up**
- The origin is field center at tile-top height

Confirm against the ftc-docs page once a BIOBUZZ diagram is posted, and against whatever convention your localizer (Pinpoint, Road Runner, etc.) uses.

**Target coordinates in that frame** (derived; ±1 in; see Section 2):
- Red HIVE: cells at Y ≈ −12.75 in. Its up-cell opening center is at **(X ≈ +16.5…+18.1, Y ≈ −12.75, Z ≈ 58.4) in** when the audience cell is up (match start). It moves to **(X ≈ −16.5…−18.1, Y ≈ −12.75, Z ≈ 58.4)** after an odd number of TIPS.
- Blue HIVE: cells at Y ≈ +12.75 in. Its up-cell opening center is at **(X ≈ −16.5…−18.1, Y ≈ +12.75, Z ≈ 58.4)** at match start (scoring cell up). It flips to X ≈ +16.5…+18.1 after a TIP.
- In mm: X ≈ ±419–460, Y ≈ ±324, Z ≈ 1483.
- The opening's outward normal is (±cos 30°, 0, sin 30°), pointing toward the side of the field it faces.

**Symmetry.** The layout is **180° point-symmetric** about the field center. To convert a red-alliance plan to blue, map (x, y, θ) → (−x, −y, θ+180°); do not mirror it. Only the HIVE state is dynamic.

**Using the tags for aiming.**
- The clusters are under the cells, facing down and toward the side the HIVE's up cell faces, about 30° from vertical. Both clusters of a HIVE tilt the same way.
- A camera must look upward (roughly 30–60° elevation, depending on distance) to see them.
- Because the cluster origin is the opening center, a cluster detection gives the target vector directly in the camera frame, robust to occlusion. Use it to correct odometry-based aiming, not as a global localizer.
- |roll| < 90° identifies which cell is scorable from your viewpoint. That gives a direct sensor for HIVE state, which shoot-on-the-move needs to pick the correct (moving) target.
- There are **no static field AprilTags** this season, so absolute localization must come from odometry/IMU or other sensors.

**Cluster frame (my reading, not official).** The SDK offsets put the tags about 7.19 in back from the opening plane (matching Fig 9-15's 9.938 − 2.75 in) and 5.62 in from the opening center perpendicular to the bottom face (about the pentagon centroid height). This suggests cluster-frame Y is along the cell depth and Z is normal to the tag sticker. Verify experimentally with the SDK's axis overlay, which SDK 12.0 shows on the preview by default.

### Gaps
- ftc-docs had no BIOBUZZ-specific field-coordinate diagram at the time of research. The +X-toward-audience conclusion is an application of the general rule and needs confirmation.
- No official field-frame coordinates for the cell openings or tag clusters exist; the SDK uses (0,0,0). The cluster-frame axis convention is not documented on the pages fetched.
- FLOWER XY positions were only read approximately from Fig 9-17: tile seams at about B/C on the rear wall, D/E on the audience wall, between rows 4/5 on the blue wall, and between rows 2/3 on the red wall. Use the CAD for exact values.

## 6. Early-season community details relevant to shooting into the HIVE

### Takeaway
Early-season community material (as of 2026-10-21) is thin and mostly speculative. Vendor starter bots use flywheel shooters that fire up to 4 POLLEN quickly. A YouTube short promotes a backspin shot for "pollinating the hive". One team is still planning to measure real HIVE spill and bounce behavior. I found **no measured bounce-out data and no Q&A rulings** on launching.

### Cited Findings
- goBILDA's StarterBot (search-result summary; page not opened): uses a shooter mechanism that can launch up to four POLLEN into the HIVE in quick succession — [goBILDA StarterBot Resource Guide 2026-2027](https://www.gobilda.com/ftc-starter-bot-resource-guide-2026-2027-season/) (unofficial; vendor)
- YouTube short "Pollinate the Hive With This Backspin Technique! #ftc #robotics": promotes backspin over a lob for POLLEN launches. Only the description was seen, not the video — [YouTube](https://www.youtube.com/watch?v=SCXx9jgLQh0) (community opinion)
- Mona Shores FTC (GitHub issue #147, community):
  - Their simulation's spill behavior is the part they "trust least" and needs real-field validation
  - They plan to film tips with loads of 3 NECTAR + 3 POLLEN ("the first TIP of every match"), 8 POLLEN, and 3 NECTAR + 5 POLLEN
  - They note that in simulation the highest-scoring robots are those that collect a tipped cell's spill
  - No real measurements are posted

  Source: [Mona Shores biobuzz issue #147](https://github.com/Mona-Shores-FTC-Robotics/biobuzz/issues/147); related [issue #146](https://github.com/Mona-Shores-FTC-Robotics/biobuzz/issues/146) (AdvantageScope sim with tipping HIVE)
- Studica has a BIOBUZZ strategy and starter-bot page, but it returned HTTP 403 to the fetch tool — [Studica: FTC BIOBUZZ Strategy](https://www.studica.co/ftc-biobuzz-strategy-starter-bot)
- Official caution on tipping: §10.5.1 says launching at the downward-facing cell during a tip "may disrupt its movement," so teams "may need to pause their LAUNCHING" — [Competition Manual TU05](https://ftc-resources.firstinspires.org/ftc/game/manual)
- The setup guide distinguishes "Gently Placed" from "Tossed In" for calibration. A tossed-in 3rd POLLEN (on 3 NECTAR + 2 POLLEN) or 8th POLLEN (on 7 POLLEN) is *required* to tip the HIVE, while a gently placed one is only "Preferred" to tip — [Event Field Setup Guide V1.0](https://ftc-resources.firstinspires.org/ftc/archive/2027/field/eventfieldguide)

### Inferences
- **Impact helps tipping.** Calibration treats "tossed-in" (dynamic) balls as more reliable tippers than gently placed ones, so launched balls with some momentum should tip at the calibrated counts. Event-to-event calibration variation means the exact count can differ by about ±1 ball; FIRST says to expect "some amount of variation".
- **Bounce-out risk.** The cell is only 12 in deep and its floor slopes back toward the pivot at 30° when up. Balls entering fast and flat may rebound off the back skin. Descending entry (about 30° below horizontal, matching the opening normal) and backspin/softer shots are plausible mitigations. This is untested and unconfirmed.
- **Spill dynamics affect pathing.** Each TIP dumps the cell's contents under the HIVE on the side of the newly-down cell. G409 forbids catching them before they touch something else. A shoot-on-the-move path near the HIVE should allow for balls rolling across the robot's path right after a TIP.

### Gaps
- No Reddit r/FTC, Game Manual 0, or Chief Delphi-style threads about BIOBUZZ HIVE shooting turned up in searches ("BIOBUZZ FTC hive shooter launch reddit" returned unrelated results). gm0 BIOBUZZ-specific content was not found.
- No official or community measurement of ball coefficient of restitution, bounce-out rate, or tip transient time was found.
- FTC Q&A content could not be read (login/JS-gated). The animation (https://youtu.be/sUH3z5a5S9I) and field tour (https://youtu.be/47X9sYnPijw) were not viewed, so visual details from them (opening lips, how balls settle) are not captured.
- Searches performed: "FTC 2026-2027 game reveal"; "FIRST Tech Challenge BioBuzz hive"; "BIOBUZZ competition manual hive pollen nectar dimensions"; "ftc-docs field coordinate system BIOBUZZ 2026"; "BIOBUZZ FTC hive shooter launch reddit"; "AndyMark am-5851 pollen ball weight"; "BIOBUZZ FTC launcher hive cell pollen bounce" (extended).
