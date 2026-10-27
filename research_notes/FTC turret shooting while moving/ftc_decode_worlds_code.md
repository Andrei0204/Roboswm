# FTC DECODE (2025–2026) World Championship robots: open-source shoot-on-the-move / turret / flywheel / hood code

Research date: 2026-10-21. Method: FTC Scout GraphQL API (`https://api.ftcscout.org/graphql`) for results, the GitHub REST API (unauthenticated repo/user search) to find team repos, shallow clones and source reads of each repo, and the official FIRST manuals. Code-level claims below come from reading the cited files directly. Line numbers refer to the HEAD commit cloned on 2026-10-21. Labels used: **Verified** means read in source or API data. **Inference** means my interpretation.

Shared URL prefixes used below:
- META = `https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`
- REV = `https://github.com/junkjunk123/RevAmped-Decode-V2/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`
- CUBE = `https://github.com/kleongf/FTC_Decode/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`
- OC = `https://github.com/Xtendera/OverclockDecode/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/overclock/`
- PAN = `https://github.com/DennNuke/ftcpanheya-worlds/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`
- LOAD = `https://github.com/LOAD-Robotics-Team-25609/Decode-Robot-Code/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`

---

## Q1. Who is Meta^Infinity, where is their code, and what exactly did their shooting / shoot-on-the-move code do?

### Takeaway
Meta^Infinity is FTC team **18221** (Highland Park, IL, USA, rookie year 2020). At DECODE Worlds 2026 they were the first pick on the **Franklin Division winning alliance** and won **3rd place for the Inspire Award in the Franklin Division**. Their code is public at **https://github.com/Meta-Infinity/DecodePublicRelease**, which uses Pedro Pathing 2.1.0-alpha.2 and a goBILDA Pinpoint. Shoot-on-the-move is a simple "virtual goal": goal − field velocity × time of flight. Time of flight, flywheel RPM, hood position and feeder power all come from 1-D distance LUTs. A 3-CR-servo turret uses an analog absolute encoder with software unwrap, a 550° wire-limited range, gain-scheduled dual PID, a slew limit and a small yaw-rate lead. The flywheel runs PIDF while idle and switches to bang-bang while feeding.

### Cited Findings

**Team identity and results (Verified via FTC Scout API)**
- Team 18221 is "Meta^Infinity", schoolName "Family/Community", Highland Park, IL, USA, rookie year 2020, website metainfinity.org — [FTC Scout API / team page](https://ftcscout.org/teams/18221)
- **FIRST World Championship, Franklin Division (event FTCCMP1FRAN, starting 2026-04-29):** rank 5, record 8-2-0, non-penalty OPR 165.3 (5th highest in the division). FTC Scout lists awards "Inspire, placement 3" and "Winner, placement 2". The Franklin "Winner" entries are 1 = 30110 Coyote, 2 = 18221 Meta^Infinity, 3 = 10949 MOB Robotics, so Meta^Infinity was the second team (first pick) on the division-winning alliance — [FTC Scout Franklin Division](https://ftcscout.org/events/2025/FTCCMP1FRAN)
- In the Worlds **Finals Division (FTCCMP1)** they have no award. The World Champion alliance was 30030 Exodus, 21087 Velocity and 11228 OverClucked Bots (the Lovelace Division winners). The Finalist alliance was 18270 RoboPlayers, 20265 Heart of RoBots and 7172 Technical Difficulties (Goodall). The overall Inspire winner was 17792 Amigos Droids — [FTC Scout Finals](https://ftcscout.org/events/2025/FTCCMP1)
- Their in-season path: North Shore league meets ranked 1/2/1 (OPR 106–118). Suburban Northeast League Tournament rank 1 (OPR 143.7). Illinois Championship Cooper Division rank 1 (6-0-1), plus "DivisionWinner", "Finalist" and the **Reach Award (1st)** at the state level. Off-season: Michiana Premier (2026-06-18), Croft Division rank 3, OPR 219.5, winning alliance (placement 2), Connect 3rd. Chicago Robotics Invitational Premier (2026-07-24), no stats. **Bucharest Twin Cup, Romania (2026-07-31)**, rank 18, OPR 190.6 — [FTC Scout team 18221](https://ftcscout.org/teams/18221)
- Team website https://metainfinity.org/ links GitHub org `github.com/Meta-Infinity/`, YouTube `@metainfinity18221` and Instagram `metainfinity18221` — [metainfinity.org](https://metainfinity.org/)
- GitHub org **Meta-Infinity** (created 2024-09-03) has 3 public repos. **DecodePublicRelease** was created and pushed 2026-10-06 (6 stars, BSD-3-Clause-Clear, a single squashed commit). The other two are a fork named "Base" and the website repo. An older user **MetaInfinityFTC** only holds 2023-24 code (MetaCode2023-24) — [GitHub Meta-Infinity](https://github.com/Meta-Infinity); [GitHub MetaInfinityFTC](https://github.com/MetaInfinityFTC)
- **Caveat on which version this is:** the public release contains off-season artifacts. Examples: a `RelocalizePosition` comment says "michiana value" next to commented "worlds value" and "practice value"; a "CRI ONLY" block in the LUT; an `auto/prebtc/` folder, likely "pre-Bucharest Twin Cup". So it is the end-of-off-season state, not necessarily the exact Worlds build — [META util/RelocalizePosition.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/RelocalizePosition.java); [META controlSystems/LookUpTable.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/controlSystems/LookUpTable.java)

**Architecture / data flow (Verified in source)**
- Loop order in teleop `RobotTeleop.update()`: `r.clearCache()`, then `f.update()` (Pedro Follower), then `r.update(f.getPose(), f.getVelocity())`. Pose and turret angle are saved to static `Storage` every loop, which carries pose and turret angle from auto into teleop. Autos use the same loop — [META robot/RobotTeleop.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/RobotTeleop.java); [META auto/RED24CLOSE.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/RED24CLOSE.java)
- **Pedro methods used:** `Follower.getPose()`, `Follower.getVelocity()` (a field-frame `Vector` in in/s), `f.setStartingPose()`, `f.setPose()` for relocalization, and `f.isBusy()` in autos. Yaw rate is **not** read from the follower. It comes directly from the Pinpoint driver: `pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)` in `Robot.update()` — [META robot/Robot.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/Robot.java)
- **Target point:** a fixed field point per alliance in Pedro coordinates (inches, 0–144). RED_GOAL = (134, 138) and BLUE_GOAL = (8, 137), deep in the goal corner rather than at the AprilTag. A driver toggle (`switchGoal()`, share button) swaps to an alternate "MIDDLE" aim point: RED (62, 126), BLUE (80, 126) — [META util/RobotConstants.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/RobotConstants.java); [META util/FieldConstants.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/FieldConstants.java)
- **Turret-center offset:** `Robot.update()` rotates a turret lever-arm offset (TURRET_X_OFFSET = 0, TURRET_Y_OFFSET = −0.197 in) by heading to get the turret's field position, and computes all distances from that point — [META robot/Robot.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/Robot.java)

**Shoot-on-the-move: virtual goal (Verified in `Robot.update(Pose, Vector)`)**
- Steps, paraphrased:
  1. `staticDistance` = |goal − turretFieldPos|.
  2. If SOTM is on: `flightTime = storage.getTime(staticDistance)` and `trackPose = goal − velocity × flightTime`, component-wise with `velocity.getXComponent()` / `getYComponent()`. Otherwise `trackPose = goal`.
  3. dx, dy = trackPose − turret position, and `distanceToGoal = hypot(dx, dy)`.
  4. Shooter and turret are updated from the virtual-goal distance and vector.
  — [META robot/Robot.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/Robot.java)
- **Single iteration.** Time of flight is looked up from the *static* distance only. There is no fixed-point iteration over the virtual-goal distance and no acceleration term — [META robot/Robot.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/Robot.java)
- The intended distance gate is a no-op: `if(sotm && (staticDistance > 30.0 || staticDistance < 110))` is always true. The team's own inline comment admits it ("the second half is true every time") — [META robot/Robot.java line ~208](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/Robot.java)
- SOTM is teleop-only and driver-armed: dpad_up calls `r.turret.startTrack()` and `r.sotmOn()`, dpad_down turns them off. The autos call `turret.startTrack()` and `shooter.startTrack()` but never `sotmOn()`, and they fire only once `!f.isBusy()` (robot stopped at the end of the path) — [META robot/RobotTeleop.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/RobotTeleop.java); [META auto/RED24CLOSE.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/RED24CLOSE.java)

**Lookup tables (Verified in `LookUpTable`, built on FTCLib `InterpLUT`, a monotone cubic spline)**
- Four tables are keyed by distance in inches: `rpm`, `hood` (servo position), `time` (time of flight in s) and `transferPower` (feeder power). Clamp rows sit at −1000 and +1000 so out-of-range distances saturate.
- Measured points (distance → RPM / hood / TOF):
  - 42.45 → 2610 / 0.40 / 0.46
  - 52.11 → 2710 / 0.34 / 0.51
  - 62.5 → 2810 / 0.30 / 0.56
  - 72.44 → 2920 / 0.25 / 0.61
  - 85.88 → 3060 / 0.21 / 0.67
  - 107.53 → 3240 / 0.18 / 0.74
  - 124.86 → 3500 / 0.17 / 0.82
  - 145.5 → 3800 / 0.17 / 0.92
  - 155.2 → 3900 / 0.17 / 0.97
- Transfer (feeder) power is 1.0 up to about 107 in and drops to 0.5 from 110 in, so far shots are fed more slowly.
- A commented-out "CRI ONLY" block extends the table to 200 in / 5200 RPM.
- Source: [META controlSystems/LookUpTable.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/controlSystems/LookUpTable.java)
- Tables were filled with a tuning OpMode (`InterpolationTuner`) that shows distance, typed vs. actual RPM and hood, sample count and loop ms — [META teleop/testing/InterpolationTuner.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/testing/InterpolationTuner.java)

**Turret control (Verified in `subsystems/Turret.java`, 362 lines)**
- **Hardware:** three continuous-rotation servos (`servo1..3`, one reversed) drive the turret together. Feedback comes from an analog absolute encoder (`AnalogInput "elcencoder"`). The encoder is wrapped at 3.2 V (`ENCODER_WRAP_VOLTAGE`) to avoid the noisy top of the range. A configurable seam shift (`ENCODER_WRAP_OFFSET_DEG = 180`) moves the unwrap boundary away from common aim angles — [META subsystems/Turret.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Turret.java); [META util/RobotConstants.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/RobotConstants.java)
- **Multi-turn unwrap:** `readEncoderUnwrapped()` adds the per-loop delta, wrapped to ±180°, into `absEncoderDeg`. `seedEncoder()` initializes it assuming the turret is within ±180° of forward. `restoreState(storedAngle)` re-seeds from the angle saved at the end of auto by choosing the rotation count k that lands closest to the stored angle — [META subsystems/Turret.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Turret.java)
- **Aim math in `update(dx, dy, heading, ω, voltage)`:**
  - `leadDeg = deg(ω) × TURRET_ANG_VEL_LEAD_SEC` (0.05 s).
  - `robotHeading = deg(heading) + leadDeg`.
  - `rawTarget = atan2(dy, dx) − robotHeading + TURRET_FORWARD_OFFSET_DEG`.
  - The target is then shifted by a multiple of 360° to the equivalent closest to the current command (shortest path).
  - Source: [META subsystems/Turret.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Turret.java)
- **Note on heading source:** `Turret.update` receives the raw odometry heading `pose.getHeading()`, not the Kalman "fusion" pose. The Kalman filter only corrects x/y — [META robot/Robot.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/Robot.java)
- **Wire limits and wraparound:**
  - Wire range is TURRET_WIRE_MIN_DEG = −455 to TURRET_WIRE_MAX_DEG = +95, about 550° of travel, with a 5° soft margin.
  - `remapTargetIntoSafeRange(desired, reference)` lists every desired + 360k inside [softMin, softMax] and picks the one closest to the current command, then clamps.
  - `getPreferredFrontHomeDeg()` picks the "forward" multiple of 360° with the most clearance to both limits.
  - Sources: [META subsystems/Turret.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Turret.java); [META util/RobotConstants.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/RobotConstants.java)
- **Slew limit:** the command moves at most SLEW_RATE_DEG_PER_SEC = 720 °/s × dt toward the target — [META subsystems/Turret.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Turret.java)
- **Gain-scheduled dual PID:**
  - "Far" gains: P 0.013, D 0.0016, F 0.09. "Close" gains: P 0.0028, D 0.0009, F 0.09.
  - The switch point is 15° of error with ±2° hysteresis.
  - Output is clipped to ±0.5 (`TURRET_MAX_POWER`) and inverted.
  - Sources: [META subsystems/Turret.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Turret.java); [META util/RobotConstants.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/RobotConstants.java)
- **Custom `PIDController`, position mode:**
  - 0.15° deadband: below it, output is 0 and the integral resets.
  - The static feedforward is *ramped*: `ff = sign(e)·kF·min(|e|/5°, 1)`. The code comment says this avoids the chatter a hard kS step causes.
  - Integral is clamped to ±1.
  - Voltage compensation multiplies the whole output by 12/V. The voltage is read once per loop from `LynxModule.getInputVoltage`.
  - Source: [META controlSystems/PIDController.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/controlSystems/PIDController.java)
- **Driver trim:** dpad left/right adds ±2° (`addOffSetDegrees`). Trim resets on relocalization — [META robot/RobotTeleop.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/RobotTeleop.java)

**Flywheel and hood (Verified in `subsystems/Shooter.java`)**
- **Hardware:** two motors (`shooterleft`, `shooterright`) run RUN_WITHOUT_ENCODER with FLOAT zero-power behavior. Velocity is read from the encoder plugged into the **`frontLeft` drive-motor port**. That port is free because the Pinpoint handles localization. RPM = −velocity × 60 / 28 — [META subsystems/Shooter.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Shooter.java)
- **Control law:**
  - When not feeding, a velocity-mode PIDF runs: P = 0.006 and F = 1, with `ff = kF·target/5200`. That is a normalized kV with 5200 RPM as full scale, multiplied by 12/V.
  - **While feeding (`isShooting`) it switches to bang-bang:** power 1.0 if RPM < target − 15 (DEADBAND_RPM), otherwise 0. This speeds recovery between balls.
  - Driver offsets: ±10 RPM per press (`OFFSET_RPM`) and a hood offset (`OFFSET_HOOD`, 0.14 by default).
  - Sources: [META subsystems/Shooter.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Shooter.java); [META util/RobotConstants.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/RobotConstants.java)
- **Hood:** one positional servo (`hood1`). Its position is `hood LUT(distance) + OFFSET_HOOD`, clipped to [0, 1], and it is written only when the change exceeds 0.002 — [META subsystems/Shooter.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Shooter.java)

**Shot gating (Verified)**
- **No software check that RPM, turret angle or hood is ready.** In teleop the driver fires with right bumper. The intake first runs a short "pre-reverse" of the transfer, then switches to feeding (`transferShoot(dynamicPower)`) with the shooter in bang-bang mode. Holding X feeds at a fixed 0.3 power — [META robot/RobotTeleop.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/RobotTeleop.java)
- In auto, firing is gated only by `!f.isBusy() && r.intake.prepareShoot()` and runs for a fixed `shootTime` — [META auto/RED24CLOSE.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/RED24CLOSE.java)
- Break-beam ball counting (EMA filtered) drives the LEDs and rumble — [META util/RobotConstants.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/RobotConstants.java)

**Limelight / AprilTag relocalization (Verified)**
- **Limelight 3A** polled at 60 Hz, with one pipeline per alliance.
  - `getPose()` converts MegaTag1 `getBotpose()` (meters, field-centered) to Pedro inches as `x = y_m·39.37 + 70.5` and `y = −x_m·39.37 + 70.5`. Heading is not used.
  - `getDistanceInches()` uses the classic ty trigonometry: Limelight height 8.39 in, mount angle 17.10°, tag height 29.5 in.
  - Source: [META subsystems/Limelight.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Limelight.java)
- **Custom 2-state Kalman filter** (independent scalar filters on x and y):
  - Predict adds the odometry deltas with Q = 0.003.
  - Correct uses R = 8 × (1 + (d/70)²), capped at 10, and rejects corrections more than 90 in from the estimate.
  - Sources: [META controlSystems/KalmanFilter.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/controlSystems/KalmanFilter.java); [META robot/Robot.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/Robot.java)
- **Fusion runs only while driver 1 holds left trigger** (`kfOn()`). After release, the last fusion offset (x/y) stays applied to the odometry pose. The Limelight is read only while the filter is on, which saves loop time — [META robot/RobotTeleop.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/RobotTeleop.java); [META robot/Robot.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/Robot.java)
- **Hard relocalization:** driver 1 "Y" calls `f.setPose(RED_GATE/BLUE_GATE)`, a known pose when the robot is pressed against the gate. The values were measured per venue; commented "michiana", "worlds" and "practice" variants exist — [META util/RelocalizePosition.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/RelocalizePosition.java)
- **Latency compensation:** none beyond the 0.05 s yaw-rate lead on the turret. Vision is not timestamped or replayed — [META robot/Robot.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/Robot.java)

### Inferences
- Meta's SOTM is the minimum viable "virtual goal" from FRC practice: goal − v·TOF, with TOF from a measured LUT. Its weaknesses:
  - TOF is taken at the static distance (no iteration).
  - No ω×r term for the offset turret, though it is nearly zero here.
  - Raw odometry heading is used.
  - No robot-acceleration term.
  - The flywheel target follows the virtual distance, so moving toward or away from the goal changes RPM automatically.
- They placed well (division winner, 165–220 OPR) with this simple approach, which suggests the precision of the measured LUT matters more than ballistic modelling.
- The turret design (3 CR servos, absolute analog encoder, multi-turn unwrap, nearest-equivalent target inside wire limits) is directly reusable for any season with a rotating launcher.
- Lowering feeder power at long range (1.0 → 0.5 beyond ~110 in) combined with bang-bang during feeding is a cheap way to keep RPM sag between consecutive balls consistent.

### Gaps
- Could not confirm whether SOTM was enabled in their actual Worlds matches. The repo is the post-off-season snapshot (Michiana, CRI and Bucharest values) and has no git history (single commit).
- Their YouTube or Instagram robot reveal for DECODE was not found by web search. The site lists `youtube.com/@metainfinity18221`, but I did not review its videos.
- Location conflict: FTC Scout lists Highland Park, IL. 2022 library event listings (search-result snippets only, not fetched) describe "FTC State Finalist team Meta^Infinity-18221" at Connecticut venues — [Easton Courier](https://eastoncourier.news/event/robotics-show-tell-w-ftc-finalist-team-metainfinity-18221-recommended-for-grades-4-7/), [Mark Twain Library](https://marktwainlibrary.org/events/robotics-show-and-tell-with-ftc-finalist-team-metainfinity-18221-recommended-for-grades-4-7-in-person/). Unresolved: the team may have relocated, or the snippet may be a mix-up.
- No engineering portfolio from Meta^Infinity was found.

---

## Q2. Which other DECODE Worlds teams published code with shoot-on-the-move or auto-aim turrets?

### Takeaway
I found four 2026 Worlds attendees with public DECODE code that does auto-aim; three of them do some form of shoot-on-the-move:
- **12808 RevAmped Robotics:** Goodall Control Award 1st. 2-D bilinear tables, iterated virtual pose, Lambert-W robot-coast prediction, turret velocity feedforward.
- **15083 Overclock:** Lovelace Division finalist alliance captain, Design 1st. Iterated projection with p + v·t + ½·a·t².
- **21058 Panheya:** Jackson Division rank 6. Constant 0.7 s look-ahead pose plus hood compensation for flywheel sag.
- **25609 LOAD Robotics:** Goodall Inspire 2nd. Static auto-aim only, but a latency-compensated EKF localizer.

A strong non-Worlds reference is **23641 Cuberobot** (`SOTMUtil`, with analytic turret-rate and flywheel-acceleration feedforward). I found no public code for the World Champion alliance (30030, 21087, 11228) or for the highest-OPR teams (e.g. 14270 Quantum Robotics, OPR 238).

### Cited Findings

**Worlds 2026 context (FTC Scout, Verified)**
- Six divisions (Ross, Edison, Jackson, Goodall, Lovelace, Franklin) plus Finals.
- Division winning alliances:
  - Ross: 14270 Quantum Robotics, 3565 Ghost Robotics, 7149 ENFORCERS
  - Edison: 19066 AiCitizens, 11329 I.C.E., 23305 Creative Minds
  - Jackson: 11260 Up-A-Creek, 19564 ITKAN Solar, 23224 Cheesy Bytes
  - Goodall: 18270 RoboPlayers, 20265 Heart of RoBots, 7172 Technical Difficulties
  - Lovelace: 30030 Exodus, 21087 Velocity, 11228 OverClucked Bots
  - Franklin: 30110 Coyote, 18221 Meta^Infinity, 10949 MOB
- Highest division non-penalty OPRs: 14270 (238.1), 19564 ITKAN Solar (226.5), 15993 infO(1) (217.2), 12682 Golden Ratio (213.8), 17861 CSH (212.2).
- Source: [FTC Scout Ross](https://ftcscout.org/events/2025/FTCCMP1ROSS), [Edison](https://ftcscout.org/events/2025/FTCCMP1EDIS), [Jackson](https://ftcscout.org/events/2025/FTCCMP1JACK), [Goodall](https://ftcscout.org/events/2025/FTCCMP1GOOD), [Lovelace](https://ftcscout.org/events/2025/FTCCMP1LOVE), [Franklin](https://ftcscout.org/events/2025/FTCCMP1FRAN)

**12808 RevAmped Robotics (Portland, OR). Worlds Goodall: rank 11, OPR 131.6, Control Award 1st. Off-season: MTI rank 3 / OPR 243.5 (winning alliance), West Coast Invitational OPR 250.9** — [FTC Scout 12808](https://ftcscout.org/teams/12808)
- Repo: https://github.com/junkjunk123/RevAmped-Decode-V2 (master, created 2025-12-14, last push 2026-09-26, so it includes MTI off-season code such as `MTITele`). Pedro `com.pedropathing:ftc:2.1.0-alpha.2` with a **custom OctoQuad localizer** (`pedro/octoquad/OctoQuadLocalizer`, registered via `FollowerBuilder.setLocalizer(...)`) — [REV pedro/Constants.java](https://github.com/junkjunk123/RevAmped-Decode-V2/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/Constants.java)
- **Active code path:** `TrackingThread.update()` calls `SimpleShooterMath.update(...)`, then commands the hood, the turret (`ServoTurretMTI`, as `AutoTrack`) and the flywheel velocity. Pose and velocity come straight from the localizer: `follower.getPoseTracker().getLocalizer()`, then `localizer.getPose()`, `getVelocity()`, `getAngularVelocity()` — [REV mechanisms/shooter/TrackingThread.java](https://github.com/junkjunk123/RevAmped-Decode-V2/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/shooter/TrackingThread.java)
- **Tables are 2-D, not 1-D:** flywheel velocity, hood (stored as **sine** of the hood angle), air time and turret position are `BilinearInterpolation` grids over (|dx|, |dy|) to the goal, with X ∈ {36, 72, 108} and Y ∈ {24, 48, 72} in — [REV utils/math/projectile/SimpleShooterMath.java](https://github.com/junkjunk123/RevAmped-Decode-V2/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/utils/math/projectile/SimpleShooterMath.java)
- **SOTM turret (`computeTurretPos`):**
  - A `LambertApproximator` ("timeTravelDevice") predicts how far and how fast the robot will coast given current speed. It is a drag-type deceleration model solved with the Lambert W function.
  - `iteratePose` then shifts the pose by `fieldVelocity × airTime` for SOTM_ITERATIONS = 10.
  - **Bug:** inside the loop, air time is always computed from `currentPos`, never from `virtualPose`, so the 10 iterations give the same result as 1.
  - Source: [REV SimpleShooterMath.java lines ~223–269](https://github.com/junkjunk123/RevAmped-Decode-V2/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/utils/math/projectile/SimpleShooterMath.java); [REV utils/math/LambertApproximator.java](https://github.com/junkjunk123/RevAmped-Decode-V2/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/utils/math/LambertApproximator.java)
- **Turret feedforward terms:**
  - A numerical derivative of the turret target, (next − current)/DT with DT = 1 ms, is multiplied by K_turretPrediction = 0.5 and added.
  - The yaw rate is subtracted as `ω × ANGULAR_CONSTANT` (0.05 s).
  - Alliance and far-zone trim offsets are added on top.
  - Source: [REV SimpleShooterMath.java](https://github.com/junkjunk123/RevAmped-Decode-V2/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/utils/math/projectile/SimpleShooterMath.java)
- **Hood SOTM offset:** above a speed threshold the hood gets a fixed offset (`HOOD_COMP_SOTM`), and a larger one when moving away from the goal (`dot(v, displacement) < 0`). There are optional gradient-feedforward terms (`K_flywheelPrediction`, `K_hoodPrediction`, both default 0). Shooting is disabled within 60 in of the goal (`tooCloseThreshold`) — [REV SimpleShooterMath.java](https://github.com/junkjunk123/RevAmped-Decode-V2/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/utils/math/projectile/SimpleShooterMath.java); [REV RobotConstants.java](https://github.com/junkjunk123/RevAmped-Decode-V2/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/RobotConstants.java)
- **Flywheel:** `FlywheelController` is a 4-mode hybrid (SPINUP with kick power 1.0, REGEN, HOLD, SPINDOWN) with hysteresis ratios (0.75 / 0.92 / 0.95). It filters measured velocity with a single-state Kalman filter and uses kV and kS feedforward — [REV mechanisms/shooter/FlywheelController.java](https://github.com/junkjunk123/RevAmped-Decode-V2/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/shooter/FlywheelController.java)
- **An older, apparently unused `ShooterMath`** shows a physics approach. It reads `follower.getVelocity()`, rotates it into the robot frame, and chooses the flywheel direction so the robot's velocity normal to the goal line is cancelled. It also adds the along-shot robot speed to the launch speed and solves hood θ with `ProjectileMathWithDrag.solveTheta`, accepting only if confidence > 0.5. Only `SimpleShooterMath` is instantiated (by `TrackingThread`) — [REV utils/math/projectile/ShooterMath.java](https://github.com/junkjunk123/RevAmped-Decode-V2/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/utils/math/projectile/ShooterMath.java)

**15083 Overclock (Allen, TX). Worlds Lovelace: rank 2, OPR 166.4, Finalist alliance captain, Design Award 1st** — [FTC Scout 15083](https://ftcscout.org/teams/15083)
- Repo: https://github.com/Xtendera/OverclockDecode (master, last push 2026-08-12; README "Official 15083 Overclock code repository"). Uses Pedro 2.1.1, `com.pedropathing:ivy:1.0.0` and the Pinpoint localizer — [GitHub](https://github.com/Xtendera/OverclockDecode)
- **SOTM in `Shooter.getCompState(...)`:**
  - Runs 5 fixed-point iterations: `t = timeLUT(distance(goal, shooterPos'))`, then `shooterPos' = shooterPos + v·t + ½·a·t²`.
  - Acceleration is low-pass filtered (α = 0.01) from `follower.getAcceleration()`. An angular term exists, but angular velocity is passed as 0.0 at the call site.
  - The shooter position is offset from the robot center by (1.7135, −2.252) in, rotated by heading.
  - Time table example: 60.4 in → 0.52 s, 75.2 → 0.66, 101.8 → 0.75.
  - Sources: [OC subsystems/Shooter.java](https://github.com/Xtendera/OverclockDecode/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/overclock/subsystems/Shooter.java); [OC globals/MConstants.java](https://github.com/Xtendera/OverclockDecode/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/overclock/globals/MConstants.java)
- **Turret:**
  - DC motor with PIDF plus `FeedforwardTurretController` (kV, kS, both 0 in the constants).
  - Error is wrapped to ±π but allowed to run **15° past ±180°** (overlap) using the sign of the previous error, so it does not flip back and forth at the seam.
  - Output is speed-capped as `|target − pos| / RAMP_DOWN_TICKS(200)`, with a floor of 0.09.
  - Source: [OC subsystems/Shooter.java](https://github.com/Xtendera/OverclockDecode/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/overclock/subsystems/Shooter.java); [OC globals/FeedforwardTurretController.java](https://github.com/Xtendera/OverclockDecode/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/overclock/globals/FeedforwardTurretController.java)
- **Flywheel and hood:**
  - Pure bang-bang with reverse braking: +1 if below target − 15, −1 if above target + 120, else 0. The target is rounded to 20 TPS.
  - The hood comes from a LUT, with a color-sorting override that sets hood 0.8611 to reject a ball.
  - Source: [OC subsystems/Shooter.java](https://github.com/Xtendera/OverclockDecode/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/overclock/subsystems/Shooter.java)
- **TeleOp calls and gating:**
  - Calls `robot.shooter.update(follower.getPose(), follower.getVelocity().getXComponent(), ...getYComponent(), follower.getAcceleration().copy(), follower.getAngularVelocity(), true)`.
  - Shot gate: the fire button rumbles and refuses when the flywheel is outside `SHOOTER_TPS_THRESHOLD` *and* the robot is slower than 4 in/s, or when turret error exceeds 12 ticks.
  - Limelight **MegaTag2** updates a position Kalman filter only when robot speed < 4 in/s and MT2 is within 4.5 in of odometry.
  - Source: [OC opmodes/teleop/TeleOp.java](https://github.com/Xtendera/OverclockDecode/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/overclock/opmodes/teleop/TeleOp.java); [OC subsystems/LimeLight.java](https://github.com/Xtendera/OverclockDecode/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/overclock/subsystems/LimeLight.java); [OC globals/PositionKalmanFilter.java](https://github.com/Xtendera/OverclockDecode/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/overclock/globals/PositionKalmanFilter.java)
- A tuning OpMode, `VelocityTableTuner`, builds the velocity table — [OC opmodes/teleop/other/VelocityTableTuner.java](https://github.com/Xtendera/OverclockDecode/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/overclock/opmodes/teleop/other/VelocityTableTuner.java)

**21058 Panheya (Almaty, Kazakhstan). Worlds Jackson: rank 6, OPR 189.0, Reach 3rd; also attended Bucharest Twin Cup** — [FTC Scout 21058](https://ftcscout.org/teams/21058)
- The index repo https://github.com/DennNuke/ftcpanheya-decode-all-time-code links per-event repos. The Worlds code is **https://github.com/DennNuke/ftcpanheya-worlds** (last push 2026-05-23), using Pedro 2.1.1 and Pinpoint — [GitHub index](https://github.com/DennNuke/ftcpanheya-decode-all-time-code)
- **SOTM:** `PoseController.getFuturePose(f)` returns `pose + follower.getVelocity() × rVM`, with a **constant** rVM = 0.7 s that does not depend on distance. Both turret aim (`AxonTurret.face(goal, futurePose, follower.getAngularVelocity())`) and distance for the shooter and hood LUTs use this future pose — [PAN robot/utils/PoseController.java](https://github.com/DennNuke/ftcpanheya-worlds/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/utils/PoseController.java); [PAN teleop/comp/CompRed2Driver.java](https://github.com/DennNuke/ftcpanheya-worlds/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/comp/CompRed2Driver.java)
- **Turret yaw lead:** `atan2(...) + angleVel·rAM` with rAM = −0.14. It is zeroed when |ω| < 2.5 rad/s, so it only acts during fast spins — [PAN robot/subsystems/AxonTurret.java](https://github.com/DennNuke/ftcpanheya-worlds/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/AxonTurret.java)
- **Hood compensates flywheel sag:**
  - In "comp mode", predicted flywheel speed = v + a·1.6.
  - If it differs from the LUT target by more than 30, the hood is offset in proportion to the error, up to `compM`.
  - Source: [PAN robot/subsystems/Hood.java](https://github.com/DennNuke/ftcpanheya-worlds/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Hood.java)
- **Flywheel:** a speed-interpolated kV (kV1 to kV2 between speed1 and speed2) plus kP and kS. "Ready" means |error| < 60 — [PAN robot/subsystems/Shooter.java](https://github.com/DennNuke/ftcpanheya-worlds/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Shooter.java)
- **Launch-zone geometry is coded as polygons:** close zone triangle (144,144)–(72,72)–(0,144) and far zone (48,0)–(72,24)–(96,0), tested against a rotated 20×20 robot footprint (`isInZone`) — [PAN robot/utils/PoseController.java](https://github.com/DennNuke/ftcpanheya-worlds/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/utils/PoseController.java)

**25609 LOAD Robotics (Odessa, DE). Worlds Goodall: rank 16, Inspire 2nd; Michiana Premier Inspire 1st** — [FTC Scout 25609](https://ftcscout.org/teams/25609)
- Repo: https://github.com/LOAD-Robotics-Team-25609/Decode-Robot-Code (master, last push 2026-07-13), using Pedro 2.1.2 — [GitHub](https://github.com/LOAD-Robotics-Team-25609/Decode-Robot-Code)
- **No SOTM.** Auto commands wait until `follower.getVelocity().getMagnitude() < 0.2` before shooting. The turret aims with `rotationalAimbotLocalizer()`: atan2 from the follower pose to a goal point, with different near and far aim points such as blue (8, 136) and (8, 138), plus hood LUTs for near and far — [LOAD LOADCode/Main_/Hardware_/Commands.java](https://github.com/LOAD-Robotics-Team-25609/Decode-Robot-Code/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/LOADCode/Main_/Hardware_/Commands.java); [LOAD LOADCode/Main_/Hardware_/Actuators_/Turret.java](https://github.com/LOAD-Robotics-Team-25609/Decode-Robot-Code/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/LOADCode/Main_/Hardware_/Actuators_/Turret.java)
- **Latency-compensated localization worth reusing:**
  - `pedroPathing/FusionLocalizer` implements Pedro's `Localizer` interface around the Pinpoint localizer.
  - It keeps time-stamped pose, twist and covariance histories in `TreeMap`s. `addMeasurement(pose, timestamp, variance)` applies the Kalman update *at the camera timestamp*, using a Joseph-form covariance update and per-axis NaN masks.
  - It then re-integrates every later odometry twist forward to the present.
  - Source: [LOAD pedroPathing/FusionLocalizer.java](https://github.com/LOAD-Robotics-Team-25609/Decode-Robot-Code/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/FusionLocalizer.java)
- RevAmped's repo contains a file with the same name (`pedro/FusionLocalizer.java`), so this is likely a shared community snippet — [REV pedro/FusionLocalizer.java](https://github.com/junkjunk123/RevAmped-Decode-V2/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/FusionLocalizer.java)

**Non-Worlds reference: 23641 Cuberobot (Palo Alto, CA). NorCal Championship Silicon Division rank 4; off-season Lobster Cup winner (OPR 211.1), MTI rank 11 (OPR 227.3); not at Worlds** — [FTC Scout 23641](https://ftcscout.org/teams/23641)
- Repo: https://github.com/kleongf/FTC_Decode ("regionals and MTI codebase"), using Pedro 2.1.0 and Pinpoint — [GitHub](https://github.com/kleongf/FTC_Decode)
- **`SOTMUtil.calculateShooterOutputs(turretPose, v, a, ω, dt, alliance)`:**
  - TOF comes from a quadratic fit, `tofFunctionTele(d) = 2.055e-5·d² − 0.001345·d + 0.6831`, iterated `TOF_ITERATIONS` times on the projected robot position, then multiplied by `tofMultiplier = 0.8` ("0.9 was good from far").
  - `virtualGoal = goal − v·tof`.
  - **Analytic turret-rate feedforward:** d/dt of atan2(−dx, dy) gives `(dx·ẏ − dy·ẋ)`-type terms over r². The yaw rate ω is subtracted and the result is sent as `wantedAngularVelocity` to the turret's kV term.
  - **Flywheel acceleration feedforward:** wheel-speed LUT slope × the rate of change of distance.
  - A per-alliance turret offset LUT is keyed by distance.
  - Sources: [CUBE util/decodeutil/SOTMUtil.java](https://github.com/kleongf/FTC_Decode/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/decodeutil/SOTMUtil.java); [CUBE decode2026/constants/ShootingConstants.java](https://github.com/kleongf/FTC_Decode/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/decode2026/constants/ShootingConstants.java)
- **Pedro reads:**
  - `follower.getVelocity()`
  - `follower.poseTracker.getAngularVelocity()`
  - `follower.poseTracker.getAcceleration()`
  - Sources: [CUBE util/decodeutil/TeleopDrivetrain.java](https://github.com/kleongf/FTC_Decode/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/decodeutil/TeleopDrivetrain.java); [CUBE decode2026/opmode/teleop/MainTeleop.java](https://github.com/kleongf/FTC_Decode/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/decode2026/opmode/teleop/MainTeleop.java)
- **Turret:** DC motor plus an analog absolute encoder that seeds the motor-encoder offset. A weighted-setpoint PID adds kS·sign(e) + kV·wantedAngularVelocity, with voltage compensation — [CUBE decode2026/subsystems/Turret.java](https://github.com/kleongf/FTC_Decode/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/decode2026/subsystems/Turret.java)
- **Flywheel (`TorqueShooter`):**
  - Power = (R·kP·e + kω·ω)/Vnom + kS·sign(e) + kV·target + kA·targetAccel, with voltage compensation and a write-caching threshold.
  - **Bug:** the "full power if error large" check is written `Math.signum(error) > 300`, so it can never trigger.
  - Source: [CUBE decode2026/subsystems/TorqueShooter.java](https://github.com/kleongf/FTC_Decode/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/decode2026/subsystems/TorqueShooter.java)

**Non-Worlds but well documented: 11138 Robo Eclipse (Bellevue, WA). WA Championship Control Award 1st; Canadian Rockies Premier Control Award 3rd** — [FTC Scout 11138](https://ftcscout.org/teams/11138)
- Repo: https://github.com/FTC11138/Decode-Premier-Ivy (mirror: https://github.com/FelixYBW/FTC11138). Uses Pedro 2.1.2 + Ivy 1.0.0, Pinpoint and Limelight MegaTag2. No SOTM; it relies on an "approach spin-up latch" and "pre-aim" instead.
- Its README documents in detail:
  - gated vision fusion
  - bilinear field-position shot tables (`WaveLength`) with zone switching and red→blue mirroring
  - a turret position computed from the lever arm (`TurretLocation` / `TractorBeam`)
  - one readiness predicate (flywheel ∧ hood) shared by auto gating, driver fire control and LEDs
  - Source: [Decode-Premier-Ivy README](https://github.com/FTC11138/Decode-Premier-Ivy)

**Other DECODE turret repos found by GitHub search but not analysed (not Worlds teams, or not verified):**
- [YoGurt31/FTC-DeCode-4780-Turret](https://github.com/YoGurt31/FTC-DeCode-4780-Turret) — team 4780; FTC Scout shows NYC Championship Design 1st.
- [Vlarkus/FTC-Decode-Turret-Yaw-Experiments](https://github.com/Vlarkus/FTC-Decode-Turret-Yaw-Experiments) and [Vlarkus/FTC-Decode-Shooter-Trajectory-Calculator](https://github.com/Vlarkus/FTC-Decode-Shooter-Trajectory-Calculator).
- [parzivalchik/ftc-28888-decode](https://github.com/parzivalchik/ftc-28888-decode) — "field-locked turret"; team 28888 InSpectors, Kazakhstan; Pedro 2.0.4/2.0.5.
- [hollow720/ftc-robot-code-21813-decode](https://github.com/hollow720/ftc-robot-code-21813-decode) — surfaced by web search; not opened.
- [WoEN239/Decode18742](https://github.com/WoEN239/Decode18742) — Kotlin, Road Runner 1.0.1. No SOTM keywords found by grep; its only FTC Scout event is the Italy Championship.

**Searches that found nothing:** GitHub repo search by team number for 14270, 15993, 17861, 19066, 20265, 15996, 15991, 30030, 21087, 11228, 18270, 7172, 19564, 12682, 30110, 20871, 11260 and 17792 returned no DECODE code repos. The `14270` GitHub account is an unrelated individual — [GitHub search API, queried 2026-10-21](https://api.github.com/search/repositories?q=14270+in:name,description)

### Inferences
- Three implementation tiers appear among Worlds-level teams:
  - **Tier 1, constant look-ahead** (Panheya): pose + v·0.7 s. Simplest, and fine when shots are 0.5–0.9 s long.
  - **Tier 2, virtual goal with a TOF LUT** (Meta^Infinity single-pass; Overclock 5-iteration with acceleration; Cuberobot iterated with a quadratic fit).
  - **Tier 3, extras:**
    - turret velocity feedforward from the analytic or numerical derivative of the aim angle (Cuberobot, RevAmped)
    - flywheel acceleration feedforward (Cuberobot)
    - hood correction for flywheel sag (Panheya)
    - robot-coast prediction (RevAmped)
- Everyone uses empirically measured tables rather than pure ballistics. RevAmped's physics path appears to have been abandoned in favor of tables, and 11138's README states that foam-ball exit dynamics resist modelling.
- Every SOTM team still uses some yaw-rate lead term (0.05 s at Meta^Infinity and RevAmped; −0.14 with a deadband at Panheya). Turret lag during chassis spins is evidently the most visible error source.

### Gaps
- No public code was found for the World Champion alliance (Exodus 30030, Velocity 21087, OverClucked Bots 11228), for 14270 Quantum Robotics (highest OPR), or for the Romanian high-OPR teams. GitHub code search requires authentication, so content-level search ("shootOnTheMove", "virtualGoal") was not possible. Only repo name/description search was done.
- Could not verify whether each team actually ran SOTM in Worlds matches (as opposed to off-season), because match video was not reviewed.

---

## Q3. Pedro Pathing versions, localizers and loop-time optimizations

### Takeaway
Every Worlds-level SOTM repo examined uses **Pedro Pathing 2.x** (2.1.0-alpha.2 to 2.1.2), and all but one use the **goBILDA Pinpoint** (RevAmped wrote an OctoQuad localizer). Shoot-on-the-move code reads `Follower.getPose()`, `Follower.getVelocity()` (field-frame inches/s), `Follower.getAngularVelocity()` / `poseTracker.getAngularVelocity()`, and sometimes `getAcceleration()`. Common loop-time techniques:
- `LynxModule.BulkCachingMode.MANUAL` with a single `clearBulkCache()` per loop
- threshold-based caching of motor and servo writes
- throttled telemetry

Pedro **3.0.0 was released 2026-09-10**, so next-season code will likely need porting.

### Cited Findings
- **Pedro versions per repo (from `build.dependencies.gradle`):**
  - Meta^Infinity `com.pedropathing:ftc:2.1.0-alpha.2` + `telemetry:1.0.0`, `com.bylazar:fullpanels:1.0.6` and FTC Dashboard 0.5.1, FTC SDK 11.0.0 — [Meta build.dependencies.gradle](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/build.dependencies.gradle)
  - RevAmped 2.1.0-alpha.2 — [REV repo](https://github.com/junkjunk123/RevAmped-Decode-V2)
  - Cuberobot 2.1.0 — [CUBE repo](https://github.com/kleongf/FTC_Decode)
  - Overclock 2.1.1 + ivy 1.0.0 — [OC repo](https://github.com/Xtendera/OverclockDecode)
  - Panheya 2.1.1 — [PAN repo](https://github.com/DennNuke/ftcpanheya-worlds)
  - LOAD 2.1.2 — [LOAD repo](https://github.com/LOAD-Robotics-Team-25609/Decode-Robot-Code)
  - 11138 2.1.2 + ivy 1.0.0 — [11138 repo](https://github.com/FTC11138/Decode-Premier-Ivy)
- **Pedro release timeline (GitHub releases):** v2.0.0 2025-08-29, v2.0.4 2025-11-06, v2.0.6 2026-01-30, v2.1.0 2026-03-21, v2.1.1 2026-03-24, v2.1.2 2026-05-07, **v3.0.0 2026-09-10**, v3.0.2 2026-10-16 — [Pedro-Pathing/PedroPathing releases](https://github.com/Pedro-Pathing/PedroPathing/releases)
- **Pedro v2.1.0 velocity semantics:**
  - `PoseTracker.getVelocity()` returns `localizer.getVelocity().getAsVector()`, cached per update.
  - `getAngularVelocity()` returns that velocity's heading component.
  - `getAcceleration()` is the finite difference of consecutive velocities divided by the pose-time delta.
  - Source: [PoseTracker.java @v2.1.0](https://github.com/Pedro-Pathing/PedroPathing/blob/v2.1.0/core/src/main/java/com/pedropathing/localization/PoseTracker.java)
- **For the Pinpoint localizer,** velocity is `new Pose(odo.getVelX(INCH), odo.getVelY(INCH), odo.getHeadingVelocity(RADIANS unnormalized))`. These are Pinpoint field-frame velocities, which is why teams subtract `v·t` directly in field coordinates — [PinpointLocalizer.java @v2.1.0](https://github.com/Pedro-Pathing/PedroPathing/blob/v2.1.0/ftc/src/main/java/com/pedropathing/ftc/localization/localizers/PinpointLocalizer.java)
- **Meta^Infinity's Pinpoint configuration:**
  - `PinpointConstants`: forwardPodY 2.2298, strafePodX −4.4317, goBILDA_4_BAR_POD, inches.
  - `FollowerConstants`: mass 12.2 kg and `PredictiveBrakingCoefficients(0.1, 0.0721, 0.00122)`.
  - Mecanum xVelocity 92.08 and yVelocity 71.61 in/s.
  - Source: [META pedroPathing/Constants.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Constants.java)
- **Meta^Infinity loop-time techniques (Verified):**
  - Every hub is set to `BulkCachingMode.MANUAL`, and `clearCache()` runs at the top of each loop, including init loops.
  - Writes are cached with thresholds: drive motor 0.02, shooter 0.005, intake/transfer 0.02, turret servo 0.001, hood 0.002, latch and light servos 0.001.
  - Telemetry is sent at most every 300 ms.
  - The Limelight is polled only while fusion is enabled.
  - The flywheel encoder reuses a drive-motor encoder port.
  - Sources: [META robot/Robot.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/Robot.java); [META util/RobotConstants.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/RobotConstants.java); [META subsystems/DrivetrainHardware.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/DrivetrainHardware.java)
- **RevAmped:** `Robot` sets `BulkCachingMode.MANUAL` and calls `clearBulkCache()` each loop. Localization comes from an OctoQuad (`OctoQuadFWv3`, with I2C recovery mode `MODE_1_PERIPH_RST_ON_FRAME_ERR`) — [REV Robot.java](https://github.com/junkjunk123/RevAmped-Decode-V2/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Robot.java); [REV pedro/Constants.java](https://github.com/junkjunk123/RevAmped-Decode-V2/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/Constants.java)
- **Cuberobot:** a `BulkRead` utility and `CachedMotor` with a caching threshold on the flywheel — [CUBE lib/util/BulkRead.java](https://github.com/kleongf/FTC_Decode/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/lib/util/BulkRead.java); [CUBE util/decodeutil/CachedMotor.java](https://github.com/kleongf/FTC_Decode/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/decodeutil/CachedMotor.java)
- **11138 README:**
  - MANUAL bulk cache invalidated once per loop; motor current kept to one read per loop because it is not in the bulk packet.
  - An exponentially smoothed loop-time HUD on the Driver Station.
  - Every controller clamps dt and substitutes 20 ms if dt is non-positive or absurdly large.
  - Source: [Decode-Premier-Ivy README](https://github.com/FTC11138/Decode-Premier-Ivy)
- **Panheya** sets MANUAL in its teleop base `DennyOpMode` — [PAN teleop/DennyOpMode.java](https://github.com/DennNuke/ftcpanheya-worlds/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/DennyOpMode.java). **Overclock:** grep found MANUAL only in Pedro's `Tuning.java`. Its competition code may rely on FTCLib defaults (not verified).

### Inferences
- A typical SOTM-ready loop on Pedro 2.x:
  1. `clearBulkCache()`
  2. `follower.update()`
  3. Read `getPose()`, `getVelocity()` and `getAngularVelocity()` once.
  4. Run the aim solver.
  5. Write actuators through caches.
- Pedro's `getAcceleration()` is a raw finite difference. Overclock low-pass filters it (α = 0.01) before use, which suggests it is noisy.
- The Pedro 3.0 release (September 2026) restructured the source tree: the v3 `core` module no longer contains the v2 file path for `Follower`. Teams porting this code should re-check method names and coordinate conventions. This is an inference; the v3 API was not studied.

### Gaps
- I did not measure or find reported loop times (ms/Hz) for these robots.
- Pedro 3.x API changes relevant to velocity and acceleration access were not verified.

---

## Q4. DECODE game specifics that shaped the designs, and what carries over to a new season with a different goal

### Takeaway
DECODE's goal is a **large, open-topped corner target**: a horizontal triangular opening about 26.5 in × 18.3 in, top lip at 38.75 in, with AprilTags 20 (blue) and 24 (red) on its front face. Artifacts are 5 in foam balls, and launching is only allowed from two triangular launch zones. Generous target size and long shot distances (about 40–155 in, 0.45–1.0 s time of flight) made LUT-based virtual-goal SOTM worthwhile.

The **2026–2027 game is BIOBUZZ**. Its targets are **CELLs on tipping HIVEs** (opening about 20 × 14 × 12 in, pivot axis 43.95 in high) and **4 in FLOWER openings at 21.5 in**. The balls are smaller (2.8 in POLLEN, 3.6 in NECTAR), and the AprilTags sit on the moving CELLs. So targets move between two states and tags are not fixed. The solver structure carries over; tables and target definitions do not.

### Cited Findings
- **DECODE field:** about 144 × 144 in, one GOAL per alliance. There are 2 LAUNCH ZONES, both infinitely tall triangles: the goal-side zone spans 6 tiles wide × 3 tiles deep, the audience-side zone 2 tiles wide × 1 tile deep. LAUNCH LINE tape counts as part of the zone — [DECODE Competition Manual §9 (archive)](https://ftc-resources.firstinspires.org/ftc/archive/2026/game/manual-09)
- **DECODE GOAL:**
  - The structure is about 27 × 27 × 54 in tall, 3-sided, with a horizontal triangular top opening.
  - The opening is about 26.5 in wide and 18.3 in deep.
  - The top lip is 38.75 in above the tiles.
  - The backboard extends up to 15 in above the opening.
  - A CLASSIFIER/RAMP holds 9 artifacts. The manual warns that artifacts launched "at a high velocity or with significant spin" may skip over.
  - Source: [DECODE Competition Manual §9](https://ftc-resources.firstinspires.org/ftc/archive/2026/game/manual-09)
- **DECODE AprilTags:** red goal ID 24, blue goal ID 20, on the goal front face. OBELISK tags 21–23 encode the MOTIF and are "not recommended for ROBOT navigational use" — [DECODE Competition Manual §9](https://ftc-resources.firstinspires.org/ftc/archive/2026/game/manual-09)
- **ARTIFACTS:** 5 in (12.70 cm) nominal Gopher ResisDent polypropylene balls, purple and green, "not perfectly spherical" — [DECODE Competition Manual §9](https://ftc-resources.firstinspires.org/ftc/archive/2026/game/manual-09). A later team update reportedly gives 4.9 ± 0.25 in at the mold seam (search-result summary of [TU 09](https://ftc-resources.firstinspires.org/ftc/archive/2026/game/tu-09); not opened).
- **Launch rule (search-result summary of the manual, not verified in the text):** robots may only launch when inside a LAUNCH ZONE or overlapping a LAUNCH LINE. Each violation is a minor foul per element, and a major foul if the element enters the goal — [DECODE manual §16 (archive)](https://ftc-resources.firstinspires.org/ftc/archive/2026/game/manual-16)
- **How teams encoded this:**
  - Panheya codes both launch-zone triangles as polygons and checks the rotated robot footprint against them — [PAN robot/utils/PoseController.java](https://github.com/DennNuke/ftcpanheya-worlds/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/utils/PoseController.java)
  - Overclock has a `LaunchZoneDetector` — [OC globals/LaunchZoneDetector.java](https://github.com/Xtendera/OverclockDecode/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/overclock/globals/LaunchZoneDetector.java)
  - Meta^Infinity, RevAmped and LOAD aim at a point inside the goal corner, not at the tag. Examples: Meta (134, 138) / (8, 137); RevAmped (144, 144) / (0, 144); LOAD near/far (8, 136) / (8, 138). Sources: [META util/RobotConstants.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/RobotConstants.java); [REV SimpleShooterMath.java](https://github.com/junkjunk123/RevAmped-Decode-V2/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/utils/math/projectile/SimpleShooterMath.java); [LOAD Turret.java](https://github.com/LOAD-Robotics-Team-25609/Decode-Robot-Code/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/LOADCode/Main_/Hardware_/Actuators_/Turret.java)
- **Next season (BIOBUZZ, 2026–2027), from the current official manual §9 (TU04):**
  - The scoring structure is HIVEs: bi-stable assemblies of two CELLs about 18.8 in apart, rotating on pivots at 43.95 in. Each HIVE tips when enough POLLEN or NECTAR is LAUNCHED into the upward-facing CELL.
  - CELL opening: about 20 in wide × 14 in tall × 12 in deep.
  - Each CELL carries a 4-tag AprilTag cluster on its *bottom* face, facing the tiles.
  - FLOWERS: 4 in diameter top opening at 21.5 in.
  - Balls: POLLEN about 2.8 in, NECTAR about 3.6 in. A match has 40 POLLEN and 8 + 8 NECTAR.
  - Source: [BIOBUZZ Competition Manual §9](https://ftc-resources.firstinspires.org/ftc/game/manual-09)
- **Chief Delphi "FTC auto-aiming" thread (Sept 13–15, 2026):**
  - Posters note that BIOBUZZ tags "are on moving elements", so vision-only pose is harder.
  - They recommend Pinpoint over OTOS or motor encoders.
  - They recommend a static shooter and fixed "sweet spots" before turrets and shoot-while-moving.
  - Source: [Chief Delphi: FTC auto-aiming](https://www.chiefdelphi.com/t/ftc-auto-aiming/524081)

### Inferences (reusable vs. game-specific)
- **Reusable as-is:**
  - Virtual-goal SOTM: `target − v·TOF(d)`, preferably iterated 3–5× (Overclock, Cuberobot), optionally with a ½·a·t² term.
  - Turret: absolute-encoder unwrap, nearest-equivalent target inside wire limits (Meta^Infinity `remapTargetIntoSafeRange`), overlap past ±180° (Overclock), slew limit, gain scheduling with hysteresis, ramped kS, voltage compensation.
  - Turret velocity feedforward from d/dt of the aim angle minus ω (Cuberobot); yaw-rate lead.
  - Flywheel: PIDF hold with bang-bang or kick during feed (Meta^Infinity, Overclock, RevAmped), flywheel acceleration feedforward (Cuberobot), hood correction for RPM sag (Panheya).
  - Pose plumbing: lever-arm turret position, auto→teleop pose and turret-angle persistence, MANUAL bulk caching and write caching, timestamped vision fusion (LOAD `FusionLocalizer`), gated MegaTag2 blending (11138).
  - Tuning OpModes that record table rows.
- **Must be redone for BIOBUZZ:**
  - All LUTs (RPM, hood, TOF), because ball size and mass change.
  - Target points: CELL positions depend on HIVE state (two candidate aim points per HIVE), and FLOWERs are 4 in openings at 21.5 in.
  - Vision relocalization: BIOBUZZ tags are on moving CELLs, so DECODE-style fixed goal-tag pose resets do not transfer directly. Fixed-feature relocalization (wall or gate presses, as Meta^Infinity did) and odometry become more important.
- A 4 in FLOWER opening is far less forgiving than DECODE's 26.5 × 18.3 in goal opening. SOTM error budgets (TOF error × robot speed, turret lag) that worked in DECODE may not be acceptable there. Teams may need tighter speed gates for SOTM shots, or shoot stationary at small targets.

### Gaps
- I did not verify BIOBUZZ launch-zone or launching rules, or whether shooting at FLOWERs is a launch or a placement action. Only §9 (Arena) was read.
- A complete DECODE §9 figure showing the exact AprilTag height was not extracted. Meta^Infinity's code uses 29.5 in as the goal-tag height ([META subsystems/Limelight.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Limelight.java)); this is unverified against the manual.

---

## Q5. Reported problems and lessons learned

### Takeaway
The clearest lessons come from the code itself: comments, abandoned paths and bugs. Two documented lessons stand out:
- Vision must be velocity-gated or latency-compensated.
- Physics-based ballistics lost out to measured tables.

Several SOTM implementations contain silent bugs (no-op gates, iterations that never update), so unit-testing the solver offline is worth doing. Community sources (Reddit, Medium) were mostly unreachable.

### Cited Findings
- **Bugs and dead logic found in shipped code:**
  - Meta^Infinity's SOTM distance gate is always true; their own comment acknowledges it — [META robot/Robot.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/Robot.java)
  - RevAmped's `iteratePose` loop never feeds `virtualPose` back into the air-time lookup — [REV SimpleShooterMath.java](https://github.com/junkjunk123/RevAmped-Decode-V2/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/utils/math/projectile/SimpleShooterMath.java)
  - Cuberobot's `Math.signum(error) > 300` can never trigger — [CUBE TorqueShooter.java](https://github.com/kleongf/FTC_Decode/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/decode2026/subsystems/TorqueShooter.java)
  - Overclock passes 0.0 for angular velocity to `getCompState`, which makes its angular-offset term inactive — [OC subsystems/Shooter.java](https://github.com/Xtendera/OverclockDecode/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/overclock/subsystems/Shooter.java)
- **TOF fudge factors show model mismatch:** Cuberobot scales TOF by 0.8 with the comment "0.9 was good from far, prob need to retune tof function" — [CUBE ShootingConstants.java](https://github.com/kleongf/FTC_Decode/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/decode2026/constants/ShootingConstants.java). RevAmped uses `K_time = 1.0` plus several manual turret and hood offsets for far shots, adjusted inside the far auto — [REV opmodes/auto/FarAuto.java](https://github.com/junkjunk123/RevAmped-Decode-V2/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/auto/FarAuto.java)
- **Vision fusion lessons (11138 README):**
  - Velocity gate at 8 in/s, given about 150 ms camera latency.
  - Reject camera poses more than 30 in from odometry or with more than 25° heading disagreement. These target PnP ambiguity: wrong-solution errors of 36–60 in and flipped solutions of about 60°.
  - Require 3-frame consensus, blend with α = 1 − e^(−dt/τ) at τ = 1 s, and deduplicate per camera frame because the loop runs about 3× faster than the camera.
  - Feed yaw to MegaTag2 every loop.
  - Source: [Decode-Premier-Ivy README](https://github.com/FTC11138/Decode-Premier-Ivy)
- **Other teams' gates:** Overclock only accepts MegaTag2 below 4 in/s and within 4.5 in of odometry — [OC opmodes/teleop/TeleOp.java](https://github.com/Xtendera/OverclockDecode/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/overclock/opmodes/teleop/TeleOp.java). Meta^Infinity scales its Kalman R with distance squared and rejects jumps over 90 in — [META controlSystems/KalmanFilter.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/controlSystems/KalmanFilter.java)
- **Per-venue calibration was needed.** Meta^Infinity keeps separate gate-relocalization poses for "michiana", "worlds", "practice" and "home" — [META util/RelocalizePosition.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/RelocalizePosition.java)
- **Auto-specific tricks (11138 README):**
  - Pre-aim at the *upcoming* shoot pose while driving, then live-aim inside 3 in.
  - Latch flywheel speed to the shoot-pose value within 20 in.
  - Compound shot gate: flywheel ∧ position ≤ 5 in (2 in for gate shots) ∧ turret ≤ 2°, raced against a timeout.
  - Source: [Decode-Premier-Ivy README](https://github.com/FTC11138/Decode-Premier-Ivy)
- **Community discussion:**
  - A Chief Delphi thread on turret vs. locked shooters argues turrets tolerate defensive contact better than fixed shooters — [Chief Delphi: Turret Shooter vs locked Shooter](https://www.chiefdelphi.com/t/turret-shooter-vs-locked-shooter/512430) (search-result summary; not opened).
  - In the Sept 2026 auto-aim thread: "The mounting direction of pinpoint is critical!", and auto-aim depends on mechanical shot consistency — [Chief Delphi: FTC auto-aiming](https://www.chiefdelphi.com/t/ftc-auto-aiming/524081)
- Medium articles "Programming a DECODE shooter" and "Programming a hooded DECODE shooter" (by vikramaditya.nishant) describe a pipeline: localization → required velocity → flywheel regression → velocity controller → turret/heading aim. The content was not retrievable (HTTP 403); only search snippets were seen — [Medium 1](https://medium.com/@vikramaditya.nishant/programming-a-decode-shooter-4ab114dac01f), [Medium 2](https://medium.com/@vikramaditya.nishant/programming-a-hooded-decode-shooter-463d6a71832f)

### Inferences
- Build the SOTM solver as a pure function: `(pose, v, a, ω) → (turretAngle, turretRate, rpm, rpmAccel, hood)`, as Cuberobot's `SOTMUtil` does. Unit-test it offline. Every team with inline SOTM math had at least one silent bug.
- Measure TOF directly (high-speed video, or timing from feeder to goal) rather than fitting it with a multiplier. A 10–20% TOF error at 40 in/s robot speed shifts the virtual goal by several inches.
- Gate shots on readiness (RPM ∧ turret error ∧ hood), as 11138 and Overclock do. Meta^Infinity left it to driver judgment, which works with a skilled driver but is fragile for auto SOTM.

### Gaps
- Reddit r/FTC, YouTube robot reveals and FTC Discord summaries specific to DECODE SOTM could not be located with the available search tool. Reddit is poorly indexed, and no relevant threads surfaced.
- No engineering portfolios from the analysed Worlds teams were found or reviewed.
- Game Manual 0 has no section on turret, SOTM or flywheel lookup tables (the search found only generic control-loop pages) — [gm0 Software](https://game-manual-0.readthedocs.io/en/latest/docs/software/)
