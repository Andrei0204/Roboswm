# Aim the turret at a virtual HIVE

Every top team that has published working shoot-on-the-move code (FRC 6328, 254, 604 and 971, and the DECODE World Championship teams led by Meta^Infinity, FTC 18221) solves the same problem at its core: predict where the shooter will be when the ball leaves, use the velocity of the turret point rather than the robot center, cancel that velocity over the ball's flight (most often by moving the goal backward by velocity × flight time to a **virtual goal**), and set turret, hood and flywheel from stationary lookup tables evaluated at the virtual distance. Meta^Infinity, first pick on the Franklin Division-winning alliance at the 2026 World Championship, shipped only the minimal version (**one pass of goal − v·TOF, a 0.05 s yaw-rate lead on the turret, and no software check that the shooter was ready**), which says that accurately measured tables matter more than ballistic sophistication. The complete algorithm for a BIOBUZZ turret, flywheel and hood robot keeps that core and adds what the best FRC code adds: release-time prediction with acceleration, the ω × r velocity of an off-center turret, a converged time-of-flight iteration with a linear-drag correction, a turret feedforward equal to the line-of-sight rate minus the chassis yaw rate, wrap-aware turret selection, and a fire gate derived from the target's real clearances. BIOBUZZ changes the target, not the solver: each alliance scores only in its HIVE's **upward-facing CELL**, a 20 × 14 in pentagonal opening centered about 58 in above the tiles that faces away from field center and tilts 30° upward, so a robot can score only from the half of the field that CELL faces, the aim point jumps about 35 in across the HIVE after every TIP, and both alliance partners fill the same CELL. That geometry makes shots **short (roughly 15–80 in) and steep**, puts robot speed in the same range as the ball's horizontal speed, and demands two table sets (POLLEN and NECTAR), a HIVE-state machine and a tip hold-off that no DECODE code needed. Pedro Pathing 3.0.2 supplies the inputs (`follower.pose()` and a field-frame `follower.velocity()` with `vx`, `vy` in in/s and `omega` in rad/s) but **no acceleration**, so the solver must difference velocity itself; the final section is an implementation-ready specification with frames, equations, gates, a calibration order and every constant that must be measured.

## Top FRC code converges on one fixed point: goal minus velocity times flight time

Public FRC code from the strongest programs falls into three algorithm families, and all three approximate the same physics. The most common is a **time-of-flight (ToF) fixed-point iteration**: look up flight time at the current distance, shift the goal backward (or the launcher forward) by velocity × flight time, recompute the distance and repeat, as 6328 in 2026, 971, 6940, 5000 and 1706 all do ([6328 LaunchCalculator](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/subsystems/launcher/LaunchCalculator.java); [971 aiming.cc](https://github.com/frc971/971-Robot-Code/blob/master/frc971/control_loops/aiming/aiming.cc); [6940 ProjectileCalculator](https://github.com/Team6940/2026Rebuilt/blob/master/src/main/java/frc/robot/util/ProjectileCalculator.java)). The second family solves directly in **velocity space**, either subtracting the shooter's velocity from the stationary launch-velocity vector (604, 5572, 3061) or solving the intercept quadratic |d − v·t| = v_shot·t with a gravity-drop term (254 in 2024) ([604 LaunchCalculator](https://github.com/frc604/2026-public/blob/main/FRC-2026/src/main/java/frc/quixlib/math/LaunchCalculator.java); [254 ShooterSetpoint](https://github.com/Team254/FRC-2024-Public/blob/main/src/main/java/com/team254/lib/util/ShooterSetpoint.java); [3061 Huskie physics](https://www.chiefdelphi.com/t/huskie-physics-shoot-on-the-move-with-equations/522805)). The third simply **extrapolates the pose by a fixed look-ahead** with no flight-time model, as 6328's 2024 robot (0.35 s in teleop) and 1678's 2026 robot do ([6328 RobotState 2024](https://github.com/Mechanical-Advantage/RobotCode2024Public/blob/main/src/main/java/org/littletonrobotics/frc2024/RobotState.java); [1678 MotionCompensatedShootingPlanner](https://github.com/frc1678/C2026-Public/blob/main/src/main/java/frc/robot/shooting/shootingplanner/MotionCompensatedShootingPlanner.java)). Without drag the first two families are exactly equivalent, because a ball launched with relative velocity u_rel from a shooter moving at v_s reaches the real goal g precisely when u_rel is the stationary shot to the virtual goal g − v_s·T; Oblarg notes on Chief Delphi that the recursion contracts toward the closed-form answer ([CD: time-of-flight determination](https://www.chiefdelphi.com/t/time-of-flight-determination/512542)).

The closest analog to an FTC turret, hood and flywheel robot is **6328's turreted 2026 "Alpha Bot"** (branch `alpha-bot-turret`). Each loop it predicts the pose **30 ms** ahead, moves to the turret point, adds ω × r to get the turret's field velocity, runs **20 fixed iterations** of offset = turretVelocity × TOF(lookaheadDistance), aims at the field angle from the look-ahead pose, and reads hood and flywheel at the look-ahead distance ([6328 alpha LaunchCalculator](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/alpha-bot-turret/src/main/java/org/littletonrobotics/frc2026/subsystems/launcher/LaunchCalculator.java)). Its turret class converts to the robot frame, feeds forward the differentiated goal angle minus ω, and picks among goal + 2πi the candidate nearest the last goal inside ±210° while launching or about ±190° while tracking ([6328 alpha Turret](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/alpha-bot-turret/src/main/java/org/littletonrobotics/frc2026/subsystems/launcher/turret/Turret.java)). The team's write-up stresses that distance and velocity must be measured at the turret, that the phase delay matters most under acceleration, and that the ToF table came from slow-motion video ([6328 build thread #272](https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2026-build-thread/509595/272)). After Oblarg observed that the 20-iteration recursion overcorrects at high speed, the main branch scaled the lead by an effective flight time **t_eff = (1 − e^(−kt))/k with k = 0.375 s⁻¹**, which models the robot-imparted velocity decaying under linear drag ([build thread #421](https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2026-build-thread/509595/421); [6328 LaunchCalculator](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/subsystems/launcher/LaunchCalculator.java)).

254 and 604 show the closed-form side. 254's 2022 robot split velocity into radial and tangential parts relative to the goal, assumed a constant **1.05 s** flight time, rotated the turret by atan2(−v_tan, range/T − v_radial) and looked up hood and RPM at the resulting effective range ([254 ShootingUtil](https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/frc2022/shooting/ShootingUtil.java)); Jared Russell later confirmed the constant-ToF tuning ([CD: optimal driving for SOTM](https://www.chiefdelphi.com/t/a-note-on-optimal-driving-for-shoot-on-the-move/512979)). 254's 2024 solver predicts the pose **50 ms** ahead, solves the intercept in 3-D from the release point with gravity drop, converts the solved speed and pitch directly into flywheel RPS and hood angle, and feeds the turret −(ω + v_tan/d) plus an elevation-rate term to the hood ([254 ShooterSetpoint](https://github.com/Team254/FRC-2024-Public/blob/main/src/main/java/com/team254/lib/util/ShooterSetpoint.java)). 604 extrapolates state **50 ms with constant acceleration**, subtracts the turret-base velocity from a stationary shot taken from an offline Monte-Carlo CSV, and re-evaluates the entire solution at ±5 ms to get turret rate and acceleration feedforward without loop-to-loop noise ([604 LaunchCalculator](https://github.com/frc604/2026-public/blob/main/FRC-2026/src/main/java/frc/quixlib/math/LaunchCalculator.java); [604 release thread](https://www.chiefdelphi.com/t/team-604-quixilver-2026-robot-cad-and-code-release/524289)). 971's shared library iterates the virtual goal exactly three times and treats robot velocity as a constant offset to ball velocity over the whole flight ([971 aiming.cc](https://github.com/frc971/971-Robot-Code/blob/master/frc971/control_loops/aiming/aiming.cc)).

How many iterations are enough follows from the contraction factor |dT/dd|·|v_radial|. 6328's alpha ToF table rises only from 0.90 s to 1.16 s across 1.38–5.68 m, about 0.06 s/m, so at 2 m/s each pass shrinks the error roughly eightfold and **3–5 passes are plenty**; 20 cost little but buy nothing ([6328 alpha LaunchCalculator](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/alpha-bot-turret/src/main/java/org/littletonrobotics/frc2026/subsystems/launcher/LaunchCalculator.java)). One pass is not enough for radial motion: in a quadratic-drag simulation run during this research (illustrative, not a published measurement), a 2.5 m shot while approaching at 1.5 m/s fell 0.42 m short with one pass, 0.14 m short with naive iteration, and landed within 0.07 m with iteration plus the effective-ToF correction. 1706 got past 3 m/s in 2022 only after adding an acceleration term, without which its turret, flywheel and hood lagged through every speed change ([CD: 1706 shoot-while-move](https://www.chiefdelphi.com/t/shoot-while-move-code-1706/410494)).

The two exact families trade different risks. The virtual-goal LUT method only ever commands (hood, RPM) pairs that were tested on a real field, at the cost of shifting the ball's field-frame trajectory slightly (approaching flattens the arrival, receding steepens it). Velocity subtraction reproduces a tested field trajectory exactly, even with drag, but commands untested hood/RPM combinations and needs calibrated maps from flywheel speed to exit speed and from hood setting to launch angle; 5000's undocumented 0.54 slip factor and 5572's 0.43 transfer coefficient show how much those maps hide ([5000 TurretCalculator](https://github.com/hammerheads5000/2026Rebuilt/blob/main/src/main/java/frc/robot/subsystems/turret/TurretCalculator.java); [5572 PR #222](https://github.com/Frc5572/FRC2026/pull/222/files)). Pure subtraction can also demand impossible geometry, such as an 80° launch from a hood that stops at 60° when driving away ([3061 Huskie physics](https://www.chiefdelphi.com/t/huskie-physics-shoot-on-the-move-with-equations/522805)). 6328 chose hand-tuned maps because they absorb real-world imperfections and can be retuned piecewise at events ([build thread #272](https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2026-build-thread/509595/272)), and for a first FTC implementation **the virtual-goal LUT with t_eff is the lower-risk choice**.

| Team, season | Aiming | Lead method | Passes | Shooter-point velocity | Latency prediction | Turret / heading feedforward | Fire gate |
|---|---|---|---|---|---|---|---|
| 6328, 2026 alpha | Turret, hood, flywheel | ToF LUT fixed point | 20 | v + ω×r | 30 ms | Filtered finite difference (0.1 s) − ω | Profile at goal position and velocity; distance window |
| 6328, 2026 main | Drive-aimed | Same, plus t_eff drag term (k = 0.375) | 20 | Setpoint velocity moved to launcher | 30 ms | Same, plus heading PD | 10° heading; 0.6–6.0 m; polar-speed cap |
| 254, 2022 | Turret, hood | Radial/tangential split, constant ToF 1.05 s | 0 | Robot velocity | None | −(ω + v_tan/d) | 20 rpm, 0.6° hood, 0.4 m lateral at goal; invalid on clamp or wrap |
| 254, 2024 | Turret, hood | Closed-form intercept with gravity | 0 | Robot field velocity | 50 ms | −(ω + v_tan/d), plus hood rate | 4% RPS; turret ε; RPS cap |
| 604, 2026 | Turret | Subtract turret velocity from stationary launch vector | 0 | v + ω×r | 50 ms, constant acceleration | Central difference at ±5 ms (rate and acceleration) | Tolerances plus a recent multi-tag vision update |
| 971, 2020–22 | Turret | Virtual goal | 3 | Robot velocity | — | atan2 derivative − ω (zeroed in 2022) | — |
| 6940, 2026 | Turret, hood, flywheel | Look-ahead, or 2-D map over (v_radial, d) | 20 | Turret field velocity | — | Profile + LOS term − ω | 0.12 s debounce on flywheel, turret, hood |
| 1706, 2022 | Turret, hood | Virtual goal with acceleration term | 1 | Field velocity + acceleration | Via acceleration factor | — | — |

Sources: rows as cited above, plus [254 Superstructure](https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/frc2022/subsystems/Superstructure.java), [254 AimFactory](https://github.com/Team254/FRC-2024-Public/blob/main/src/main/java/com/team254/frc2024/command_factories/AimFactory.java), [604 TrackAndLaunchCommand](https://github.com/frc604/2026-public/blob/main/FRC-2026/src/main/java/frc/robot/commands/TrackAndLaunchCommand.java), [971 y2022 aiming](https://github.com/frc971/971-Robot-Code/blob/master/y2022/control_loops/superstructure/turret/aiming.cc), [6940 HybridShootCommand](https://github.com/Team6940/2026Rebuilt/blob/master/src/main/java/frc/robot/commands/HybridShootCommand.java).

## Meta^Infinity won its division with one pass, good tables and no fire gate

Meta^Infinity is FTC 18221 from Highland Park, Illinois. At the 2026 World Championship it ranked **5th in the Franklin Division (8-2-0, non-penalty OPR 165.3)**, was the first pick of the division-winning alliance with 30110 Coyote and 10949 MOB Robotics, and took 3rd place for the division's Inspire Award; the World Champion alliance (30030, 21087, 11228) came from Lovelace ([FTC Scout Franklin](https://ftcscout.org/events/2025/FTCCMP1FRAN); [FTC Scout Finals](https://ftcscout.org/events/2025/FTCCMP1)). Its repository, `DecodePublicRelease`, runs Pedro Pathing 2.1.0-alpha.2 with a goBILDA Pinpoint, but it is a single squashed commit from October 2026 carrying off-season values (Michiana, CRI, Bucharest), so it shows the end-of-season state rather than a confirmed Worlds build ([Meta repo](https://github.com/Meta-Infinity/DecodePublicRelease); [Meta RelocalizePosition](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/RelocalizePosition.java)).

The SOTM path in `Robot.update(Pose, Vector)` is short. It computes the static distance from the turret's field position to a fixed aim point deep in the goal corner, (134, 138) or (8, 137), looks up flight time at that static distance, sets `trackPose = goal − velocity × flightTime` with Pedro's field-frame velocity, and then aims the turret and indexes RPM and hood by the distance to `trackPose` ([Meta Robot.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/Robot.java)). There is no iteration, no ω × r term (the turret sits only 0.197 in off center), no acceleration, and a distance gate that the team's own comment admits is always true. SOTM is armed by the driver in teleop only; the autos track with the turret but fire only after the follower stops ([Meta RobotTeleop](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/RobotTeleop.java); [Meta RED24CLOSE](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/RED24CLOSE.java)).

The accuracy lives in the tables. FTCLib `InterpLUT` monotone splines keyed by inches hold RPM, hood servo position, ToF and feeder power, running from **42.45 in → 2610 RPM, hood 0.40, 0.46 s** to **155.2 in → 3900 RPM, hood 0.17, 0.97 s**, with clamp rows at ±1000 in so out-of-range distances saturate. Feeder power drops from 1.0 to 0.5 beyond about 110 in, which keeps far-shot RPM sag consistent between balls ([Meta LookUpTable](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/controlSystems/LookUpTable.java)), and a dedicated tuning OpMode displays distance, commanded and actual RPM, and sample count while rows are collected ([Meta InterpolationTuner](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/testing/InterpolationTuner.java)).

The engineering effort went into the turret. Three continuous-rotation servos are read by an analog absolute encoder unwrapped in software, with the unwrap seam moved 180° away from common aim angles. The wire-limited range runs **−455° to +95° (550°)** with a 5° soft margin, and every target is remapped to the 360°-equivalent closest to the current command inside that range, then slew-limited at 720°/s. A gain-scheduled PID switches gains at 15° of error with ±2° hysteresis, ramps its static-friction term over the first 5° of error to stop chatter, and scales output by 12/V ([Meta Turret.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Turret.java); [Meta PIDController](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/controlSystems/PIDController.java)). The aim angle is atan2(dy, dx) − (θ + ω·0.05 s), so the heading is predicted 50 ms ahead from a yaw rate read straight off the Pinpoint; that is the **only latency term in the stack** ([Meta Robot.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/Robot.java)). The flywheel runs a normalized-kV PIDF when idle and switches to **bang-bang while feeding** (full power below target − 15 RPM, otherwise zero); the hood servo takes the table value plus a driver offset; and the driver alone decides when to fire, with no software readiness check ([Meta Shooter.java](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Shooter.java)).

Other Worlds teams fill in what Meta skipped. **12808 RevAmped** (Goodall Control Award 1st) uses 2-D bilinear tables over (|dx|, |dy|), a Lambert-W predictor of robot coast, and a numerical-derivative turret feedforward, but its 10-iteration loop never feeds the virtual pose back into the air-time lookup, so it is effectively one pass ([RevAmped SimpleShooterMath](https://github.com/junkjunk123/RevAmped-Decode-V2/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/utils/math/projectile/SimpleShooterMath.java)). **15083 Overclock** (Lovelace finalist captain, Design 1st) iterates five times on p + v·t + ½a·t² with acceleration low-passed at α = 0.01 and makes the fire button rumble and refuse when the flywheel or turret is out of tolerance, yet passes zero for angular velocity and so silences its own rotation term ([Overclock Shooter](https://github.com/Xtendera/OverclockDecode/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/overclock/subsystems/Shooter.java); [Overclock TeleOp](https://github.com/Xtendera/OverclockDecode/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/overclock/opmodes/teleop/TeleOp.java)). **21058 Panheya** (Jackson rank 6) uses a constant 0.7 s look-ahead, a yaw lead active only above 2.5 rad/s, and a hood offset that compensates predicted flywheel sag ([Panheya PoseController](https://github.com/DennNuke/ftcpanheya-worlds/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/utils/PoseController.java); [Panheya Hood](https://github.com/DennNuke/ftcpanheya-worlds/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/subsystems/Hood.java)). **25609 LOAD** (Goodall Inspire 2nd) stops to shoot but wrote a latency-compensated fusion localizer that applies camera corrections at the frame timestamp and re-integrates odometry forward ([LOAD FusionLocalizer](https://github.com/LOAD-Robotics-Team-25609/Decode-Robot-Code/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/FusionLocalizer.java)). Outside Worlds, **23641 Cuberobot** has the cleanest solver, a pure `SOTMUtil` function with an iterated quadratic ToF fit, analytic line-of-sight-rate turret feedforward minus ω and flywheel acceleration feedforward from the table slope, although its 0.8 ToF fudge factor signals model mismatch ([Cuberobot SOTMUtil](https://github.com/kleongf/FTC_Decode/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/decodeutil/SOTMUtil.java); [Cuberobot ShootingConstants](https://github.com/kleongf/FTC_Decode/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/decode2026/constants/ShootingConstants.java)). No public code turned up for the World Champion alliance or for the highest-OPR team, 14270 Quantum Robotics ([FTC Scout Ross](https://ftcscout.org/events/2025/FTCCMP1ROSS); [GitHub search](https://api.github.com/search/repositories?q=14270+in:name,description)).

Three lessons follow. Everyone tunes measured tables rather than trusting ballistics; RevAmped's physics solver sits unused beside its table code. Every SOTM team adds a yaw-rate lead, which marks turret lag during chassis spins as the most visible error. And shipped code is full of silent bugs: Meta's always-true gate, RevAmped's non-updating loop, Overclock's zeroed ω, Cuberobot's `Math.signum(error) > 300` that can never fire ([Cuberobot TorqueShooter](https://github.com/kleongf/FTC_Decode/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/decode2026/subsystems/TorqueShooter.java)), 1678's offset multiplied by `times(0.0)` ([1678 planner](https://github.com/frc1678/C2026-Public/blob/main/src/main/java/frc/robot/shooting/shootingplanner/MotionCompensatedShootingPlanner.java)), 971's feedforward clamped to zero, and a possible ω × r sign flip in 6328's alpha branch that differs from the correct form in its main-branch `GeomUtil` ([6328 GeomUtil](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/util/geometry/GeomUtil.java)). The solver should be a pure function with unit tests before it touches a robot.

| Team (Worlds 2026) | Pedro / localizer | Lead method | Effective passes | Turret | Flywheel | Software fire gate |
|---|---|---|---|---|---|---|
| 18221 Meta^Infinity (Franklin winning alliance, 1st pick) | 2.1.0-α2 / Pinpoint | g − v·TOF(d_static) | 1 | 3 CR servos, analog absolute encoder, 550°, 0.05 s yaw lead | PIDF idle, bang-bang feeding | None |
| 12808 RevAmped (Goodall, Control 1st) | 2.1.0-α2 / OctoQuad | 2-D tables, coast predictor | 1 (bug) | Numerical-derivative FF × 0.5, −0.05·ω | 4-mode hybrid with Kalman filter | Too-close cutoff (60 in) |
| 15083 Overclock (Lovelace finalist captain) | 2.1.1 / Pinpoint | p + v·t + ½a·t² | 5 | DC motor, 15° overlap past ±180° | Bang-bang with reverse brake | Flywheel and turret tolerance |
| 21058 Panheya (Jackson rank 6) | 2.1.1 / Pinpoint | Constant 0.7 s look-ahead | — | Yaw lead above 2.5 rad/s | kV, kP, kS | Flywheel error < 60 |
| 25609 LOAD (Goodall, Inspire 2nd) | 2.1.2 / Pinpoint + fusion | None (stops to shoot) | — | atan2 aim | — | Speed < 0.2 |
| 23641 Cuberobot (not at Worlds) | 2.1.0 / Pinpoint | Iterated quadratic ToF × 0.8 | N | Analytic LOS-rate FF − ω | FF with kA from table slope | — |

## Feedforward, wrap logic and fire gates decide whether the math lands

Jared Russell (254) put it bluntly on Chief Delphi: "the math was the easier part of shoot-on-the-move." The hard part is stacking up errors from vision, odometry, latency, control and the shooter ([CD: advice on shooting while moving](https://www.chiefdelphi.com/t/advice-on-shooting-while-moving/405472)). Scaled to a typical BIOBUZZ shot of 48 in with about 0.6 s of flight, and compared against the roughly 6.6 in lateral allowance for POLLEN derived in the next section, the uncompensated terms computed here from first principles look like this:

| Uncompensated error | Calculation | Lateral miss at 48 in |
|---|---|---|
| No lead, 40 in/s tangential | 40 in/s × 0.6 s | ≈ 24 in |
| ω × r ignored, 5 in turret offset, ω = 3 rad/s | 15 in/s × 0.6 s | ≈ 9 in |
| Heading 50 ms stale while spinning at 3 rad/s | 8.6° × 48 in | ≈ 7.2 in |
| Velocity low-pass τ = 0.1 s during 80 in/s² acceleration | 8 in/s × 0.55 s | ≈ 4.4 in |
| ToF 10% wrong at 40 in/s | 0.06 s × 40 in/s | ≈ 2.4 in |
| 1° heading error | 48 in × 0.0175 | ≈ 0.8 in |

Four of these terms (no lead, ignored ω × r, stale heading and over-filtered velocity) can each consume the whole allowance alone, which is why the specification below predicts heading to release, uses turret-point velocity and filters velocity lightly.

**The turret needs feedforward, not just feedback.** WPILib states that feedforward-only turret control is not viable and must be paired with feedback ([WPILib: tuning a turret](https://docs.wpilib.org/en/2024/docs/software/advanced-controls/introduction/tuning-turret.html)), while feedback alone lags a moving setpoint. Differentiating the field line-of-sight angle ψ = atan2(r_y, r_x), with r the vector from turret to (virtual) goal and v the turret point's velocity, gives ψ̇ = (r_y·v_x − r_x·v_y)/|r|² = −v_⊥/d, and the robot-frame turret must turn at **φ̇_ff = ψ̇ − ω**. That is exactly 254's −(ω + v_tan/d) ([254 ShooterSetpoint](https://github.com/Team254/FRC-2024-Public/blob/main/src/main/java/com/team254/lib/util/ShooterSetpoint.java)) and the analytic form Cuberobot implements ([Cuberobot SOTMUtil](https://github.com/kleongf/FTC_Decode/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/decodeutil/SOTMUtil.java)). 6328 instead differentiates the solved angle and smooths it with a 0.1 s moving average, 604 differentiates the whole solver at ±5 ms, and 971 derived the term from atan2 but zeroed lateral velocity (a tank-drive simplification) and clamped the result to zero in 2022 ([6328 alpha LaunchCalculator](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/alpha-bot-turret/src/main/java/org/littletonrobotics/frc2026/subsystems/launcher/LaunchCalculator.java); [604 LaunchCalculator](https://github.com/frc604/2026-public/blob/main/FRC-2026/src/main/java/frc/quixlib/math/LaunchCalculator.java); [971 y2022 aiming](https://github.com/frc971/971-Robot-Code/blob/master/y2022/control_loops/superstructure/turret/aiming.cc)). FRC 5987 reports that adding the ω·r rotation correction alone fixed its misses during turns ([CD: 5987 model-based SOTM](https://www.chiefdelphi.com/t/a-model-based-approach-to-shooting-on-the-move-team-5987-galaxia/518256)).

Wrap handling is the same everywhere: generate the 2π-equivalents of the robot-relative target, keep those inside the soft limits, pick the one closest to the last command, and never fire while the turret is unwinding. 254 in 2022 zeroed the feedforward and invalidated the shot on a wrap ([254 Superstructure](https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/frc2022/subsystems/Superstructure.java)); 5572's fix for partial feeds counts the turret as aimed only when it is not whipping around its travel limit ([5572 issue #230](https://github.com/Frc5572/FRC2026/issues/230)); and Overclock lets the turret run 15° past ±180° so it does not hunt at the seam ([Overclock Shooter](https://github.com/Xtendera/OverclockDecode/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/overclock/subsystems/Shooter.java)).

The flywheel is constrained by the FTC SDK as much as by tuning. In SDK 12.0, `getVelocity()` returns ticks/s measured over the hub's **50 ms window in 20 ticks/s steps** and carried as a 16-bit value, and any `setVelocity()` call silently switches a `RUN_WITHOUT_ENCODER` motor to the hub's own PID ([FTC SDK Hardware 12.0.0](https://repo1.maven.org/maven2/org/firstinspires/ftc/Hardware/12.0.0/); [Road Runner Encoders.kt](https://raw.githubusercontent.com/acmerobotics/road-runner-ftc/master/RoadRunner/src/main/java/com/acmerobotics/roadrunner/ftc/Encoders.kt)). The inspected DECODE teams therefore run their own voltage-compensated feedforward + P each loop with a bang-bang boost while feeding (Meta, Overclock, RevAmped), and REV's starter-bot guidance gates feeding on a window from target − 50 to target + 100 ticks/s ([REV DECODE tips](https://docs.revrobotics.com/ftc-kickoff-concepts/decode-2025-26/programming-tips-and-tricks)). The hood has a trap of its own: `Servo.getPosition()` returns the last commanded value, not a measurement ([Servo javadoc](https://javadoc.io/static/org.firstinspires.ftc/RobotCore/10.3.0/com/qualcomm/robotcore/hardware/Servo.html)), so a readiness check comparing it with the target, as 11138's does, passes one loop after any command ([11138 Shooter.java](https://github.com/FelixYBW/FTC11138/blob/HEAD/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Shooter.java)). Hood readiness has to come from a travel-time model or from Axon analog feedback, which costs nothing extra in the bulk read.

The best FRC fire gates combine mechanism-at-setpoint, solution validity and platform state. 254 in 2022 required the flywheel within 20 rpm, the hood within 0.6°, and the turret within **0.4 m of lateral miss at the goal**, a target-space tolerance that loosens up close and tightens far away; it invalidated the shot on any clamp or wrap and latched firing for 0.25 s ([254 Superstructure](https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/frc2022/subsystems/Superstructure.java)). 6940 debounces readiness for 0.12 s ([6940 HybridShootCommand](https://github.com/Team6940/2026Rebuilt/blob/master/src/main/java/frc/robot/commands/HybridShootCommand.java)), 604 also requires a recent multi-tag vision update ([604 TrackAndLaunchCommand](https://github.com/frc604/2026-public/blob/main/FRC-2026/src/main/java/frc/robot/commands/TrackAndLaunchCommand.java)), 5987 caps robot speed at 2.0 m/s while shooting on the move ([CD: 5987](https://www.chiefdelphi.com/t/a-model-based-approach-to-shooting-on-the-move-team-5987-galaxia/518256)), and 6328 caps speed from the maximum polar velocity its heading loop can follow ([6328 DriveCommands](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/commands/DriveCommands.java)). On the Control Hub, loop hygiene feeds directly into latency: `BulkCachingMode.MANUAL` with one `clearBulkCache()` per loop ([gm0: bulk reads](https://gm0.org/en/latest/docs/software/tutorials/bulk-reads.html)), thresholded write caching, and battery-voltage reads only every few hundred milliseconds, because voltage is not in the bulk packet ([FTC SDK RobotCore 12.0.0](https://repo1.maven.org/maven2/org/firstinspires/ftc/RobotCore/12.0.0/)).

## BIOBUZZ moves the target, not the solver

### The scorable opening faces outward and flips with every TIP

BIOBUZZ has two HIVEs, red and blue, hung side by side from one central structure over tiles C3, C4, D3 and D4. Each HIVE is a bi-stable seesaw holding two CELLs about 18.8 in apart on a pivot **43.95 in** above the tiles; it rests tilted **30°** with one CELL facing up and tips only when enough game pieces are launched into that CELL. The opening is a house-shaped pentagon **20 in wide, 14 in tall and 12 in deep**, spanning **53.5–65.6 in** above the tiles, and the red and blue HIVE centerlines are **25.5 in** apart, leaving only about 5.5 in between adjacent CELLs of opposite colors ([Competition Manual TU05 §9.6, Figs 9-9 to 9-11](https://ftc-resources.firstinspires.org/ftc/game/manual)). The 12.1 in vertical span of a 14 in opening implies a plane tilted 60°, the ballast sits at the opening end, and staged NECTAR rests against the back skin; together these place each opening at the outer end of its CELL with its normal pointing **away from field center and 30° above horizontal** ([Competition Manual TU05](https://ftc-resources.firstinspires.org/ftc/game/manual); [Event Field Setup Guide](https://ftc-resources.firstinspires.org/ftc/archive/2027/field/eventfieldguide)). A ball therefore enters only if it crosses that plane inward, with vertical speed below 1.73 × its horizontal speed toward center (rising at under 60°, or descending), and the opening presents a projected height of **14·cos(γ − 30°) in** to a ball descending at angle γ, largest at 30° of descent and still 12.1 in for a flat shot. The SDK puts each AprilTag cluster's origin at the center of a CELL opening, but because the tags ride on the moving CELLs, FIRST says they are not suitable for absolute localization ([FtcRobotController README](https://github.com/FIRST-Tech-Challenge/FtcRobotController)).

The aim point is two-state. At match start the red **audience** CELL and the blue **rear** CELL are up ([Event Field Setup Guide §11.1](https://ftc-resources.firstinspires.org/ftc/archive/2027/field/eventfieldguide)), and every TIP moves the target to the other end of the HIVE, about 35 in away. The four aim points, in a Pedro-style frame anchored to the walls, appear in the specification table below. They are derived from the manual's dimensioned figures because no official coordinates exist (the SDK sets cluster field positions to zero), they carry the manual's ±1 in tolerance, and they should be measured in the official Onshape CAD and on a real field ([Onshape field CAD](https://cad.onshape.com/documents/a355e772e3d24813de7852ee/w/f106353168f1f92100b81259/e/95d1e1e442b4138cccaf2d73)). The layout is **180° point-symmetric, not mirror-symmetric**, so a red plan maps to blue as (x, y, θ) → (144 − x, 144 − y, θ + π).

### Short, steep shots make robot speed rival ball speed

Because the robot must sit in front of the up CELL, outside the HIVE frame's roughly 49.5 × 39 in footprint and inside the field, turret-to-target horizontal distances fall roughly between **15 and 80 in**, against 40–155 in in Meta^Infinity's DECODE table. With robots capped at 29 in tall ([Competition Manual TU05 R105](https://ftc-resources.firstinspires.org/ftc/game/manual)), the ball must climb 25–45 in. For a ball that must rise Δz and arrive descending at angle γ after horizontal distance d, the drag-free relations derived here are **tan α = 2Δz/d + tan γ** for launch elevation α and **T = √(2(Δz + d·tan γ)/g)** for flight time. With the exit 18.4 in above the tiles (Δz = 40 in) and g = 386.09 in/s²:

| Horizontal distance | Launch elevation for 30° descent | Flight time | Ball horizontal speed | Launch elevation for flat (apex) entry |
|---|---|---|---|---|
| 24 in | 75.7° | 0.53 s | 45 in/s | 73.3° |
| 48 in | 66.0° | 0.59 s | 81 in/s | 59.0° |
| 72 in | 59.4° | 0.65 s | 111 in/s | 48.0° |

These numbers reshape the problem. The hood must reach roughly **60–76° of launch elevation**, the shortest legal shot is d_min = 2Δz/(tan α_max − tan γ), and mounting the exit as high as the 29 in envelope allows flattens every shot. More importantly, a robot at 40 in/s moves at **half the ball's horizontal speed** at 48 in, so the virtual distance swings hard with direction of travel. Computed here with t_eff and c = 0.3 s⁻¹, approaching from 48 in at 40 in/s pulls the virtual distance to about 28 in and demands about a 74° launch, beyond most hoods, so the gate must reject it; receding pushes it to about 72 in at 60°; driving sideways yields about 53 in with a yaw lead near 25°. Feasibility, not convergence, limits SOTM here, since dT/dd ≈ 0.0025 s/in keeps the contraction factor near 0.1. The driving plan should cross the face of the CELL rather than charge it, which matches Oblarg's advice to drive askew of the target ([CD: optimal driving for SOTM](https://www.chiefdelphi.com/t/a-note-on-optimal-driving-for-shoot-on-the-move/512979)).

Tolerances are tight but workable. Aim at the lateral center of the pentagon, where the full 14 in height exists, because the side walls are only 7.61 in tall ([Competition Manual TU05 Fig 9-11](https://ftc-resources.firstinspires.org/ftc/game/manual)). With β the horizontal approach angle off the opening's axis and D the ball diameter (2.8 in POLLEN, 3.6 in NECTAR), the lateral allowance is (20·cos β − D)/2 minus a margin, about **6.6 in for POLLEN and 6.2 in for NECTAR** head-on with a 2 in margin, and it shrinks quickly with β. Since the ball's horizontal travel direction is set by where it is released, not by robot motion, β is a property of the release position; capping it also keeps the final approach inside one's own HIVE's lateral band and away from the opponent CELL 5.5 in away (derived here).

### HIVE rules and the alliance partner turn aiming into state estimation

Calibration requires a HIVE to tip on the 3rd POLLEN tossed onto 3 NECTAR plus 2 POLLEN, and on the 8th POLLEN tossed onto 7 ([Event Field Setup Guide §12](https://ftc-resources.firstinspires.org/ftc/archive/2027/field/eventfieldguide)). Because the up CELL starts each match holding 3 NECTAR, **three POLLEN tip it the first time**; later TIPs need about **eight POLLEN-equivalents**, more than a robot may hold under the 4-element limit (G407), and the guide's mass inequalities put one NECTAR at 1.33–1.67 POLLEN. A TIP is worth **20 points** in AUTO or TELEOP and feeds the POLLINATOR ranking points at 4 and 7 TIPs, while pieces left in an up-facing CELL at the end score only 2 each ([Competition Manual TU05 Table 10-2](https://ftc-resources.firstinspires.org/ftc/game/manual)). The rules bind the shooter in four ways. Launching during a tip may disrupt it, so teams may need to pause their launching (§10.5.1). G417 lets only launches into the up CELL move a HIVE and treats deliberate hits on a CELL's outer faces, or impeding an opponent's TIP, as likely STRATEGIC, though honest misses are not. G403 and G404 forbid mechanism motion in the 8 s AUTO-to-TELEOP transition and after the match ends. And the manual defines no launch zones at all ([Competition Manual TU05 §10.5.1, §11.4](https://ftc-resources.firstinspires.org/ftc/game/manual)).

The robot must therefore know which CELL is up, stop firing while the HIVE tips, and retarget about 35 in away, which usually means relocating to the other half of the field. Counting its own shots cannot establish that state, because the partner fills the same CELL and misses happen. Vision can: an upward-looking camera sees the cluster under each CELL, a cluster with |roll| < 90° marks the scorable CELL, and its ID says which CELL it is ([ftc-docs: AprilTag clusters](https://ftc-docs.firstinspires.org/en/latest/tech_tips/tech-tips/tech-tip-apriltag-clusters/tech-tip-apriltag-clusters.html)). An operator button, like Meta^Infinity's goal-switch toggle, is the robust fallback ([Meta RobotConstants](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/RobotConstants.java)). As of October 2026 no one has published measured tip-transient times, ball masses or bounce-out rates, and Mona Shores lists spill behavior as the part of its simulation it trusts least ([Mona Shores issue #147](https://github.com/Mona-Shores-FTC-Robotics/biobuzz/issues/147)).

The case against SOTM deserves a hearing. FTC posters in September 2026 advised a static shooter and fixed sweet spots before turrets and shoot-while-moving ([CD: FTC auto-aiming](https://www.chiefdelphi.com/t/ftc-auto-aiming/524081)), and FRC teams report dropping SOTM to avoid integration problems ([CD poll](https://www.chiefdelphi.com/t/poll-how-are-you-shooting-on-the-move/514641)). The specification below degrades gracefully to that strategy, because at zero velocity it reduces to ordinary turret auto-aim.

### Pedro Pathing 3.0.2 supplies field velocity but no acceleration

Pedro 3 (released 2026-09-10, with 3.0.2 on 2026-10-16) is a rewrite, so v2 code from DECODE repos will not compile against it ([Pedro releases](https://github.com/Pedro-Pathing/PedroPathing/releases)). In the 3.0.2 jar, `follower.pose()`, `follower.velocity()` and `follower.twist()` delegate to the localizer; `Velocity` has public `vx` and `vy` in the field frame (in/s) and `omega` (rad/s, CCW-positive), `Twist` is the robot-frame equivalent (x forward, y left), heading is normalized to **[0, 2π)** and must be wrapped with `Angle.normalizeSigned` before taking differences, and v2's `getAcceleration()` is gone with no replacement ([core-3.0.2.jar](file:///home/andrei0204/.gradle/caches/modules-2/files-2.1/com.pedropathing/core/3.0.2/971d7ef36fa68b8499e867efff8e915c2a7bc3e6/core-3.0.2.jar); [Pedro docs: velocity types](https://pedropathing.com/docs/pathing/reference/velocity)). `follower.update()` must run exactly once per loop and is the only call that refreshes the localizer. For the Pinpoint, Pedro passes `getVelX`/`getVelY`/`getHeadingVelocity` through unfiltered as field-frame velocity, while the SDK javadoc calls `getVelX` the "forward" velocity, an ambiguity a drive test must settle ([revhub sources](file:///home/andrei0204/.gradle/caches/modules-2/files-2.1/com.pedropathing/revhub/3.0.2/764a1913a7175783a47cf7832b0c1aaab72ccfaf/revhub-3.0.2-sources.jar); [Road Runner PinpointLocalizer](https://github.com/acmerobotics/road-runner-quickstart/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/PinpointLocalizer.java)). Two- and three-wheel localizers compute velocity as a raw one-loop finite difference, so differencing it again for acceleration amplifies noise badly.

The only `Follower` constructor is `new Follower(localizer, drivetrain, algorithm)`, localizer first; the comment in this project's `Constants.java` stub has the order wrong ([local Tests.java](file:///home/andrei0204/Coding/Robo/FtcRobotController-12.0/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/procedures/Tests.java)). Teleop calls `follower.manual(forward, lateral, turn)` every loop before `update()` ([Pedro docs: TeleOp usage](https://pedropathing.com/docs/pathing/guide/teleop-usage)). `setPose()` keeps velocity, so a mid-match wall relocalization does not upset turret aiming, and `FusionLocalizer.addMeasurement(pose, nanoTime)` applies latency-correct corrections but silently drops measurements outside its history buffer and publishes the raw odometry velocity without rotating it into the corrected frame ([core-3.0.2.jar](file:///home/andrei0204/.gradle/caches/modules-2/files-2.1/com.pedropathing/core/3.0.2/971d7ef36fa68b8499e867efff8e915c2a7bc3e6/core-3.0.2.jar)).

## Implementation-ready specification for a BIOBUZZ turret shooter

This specification is the virtual-goal LUT method with every correction the evidence supports. Steps 3–7 form a **pure function** of (pose, velocity, acceleration, HIVE state, ball type) that can be unit-tested offline, and every constant in `CAPS` appears in the measurement table at the end.

### Frames, units and signs

| Quantity | Symbol | Unit | Convention |
|---|---|---|---|
| Field frame | F | in | Seeded with `follower.setPose()`. Origin at the corner where the audience wall meets the red alliance's wall; +x toward the blue wall (tile columns A→F); +y away from the audience toward the rear (scoring) wall (rows 1→6); z up from tile top; field center (72, 72). This equals Pedro's documented FTC→Pedro conversion of the official frame if +X_FTC points toward the audience, as the ftc-docs rule implies for red-on-the-left fields ([Pedro coordinates](https://pedropathing.com/docs/pathing/reference/coordinates); [ftc-docs field frame](https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html)). |
| Robot frame | B | in | +x forward, +y left (Pedro `Twist`) |
| Heading | θ | rad | CCW-positive from +x; Pedro returns [0, 2π) |
| Yaw rate | ω | rad/s | CCW-positive, `velocity().omega` |
| Robot velocity | v = (v_x, v_y) | in/s | Field frame, `velocity().vx/vy` |
| Turret angle | φ | rad | Robot frame, CCW-positive from robot +x, kept continuous (never wrapped) |
| Shot field yaw | ψ = θ + φ | rad | Field frame |
| Turret axis offset | r_B = (R_X, R_Y) | in | Turret rotation axis in robot frame |
| Flywheel speed | Ω | ticks/s | `DcMotorEx.getVelocity()` |
| Hood | h | servo units [0, 1] | Tables store servo units directly |
| Flight time | T | s | Ball leaves hood → ball crosses the CELL opening plane |
| Distances | d, d_v | in | Horizontal, turret axis → real / virtual aim point |
| Angle wrap | wrap(x) | rad | `Angle.normalizeSigned(x)` → [−π, π) |

### Targets and the HIVE-state machine

| Own alliance | Up CELL | Aim point g = (x, y) [in] | Horizontal outward normal n̂ | Shoot from | Cluster IDs |
|---|---|---|---|---|---|
| Red | AUDIENCE (match start) | (59.25, 54.7) | (0, −1) | y < g_y | 34–37 |
| Red | REAR | (59.25, 89.3) | (0, +1) | y > g_y | 30–33 |
| Blue | AUDIENCE | (84.75, 54.7) | (0, −1) | y < g_y | 38–41 |
| Blue | REAR (match start) | (84.75, 89.3) | (0, +1) | y > g_y | 42–45 |

The aim height z_g ≈ 58.4 in is implicit in the tables because exit and target heights are both fixed. The y values come from two geometric fits (53.9–55.5 and 88.5–90.1 in) and the x values assume the structure is centered, as the setup guide requires; all carry the manual's ±1 in tolerance ([Competition Manual TU05 Figs 9-10, 9-17](https://ftc-resources.firstinspires.org/ftc/game/manual)).

```
enum Cell { AUDIENCE, REAR }
state:  upCell, confidence ∈ {VISION, OPERATOR, ASSUMED, UNKNOWN},
        tTip (s), fillEq (POLLEN-equivalents, own estimate), tLastShot

AUTO start:   upCell = (alliance == RED) ? AUDIENCE : REAR; confidence = ASSUMED;
              fillEq = 3·RHO_N            // 3 staged NECTAR, RHO_N ≈ 1.5 POLLEN-eq
TELEOP start: restore upCell, confidence, fillEq from static storage written every AUTO loop

every loop:
  if (vision sees own-alliance cluster with |roll| < 90° for ≥ N_VIS frames
      && its cell != upCell)                       flip(VISION)
  if (operator TIP button)                         flip(OPERATOR)   // UNDO button flips back
  tipping   = (now − tTip) < T_TIP_HOLD
  tipLikely = fillEq ≥ TIP_EQ − 1                  // TIP_EQ ≈ 7.5 ± 1 POLLEN-eq

after each fired shot of type k:  fillEq += P_MAKE·(k == NECTAR ? RHO_N : 1); tLastShot = now

flip(src): upCell = other(upCell); tTip = now; confidence = src;
           fillEq = 0                              // newly-up CELL starts ~empty
```

The fill estimate never flips the target by itself; it only triggers a **tip-check pause** of T_CHECK after each shot once a TIP is likely, so the robot does not fire into a HIVE that has started to move. In AUTO, the third POLLEN should produce the first TIP, so the robot fires three and holds, because a fourth ball cannot score from the same half once the HIVE flips; partners must coordinate because both fill the same CELL. The HIVE state should persist across the AUTO→TELEOP boundary exactly as Meta^Infinity persists pose and turret angle ([Meta RobotTeleop](https://github.com/Meta-Infinity/DecodePublicRelease/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/RobotTeleop.java)).

### The per-loop pipeline

```java
// ---- 0. Hardware reads (BulkCachingMode.MANUAL set once at init) ----
for (LynxModule hub : hubs) hub.clearBulkCache();
double now = System.nanoTime() * 1e-9;
double dt  = now - tPrev; if (dt <= 0 || dt > 0.25) dt = 0.02; tPrev = now;
double phiMeas    = turretAngleRad();          // continuous, CCW+, 0 = robot +x (includes PHI_0)
double phiDotMeas = turretRateRadPerSec();
double flyMeas    = flywheel.getVelocity();    // ticks/s; NEVER call setVelocity() on this motor
if (now - tVolt > 0.2) { vBat += 0.3 * (voltageSensor.getVoltage() - vBat); tVolt = now; }

// ---- 1. Drive, then exactly one localizer update ----
follower.manual(fwd, lat, turn);               // TeleOp (verify signs); AUTO uses follow(path)
follower.update();
MotionState s = follower.localizer.state();    // one consistent snapshot
double x = s.pose().x(), y = s.pose().y();     // in
double th = s.pose().heading();                // rad, [0, 2π)
double vx = s.velocity().vx, vy = s.velocity().vy;   // in/s, FIELD frame
double om = s.velocity().omega;                // rad/s, CCW+

// ---- 2. Acceleration (Pedro v3 has none) ----
double kA = 1 - Math.exp(-dt / TAU_A);         // TAU_A = 0.05–0.10 s
ax += kA * ((vx - vxPrev) / dt - ax);  ay += kA * ((vy - vyPrev) / dt - ay);
clampMagnitude(ax, ay, A_CLIP);  vxPrev = vx; vyPrev = vy;

// ---- 3. Predict to ball release ----
double L   = L_SENSE + L_FEED;                 // s
double px  = x + vx*L + 0.5*ax*L*L,  py = y + vy*L + 0.5*ay*L*L;
double ux  = vx + ax*L,              uy = vy + ay*L;
double thR = th + om*L;                        // heading at release

// ---- 4. Turret axis: position and rigid-body velocity v + ω × r ----
double rFx = R_X*Math.cos(thR) - R_Y*Math.sin(thR);
double rFy = R_X*Math.sin(thR) + R_Y*Math.cos(thR);
double sx = px + rFx,       sy = py + rFy;     // in
double vsx = ux - om*rFy,   vsy = uy + om*rFx; // in/s

// ---- 5. Target and ball type ----
Target tg = TARGET[alliance][hive.upCell];     // tg.gx, tg.gy, tg.nx, tg.ny
BallTables tb = TABLES[nextBallType()];        // POLLEN or NECTAR (color sensor)

// ---- 6. Time-of-flight fixed point on the virtual goal ----
double t = tb.tof(Math.hypot(tg.gx - sx, tg.gy - sy));
boolean converged = false;
for (int i = 0; i < 10; i++) {
    double te = teff(t, tb.c);                 // (1 - e^{-c t})/c ; = t when c == 0
    double tn = tb.tof(Math.hypot(tg.gx - vsx*te - sx, tg.gy - vsy*te - sy));
    if (Math.abs(tn - t) < 1e-3) { t = tn; converged = true; break; }
    t = (i < 3) ? tn : t + 0.5*(tn - t);       // damp if still oscillating
}
double te = teff(t, tb.c);
double rx = tg.gx - vsx*te - sx, ry = tg.gy - vsy*te - sy;   // turret → virtual goal
double dv = Math.hypot(rx, ry);                              // in

// ---- 7. Setpoints and feedforward ----
double psi      = Math.atan2(ry, rx);                         // field yaw of relative shot
double phiDes   = Angle.normalizeSigned(psi - thR) + PHI_TRIM;
double hoodCmd  = clip(tb.hood(dv) + HOOD_TRIM, H_MIN, H_MAX);
double flyCmd   = tb.fly(dv) + FLY_TRIM;                      // ticks/s
double psiDot   = (ry*vsx - rx*vsy) / (dv*dv);                // rad/s, LOS rate
double phiDotFF = psiDot - om;                                // rad/s, turret-frame rate
double flyDotFF = tb.flySlope(dv) * (-(rx*vsx + ry*vsy) / dv);// ticks/s², from d(dv)/dt

// ---- 8. Turret selection and control laws, 9. gate, 10. feed (below) ----
```

Each step has a lineage. The clock guard in step 0 copies 11138's protection against garbage-collection pauses ([11138 README](https://github.com/FTC11138/Decode-Premier-Ivy)), and step 1 takes pose and velocity from one snapshot after exactly one update. Step 2 filters acceleration lightly because a heavy filter lags during acceleration and that lag converts directly into miss distance. Step 3 combines 6328's 30 ms phase delay, 254's and 604's 50 ms prediction, 604's constant-acceleration extrapolation and Meta^Infinity's 50 ms heading lead, and adds L_FEED (feed command until the ball leaves the hood) because a ball fired now leaves later. Step 4 is 6328's `transformVelocity`, vx − ω(R_X sinθ + R_Y cosθ) and vy + ω(R_X cosθ − R_Y sinθ) ([6328 GeomUtil](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/util/geometry/GeomUtil.java)). Step 6 is 6328's iteration with its drag correction plus a convergence test and damping, and step 7 uses the analytic line-of-sight rate minus ω for the turret and Cuberobot's table-slope term for flywheel acceleration. Hood and flywheel are read at the **virtual** distance, which is what makes radial motion change RPM and hood automatically, and every table is keyed by turret-axis distance, never robot-center distance, the mistake 5000 made ([5000 TurretCalculator](https://github.com/hammerheads5000/2026Rebuilt/blob/main/src/main/java/frc/robot/subsystems/turret/TurretCalculator.java)).

### Turret selection and control laws

```java
// 8a. Wrap-aware turret target (continuous angle, cable limits may span > 360°)
double lo = PHI_MIN + (firing ? M_FIRE : M_TRACK), hi = PHI_MAX - (firing ? M_FIRE : M_TRACK);
double best = Double.NaN;
for (int k = -2; k <= 2; k++) {
    double c = phiDes + 2*Math.PI*k;
    if (c >= lo && c <= hi && (Double.isNaN(best) || Math.abs(c - phiCmdPrev) < Math.abs(best - phiCmdPrev))) best = c;
}
boolean turretInRange = !Double.isNaN(best);
double phiCmd = turretInRange ? best : clip(nearestEquivalent(phiDes, phiCmdPrev), lo, hi);
boolean whipping = Math.abs(phiCmd - phiMeas) > PHI_WHIP;     // ~0.5 rad: unwinding, never fire
phiCmdPrev = phiCmd;

// 8b-i. Turret motor (or CR-servo) power, +power → +φ (CCW)
double e = phiCmd - phiMeas;
double sgn = Math.abs(phiDotFF) > 0.05 ? Math.signum(phiDotFF)
           : Math.signum(e) * Math.min(Math.abs(e) / E_RAMP, 1.0);     // ramped kS (Meta)
double uT = (12.0/vBat) * (KS_T*sgn + KV_T*phiDotFF + KA_T*phiDDotFF)
          + KP_T*e + KD_T*(phiDotFF - phiDotMeas);
uT = zeroIfPushingPastLimit(clip(uT, -U_T_MAX, U_T_MAX), phiMeas);

// 8b-ii. Flywheel: RUN_WITHOUT_ENCODER, FLOAT, custom FF + P with bang-bang assist
double eF = flyCmd - flyMeas;
double uF = (12.0/vBat) * (KS_F + KV_F*flyCmd + KA_F*flyDotFF) + KP_F*eF;
if (feeding && eF > BAND_F) uF = 1.0;
uF = clip(uF, 0.0, 1.0);

// 8b-iii. Hood servo (setPwmRange(500, 2500) for goBILDA/Axon), write-cached
if (Math.abs(hoodCmd - hoodLast) > 0.002) {
    hood.setPosition(hoodCmd);
    tHoodReady = now + Math.abs(hoodCmd - hoodLast) * SERVO_DEG_PER_UNIT / SERVO_DEG_PER_S + T_SETTLE;
    hoodLast = hoodCmd;
}
```

The two turret margins copy 6328's trick of using a narrower range while merely tracking, so the turret unwinds between bursts rather than during one ([6328 alpha Turret](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/alpha-bot-turret/src/main/java/org/littletonrobotics/frc2026/subsystems/launcher/turret/Turret.java)). φ̈_ff (`phiDDotFF`) is a lightly filtered finite difference of φ̇_ff and can stay at zero until kV is tuned. The flywheel law follows the SDK constraints and team practice above, and the hood uses a travel-time model because a plain servo cannot report its position; if the hood servo has an analog feedback wire, a measured angle should replace the model.

### The fire gate

```java
double fx = sx - tg.gx, fy = sy - tg.gy, fd = Math.hypot(fx, fy);
double front  = fx*tg.nx + fy*tg.ny;                 // in; > 0 means in front of the opening
double cosB   = front / fd;                          // cos of horizontal approach angle β
double eLat   = (20.0*cosB - tb.D)/2.0 - M_LAT;      // in; D = 2.8 POLLEN, 3.6 NECTAR
double tolPhi = Math.atan2(Math.max(eLat, 0), dv);   // 254-style "miss at the goal" tolerance

boolean ready =
      converged && dv >= tb.dMin && dv <= tb.dMax && !tb.clamped(dv) && turretInRange   // solution
   && hive.confidence != UNKNOWN && !hive.tipping
   && !(hive.tipLikely && now - hive.tLastShot < T_CHECK)                              // HIVE state
   && (phase == AUTO || phase == TELEOP)                                                // G403/G404
   && front >= S_FRONT && cosB >= Math.cos(BETA_MAX)                                    // geometry, G417
   && Math.abs(phiCmd - phiMeas) < tolPhi && !whipping
   && Math.abs(phiDotMeas - phiDotFF) < TOL_PHIDOT                                      // turret
   && Math.abs(flyCmd - flyMeas) < tb.tolFly(dv)                                        // flywheel
   && now >= tHoodReady                                                                 // hood
   && Math.hypot(vx, vy) <= V_MAX && Math.hypot(ax, ay) <= A_MAX && Math.abs(om) <= OM_MAX
   && nextBallTypeKnown();

readyCount = ready ? readyCount + 1 : 0;
boolean fire = readyCount >= N_DB || (feeding && !hive.tipping && !whipping);   // latch a started feed
```

The turret tolerance is 254's target-space idea expressed with BIOBUZZ clearances, so it tightens automatically at long range and oblique approach. The flywheel tolerance `tolFly(dv)` should be about half of the measured RPM "make window" at each table distance (the RPM range over which stationary shots still score), which replaces fixed thresholds that a moving setpoint may never meet ([5572 issue #230](https://github.com/Frc5572/FRC2026/issues/230)). The `front` and `cosB` checks encode the outward-facing opening and keep shots off the CELL's outer faces and away from the opponent HIVE (G417). When the driver holds the shoot button and `ready` fails only on the motion or approach checks, the robot should scale down the manual drive powers or rumble, as Overclock does ([Overclock TeleOp](https://github.com/Xtendera/OverclockDecode/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/overclock/opmodes/teleop/TeleOp.java)); steering across the CELL face recovers most failed approach shots.

### Offline tests before the robot moves

Each test below runs as a plain JUnit case against the pure solver and targets a bug class found in published code.

| Test | Input | Required result |
|---|---|---|
| Zero motion | v = 0, ω = 0 | g_v = g; φ̇_ff = 0 |
| Pure rotation (ω × r sign) | r_B = (5, 0) in, θ = 0, v = 0, ω = 2 rad/s | v_s = (0, +10) in/s |
| Feedforward sign | Turret at origin, g = (48, 0), v_s = (0, 30) in/s | ψ̇ = −0.625 rad/s (clockwise) |
| Fixed-point closure | Any converged case | \|TOF(d_v) − t\| < 1 ms; g_v + v_s·t_eff = g |
| Alliance symmetry | Blue solver on the 180°-rotated red state | Same turret angle and virtual distance |
| Wrap continuity | φ_des swept through ±π | φ_cmd continuous and inside limits; unwinds only when not firing |
| Convergence and validity | Random velocities up to drivetrain maximum over the shooting region | Converges within 10 passes; out-of-table cases report invalid, never NaN |

### Calibration order

| Stage | Procedure | Pass criterion |
|---|---|---|
| 1. Localization | Run Pedro AutoTune for Pinpoint, Mecanum and Foresight. Push the robot along +x and check vx > 0; turn to θ = 90°, drive forward and check vy > 0 with vx ≈ 0; spin CCW and check ω > 0. Log velocity noise at rest and at constant speed, and the loop-time distribution. | Frames confirmed; σ_v and dt known |
| 2. Geometry | Measure r_B, exit height, turret zero φ_0, cable limits and ticks per turret radian; map hood servo units to travel; tape-measure all four aim points on a real field and compare to CAD. | Aim points within ±0.5 in of measured |
| 3. Mechanism loops | Flywheel kS and kV from a P = 0 steady-state sweep, then kP and BAND_F for recovery, logging dip and recovery per shot. Turret kS, kV and kA from ramps, then kP and kD from steps, recording maximum rate. Hood settle time vs. travel. | Flywheel back in its make window before the next feed; turret ramp error < 1° |
| 4. Static tables, per ball type | At 6–10 head-on distances from d_min to d_max, pick the hood giving roughly 20–40° of descent into the opening (aperture ≥ 13 in), tune RPM to center, fire ≥ 5 shots, and record the RPM and hood make windows. Measure ToF with 240 fps video from leaving the hood to crossing the opening plane (≈ 4 ms per frame) and fit a smooth monotone curve. Validate at midpoints and at β = 30°, 45° and 60°. | ≥ 90% stationary makes across the range; β_max chosen |
| 5. Latency | Spin test: hold the turret on target while the chassis spins at constant ω at a fixed distance, and fire; a lateral miss m implies a latency error δL ≈ m/(d·ω), so adjust L_SENSE and check turret error against KV_T. Measure L_FEED from the feed-command timestamp to the flywheel speed dip. | Spin-test misses centered |
| 6. Drag and lead | Drive-by tests at 20 and 40 in/s, tangential, toward and away. Tangential bias tunes L and the turret feedforward; radial long/short bias tunes c (start at 0.3 s⁻¹) and the ToF fit. | Moving-shot bias < 1 in |
| 7. Gates | Set tolFly and tolPhi margins from the make windows and N_DB = 2–3; set V_MAX, A_MAX and OM_MAX from the speed at which moving hit rate falls below the team's threshold. | Gate passes ≥ 90% makes |
| 8. HIVE | On a calibrated HIVE, video the tip transient and settling to set T_TIP_HOLD and T_CHECK; verify the vision and operator flip paths and the AUTO three-shots-then-hold sequence. | No shots into a tipping HIVE |
| 9. Every event | Re-measure aim points (structure centering is ±1 in), confirm the event's tip counts, check the ball batch. | Static makes restored |

Stages 4–6 follow FRC practice of slow-motion ToF measurement and empirical drag tuning ([6328 build thread #272](https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2026-build-thread/509595/272); [CD: 3061 calibration](https://www.chiefdelphi.com/t/huskie-physics-shoot-on-the-move-with-equations/522805)), and stage 3 follows REV's feedforward-first flywheel recipe ([REV DECODE tips](https://docs.revrobotics.com/ftc-kickoff-concepts/decode-2025-26/programming-tips-and-tricks)).

### Values that must be measured

| Value | Symbol | Unit | How to measure | Starting value |
|---|---|---|---|---|
| Four CELL aim points | g | in | CAD, then tape on a real field per state | Table above |
| Opening tilt / normal | n̂ | — | CAD or inclinometer | 30° up, outward |
| Field interior size | — | in | Tape | 144 |
| Tip threshold at the event | TIP_EQ | POLLEN-eq | Field calibration check | 7.5 ± 1 |
| NECTAR/POLLEN mass ratio | RHO_N | — | Scale | 1.5 (bounds 1.33–1.67) |
| Ball diameters | D | in | Calipers, several balls | 2.8 / 3.6 |
| Tip transient and settle | T_TIP_HOLD | s | 240 fps video of a tip | Measure; no published value |
| Post-shot tip-check pause | T_CHECK | s | ToF + tip onset from video | Measure |
| Turret axis offset | R_X, R_Y | in | CAD or ruler from robot center | — |
| Exit height | z₀ | in | Ruler | As high as the 29 in envelope allows |
| Turret zero, cable limits | PHI_0, PHI_MIN, PHI_MAX | rad | Jig or hard stop | — |
| Turret margins | M_TRACK, M_FIRE, PHI_WHIP | rad | Tuning | 20°, 0°, 0.5 rad |
| Turret gains | KS_T, KV_T, KA_T, KP_T, KD_T, E_RAMP | — | Ramp and step tests | E_RAMP = 5° |
| Flywheel gains | KS_F, KV_F, KA_F, KP_F, BAND_F | — | Steady-state sweep, recovery tests | BAND_F ≈ 100–200 ticks/s |
| Flywheel dip and recovery per shot | — | ticks/s, s | Logs | — |
| Hood servo travel and speed | SERVO_DEG_PER_UNIT, SERVO_DEG_PER_S, T_SETTLE | °, °/s, s | Video or feedback wire | 300° (500–2500 µs), spec speed, 0.03 s |
| Sensing latency | L_SENSE | s | Spin test | 0.03–0.05 |
| Feed latency | L_FEED | s | Feed-command timestamp → flywheel dip | Measure |
| Acceleration filter and clip | TAU_A, A_CLIP | s, in/s² | Noise logs | 0.05–0.10 s |
| Shot tables per ball type | hood(d), fly(d), tof(d) | servo, ticks/s, s | Stage 4 | 6–10 rows each |
| Table range | dMin, dMax | in | Stage 4; d_min ≈ 2Δz/(tan α_max − tan γ) | ~15–80 in |
| Drag constant | c | s⁻¹ | Stage 6 | 0.3 |
| RPM make window | tolFly(d) | ticks/s | Stage 4 | ½ of window |
| Approach cone, front distance | BETA_MAX, S_FRONT | rad, in | Stage 4 oblique tests | 45°, d_min |
| Lateral margin | M_LAT | in | Stage 7 | 2 |
| Turret rate tolerance | TOL_PHIDOT | rad/s | Stage 7 | — |
| Motion caps | V_MAX, A_MAX, OM_MAX | in/s, in/s², rad/s | Stage 7 hit-rate curves | Start low; raise with data |
| Gate debounce | N_DB | loops | Stage 7 | 2–3 |
| Vision confirmation | N_VIS | frames | HIVE stage | 3 |
| Shot make probability for fill estimate | P_MAKE | — | Match logs | 0.8 |

## Conclusion

BIOBUZZ shifts where shoot-on-the-move wins and loses points. In DECODE the hard problem was precision against a large fixed goal, and Meta^Infinity showed that good tables plus a one-line virtual goal were enough to win a division. In BIOBUZZ the solver is the solved part: the scoring zone is half the field and flips with every TIP, the partner changes the target's state, and a 20-point TIP can be spoiled by one badly timed ball, so the **HIVE-state machine, the tip hold-off and the approach-cone gate** will decide more points than the last inch of lead accuracy. The short, steep geometry also inverts the FRC intuition: with the robot moving at half the ball's horizontal speed, feasibility (hood range and approach direction) limits moving shots long before iteration count or convergence does, and the practical driving rule is to cross the face of the up CELL rather than charge it.

The remaining uncertainty is empirical rather than mathematical, and it is concentrated in a handful of quantities nobody has published yet: exact CELL coordinates, the Pinpoint velocity frame, ball masses, tip-transient time and bounce-out behavior at different entry angles. A team that measures those in the first weeks, builds the solver as a tested pure function and starts from the virtual-goal LUT method will have a sound stack, with velocity subtraction held back as an upgrade that preserves the stationary entry angle exactly if bounce-outs turn out to depend on it.
