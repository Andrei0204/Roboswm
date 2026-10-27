# FRC "Shoot on the Move" (SOTM): Code-Level Notes from Top Teams' Open-Source Robot Code

Research date: 2026-10-21. These notes come from reading the public source files listed below, fetched from GitHub and Chief Delphi (CD). The season is labelled on every repo. Code excerpts are short and were transcribed by a fetch tool, so re-check any line before copying it into robot code. Notation used throughout: `v` = robot (or shooter-point) field velocity, `ω` = robot yaw rate, `d` = horizontal distance to the goal, `ToF` = time of flight.

---

## Q1. Which FRC teams/repos have working shoot-on-the-move code? (repos, files, seasons)

### Takeaway
Several teams have working, readable SOTM code. The best turret, hood and flywheel references are 254 (2022 Rapid React and 2024 Crescendo), 6328's 2026 REBUILT turreted "Alpha Bot" (branch `alpha-bot-turret`), 604 (2026), 971 (shared aiming library) and several smaller 2026 teams (6940, 5572, 5000, 5987). On 6328's 2026 competition robot and on 1678's and 2910's 2026 robots, the drivetrain aims instead of a turret. Team 254 had not published 2026 code as of this research.

### Cited Findings
**2022 Rapid React**
- **254 (2022)**, turret, hood and flywheel. The SOTM math is in `src/main/java/com/team254/frc2022/shooting/ShootingUtil.java` (`getShootingParameters(...)`). Orchestration is in `subsystems/Superstructure.java` (`setShooterSetpoints`, `getHoodAndRpm`, `handleTurretLimit`, `handleShooting`). Tolerances are in `com/team254/lib/util/ShootingParameters.java` and constants in `Constants.java`. — [ShootingUtil](https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/frc2022/shooting/ShootingUtil.java), [Superstructure](https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/frc2022/subsystems/Superstructure.java), [ShootingParameters](https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/lib/util/ShootingParameters.java), [Constants](https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/frc2022/Constants.java)
- **971 (2020–2022)**, C++. A shared turret aiming library, `frc971/control_loops/aiming/aiming.cc` (`AimerGoal`, `IterateVirtualGoal`), is used by `y2022/control_loops/superstructure/turret/aiming.cc` (`Aimer::Update`). — [aiming.cc](https://github.com/frc971/971-Robot-Code/blob/master/frc971/control_loops/aiming/aiming.cc), [y2022 aiming.cc](https://github.com/frc971/971-Robot-Code/blob/master/y2022/control_loops/superstructure/turret/aiming.cc)
- **1706 (2022)**, turret and hood. The team posted a `ShootWhileMove` command on CD (Apr 27, 2022). No repo or file path was linked. — [CD: Shoot While Move Code (1706)](https://www.chiefdelphi.com/t/shoot-while-move-code-1706/410494)
- **1690 Orbit (2022, 2024)**: closed source. The team presented the SOTM math in a software session video ([youtube.com/watch?v=vUtVXz7ebEE](https://www.youtube.com/watch?v=vUtVXz7ebEE)), which a CD post recommended. Per another CD post, the slides described a shooter trajectory-optimization formulation but did not include an implementation. — [CD: SOTM from the code perspective](https://www.chiefdelphi.com/t/shoot-on-the-move-from-the-code-perspective/511815), [CD: Shooting on the Move Thoughts](https://www.chiefdelphi.com/t/shooting-on-the-move-thoughts/512169)

**2024 Crescendo**
- **254 (2024)**, turret and hood. The SOTM solver is `src/main/java/com/team254/lib/util/ShooterSetpoint.java` (closed-form intercept, turret and hood feedforward, validity). The readiness gate is `frc2024/command_factories/AimFactory.java` (`onTarget`). Turret commanding is in `TurretFactory.aimTurretToPose`. Prediction is in `frc2024/RobotState.java` (`getPredictedFieldToRobot`). — [ShooterSetpoint](https://github.com/Team254/FRC-2024-Public/blob/main/src/main/java/com/team254/lib/util/ShooterSetpoint.java), [AimFactory](https://github.com/Team254/FRC-2024-Public/blob/main/src/main/java/com/team254/frc2024/command_factories/AimFactory.java), [TurretFactory](https://github.com/Team254/FRC-2024-Public/blob/main/src/main/java/com/team254/frc2024/command_factories/TurretFactory.java), [RobotState](https://github.com/Team254/FRC-2024-Public/blob/main/src/main/java/com/team254/frc2024/RobotState.java)
- **6328 (2024, "Presto")**, drive-aimed with an arm/pivot. Code is `RobotState.getAimingParameters()` (pose lookahead with no ToF) and `getSuperPoopAimingParameters()` (flywheel radial-velocity compensation). — [RobotState 2024](https://github.com/Mechanical-Advantage/RobotCode2024Public/blob/main/src/main/java/org/littletonrobotics/frc2024/RobotState.java)

**2026 REBUILT** (fuel into hub)
- **6328 (2026, "Darwin")**, main branch. Drive-aimed hood and flywheel launcher: `subsystems/launcher/LaunchCalculator.java`, `util/geometry/GeomUtil.java` (`transformVelocity`), `commands/DriveCommands.java` (`joystickDriveWhileLaunching`). — [LaunchCalculator (main)](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/subsystems/launcher/LaunchCalculator.java), [GeomUtil](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/util/geometry/GeomUtil.java), [DriveCommands](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/commands/DriveCommands.java)
- **6328 (2026) Alpha Bot, turreted**, branch `alpha-bot-turret`. This is the closest analog to the target FTC robot (turret, flywheel and hood): `subsystems/launcher/LaunchCalculator.java` plus `subsystems/launcher/turret/Turret.java`. — [LaunchCalculator (alpha)](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/alpha-bot-turret/src/main/java/org/littletonrobotics/frc2026/subsystems/launcher/LaunchCalculator.java), [Turret (alpha)](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/alpha-bot-turret/src/main/java/org/littletonrobotics/frc2026/subsystems/launcher/turret/Turret.java). The design is described in CD post #272 of the build thread (nharnwal, Feb 5, 2026). — [6328 build thread #272](https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2026-build-thread/509595/272). An early-January version used `subsystems/shooter/ShotCalculator.java`, per a CD post. — [CD 511815](https://www.chiefdelphi.com/t/shoot-on-the-move-from-the-code-perspective/511815)
- **604 Quixilver (2026)**, single-stream turret. Code is `FRC-2026/src/main/java/frc/quixlib/math/LaunchCalculator.java` and `frc/robot/commands/TrackAndLaunchCommand.java`, with a CSV launch table (`src/test/java/frc/quixlib/math/test_launch_table.csv`). — [LaunchCalculator](https://github.com/frc604/2026-public/blob/main/FRC-2026/src/main/java/frc/quixlib/math/LaunchCalculator.java), [TrackAndLaunchCommand](https://github.com/frc604/2026-public/blob/main/FRC-2026/src/main/java/frc/robot/commands/TrackAndLaunchCommand.java), [CD release thread](https://www.chiefdelphi.com/t/team-604-quixilver-2026-robot-cad-and-code-release/524289)
- **1678 (2026, "Limestone")**, drive-aimed. Code is `src/main/java/frc/robot/shooting/shootingplanner/MotionCompensatedShootingPlanner.java`, `shooting/Shooting.java`, `shooting/setpoints/ShootingSetpoints.java` and `shooting/verification/ShotVerifier.java`. — [MotionCompensatedShootingPlanner](https://github.com/frc1678/C2026-Public/blob/main/src/main/java/frc/robot/shooting/shootingplanner/MotionCompensatedShootingPlanner.java), [Shooting](https://github.com/frc1678/C2026-Public/blob/main/src/main/java/frc/robot/shooting/Shooting.java), [ShotVerifier](https://github.com/frc1678/C2026-Public/blob/main/src/main/java/frc/robot/shooting/verification/ShotVerifier.java)
- **2910 (2026)**, drive-aimed with hood and flywheel. Code is `src/main/java/org/frc2910/robot/subsystems/SuperStructure.java` and `util/ShooterInterpolation.java`. — [SuperStructure](https://github.com/FRCTeam2910/2026CompetitionRobot-Public/blob/main/src/main/java/org/frc2910/robot/subsystems/SuperStructure.java), [ShooterInterpolation](https://github.com/FRCTeam2910/2026CompetitionRobot-Public/blob/main/src/main/java/org/frc2910/robot/util/ShooterInterpolation.java)
- **6940 (2026)**, turret, hood and flywheel. Code is `src/main/java/frc/robot/util/ProjectileCalculator.java`, `util/TurretVelocityCalculator.java`, `commands/HybridShootCommand.java` and `util/Interpolating2DMap.java`. — [ProjectileCalculator](https://github.com/Team6940/2026Rebuilt/blob/master/src/main/java/frc/robot/util/ProjectileCalculator.java), [TurretVelocityCalculator](https://github.com/Team6940/2026Rebuilt/blob/master/src/main/java/frc/robot/util/TurretVelocityCalculator.java), [HybridShootCommand](https://github.com/Team6940/2026Rebuilt/blob/master/src/main/java/frc/robot/commands/HybridShootCommand.java)
- **5572 (2026)**, turret and hood. PR #222 "Shooting while moving" adds `math/ShootOnMove.java`. — [PR #222](https://github.com/Frc5572/FRC2026/pull/222/files)
- **5000 Hammerheads (2026)**, turret. Code is `src/main/java/frc/robot/subsystems/turret/TurretCalculator.java`. — [TurretCalculator](https://github.com/hammerheads5000/2026Rebuilt/blob/main/src/main/java/frc/robot/subsystems/turret/TurretCalculator.java)
- **5987 Galaxia (2026)**, turret, hood and flywheel, Kotlin. Uses an offline optimizer plus polynomial fits. Code is `src/main/kotlin/frc/robot/states/setpoints_manager/shooting_modes/ShootOnMove.kt`, with the simulator in the `Galaxia5987/ballistic-simulator` repo. — [ShootOnMove.kt](https://github.com/Galaxia5987/robot-2026/blob/main/src/main/kotlin/frc/robot/states/setpoints_manager/shooting_modes/ShootOnMove.kt), [CD 5987 thread](https://www.chiefdelphi.com/t/a-model-based-approach-to-shooting-on-the-move-team-5987-galaxia/518256), [ballistic-simulator](https://github.com/Galaxia5987/ballistic-simulator)
- **3061 Huskie (2026)** wrote up closed-form "velocity subtraction" equations on CD. The post links no repo. — [CD Huskie Physics](https://www.chiefdelphi.com/t/huskie-physics-shoot-on-the-move-with-equations/522805)

### Inferences
- For a turret, flywheel and hood FTC robot, the most directly portable references are:
  - 6328 `alpha-bot-turret`, a LUT plus ToF fixed-point iteration with turret feedforward.
  - 254 2024 `ShooterSetpoint`, a closed-form physics solution with turret and hood feedforward.
  - 604 `LaunchCalculator`, a LUT plus velocity subtraction with latency and acceleration extrapolation and finite-difference yaw rate and acceleration feedforward.

### Gaps
- No public 254 2026 repo exists: `api.github.com/repos/Team254/FRC-2026-Public` returned 404 on 2026-10-21.
- 1690's code is not public, and the software session video could not be read.
- 3476, 4414, 195, 3015, 5940, 1323, 4481 and 3636: no public SOTM source was located in this pass.
- Unverified leads, known from search-result snippets only:
  - 2471 (Kotlin): field-centric turret using a Pigeon gyro with wrap limits. — [TeamMeanMachine/FRC2026](https://github.com/TeamMeanMachine/FRC2026)
  - 340: SOTM plus a Kalman filter. — [Rebuilt2026-340](https://github.com/Greater-Rochester-Robotics/Rebuilt2026-340)
  - 2826: triple coaxial turret. — [wave-2826/Rebuilt2026](https://github.com/wave-2826/Rebuilt2026)
- 1678's 2022 and 2024 repos were not inspected.

---

## Q2. What exact algorithm does each team use? (virtual goal, iteration, latency look-ahead, velocity frame, shooter-point velocity, acceleration)

### Takeaway
Three families of algorithm appear:
- **(A) ToF fixed-point iteration ("virtual goal" / "lookahead launcher position").** Used by 6328 2026, 971, 6940, 5000 and 1706.
- **(B) Closed-form velocity-space solutions.** Either subtract the robot velocity from the stationary launch-velocity vector (604, 5572, Huskie 3061), or solve the intercept quadratic |d − v·t| = v_shot·t (254 2024). 254 2022 is the special case of a constant ToF.
- **(C) Simple pose look-ahead with no ToF.** Used by 6328 2024 and 1678 2026.

Good implementations use the field-frame velocity of the shooter/turret point (`v + ω × r`), not the robot-center velocity. They also add a small "phase delay" pose prediction of 30–50 ms.

### Cited Findings
**6328 2026 main ([LaunchCalculator](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/subsystems/launcher/LaunchCalculator.java))**, in order:
1. **Phase delay:** `estimatedPose.exp(new Twist2d(vx*phaseDelay, vy*phaseDelay, ω*phaseDelay))` with `phaseDelay = 0.03` s, using robot-relative velocity.
2. **Launcher pose:** `estimatedPose.transformBy(robotToLauncher)`.
3. **Launcher velocity:** in teleop, `GeomUtil.transformVelocity(fieldSetpointVelocity, robotToLauncher.xy, robotAngle)`. This uses the drive's commanded field-relative setpoint velocity, not the measured velocity. In autonomous, the raw velocity is used without the offset transform.
4. **Iteration:** 20 fixed iterations with no early exit:
   ```java
   timeOfFlight = timeOfFlightMap.get(lookaheadDist);
   effectiveTOF = (1 - Math.exp(-timeOfFlight * k)) / k;   // k = linearDragTimeConstant (0.375)
   lookaheadPose = launcherPos + launcherVel * effectiveTOF;
   lookaheadDist = target.getDistance(lookaheadPose);
   ```
   The launcher is shifted forward rather than the goal backward, which is equivalent. Hood and flywheel are then looked up at `lookaheadDist`, and the heading is aimed from the lookahead pose.
- **`transformVelocity` (the ω × r helper):** `vx' = vx − ω(x·sinθ + y·cosθ)` and `vy' = vy + ω(x·cosθ − y·sinθ)`, where (x, y) is the robot-frame offset and θ is the robot heading. — [GeomUtil](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/util/geometry/GeomUtil.java)

**6328 2026 Alpha Bot, turret ([alpha LaunchCalculator](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/alpha-bot-turret/src/main/java/org/littletonrobotics/frc2026/subsystems/launcher/LaunchCalculator.java))**
- Uses the same structure: 0.03 s phase delay, turret pose from `robotToTurret`, and a 20-iteration loop. There is no drag term: offset = `turretVelocity * timeOfFlight`.
- The turret velocity is computed inline from `RobotState.getFieldVelocity()` plus an ω × r term.
- The turret angle is `target.minus(lookaheadPose.getTranslation()).getAngle()`, a field-relative angle.
- 6328's description (nharnwal, Feb 5, 2026): distance and velocity are measured at the turret, not the robot center. The launcher position is shifted forward by ToF and ToF is recomputed "several times." The phase delay compensates for subsystems not tracking setpoints perfectly and matters most under acceleration. — [build thread #272](https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2026-build-thread/509595/272)
- The drag term in main appears to follow Oblarg's suggestion (Feb 22, 2026) that 6328's 20-iteration ToF recursion "overcorrects when moving fast" and that linear drag compensation would fix it. — [build thread #421](https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2026-build-thread/509595/421)

**971 ([aiming.cc](https://github.com/frc971/971-Robot-Code/blob/master/frc971/control_loops/aiming/aiming.cc))**
- The file comment says the robot's velocity is assumed to be "a constant offset to the ball's velocity over the entire trajectory." Each `IterateVirtualGoal` step:
  ```cpp
  air_time = |virtual_goal − robot| / ball_speed_over_ground;
  virtual_target = goal − air_time * robot_velocity;
  ```
- In `ShotMode::kShootOnTheFly` the step runs exactly 3 times (`for (int ii = 0; ii < 3; ++ii)`), starting from the real goal, with no convergence check.
- The output includes `virtual_shot_distance`, which is used for the shot lookups.
- In 2022, `ball_speed_over_ground` comes from `shot_velocity_interpolation_table`, looked up by the previous `target_distance`. — [y2022 aiming.cc](https://github.com/frc971/971-Robot-Code/blob/master/y2022/control_loops/superstructure/turret/aiming.cc)

**254 2022 ([ShootingUtil](https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/frc2022/shooting/ShootingUtil.java))**
- Radial/tangential decomposition with a constant ToF, `kShotTime = 1.05` s (set in `Superstructure.setShooterSetpoints`):
  ```java
  v = new Translation2d(dx, dy).rotateBy(robotToGoalRotation.inverse());   // x = radial, y = tangential
  shot_speed = range / shotTime - radial;  (clamped >= 0)
  yaw += atan2(-tangential, shot_speed);
  effective_range = shotTime * hypot(tangential, shot_speed);
  ```
- Hood and RPM are then looked up at `effective_range_m`.
- Velocity comes from `RobotState.getMeasuredVelocity()`, a raw, unfiltered twist. The frame is not stated; `getPredictedFieldToVehicle` uses `Pose2d.exp(twist)`, which suggests robot-relative. — [RobotState 2022](https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/frc2022/RobotState.java)
- Jared Russell (254) confirmed on CD (Jan 28, 2026) that 254 used constant ToF tuning in 2022. With a constant ToF the constant-shot-velocity curve becomes a circle around the goal. — [CD optimal driving](https://www.chiefdelphi.com/t/a-note-on-optimal-driving-for-shoot-on-the-move/512979)

**254 2024 ([ShooterSetpoint](https://github.com/Team254/FRC-2024-Public/blob/main/src/main/java/com/team254/lib/util/ShooterSetpoint.java))**
- **Inputs:**
  - Pose from `robotState.getPredictedFieldToRobot(kPredictionLookaheadTime = 0.05)`.
  - Velocity from `getLatestMeasuredFieldRelativeChassisSpeeds()`.
  - `d` = 3D vector from the release point (`kNoteReleaseHeight`) to the target.
- **Seed:** a nominal shot speed from a distance-based RPS ramp × `kRingLaunchVelMetersPerSecPerRotPerSec`.
- **Closed-form intercept:**
  ```java
  a = vx*vx + vy*vy - vShot*vShot;  b = -2*(dx*vx + dy*vy);  c = dx*dx + dy*dy + dz*dz;
  t = (-b - sqrt(b*b - 4*a*c)) / (2*a);
  shot = ((dx - vx*t)/t, (dy - vy*t)/t, dz/t);
  ```
- **Gravity:** `drop = 0.5*g*t*t` (+ a lift term `0.5*kRingLaunchLiftCoeff*c`). Then `pitch = atan2((dz − drop)/t, xyVel)` and `vShot = hypot((dz − drop)/t, xyVel)`. Turret yaw is the field angle of `shot`.
- No iteration in the speaker path.
- **Prediction:** `RobotState.getPredictedFieldToRobot(lookahead)` applies `pose.exp(robotRelativeSpeeds * lookahead)`. Acceleration is buffered and logged only, not used. — [RobotState 2024](https://github.com/Team254/FRC-2024-Public/blob/main/src/main/java/com/team254/frc2024/RobotState.java)

**604 2026 ([LaunchCalculator](https://github.com/frc604/2026-public/blob/main/FRC-2026/src/main/java/frc/quixlib/math/LaunchCalculator.java))**
- **Latency:** `kSystemLatencySec = 0.05`. The robot state is extrapolated with constant acceleration (`v*t + 0.5*a*t²`, and `v + a*t` for velocity).
- **Shooter point:** the turret-base field velocity is `MathUtils.getVelocityAtOffset(pose, robotToTurretBaseT2d, fieldVel)`.
- **Stationary shot:** `(v0, phi0)` comes from the CSV table at the turret-to-target distance, and `theta0` is the field azimuth.
- **Velocity subtraction (field plane):** `vx = v0·cosφ0·cosθ0 − turretVel.dx`, `vy = … − turretVel.dy`, `vz = v0·sinφ0`.
- Tilt (pitch/roll) compensation is applied, then `yaw = atan2(vy, vx) − turretHeading`, `elevation = asin(vz/|v|)`, `speed = |v|`.
- No ToF is used.
- 604 described the method as adjusting the launch velocity vector to "cancel out" robot velocity. — [CD 604](https://www.chiefdelphi.com/t/team-604-quixilver-2026-robot-cad-and-code-release/524289)

**5572 2026 ([PR #222](https://github.com/Frc5572/FRC2026/pull/222/files) `ShootOnMove.java`)**
- **Shooter-point velocity:** `svx = vx − ω·offY_field` and `svy = vy + ω·offX_field`.
- **Decomposition:** split into hub-radial `vr` and tangential `vt`. Take the stationary `(pitch, s)` from the LUT, giving target components `(s·cos p, 0, s·sin p)`.
- **Launch vector:** `a = vrT − vr`, `b = −vt`, `c = vzT`. Then `deltaYaw = atan2(b, a)`, `pitch = atan2(c, hypot(a, b))`, `exitSpeed = |(a, b, c)|`.
- `LATENCY_SEC = 0.0`, marked "tune." A commented-out loop suggests an earlier iterative attempt was abandoned.

**6940 2026 ([ProjectileCalculator](https://github.com/Team6940/2026Rebuilt/blob/master/src/main/java/frc/robot/util/ProjectileCalculator.java))**
- **LOOKAHEAD mode** (`LOOKAHEAD_ITERATIONS = 20`, no tolerance):
  - Start from the static distance. Each pass: `tof = ToF(d)`, `l = turretPos + v·tof`, `d = |l − target|`.
  - Then `virtualTarget = target − v·tof` and `aim = angle(virtualTarget − turretPos)`. RPS and hood come from the static maps at `d`.
  - The velocity is the turret field velocity from `drive.getTurretFieldVelocity()`, which is meant to include ω × r.
- **DIRECT mode:**
  - RPS and hood come from a 2-D bilinear map over (radialVelocity, distance).
  - Lead is `atan(v_tan·tof/d)`, with ToF taken at the current distance (a documented limitation). — [HybridShootCommand](https://github.com/Team6940/2026Rebuilt/blob/master/src/main/java/frc/robot/commands/HybridShootCommand.java)

**5000 Hammerheads 2026 ([TurretCalculator](https://github.com/hammerheads5000/2026Rebuilt/blob/main/src/main/java/frc/robot/subsystems/turret/TurretCalculator.java))**
- **Physics shot (`calculateShotFromFunnelClearance`):** a parabola `y = a·x² + b·x` through the target and a funnel-clearance point. Then `theta = atan(b)` and `v0 = sqrt(−g/(2a·cos²θ))`.
- **ToF:** `d/(v·cosθ)`.
- **`iterativeMovingShotFromFunnelClearance`:** shifts the target by `−v_field·ToF` and re-solves for a caller-chosen number of iterations. The author said 3 iterations, and that 1–2 is often enough. — [CD 511815](https://www.chiefdelphi.com/t/shoot-on-the-move-from-the-code-perspective/511815)
- **Caveat:** distances are measured from the robot center, while azimuth uses the turret point.

**1706 2022 ([CD](https://www.chiefdelphi.com/t/shoot-while-move-code-1706/410494))**
- Uses a ToF LUT and field-relative velocity plus acceleration (`kAccelCompFactor`, described as roughly the velocity ~0.1 s ahead) to build a virtual goal.
- Single correction, no loop. Shooter and hood come from LUTs at the virtual-goal distance.

**1678 2026 ([MotionCompensatedShootingPlanner](https://github.com/frc1678/C2026-Public/blob/main/src/main/java/frc/robot/shooting/shootingplanner/MotionCompensatedShootingPlanner.java))**
- Single pass: `lookahead = pose.exp(Twist2d(vx·ToF, vy·ToF, 0))`. The heading is aimed at the target from the lookahead translation.
- ToF comes from a linear trend line of distance (`TrendLines.HUB_TREND_LINE`). — [Shooting.java](https://github.com/frc1678/C2026-Public/blob/main/src/main/java/frc/robot/shooting/Shooting.java)
- The look-ahead is rejected (falls back to the current pose) when `(yawUnstable && error not tolerable) || speed < 0.09 m/s`.
- The shooter-offset and launch-delay terms are present but disabled (`times(0.0)`, `FUEL_HOPPER_TO_EXIT_TIME = 0`).

**2910 2026 ([SuperStructure](https://github.com/FRCTeam2910/2026CompetitionRobot-Public/blob/main/src/main/java/org/frc2910/robot/subsystems/SuperStructure.java))**
- Hub scoring in `scoringOnTheMove()` aims from the current pose with no extrapolation. It is gated on chassis speed ≤ `SCORING_ON_MOVE_SPEED_TOLERANCE = 0.15`.
- Only passing extrapolates, once, by ToF × field-relative velocity (`getExtrapolatedTranslation`).

**6328 2024 ([RobotState](https://github.com/Mechanical-Advantage/RobotCode2024Public/blob/main/src/main/java/org/littletonrobotics/frc2024/RobotState.java))**
- `getPredictedPose(lookahead, lookahead)` uses constant-velocity extrapolation: `lookahead = 0.35` s in teleop, `autoLookahead = 0.5` s in autonomous (using trajectory velocity).
- Aim, distance and arm angle are computed from the predicted pose. No ToF iteration.

**Physics framing (Huskie 3061, [CD](https://www.chiefdelphi.com/t/huskie-physics-shoot-on-the-move-with-equations/522805))**
- `v_shot = v★(x, y) − v_robot` in 3D, where `v★` is the static-shot launch-velocity vector.
- For an off-center shooter, add `(−y_s·ω, x_s·ω)` to `v_robot`.
- Velocity subtraction needs no ToF and no iteration. ToF correction needs about 2–3 iterations.
- For latency, a fallback is to evaluate the static map at a predicted position `(x + v_x·δt, y + v_y·δt)`.

### Inferences
- Families A and B are equivalent in the ideal case. ToF recursion converges to the same solution as the velocity-space direct solve; Oblarg (Jan 26, 2026) said the recursion contracts toward it ([CD 512542](https://www.chiefdelphi.com/t/time-of-flight-determination/512542)). A ToF LUT plus iteration needs only measured tables (hood, RPM and ToF vs distance). Velocity subtraction needs a reliable exit-velocity ↔ flywheel-speed model and a launch-angle ↔ hood-angle model.
- Convergence of the fixed-point iteration is fast when |dToF/dd|·|v| ≪ 1. In 6328's alpha table, ToF goes from 0.90 s to 1.16 s over 1.38–5.68 m, roughly 0.06 s/m. At 2 m/s the contraction factor is about 0.12 per iteration, so 3–5 iterations are plenty. 20 iterations costs little but is not needed. (Own calculation from the cited table.)
- **Porting to FTC with Pedro Pathing.** Feed the field-frame velocity and heading rate from odometry into a `transformVelocity` equivalent (`v + ω × r_turret`). Apply a ~30–50 ms pose prediction (`pose.exp(twist·dt)`). Then run the 6328-alpha loop or the 254-2024 closed form.

### Gaps
- 1690's exact algorithm is unknown.
- 1706's exact acceleration formula was described in prose only.
- No team's code models acceleration over the ToF horizon. 604 and 1706 use acceleration only for latency or near-term prediction.

---

## Q3. Time-of-flight source and lookup-table structure (InterpolatingDoubleTreeMap / InterpolatingTreeMap)

### Takeaway
Most top teams use empirically tuned 1-D maps keyed by horizontal distance from the launcher/turret point (not the robot center) to the goal. There is one map each for hood angle, flywheel speed and ToF, typically 5–10 points. ToF is measured with slow-motion video or a physical sensor. Physics models appear in 254 2024 (closed form plus gravity and lift), 5000 (parabola), 604 (offline Monte-Carlo CSV) and 5987 (offline Sleipnir optimization with quadratic drag).

### Cited Findings
- **6328 2026 main** ([LaunchCalculator](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/subsystems/launcher/LaunchCalculator.java)), keyed by lookahead distance in meters:
  - Hood: 1.12→10°, 1.52→20°, 2.01→23°, 2.73→26°, 3.51→29°, 3.93→32°, 4.40→34°, 4.68→36°, 4.90→38°, 5.01→38°.
  - Flywheel (units likely rad/s): 145, 145, 153, 160, 175, 188, 192, 195, 195, 195 at the same distances.
  - ToF (s): 1.63→1.017, 2.40→0.967, 3.25→1.19, 4.15→1.18, 4.875→1.25.
  - There are separate passing maps; passing ToF reaches 17.16 m→2.5 s.
  - The commanded flywheel is the map value plus a runtime `flywheelSpeedOffsetRadsPerSec` trim.
  - The hood map stores `Rotation2d` values.
- **6328 2026 alpha (turret):**
  - Hood: 1.34→19°, 1.78→19°, 2.17→24°, 2.81→27°, 3.82→29°, 4.09→30°, 4.40→31°, 4.77→32°, 5.57→32°, 5.60→35°.
  - Flywheel: 210…290.
  - ToF: 1.38→0.90, 1.88→1.09, 3.15→1.11, 4.55→1.12, 5.68→1.16.
  - Valid range: 1.34–5.60 m. — [alpha LaunchCalculator](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/alpha-bot-turret/src/main/java/org/littletonrobotics/frc2026/subsystems/launcher/LaunchCalculator.java)
- **6328's rationale** (nharnwal, Feb 5, 2026): manually tuned distance → hood/flywheel maps "gracefully handle real-world imperfections." Parts of the range can be retuned without affecting the rest. The ToF LUT came from sampling flight time with slow-motion video. — [build thread #272](https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2026-build-thread/509595/272)
- **254 2022** ([Constants](https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/frc2022/Constants.java), [Superstructure](https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/frc2022/subsystems/Superstructure.java)):
  - The tables use 254's own `InterpolatingTreeMap<InterpolatingDouble, …>`.
  - RPM: 1.5→740, 2→730, 2.5→770, 3.5→813, 4.5→900, 5.5→990, 6.7→1084.
  - Hood: 1.5→0, 2→4.5, 2.5→7, 3.5→12, 4.5→19.2, 5.5→21.66, 6.7→25.
  - With `kUseSinMap = true`, an analytic model is used instead: `horizontalRpm = kHOffset + kHPerMeter·range`, `verticalRpm = kVOffset + kVPerMeter·range`, `hood = atan(hRpm/vRpm) − 12°`, `rpm = hypot(hRpm, vRpm)`.
  - Both paths are keyed by the compensated `effective_range_m` plus an operator `rangeOffset`.
  - ToF is a constant 1.05 s.
- **254 2024**: there is no hood table. Hood comes from the solved pitch (`hoodZeroedAngle − pitch`). Flywheel RPS = solved `vShot / kRingLaunchVelMetersPerSecPerRotPerSec`. The nominal speed is a two-endpoint linear ramp between short- and long-range RPS. — [ShooterSetpoint](https://github.com/Team254/FRC-2024-Public/blob/main/src/main/java/com/team254/lib/util/ShooterSetpoint.java)
- **971**: ToF is implicit as `distance / ball_speed_over_ground`, where ball speed comes from `shot_velocity_interpolation_table`. — [y2022 aiming.cc](https://github.com/frc971/971-Robot-Code/blob/master/y2022/control_loops/superstructure/turret/aiming.cc)
- **2910 2026**: two `InterpolatingDoubleTreeMap`s keyed by distance in meters:
  - RPM: 1.3→2100, 2→2250, 3→2400, 4→2600, 5→2900.
  - Hood: 1.3→0°, 2→8°, 3→17°, 4→23°, 5→28°.
  - No ToF table. — [ShooterInterpolation](https://github.com/FRCTeam2910/2026CompetitionRobot-Public/blob/main/src/main/java/org/frc2910/robot/util/ShooterInterpolation.java)
- **1706 2022**: ToF LUT of (105 in, 0.82 s), (135, 0.82), (165, 0.85), (195, 0.85), (250, 1.05). The team found a "straight trig approach" gave poor results and switched to LUTs. — [CD 1706](https://www.chiefdelphi.com/t/shoot-while-move-code-1706/410494)
- **604 2026**: an `InterpolatingTreeMap<Double, LaunchParameter>` loaded from a CSV with columns `distance_m, optimal_v_mps, optimal_angle_deg`. It is built with `InverseInterpolator.forDouble()` and `LaunchParameter::interpolate`, and has an `inTableRange` flag. The table comes from offline Monte-Carlo simulation that picks angle/velocity pairs least sensitive to error. — [LaunchCalculator](https://github.com/frc604/2026-public/blob/main/FRC-2026/src/main/java/frc/quixlib/math/LaunchCalculator.java), [CD 604](https://www.chiefdelphi.com/t/team-604-quixilver-2026-robot-cad-and-code-release/524289)
- **6940 2026**: `Interpolating2DMap` (bilinear) keyed by (radial velocity, distance) for RPS and hood, plus a 1-D `DistanceToFlightTimeSecs` map. Static shots query the 2-D map at radial velocity 0. — [ProjectileCalculator](https://github.com/Team6940/2026Rebuilt/blob/master/src/main/java/frc/robot/util/ProjectileCalculator.java)
- **5000 2026**: a `shotMap` (distance → exit velocity, hood) plus a `tofMap` with a `tofFudgeFactor`. Flywheel `ω = v / r / 0.54`, where 0.54 is an undocumented efficiency/slip factor. — [TurretCalculator](https://github.com/hammerheads5000/2026Rebuilt/blob/main/src/main/java/frc/robot/subsystems/turret/TurretCalculator.java)
- **5572 2026**: exit speed ↔ flywheel uses `k = 0.43·2π·inchesToMeters(2)`, so m/s per rev/s with a 0.43 transfer coefficient. — [PR #222](https://github.com/Frc5572/FRC2026/pull/222/files)
- **3061**: exit speed `v = π(R1 + R2)·RPS` with roughly 90% efficiency, calibrated using 240 fps video, a ruler in frame and about 10 shots per clip. — [CD Huskie](https://www.chiefdelphi.com/t/huskie-physics-shoot-on-the-move-with-equations/522805)
- **5987**: offline grid over (d, vx, vy) solved with Sleipnir. Physics is RK4 with quadratic drag, Cd ≈ 0.63 measured by a drop test. Polynomial fits are generated into `ShotCalculator.java`. Exit velocity ↔ wheel speed is a 2nd-degree polynomial. — [CD 5987](https://www.chiefdelphi.com/t/a-model-based-approach-to-shooting-on-the-move-team-5987-galaxia/518256)
- **ToF measurement advice** ([CD Time of Flight Determination](https://www.chiefdelphi.com/t/time-of-flight-determination/512542)):
  - Oblarg (Jan 23, 2026): empirical measurement is "typically more accurate at first-pass" than a physics model. Video frame counting is accurate enough.
  - truher: a hand stopwatch has about 0.15 s mean absolute error, roughly 0.75 m at 5 m/s, so average many samples.
  - ArchdukeTim: a CANrange in the hub plus shooter current/velocity drop timestamps each shot's ToF automatically.
- **WPILib `InterpolatingTreeMap(InverseInterpolator<K>, Interpolator<V>)`** offers `put`, `get` (interpolates between neighboring keys) and `clear`. `InterpolatingDoubleTreeMap` is its subclass. — [WPILib javadoc](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/interpolation/InterpolatingTreeMap.html)

### Inferences
- The ToF tables are fairly flat: about 0.9–1.25 s across 1.3–5.7 m for REBUILT fuel. This is why constant-ToF approaches (254 2022) and few iterations work.
- For FTC, a 3-column LUT (distance → hood, flywheel, ToF) keyed by turret-to-goal distance is the lowest-risk start. It can be built with a simple sorted-array linear interpolator, with no WPILib dependency.
- 254 2022's "sin map" is a good middle ground: two linear fits (horizontal and vertical exit-velocity components vs range) give a physically consistent hood angle and RPM from just 4 constants.

### Gaps
- WPILib's out-of-range behavior (whether `get` clamps to the end values) was not confirmed from the docs page in this pass. Check the source before relying on extrapolation. 604 tracks `inTableRange` explicitly, and 6328 gates on min/max distance.
- 254 2022's `kHorizontalRpmPerMeter`/`kVerticalRpmPerMeter` values were not fetched.

---

## Q4. Turret setpoint, turret feedforward (counter-rotation + line-of-sight rate), and wraparound/limits

### Takeaway
Every turret implementation computes a field-relative aim angle, then converts it to robot-relative (turret angle = field angle − robot heading). The feedforward is turret velocity = (d/dt of the field line-of-sight angle) − ω_robot. Teams get the line-of-sight rate either analytically (254: −v_tangential/d; 971: derivative of atan2) or by finite-differencing the solver output (6328: filtered finite difference; 604: central differences that also give acceleration). For wrap, teams pick the 2π-equivalent setpoint closest to the last goal that lies within the limits, and invalidate the shot or zero the feedforward when a wrap happens.

### Cited Findings
- **254 2022** ([ShootingUtil](https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/frc2022/shooting/ShootingUtil.java), [Superstructure](https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/frc2022/subsystems/Superstructure.java)):
  - Feedforward: `effective_yaw_feedforward_velocity_deg_s = −(ω_deg + toDegrees(tangential / uncompensated_range))`. The range feedforward is `−radial`. A TODO notes the feedforward ignores the derivative of the motion compensation.
  - The feedforward goes into a `LinearTimeVaryingMotionProfileGoal`.
  - Compensation is clamped to `kMaxTurretAdjustmentForShootOnMove = 45°`; when clamped, the setpoint is marked invalid.
  - `handleTurretLimit`: setpoint ≥ 190 → −90, ≤ −190 → +90. On a "wrap," the feedforward is zeroed and shooting is invalidated.
  - Turret soft limits are `kMinUnitsLimit = −115` and `kMaxUnitsLimit = 115`.
- **254 2024** ([ShooterSetpoint](https://github.com/Team254/FRC-2024-Public/blob/main/src/main/java/com/team254/lib/util/ShooterSetpoint.java)):
  - Turret: `turretRobotFrame = shotFieldAngle − robotHeading`, then rotated by π and `kTurretToShotCorrection`.
  - Feedforward: in the target frame, `tangent = v.rotateBy(robotToTarget.angle).y` and `turretFF = −(ω + tangent/distance)`.
  - Hood feedforward: `hoodFF = radial·(−z)/(d² + z²)`, the elevation-angle rate.
  - Wrap is not handled in this file. `TurretFactory.aimTurretToPose` passes `getTurretRadiansFromCenter()` and `getTurretFF()` suppliers to `positionSetpointCommand`. — [TurretFactory](https://github.com/Team254/FRC-2024-Public/blob/main/src/main/java/com/team254/frc2024/command_factories/TurretFactory.java)
- **971** ([aiming.cc](https://github.com/frc971/971-Robot-Code/blob/master/frc971/control_loops/aiming/aiming.cc)):
  - LOS rate: `atan_diff = (rel_x·rel_ydot − rel_y·rel_xdot)/(rel_x² + rel_y²)` (returns 0 when the squared norm < 1e-3), then `dheading_dt = atan_diff − yaw_rate`.
  - `rel_ydot = 0.0` because 971's tank drive has no lateral velocity. The comment says it ignores lateral velocity and the SOTM correction ("close enough").
  - Position is unwrapped relative to the last goal: `last_goal + NormalizeAngle(heading − zero_offset − last_goal)`. If within `anti_wrap_buffer` of a limit, it is wrapped by 2π toward `turret_range.middle_soft()`.
  - In y2022 the goal velocity was clamped to `[0, 0]`, so the feedforward was effectively disabled that season. — [y2022 aiming.cc](https://github.com/frc971/971-Robot-Code/blob/master/y2022/control_loops/superstructure/turret/aiming.cc)
- **6328 2026 alpha** ([LaunchCalculator](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/alpha-bot-turret/src/main/java/org/littletonrobotics/frc2026/subsystems/launcher/LaunchCalculator.java), [Turret](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/alpha-bot-turret/src/main/java/org/littletonrobotics/frc2026/subsystems/launcher/turret/Turret.java)):
  - Field LOS rate: `turretVelocity = movingAverage(0.1 s)( (turretAngle − lastTurretAngle)/loopPeriod )`, with the hood velocity computed the same way.
  - In `Turret`: `robotRelativeGoalAngle = goalAngle − robotAngle` and `robotRelativeGoalVelocity = goalVelocity − ω`.
  - Wrap: try `goal + 2π·i` for i ∈ [−2, 2] and keep candidates within ±210° (ACTIVE_LAUNCHING) or about ±190° (TRACKING). Pick the one closest to `lastGoalAngle`, then clamp.
  - A `TrapezoidProfile` runs to `State(bestAngle, robotRelativeGoalVelocity)`.
  - `atGoal` requires both position and velocity epsilon-equal between profile setpoint and goal.
  - The thread says turret and hood setpoint derivatives are fed forward "to reduce tracking lag." — [#272](https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2026-build-thread/509595/272)
- **604 2026** ([LaunchCalculator](https://github.com/frc604/2026-public/blob/main/FRC-2026/src/main/java/frc/quixlib/math/LaunchCalculator.java), [TrackAndLaunchCommand](https://github.com/frc604/2026-public/blob/main/FRC-2026/src/main/java/frc/robot/commands/TrackAndLaunchCommand.java)):
  - The full solution is evaluated at `latency − dt`, `latency` and `latency + dt` with `dt = 0.005` s.
  - `yawVelocity = wrap(yaw+ − yaw−)/(2dt)`. Yaw acceleration is the second difference over `dt²`.
  - Both are sent through `m_turret.setTargetYaw(yaw, yawVelocity, yawAcceleration)`.
  - Because yaw is computed robot-relative, robot rotation is automatically included in the numerical derivative.
- **6940 2026** ([TurretVelocityCalculator](https://github.com/Team6940/2026Rebuilt/blob/master/src/main/java/frc/robot/util/TurretVelocityCalculator.java), [HybridShootCommand](https://github.com/Team6940/2026Rebuilt/blob/master/src/main/java/frc/robot/commands/HybridShootCommand.java)):
  - Turret velocity = `TrapezoidProfile` velocity + (−toDegrees(ω_chassis)) + target LOS feedforward `toDegrees(tangentialToVirtual / distance)`.
  - The output goes to `PositionTorqueCurrentFOC.withVelocity()`. The sum is not clamped to the max velocity.
  - The README says speed tapers near mechanical limits. — [README](https://github.com/Team6940/2026Rebuilt)
- **5000 2026**: `calculateAzimuthAngle` wraps (field angle − yaw) to [−0.5, 0.5) rotations. It then adds or subtracts one rotation to stay near the current side if still within `MAX_TURN_ANGLE`/`MIN_TURN_ANGLE`. No feedforward. — [TurretCalculator](https://github.com/hammerheads5000/2026Rebuilt/blob/main/src/main/java/frc/robot/subsystems/turret/TurretCalculator.java)
- **5987 2026**:
  - The turret setpoint is `turretAngleToHub − calculateYaw(d, vx, vy)`, with velocities in the turret frame including the gyro-ω tangential term.
  - If the compensated and static constrained setpoints differ by more than 180°, the static one is used to avoid a long unwind. — [ShootOnMove.kt](https://github.com/Galaxia5987/robot-2026/blob/main/src/main/kotlin/frc/robot/states/setpoints_manager/shooting_modes/ShootOnMove.kt)
  - Turret feedforward = setpoint rate × a calibrated constant. Adding ω·r "fixed misses during turns." — [CD 5987](https://www.chiefdelphi.com/t/a-model-based-approach-to-shooting-on-the-move-team-5987-galaxia/518256)
- **6328 2026 main (drive-aimed analog of turret FF):**
  - Heading command: `omega = driveVelocity + Kp(8.0)·(driveAngle − heading) + Kd(0.5)·(driveVelocity − ω_measured)`, where `driveVelocity` is a 0.1 s moving average of the finite-differenced aim angle.
  - The swerve center of rotation is shifted toward the launcher as heading error shrinks: `corScalar` ramps over 15°–30° error.
  - `getDriveAngleWithLauncherOffset` adds `asin(launcherY/d)` for an off-axis launcher. — [DriveCommands](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/commands/DriveCommands.java), [LaunchCalculator](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/subsystems/launcher/LaunchCalculator.java)

### Inferences
- For a holonomic FTC robot, use the full LOS rate. The field LOS angle φ = atan2(g_y − p_y, g_x − p_x) has dφ/dt = ((g_y − p_y)·v_x − (g_x − p_x)·v_y)/d² = −v_tan/d (counter-clockwise positive). The turret feedforward is then dφ/dt − ω. This matches 254's `−(ω + tangent/d)` (own derivation).
- Do not copy 971's `rel_ydot = 0`, which is a tank-drive simplification.
- With SOTM, the aim point is the virtual goal. Computing the LOS rate to the virtual goal (6940), or finite-differencing the final solved angle (6328, 604), captures the compensation's own rate of change. 254 2022 and 971 explicitly do not.
- Finite differences need filtering: 6328 uses a 0.1 s moving average, while 604 differentiates the model analytically-numerically at ±5 ms from the same state, which avoids loop-to-loop noise.

### Gaps
- 254 2024's turret wrap and limit handling (in `TurretSubsystem.positionSetpointCommand`) was not fetched.
- The 254 2022 `handleTurretLimit` units (±190 threshold vs ±115 soft limits) were not reconciled.

---

## Q5. Shot-readiness gating (when is it OK to fire?)

### Takeaway
Gates combine three groups of checks:
- **Mechanism at setpoint:** flywheel within a fixed RPM tolerance or about 4%, hood within about 0.6–3°, and turret/heading within an angle tolerance. 254 2022 converts the turret tolerance to meters of lateral miss at the goal.
- **Solution validity:** distance in range, no clamping of hood, RPM or turret compensation, no turret wrap in progress, and no forbidden field zones.
- **Optional platform checks:** robot speed caps, tilt (pitch/roll) and vision recency.

### Cited Findings
- **254 2022** ([Superstructure](https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/frc2022/subsystems/Superstructure.java), [ShootingParameters](https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/lib/util/ShootingParameters.java), [Constants](https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/frc2022/Constants.java)):
  - Fires when `hoodReady && shooterReady && turretReady && mShooterSetpointValidForShooting`.
  - Tolerances: shooter 20 rpm, hood 0.6°, turret 0.4 m at goal via `tan(angleErr)·range`.
  - The setpoint is invalidated on SOTM yaw clamp (45°), turret wrap, or hood/RPM clamp.
  - Firing stays latched for `kShootingTimeout = 0.25` s.
- **254 2024** ([AimFactory](https://github.com/Team254/FRC-2024-Public/blob/main/src/main/java/com/team254/frc2024/command_factories/AimFactory.java)):
  - `onTarget = hoodOnTarget && shooterOnTarget && turretOnTarget && isValid`.
  - Shooter must be within 4% of commanded RPS. The turret error is checked with `Rotation2d` (wrap-safe) against `kTurretShootingEpsilon`.
  - `isValid` is false if the solved RPS exceeds `kShooterStage2RPSCap`. — [ShooterSetpoint](https://github.com/Team254/FRC-2024-Public/blob/main/src/main/java/com/team254/lib/util/ShooterSetpoint.java)
- **6328 2026**:
  - `isValid` requires being outside the "bad boxes" (tower, near hub, far hub) and lookahead distance within [0.6, 6.0] m. Passing uses [3.0, 17.16] m. — [LaunchCalculator](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/subsystems/launcher/LaunchCalculator.java)
  - Heading tolerance is 10° for the hub and 15° for passing. Pitch/roll must be within 5°; this is logged in `DriveCommands`.
  - The linear speed cap uses `driveLaunchMaxPolarVelocityRadPerSec = 0.4` and the naive ToF: `hubAngle = maxPolar·ToF`, `lookaheadAngle = π − robotAngle − hubAngle`, `maxSpeed = (d·sin(hubAngle)/sin(lookaheadAngle))/ToF`. — [DriveCommands](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/commands/DriveCommands.java)
  - The alpha turret's `atGoal` requires the profile to have reached both goal position and goal velocity. — [Turret (alpha)](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/alpha-bot-turret/src/main/java/org/littletonrobotics/frc2026/subsystems/launcher/turret/Turret.java)
- **604 2026**: rollers run only when `inLaunchZone && turretReady && hasRecentCameraUpdate`. `turretReady` checks yaw, launch angle and launch velocity tolerances, with separate values for feeding and scoring. Vision recency means a recent multi-tag update (≥ 2 tags). — [TrackAndLaunchCommand](https://github.com/frc604/2026-public/blob/main/FRC-2026/src/main/java/frc/robot/commands/TrackAndLaunchCommand.java), [CD 604](https://www.chiefdelphi.com/t/team-604-quixilver-2026-robot-cad-and-code-release/524289)
- **6940 2026**: `readyToAutoFeed = debounce(0.12 s falling)(shooter.isAtTargetRps()) && turret.isAtTargetPosition(distance) && hood.isAtTargetPosition()`. The turret tolerance takes distance as an argument. — [HybridShootCommand](https://github.com/Team6940/2026Rebuilt/blob/master/src/main/java/frc/robot/commands/HybridShootCommand.java)
- **5572 2026**: `okayToShoot = movingShot.feasible() && currentFlywheel > desiredFlywheel − 6`. Feasible means pitch is within the hood-derived [MIN_PITCH, MAX_PITCH] and exit speed ≤ MAX_EXIT_SPEED. — [PR #222](https://github.com/Frc5572/FRC2026/pull/222/files)
- **1678 2026**: `verifyShot = !tilted && aligned`. Tilted means pitch/roll beyond `kAcceptableTiltError`, or the gyro is disconnected. Alignment uses a hub- or ferry-specific angle tolerance with a latch. An auto timeout is also available. — [ShotVerifier](https://github.com/frc1678/C2026-Public/blob/main/src/main/java/frc/robot/shooting/verification/ShotVerifier.java)
- **2910 2026**: `isAtShotConditions()`, distance ≥ 1.5 m, heading within 4°, and chassis speed ≤ 0.15. Passing allows 15° and a speed of 10. — [SuperStructure](https://github.com/FRCTeam2910/2026CompetitionRobot-Public/blob/main/src/main/java/org/frc2910/robot/subsystems/SuperStructure.java)
- **5987**: robot speed is capped at 2.0 m/s while shooting on the move toward the hub. — [CD 5987](https://www.chiefdelphi.com/t/a-model-based-approach-to-shooting-on-the-move-team-5987-galaxia/518256)

### Inferences
- 254's "meters at goal" turret tolerance is a good FTC choice: allowed angle error = atan(goalHalfWidthMargin / d). It loosens up close and tightens far away.
- The gate should also require that the solution was not clamped (hood or flywheel limits, maximum turret compensation) and that the turret is not mid-wrap.
- 6328's polar-velocity speed cap is a principled way to keep the virtual goal from swinging faster than the turret/heading loop can track.

### Gaps
- Numeric values for 254 2024's `kTurretShootingEpsilon`/`kHoodShootingEpsilon`, 604's tolerances and 1678's `ShotVerifierConstants` were not fetched.
- No team was found that gates on robot acceleration explicitly.

---

## Q6. Projectile inherits robot velocity: flywheel-speed adjustment vs re-aiming only

### Takeaway
The ball's field velocity is the launch velocity relative to the shooter plus the shooter point's field velocity. A correct solution must therefore change both direction (turret yaw) and magnitude and elevation (flywheel and hood).
- **ToF-LUT teams** (6328, 971, 6940, 1706) do this implicitly. They re-look-up hood and flywheel at the virtual-goal distance: moving toward the goal shortens the virtual distance, giving a lower RPM and flatter hood.
- **Closed-form teams** (254 2022/2024, 604, 5572, 3061) explicitly recompute exit speed and pitch from the subtracted velocity vector.
- A few add ad-hoc flywheel trims.

### Cited Findings
- **254 2022**: `shot_speed = range/T − v_radial` and `effective_range = T·hypot(v_tan, shot_speed)`. Hood and RPM come from the map at `effective_range`, so radial motion changes RPM and hood, and tangential motion changes yaw plus a small range increase. — [ShootingUtil](https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/frc2022/shooting/ShootingUtil.java)
- **254 2024**: the solved `vShot` (with gravity drop) is converted directly to flywheel RPS, and the pitch to hood angle. The setpoint is invalid if RPS exceeds the cap. — [ShooterSetpoint](https://github.com/Team254/FRC-2024-Public/blob/main/src/main/java/com/team254/lib/util/ShooterSetpoint.java)
- **604 2026**: subtracts the planar turret velocity from the 3D stationary launch vector, then sets turret yaw, elevation and launch speed from the result. — [LaunchCalculator](https://github.com/frc604/2026-public/blob/main/FRC-2026/src/main/java/frc/quixlib/math/LaunchCalculator.java)
- **6328 2024 "super poop" shot**: subtracts `RPM(v_radial / 1.5 in)·cos(armAngle)` from both flywheel speeds. This is a direct radial-velocity flywheel compensation projected on the launch elevation. — [RobotState 2024](https://github.com/Mechanical-Advantage/RobotCode2024Public/blob/main/src/main/java/org/littletonrobotics/frc2024/RobotState.java)
- **6940 DIRECT mode**: RPS and hood come from a 2-D map indexed by radial velocity, an empirical flywheel adjustment. **LOOKAHEAD mode**: the static map at the lookahead distance. — [ProjectileCalculator](https://github.com/Team6940/2026Rebuilt/blob/master/src/main/java/frc/robot/util/ProjectileCalculator.java)
- **5987**: flywheel = `(kStatic 0.97 + kMoving 0.05·tanh(vx))·ω(v_exit)`, an empirical moving-shot trim on top of the model's exit velocity, adjustable by the driver in 0.01 steps. — [ShootOnMove.kt](https://github.com/Galaxia5987/robot-2026/blob/main/src/main/kotlin/frc/robot/states/setpoints_manager/shooting_modes/ShootOnMove.kt)
- **3061**: hardware limits can make the subtracted solution infeasible. Example: driving away from the hub needs a near-vertical shot, as in tcrvo's case of a computed 80° when the hood maxes at 60°. Rule of thumb: test the shooter beyond its operating range by the ToF offset; at 6 m with 2 m/s retreat and 1.5 s ToF, test to about 9 m. — [CD Huskie](https://www.chiefdelphi.com/t/huskie-physics-shoot-on-the-move-with-equations/522805)
- **Drag and windage:**
  - Omer_Prag argued that subtracting robot velocity changes drag and Magnus forces and may change spin when speed and spin are not independently controlled. Oblarg replied that either method can add a spin term. — [CD 512542](https://www.chiefdelphi.com/t/time-of-flight-determination/512542)
  - 6328 main applies `effectiveTOF = (1 − e^(−k·ToF))/k` to the robot-imparted displacement (k = 0.375). — [LaunchCalculator](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/subsystems/launcher/LaunchCalculator.java)

### Inferences
- The term `(1 − e^(−kt))/k` is the displacement of a linear-drag body with unit initial velocity after time t. 6328 therefore models the inherited robot velocity decaying in flight, which shrinks the lead at long ToF and high speed and fixes the overcorrection Oblarg observed (own interpretation of the formula).
- If the closed-form subtraction makes the ball's field-frame launch velocity exactly equal the stationary solution's, the trajectory, including air drag, is identical in still air. Only spin/Magnus and the exit-velocity ↔ RPM mapping differ. Omer_Prag's drag concern therefore applies mainly to ToF-offset methods, not exact velocity matching (own reasoning; not verified by a cited team).
- For an FTC turret, flywheel and hood robot, the LUT-iteration path (hood/flywheel at virtual distance) gets most of the speed and pitch effect without an exit-velocity model. A small radial-velocity flywheel trim (6328 2024 / 6940-style) can be tuned on top.

### Gaps
- No team published measured before/after accuracy comparing "re-aim only" with "re-aim + flywheel/hood adjust."

---

## Q7. Known pitfalls, bugs, and lessons learned

### Takeaway
The recurring lessons:
- Measure distance and velocity at the turret point and include ω × r.
- Add a small phase-delay or latency prediction, because errors are worst under acceleration.
- Feed forward the turret, hood and heading setpoint derivatives.
- Damp overcorrection at high speed (drag term or speed cap).
- Validate against mechanism limits, and handle turret wrap without firing mid-unwind.
- Prefer LUTs tuned on a real field.
- Several published codebases contain disabled or buggy compensation terms, so read code critically.

### Cited Findings
- **6328** (Feb 5, 2026):
  - Turret-relative measurement and lookahead consistency are easy to get wrong.
  - Phase delay matters most under acceleration.
  - Map-based tuning allows low-risk targeted retuning at events. — [#272](https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2026-build-thread/509595/272)
- **Oblarg on 6328's 20-iteration recursion:** it "overcorrects when moving fast"; add linear drag compensation. — [#421](https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2026-build-thread/509595/421)
- **1706 (2022):**
  - Without acceleration compensation, turret, shooter and hood lagged during acceleration and deceleration, and the turret took time to settle after stops.
  - With `kAccelCompFactor`, the team shot on the move at over 3 m/s and used move-and-shoot all of Houston.
  - A trig approach did worse than LUTs. — [CD 1706](https://www.chiefdelphi.com/t/shoot-while-move-code-1706/410494)
- **5987:** correcting turret velocity for robot rotation (ω·r) fixed misses during turns. Minimizing ToF in the optimizer gave flat, low-entry shots; minimizing launch speed was better. Hard if-statement constraints produced unfittable discontinuities. — [CD 5987](https://www.chiefdelphi.com/t/a-model-based-approach-to-shooting-on-the-move-team-5987-galaxia/518256)
- **Optimal driving** (Oblarg, Jan 28 and Feb 10, 2026):
  - The ToF iteration converges only within an envelope of robot velocities that widens with shot speed.
  - The most stable approach direction makes the virtual shot perpendicular to the robot's motion, so drivers should drive "askew" of the target.
  - Driving straight at or away from the goal at low shot speeds makes velocity uncertainty dominate.
  - Hjelstrom reported a 2017 robot that "shot backwards" when driving toward the goal. — [CD 512979](https://www.chiefdelphi.com/t/a-note-on-optimal-driving-for-shoot-on-the-move/512979)
- **Velocity noise:** finite-difference velocity estimates are noisy. Integrated ToF formulations avoid this, while velocity-space methods need filtering. — [CD 512542](https://www.chiefdelphi.com/t/time-of-flight-determination/512542)
  - 6328 main uses the commanded drive setpoint velocity in teleop rather than measured velocity. — [LaunchCalculator](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/subsystems/launcher/LaunchCalculator.java)
  - 254 2022 keeps a 25-sample moving-average twist but passes the raw measured velocity to SOTM. — [RobotState 2022](https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/frc2022/RobotState.java)
- **Infeasible solutions:** pure subtraction can demand hood angles beyond the mechanism (an 80° request on a 60° hood). — [CD Huskie](https://www.chiefdelphi.com/t/huskie-physics-shoot-on-the-move-with-equations/522805)
- **Bugs and disabled terms found in published code:**
  - 1678 2026's shooter-offset term is zeroed by `times(0.0)`, its launch delay is 0, and `m_currentLookAheadOffset` is never written. — [MotionCompensatedShootingPlanner](https://github.com/frc1678/C2026-Public/blob/main/src/main/java/frc/robot/shooting/shootingplanner/MotionCompensatedShootingPlanner.java)
  - 971 y2022 clamps the turret goal velocity to 0. — [y2022 aiming.cc](https://github.com/frc971/971-Robot-Code/blob/master/y2022/control_loops/superstructure/turret/aiming.cc)
  - 254 2024's near-zero `a` fallback multiplies `vShot` after `a` is already computed, so it has no effect. — [ShooterSetpoint](https://github.com/Team254/FRC-2024-Public/blob/main/src/main/java/com/team254/lib/util/ShooterSetpoint.java)
  - 6328 main's `Double.isNaN(lastHoodAngle)` init guard likely never fires because the primitive defaults to 0. — [LaunchCalculator](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/subsystems/launcher/LaunchCalculator.java)
  - 5000 measures distance from the robot center but azimuth from the turret. — [TurretCalculator](https://github.com/hammerheads5000/2026Rebuilt/blob/main/src/main/java/frc/robot/subsystems/turret/TurretCalculator.java)
  - 5572's exact-key `shotMap.get(distance)` (no interpolation) can return null. — [PR #222](https://github.com/Frc5572/FRC2026/pull/222/files)
- **Possible sign error in 6328's alpha branch:** the fetched inline turret-velocity X term reads `vx + ω·(r_y·cosθ − r_x·sinθ)`. The rigid-body result is `vx − ω·(r_x·sinθ + r_y·cosθ)`, so if the transcription is exact, the `r_y·cosθ` sign is flipped. The main branch's `GeomUtil.transformVelocity` has the correct form. This needs verifying against the raw file. — [alpha LaunchCalculator](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/alpha-bot-turret/src/main/java/org/littletonrobotics/frc2026/subsystems/launcher/LaunchCalculator.java), [GeomUtil](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/util/geometry/GeomUtil.java)
- **Community skepticism:** many teams planned SOTM until they saw what it requires, and expected reports that it "works great in simulation." One team skipped SOTM to avoid "integration hell." — [CD poll](https://www.chiefdelphi.com/t/poll-how-are-you-shooting-on-the-move/514641)
- **Calibration:** use 240 fps video with a ruler at the right depth. A single far ruler overestimated speed. — [CD Huskie](https://www.chiefdelphi.com/t/huskie-physics-shoot-on-the-move-with-equations/522805)

### Inferences
- A robust FTC port checklist distilled from the above:
  1. Use the turret-point velocity `v + ω × r`, with correct signs. Unit-test it against a pure rotation.
  2. Predict pose by about 30–50 ms.
  3. Run 3–5 ToF iterations, optionally with 6328's linear-drag effective ToF.
  4. Look up hood and flywheel at the virtual distance.
  5. Turret setpoint = field aim − heading, with wrap selection closest to the last goal within limits.
  6. Turret feedforward = LOS rate − ω, filtered.
  7. Fire only if flywheel, hood and turret (meters-at-goal) are within tolerance, the solution is unclamped and in range, and speed is under a cap.
- FTC robots usually run slower loops and slower odometry updates than FRC. The phase-delay constant and feedforward filtering windows (6328: 0.1 s moving average) will likely need retuning.

### Gaps
- The robototes 2022 `yawAdjustment()` `Math.asin` NaN pitfall (lateral velocity > distance) came from a search snippet and could not be verified from the PR page.
- Eeshwar Krishnan's blog posts on "shooting-on-the-fly" ([part 1](https://blog.eeshwark.com/robotblog/shooting-on-the-fly), [part 2](https://blog.eeshwark.com/robotblog/shooting-on-the-fly-pt2)) are JavaScript-rendered and could not be read.
- No quantitative hit-rate data comparing SOTM methods was found.
