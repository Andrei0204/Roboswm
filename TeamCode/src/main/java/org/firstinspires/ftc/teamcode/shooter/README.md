# Shoot on the move (BIOBUZZ, turret + flywheel + hood, Pedro Pathing 3.0.2)

The robot drives freely while the turret, flywheel and hood keep a shot solved for the up CELL
of our HIVE. A ball is fed only when every readiness check passes. The full research and design
rationale is in `reports/FTC turret shooting while moving.md`.

## Algorithm (each loop, `ShotSolver`)

1. Predict the robot `LOOKAHEAD_SEC` ahead (p + v t + a t²/2, heading + ω t), using
   `follower.pose()` and `follower.velocity()` (field frame, in/s and rad/s). The lookahead
   covers sensor, loop and mechanism lag, not the feed delay. The mechanisms keep tracking
   until the ball leaves, so adding the feed delay would over-lead.
2. Move to the turret pivot. The ball inherits the pivot's velocity, `v + ω × r`.
3. Iterate the time of flight T. The virtual goal is `g − v_pivot · lead(T)`, then T is looked
   up again at the new distance, until T changes by less than 1 ms. `lead(T) = T`, or
   `(1 − e^(−cT))/c` with the drag correction `LEAD_DRAG_COEFF`.
4. The turret aims at the virtual goal. Flywheel rpm and hood come from the stationary table of
   the next ball type (POLLEN or NECTAR) at the virtual distance.
5. The turret feedforward is a central difference of steps 1–4 along the predicted motion. The
   simpler v/d line-of-sight formula ignores the pivot's centripetal motion and the change in T,
   and was off by up to 412°/s in testing.
6. Firing is gated on all of these, for `READY_DEBOUNCE_LOOPS` loops in a row:
   - distance inside the table
   - the ball can enter the outward-facing, 30°-tilted CELL opening: correct side of the HIVE,
     approach angle at most `MAX_APPROACH_ANGLE_DEG`
   - speed, turn rate and acceleration under their limits
   - flywheel inside its rpm window
   - hood settled
   - turret error under `atan(allowance / distance)`, where allowance =
     (20 in · cos(approach) − ball diameter)/2 − `AIM_MARGIN_IN`, measured by the turret's
     external encoder, with the encoder rate matching the commanded rate
   - turret homed, and its target inside the reachable range (not in the dead zone behind it)
   - no HIVE TIP in the last `HIVE_TIP_PAUSE_SEC`

## Off-robot tests

Both tests are in `TeamCode/src/test/java/.../shooter/`.

- **`SolverSimTest`** launches simulated balls the way the solver commands, from random moving
  states. It covers turret offset, rotation, acceleration, linear drag, both ball types and
  both alliances. The worst miss is 0.0001 in, against a 17 in average miss when aiming
  straight at the goal. It also checks the signs, wrap logic and symmetry, and that the
  feedforward matches a finite difference within 0.3°/s.
- **`TurretSimTest`** runs `TurretController` against a deliberately imperfect Axon turret:
  3% scale error, 50 ms lag, 400°/s slew, 0.8° backlash, and a quantized encoder with
  hub-style velocity. It checks:
  - nothing moving in INIT without the button
  - homing from any start angle, an encoder reset that only shows up after one stale
    bulk-cached read, and catching a reset that never happens
  - the count carrying to the next OpMode, an app restart forgetting the homing, and an
    impossible count being dropped
  - an un-homed turret never counting as aimed
  - static error (0.03°)
  - 60°/s tracking error (1.6°)
  - no hunting
  - dead-zone handling
  - that "aimed" is never reported while the true error exceeds the tolerance

Run both from the project root after changing tables or turret constants:

```bash
mkdir -p build/sotm-test && javac --release 8 -d build/sotm-test TeamCode/src/main/java/org/firstinspires/ftc/teamcode/shooter/{ShooterConstants,InterpolatingTable,ShotSolution,ShotSolver,BioBuzzField,Alliance,HiveCell,BallType,MotionEstimator,TurretMath,TurretController}.java TeamCode/src/test/java/org/firstinspires/ftc/teamcode/shooter/*.java && java -cp build/sotm-test org.firstinspires.ftc.teamcode.shooter.SolverSimTest && java -cp build/sotm-test org.firstinspires.ftc.teamcode.shooter.TurretSimTest
```

## Files

| File | Purpose |
|---|---|
| `ShooterConstants` | every tunable number (MEASURE = placeholder), both shot tables |
| `BioBuzzField` | HIVE CELL aim points, field frame, reset pose, AprilTag IDs |
| `BallType` | POLLEN / NECTAR (diameter, table selection) |
| `ShotSolver`, `ShotSolution` | the math above (no hardware, testable) |
| `MotionEstimator` | velocity smoothing and acceleration estimate (Pedro 3 has none) |
| `Turret` | turret hardware: Axon servo(s) out, external encoder in |
| `TurretController`, `TurretMath` | homing, servo lead, encoder correction, encoder aim check, angle↔position map, range handling (no hardware, testable) |
| `Flywheel` | kS + kV feedforward + P, full power when far below target |
| `Hood` | servo hood with a travel-time model for "settled" |
| `Feeder` | motor or CR servo transfer, one pulse per ball |
| `HiveState` | which CELL is up (swaps on every TIP) |
| `ShootOnTheMove` | runs everything each loop |
| `PoseStorage` | hands pose, alliance and up CELL from AUTO to TeleOp |
| `opmodes/ShootOnTheMoveTeleOp` | match TeleOp |
| `opmodes/ShooterCalibration` | builds the shot tables |
| `opmodes/TurretServoSetup` | finds the turret servo zero and scale, encoder sign and mounting, and limits |
| `opmodes/FlywheelTuner` | measures flywheel kS/kV (open loop) and shot recovery (closed loop) |
| `opmodes/HoodFeederTest` | finds hood servo limits and travel time, checks feeder direction |
| `TUNING.md` | the tuning tutorial |

## Setup and tuning

Follow **[TUNING.md](TUNING.md)**, step by step and in order: Pedro, field frame, turret,
flywheel, hood and feeder, aim points, shot table, then still, spin and moving shot tests, gates
and HIVE tips. It names the OpMode, the constants and the pass check for every step.

## Controls (TeleOp)

- **INIT:** gamepad1 X = blue, B = red, Y = home the turret.

- **Driver (gamepad1):**
  - left stick: drive (field-centric)
  - right stick X: turn
  - right bumper: slow
  - left bumper: arm flywheel
  - right trigger: fire. It rumbles when the driving is what blocks the shot: side, angle,
    speed, or the CELL behind the turret's dead zone (rotate the robot).
  - left trigger: **force fire**. It feeds no matter what: no aim, flywheel, range or HIVE-tip
    check, not even "flywheel armed" or gamepad2's hold fire. Telemetry shows (FORCED).
  - back: reset pose in the alliance corner
- **Operator (gamepad2):**
  - Y: our HIVE tipped
  - X: undo TIP
  - B: next ball POLLEN or NECTAR (only if `SHOOTS_NECTAR`)
  - left bumper: hold fire
  - dpad: aim and distance trims
  - A: clear trims

## Not included yet

- AprilTag sensing of the up CELL. `HiveState.setUpCell` is the hook; a CELL whose tag cluster
  reads |roll| < 90° is up. Your partner fills the same CELL, so counting your own shots
  cannot tell when it tips.
- Ball-type sensing (color sensor at the feeder). The hook is `ShootOnTheMove.setNextBall`.
- A fill estimate that pauses after each shot once a TIP is likely (report, HIVE-state machine).
- Vision relocalization. Pedro's `FusionLocalizer` handles latency-stamped poses.
- AUTO routines. Call `shooter.update(fire)` after `follower.update()`, then save
  `PoseStorage` at the end. Expect the first TIP after about 3 POLLEN.
