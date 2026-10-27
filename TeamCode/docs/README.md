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

## Code layout

All under `src/main/java/org/firstinspires/ftc/teamcode/`.

| Package | Class | Purpose |
|---|---|---|
| `robot` | `RobotSys` | the whole robot for OpModes: Pedro, shooter, feeder, gate, intake, bulk caching AUTO |
| `aiming` | `ShootOnTheMove` | aiming: turret, flywheel, hood and the ready check (no feeder) |
| | `ShotSolver`, `ShotSolution` | the math above (no hardware, testable) |
| | `MotionEstimator` | velocity smoothing and acceleration estimate (Pedro 3 has none) |
| | `InterpolatingTable`, `BallType` | shot-table lookup; POLLEN / NECTAR |
| `subsystems` | `Turret` | turret hardware: Axon servo(s) out, external encoder in |
| | `TurretController`, `TurretMath` | homing, servo lead, encoder correction, encoder aim check, angle↔position map, range handling (no hardware, testable) |
| | `Flywheel` | kS + kV feedforward + P, full power when far below target |
| | `Hood` | servo hood with a travel-time model for "settled" |
| | `Feeder` | motor or CR servo transfer: one pulse per ball, separate from the shooter |
| | `Gate` | servo between feeder and launcher: open/closed positions and an open delay |
| | `Intake` | intake motor: always on while launching, on/off by hand, reverse to clear jams |
| | `IntakeServo` | the intake's 3-position servo; its `Mode` enum (INIT, COLLECT_ALL, COLLECT_POLLEN) holds the positions |
| `field` | `BioBuzzField` | HIVE CELL aim points, field frame, reset pose, AprilTag IDs |
| | `HiveState`, `HiveCell`, `Alliance` | which CELL is up (swaps on every TIP); sides |
| | `PoseStorage` | hands pose, alliance and up CELL from AUTO to TeleOp |
| `config` | `ShooterConstants` | every tunable number (MEASURE = placeholder), both shot tables |
| `opmodes` | `ShootOnTheMoveTeleOp` | match TeleOp |
| | `ShooterCalibration` | builds the shot tables |
| | `TurretServoSetup` | finds the turret servo zero and scale, encoder sign and mounting, and limits |
| | `FlywheelTuner` | measures flywheel kS/kV (open loop) and shot recovery (closed loop) |
| | `HoodFeederTest` | finds hood, gate and intake servo positions, checks feeder direction |
| `pedro` | `Constants`, tuners | Pedro Pathing setup |

Docs are in `TeamCode/docs/` ([TUNING.md](TUNING.md) is the tuning tutorial).

## Setup and tuning

Follow **[TUNING.md](TUNING.md)**, step by step and in order: Pedro, field frame, turret,
flywheel, hood and feeder, aim points, shot table, then still, spin and moving shot tests, gates
and HIVE tips. It names the OpMode, the constants and the pass check for every step.

## Controls (TeleOp)

- **INIT:** gamepad1 X = blue, B = red, Y = home the turret, A = move all servos to their
  start position (turret forward, hood start, gate closed, intake servo INIT). Those buttons are
  the only movement in INIT; the flywheel, feeder and intake motor stay still until START.

- **Driver (gamepad1):**
  - left stick: drive (field-centric)
  - right stick X: turn
  - right bumper: slow
  - left bumper: arm flywheel
  - right trigger: fire. It rumbles when the driving is what blocks the shot: side, angle,
    speed, or the CELL behind the turret's dead zone (rotate the robot).
  - A: intake on/off (it always runs during launches). X held: intake reverse.
  - dpad up: intake servo collect all. Dpad left: collect pollen only. Dpad down: back to init.
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
- AUTO routines. Build a `RobotSys`, follow paths with `robot.follower()`, and call
  `robot.update()` then `robot.shoot(fire, false)` once per loop (update also saves `PoseStorage`). Expect the first TIP
  after about 3 POLLEN.
