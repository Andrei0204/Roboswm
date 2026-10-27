# Shooter tuning guide

How to tune the BIOBUZZ shoot-on-the-move robot: Pedro Pathing 3, Axon servo turret with
external encoder, flywheel, hood and feeder. **Do the steps in order.** Each one relies on the
ones before it: a perfect shot table is useless if the turret zero is wrong, and lead tuning is
useless if the table is wrong.

## How to change a value

Every number is a `public static` field in `ShooterConstants.java` or `BioBuzzField.java`. Edit
the field, then redeploy the app (Run in Android Studio). No live dashboard is installed. Panels
or FTC Dashboard could be added later; all fields are already public static.

Ground rules:
- Change **one thing at a time** and keep a log (value, result).
- Use a **charged battery (≥ 12.5 V)** for anything that involves shots.
- Use **fresh, clean POLLEN**. Worn balls fly differently.

## Overview

| # | Step | Tool (OpMode) | You set | Done when |
|---|---|---|---|---|
| 0 | Before you start | – | Robot Configuration names | every device found at INIT |
| 1 | Pedro Pathing | Pedro AutoTune, *Shoot On The Move* telemetry | `pedro/Constants.java` | pose and velocity signs correct, 24 in drive reads 24 ± 0.5 |
| 2 | Field frame and reset pose | *Shoot On The Move* | `RED_RESET_*` | pose correct at a known tile corner |
| 3 | Turret servo and encoder | *Turret Servo Setup* | `TURRET_*` | sweep check within 2%, homing works |
| 4 | Flywheel | *Flywheel Tuner* | `FLYWHEEL_*` | back at speed quickly after every shot |
| 5 | Hood and feeder | *Hood & Feeder Test*, *Flywheel Tuner* | `HOOD_*`, `FEEDER_*` | hood never hits a stop; one ball per pulse |
| 6 | HIVE aim points | tape measure / CAD | `BioBuzzField` | telemetry distance matches the tape |
| 7 | Shot table | *Shooter Calibration* | `SHOT_TABLE_POLLEN` | 5/5 still shots at every row |
| 8 | Still-shot checks | *Shoot On The Move* | trims, `MAX_APPROACH_ANGLE_DEG` | scores from every spot you want to shoot from |
| 9 | Turret lag and latency | *Shoot On The Move* (spin tests) | `TURRET_SERVO_LEAD_SEC`, `LOOKAHEAD_SEC` | no miss bias while spinning |
| 10 | Moving shots | *Shoot On The Move* (drive-bys) | `LEAD_DRAG_COEFF`, ToF column | no bias while strafing or driving in and out |
| 11 | Gates and limits | *Shoot On The Move* | tolerances, `MAX_SHOT_*`, filters | moving hit rate meets your target |
| 12 | HIVE tips | *Shoot On The Move* | `HIVE_TIP_PAUSE_SEC` | never fires into a tipping HIVE |
| 13 | Match day | – | – | checklist below |

---

## 0. Before you start

**Robot Configuration.** Device names must match `ShooterConstants`:

| Device | Type in Robot Configuration | Constant (default name) |
|---|---|---|
| Turret Axon MAX | Servo | `TURRET_SERVO_NAMES` (`turret`) |
| Turret encoder | Motor on the port whose encoder slot it uses | `TURRET_ENCODER_NAME` (`turretEncoder`) |
| Flywheel motor(s); the first has the encoder | Motor | `FLYWHEEL_MOTOR_NAMES` (`flywheel`) |
| Hood servo | Servo | `HOOD_SERVO_NAME` (`hood`) |
| Feeder | Motor or CR Servo | `FEEDER_NAME` (`feeder`) |
| Gate (between feeder and launcher) | Servo | `GATE_SERVO_NAME` (`gate`) |
| Intake | Motor | `INTAKE_MOTOR_NAME` (`intake`) |
| Intake servo (3 positions) | Servo | `INTAKE_SERVO_NAME` (`intakeServo`) |

The turret encoder must **not** be on a Pedro dead-wheel port, and no code of yours may reset
that port. Homing resets it, and that briefly stops any motor plugged into the same port, so use
a free port if you can.

**Bring:**
- tape measure
- a carpenter's square or protractor
- phone with 240 fps slow motion
- ≥ 10 POLLEN
- a CELL opening at the right height: a real HIVE, or a 20 in wide mock-up centered 58.4 in high
  and tilted 30°

---

## 1. Pedro Pathing (drive and localization)

Nothing aims correctly until the pose and velocity are right.

1. Register the AutoTune procedures in `pedro/Tuning.java`, as Pedro's AutoTune docs show:
   static methods annotated `@Tuner(name = "...")` that take no parameters and return a
   `Procedure`.
   - `MecanumTuner` (wheel directions)
   - your localizer's tuner, e.g. `PinpointTuner` (pod scalar, directions, offsets)
   - `ForesightTuner`
   - `Tests`

   `ForesightTuner` and `Tests` take your localizer and drivetrain factories.
2. Open the AutoTune page while connected to the Control Hub's Wi-Fi. It is served on port 10158
   (normally `http://192.168.43.1:10158`). Run the procedures in the order above.
3. Put the generated configs into `Constants.create()`:
   `return new Follower(localizer, drivetrain, new Foresight(foresightConfig));`
   The order is localizer, then drivetrain, then algorithm.
4. Check the frames in **Shoot On The Move** (*Pose* and *Velocity (filtered)*):

| Test | Must read |
|---|---|
| Push the robot toward the blue wall (+x) | vx > 0 |
| Turn it to face the rear wall, then drive forward | vy > 0, vx ≈ 0 |
| Spin counter-clockwise (seen from above) | ω > 0 |
| Drive exactly 24 in (tape) | pose changes 24 ± 0.5 in |
| Spin 10 turns and return to the start heading | heading error < 2° |

If vx/vy look rotated by the robot heading, the localizer's velocity is not field-frame. Fix
that in Pedro before continuing.

---

## 2. Field frame and reset pose

The code uses Pedro's frame:
- origin at the corner where the **red** alliance wall meets the **audience** wall
- +x toward the blue wall, +y toward the rear wall, inches
- heading counter-clockwise, with 0 facing +x

TeleOp without AUTO assumes the robot starts in the **reset corner**: back against the red wall,
right side against the audience wall, facing +x.

1. Measure from the robot center (the point your localizer offsets are measured from):
   - `RED_RESET_X_IN` = distance to the robot's **back** edge
   - `RED_RESET_Y_IN` = distance to its **right** edge

   Blue uses the same pose rotated 180°, automatically.
2. Check: place the robot in the corner, start *Shoot On The Move*, press **gamepad1 back**.
   Drive the robot center over the tile-seam corner two tiles from each wall. *Pose* should read
   about (48, 48) ± 1.5 in; real tiles are slightly under 24 in.

---

## 3. Turret: servo and encoder

Tool: **Turret Servo Setup** (no Pedro needed).

1. Check the defaults first:
   - `TURRET_SET_PWM_RANGE = true` gives the Axon's full 500–2500 µs travel. If you change it
     later, redo this whole step.
   - `TURRET_ENCODER_TICKS_PER_REV` is 8192 for a REV Through Bore.
2. Jog with dpad left/right (hold right bumper for fine steps). Use a square against the frame
   for the angles.
   - turret shooting **straight forward** → press **A**
   - turret exactly **90° to the robot's left** (counter-clockwise) → press **B**
3. Paste the four printed lines into `ShooterConstants`:
   - `TURRET_SERVO_POSITION_AT_ZERO`
   - `TURRET_DEG_PER_SERVO_UNIT`: expect about 299, from ~350° × the 0.855 gearing. Far off
     means a wrong angle or a slipping gear.
   - `TURRET_ENCODER_SIGN`
   - `TURRET_ENCODER_REVS_PER_TURRET_REV`: 1.000 means the encoder is on the turret shaft,
     2.923 on the 24T/26T shaft, 1.169 on the servo shaft. Anything else means the ticks per
     rev or the mounting are wrong.
4. Redeploy and run it again. Dpad up goes to zero, and the turret must point straight forward.
5. **Limits.** Press dpad up (turret to zero), then **A**, so *Encoder angle from A* reads from
   straight forward. Jog slowly to each end of safe travel (cables, frame, hard stops). Set
   `TURRET_MIN_DEG` / `TURRET_MAX_DEG` from that angle, minus about 3° margin.
6. Press **Y** to sweep end to end. The encoder span must be within **2%** of the expected
   span. A negative percentage means the encoder sign is wrong. Otherwise redo A/B.
7. **Turret offset.** Measure the turret's rotation axis from the robot center:
   - `TURRET_OFFSET_X_IN`, + forward
   - `TURRET_OFFSET_Y_IN`, + left

   This matters while the robot spins: at 3 rad/s a 5 in offset is a 15 in/s error.
8. **Homing check** (*Shoot On The Move*):
   - In INIT, press **gamepad1 Y**. The turret goes to zero and INIT shows "homed, encoder angle
     ~0.0". Nothing else moves in INIT.
   - Stop and start the OpMode again. It should already say "homed": the count carried over.
   - Power-cycle the robot. It must say "NOT HOMED since power-on".

---

## 4. Flywheel

Tool: **Flywheel Tuner** (no Pedro needed). Keep hands clear: the flywheel spins at 0.3 power
as soon as you press START. **B** stops and starts it, **A** switches open/closed loop.

1. **Encoder and direction.**
   - `FLYWHEEL_TICKS_PER_REV` is 28 for a goBILDA 5203 read at the motor. If the encoder sits
     after a gear, use counts per flywheel-side revolution, and remember the table rpm is then
     at that shaft.
   - In open loop at 0.3 power, *Measured* rpm must be **positive** and the wheel must spin in
     the shooting direction. If not, flip `FLYWHEEL_MOTOR_REVERSED`.
   - With two motors, each must push the same way. Test them one at a time by unplugging the
     other.
2. **kS and kV (open loop):**
   - Set 0.30 power with dpad, wait until the rpm is steady, press **X**.
   - Set 0.80, wait, press **Y**.
   - Paste `FLYWHEEL_KV` and `FLYWHEEL_KS`. They are normalized to 12 V.
   - Note the *top speed* line: keep every shot table rpm under about 85% of it.
3. **Recovery (closed loop, A).**
   - Set a typical target (e.g. 3000 rpm) and let it settle; it must sit inside the window.
   - Load one ball and press **right trigger**. Repeat 5 times and read *Last shot*.

| What you see | Change |
|---|---|
| Never gets inside the window when idle | kV is off: redo step 2 |
| Back at speed in > 300 ms | `FLYWHEEL_KP` × 1.5, or lower `FLYWHEEL_BANG_BANG_RPM` (e.g. 200) |
| Rpm swings above the window after recovering | `FLYWHEEL_KP` × 0.7 |
| Dip is huge (> 400 rpm) | mechanical: more flywheel mass or compression check |

Leave `FLYWHEEL_TOLERANCE_*` at their defaults for now; step 11 sets them from real data.

---

## 5. Hood and feeder

Tool: **Hood & Feeder Test**. At START the hood moves to the middle of its current limits.
After that the limits are **not** applied, so move in small steps.

1. If the hood servo is an Axon or goBILDA and you want its full travel, set
   `HOOD_SET_PWM_RANGE = true` **before** this step. Positions depend on the pulse range.
2. Jog with dpad left/right (right bumper = fine) to just before each hard stop. Press **X** at
   one end and **Y** at the other, then paste `HOOD_MIN_POSITION` / `HOOD_MAX_POSITION`.
3. Press **A** to swing between the ends and film it. Time one swing (t, in seconds):
   `HOOD_SECONDS_PER_FULL_RANGE = t / (HOOD_MAX_POSITION − HOOD_MIN_POSITION)`, plus about 15%.
   Too small a value fires before the hood arrives.
4. **Gate.** With dpad up/down, move the gate to just short of fully closed (balls held back) and
   press **left bumper**, then to open (balls pass freely) and press **B**. Paste
   `GATE_CLOSED_POSITION` / `GATE_OPEN_POSITION`. Film it opening and set `GATE_OPEN_DELAY_SEC`
   to that time plus a little. The robot opens the gate when a feed is wanted, and the feeder
   only starts after that delay.
5. **Intake servo.** In **Hood & Feeder Test**, use **gamepad2** dpad up/down to move it. Press
   **X** at the stowed position, **A** at collect-all and **B** at collect-pollen-only, then paste
   the printed numbers into the `IntakeServo.Mode` enum values (`INIT(...)`, `COLLECT_ALL(...)`, `COLLECT_POLLEN(...)`). In the TeleOp, dpad up/left/down switch between them.
6. **Intake direction.** In *Shoot On The Move*, press **A** with a ball near the intake: it must
   pull balls in toward the feeder. If not, make `INTAKE_POWER` negative. Hold **X** to check
   reverse. The intake also always runs during launches.
7. **Feeder direction.** With no balls loaded, holding **right trigger** must move balls toward
   the flywheel. If not, make `FEEDER_FIRE_POWER` negative.
8. **One ball per pulse.** In **Flywheel Tuner** closed loop, load 2 balls and press the right
   trigger once.
   - Exactly one ball must leave.
   - The ball doesn't fully leave → raise `FEEDER_PULSE_SEC`.
   - Two leave → lower it.
   - If balls creep into the flywheel while idle, set `FEEDER_IDLE_POWER` slightly negative
     (about −0.1).

---

## 6. HIVE aim points

FIRST publishes no coordinates for the CELL openings. The `BioBuzzField` defaults are derived
from the manual's figures, about ±1 in.

1. Measure, in the official Onshape field CAD or on a real field:
   - `CELL_OPENING_AXIAL_OFFSET_IN`: field center to the opening center, along the
     audience–rear axis (manual figures give 16.5–18.1)
   - `HIVE_SIDE_OFFSET_IN`: field center to the HIVE centerline (12.75)
2. `FIELD_CENTER_IN`:
   - **72** if your poses use Pedro's 144 in convention, as in step 2
   - **half the measured wall-to-wall distance** if you measure poses from the real walls
     (about 70.75 on a 141.5 in field)
3. **Check.** Park the robot still in front of the up CELL. Tape-measure the horizontal distance
   from the turret axis to the point under the opening center. *Distance real* in telemetry must
   match within 1 in, and the shooter must visibly point at the opening's lateral center.

---

## 7. Shot table

Tool: **Shooter Calibration**. Steps 1–6 must be done.

1. In INIT, choose the alliance (X blue / B red) and press **Y** to home the turret: it won't
   fire unhomed. Start with the robot in the reset corner, or with a pose from AUTO.
2. Press **left bumper** to arm the flywheel.
3. For each distance from about **16 to 88 in**, every 8–12 in (only distances you can really
   shoot from):
   1. Park **still** and **head-on** to the up CELL (*Approach angle* < 10°).
   2. Press **Y** to load the current table values for this distance.
   3. **Hood** (dpad left/right, left trigger = fine): pick the angle where the ball comes
      **down into** the opening at about 20–40°. The opening faces 30° up, so a ball descending
      along it has the most room.
   4. **Rpm** (dpad up/down, left trigger = fine): find the lowest and highest rpm that still
      score. Set the middle, then fire 5 balls (right trigger): 5/5 must score.
   5. Write down the **make window** (highest − lowest rpm); step 11 needs it.
   6. Press **A** to record the row.
4. **Time of flight.** Film each row from the side at 240 fps. ToF = frames from leaving the
   hood to crossing the opening plane ÷ 240. Average 3 shots.
5. Paste the rows into `SHOT_TABLE_POLLEN` with **distances strictly increasing** (otherwise the
   OpMode stops at INIT with an error), replacing `TOF` with your values. Redeploy.

`SHOT_TABLE_NECTAR` only matters if you set `SHOOTS_NECTAR = true`.

---

## 8. Still-shot checks

Tool: **Shoot On The Move**, robot not moving.

1. Pick 5 spots in front of the up CELL: near, middle and far, and at 0°, 20° and 40° to the
   opening. Fire 5 balls from each.
2. Read the misses:

| Pattern | Cause | Fix |
|---|---|---|
| Always left or always right, at every distance | turret zero | redo step 3 A/B. On the field: gamepad2 dpad left/right trim (0.5° steps) |
| Left/right miss that grows with distance | turret scale | redo step 3 (the 90° point) |
| Always short or always long | table | gamepad2 dpad up/down distance trim (1 in steps), then fix the table rows |
| Short only far away | table end or flywheel top speed | add a far row; check the 85% top-speed rule |
| Misses only at large angles | opening too oblique | lower `MAX_APPROACH_ANGLE_DEG` to ~5° below where makes start failing |

Trims reset when the OpMode restarts, so put the real fix in the constants.

---

## 9. Turret lag and latency (spin tests)

Tool: **Shoot On The Move**. Park about 50 in from the CELL and spin slowly in place at a steady
rate (small right-stick input, about 60–90°/s). Do it in both directions.

1. **`TURRET_SERVO_LEAD_SEC`** (no shooting). Watch *Turret error / allowed* and *Turret ff
   rate*.
   - Error has the **same sign** as the ff rate (the turret lags): raise the lead by 0.01 s.
   - **Opposite** sign: lower it.
   - Done when the error averages under 1° while spinning.
2. **`LOOKAHEAD_SEC`** (now shoot while spinning). The turret tracks its target from step 1, so
   any bias left is sensor and loop delay.
   - Shots offset **in the direction the robot spins** → raise `LOOKAHEAD_SEC` by 0.01 s.
   - Offset the other way → lower it.
   - Typical range: 0.03–0.08 s.

---

## 10. Moving shots

Tool: **Shoot On The Move**, holding the trigger. It fires only when every check passes.

1. **Sideways (`LEAD_DRAG_COEFF`).** Strafe across the face of the CELL at about 50 in, at a
   steady 20 in/s, then 40 in/s.
   - Shots land **behind** (offset opposite to the robot's motion), meaning too much lead: raise
     `LEAD_DRAG_COEFF`, starting at 0.3 and then in steps of 0.1.
   - Shots land **ahead**: lower it (it may go slightly negative).
2. **In and out (time of flight).** Drive slowly straight away from, then toward, the CELL.

| Pattern | Cause | Fix |
|---|---|---|
| Short going away, long coming toward | ToF column too small | raise the ToF values ~5–10% (or re-film them) |
| Long going away, short coming toward | ToF column too large | lower them |

Driving **straight at** the CELL shortens the virtual distance below the table (TOO_CLOSE)
quickly. The drivers should cross the face instead.

---

## 11. Gates and limits

- **Flywheel window.** Set `FLYWHEEL_TOLERANCE_BELOW_RPM` / `FLYWHEEL_TOLERANCE_ABOVE_RPM` to
  about **half the make window** from step 7. Too tight wastes time; too loose misses.
- **Speed limits.** Start with the defaults: 60 in/s, 3 rad/s, 120 in/s².
  - While moving shots stay at or above your target (e.g. 90% makes), raise
    `MAX_SHOT_SPEED_IN_PER_SEC`, `MAX_SHOT_OMEGA_RAD_PER_SEC` and `MAX_SHOT_ACCEL_IN_PER_SEC2`
    by 10–20% at a time.
  - *Problems* in telemetry shows which gate blocks, and the driver's gamepad rumbles when
    driving is the reason.
- **Motion filters.** At rest, *Accel (filtered)* should stay under about 10 in/s².
  - If it's noisy (common with two- or three-wheel odometry), raise `ACCEL_FILTER_SEC` to
    0.10–0.15, or set `USE_ACCELERATION = false`.
  - Keep `VELOCITY_FILTER_SEC = 0` unless the velocity visibly jumps.
- **`READY_DEBOUNCE_LOOPS`.** Use 2–3; raise it if single bad loops cause misses.
- **`AIM_MARGIN_IN`.** Keep it at 2 in. The opponent's CELL is only ~5.5 in away (G417).
- **Turret correction.** Only touch it if needed:

| Symptom | Fix |
|---|---|
| Turret buzzes or hunts when still | raise `TURRET_P_DEADBAND_DEG` to half the backlash, or lower `TURRET_ENCODER_KP` (must stay < 1) |
| Slow to settle on target | `TURRET_CORRECTION_GAIN` up to 6 (simulated stable to 6) |
| *correction* sits near ±8° | servo map is off: redo step 3 |

---

## 12. HIVE tips

1. Film a real TIP. Set `HIVE_TIP_PAUSE_SEC` to the time from tip start to the HIVE settling,
   plus 0.2 s.
2. Check `RED_START_UP_CELL` / `BLUE_START_UP_CELL` against the current manual. Defaults: red
   audience, blue rear.
3. **Operator drill:** press **gamepad2 Y** on every TIP of your HIVE (X undoes a mistake). The
   target jumps to the other CELL, and the robot must shoot from the other half of the field;
   telemetry shows WRONG_SIDE_OR_ANGLE until it gets there.

---

## 13. Match-day checklist

**At each event:**
- Re-check step 6 (the HIVE structure may be off center by ±1 in).
- Fire one still-shot check (step 8) at two distances.

**Before each match:**
1. Battery ≥ 12.8 V. Turret free to move.
2. AUTO INIT: choose the alliance, press **gamepad1 Y**, and wait for "homed, encoder angle
   ~0.0". Your AUTO OpMode needs the same INIT code as *Shoot On The Move*: build a `RobotSys`,
   call `robot.initLoop()` every INIT loop, and call `robot.homeTurret()` on Y.
3. Place the robot exactly on the start pose.
4. TeleOp INIT: the turret should already say "homed". If it says NOT HOMED, press Y. Press
   **A** to put all servos in their start positions (and set `HOOD_START_POSITION` once).

**After any power cycle:** home again, or the turret will not shoot.

---

## Symptom → constant quick reference

| Symptom | Look at |
|---|---|
| Won't fire at all | *Problems* line; turret not homed; flywheel not *at speed*; `HOOD_SECONDS_PER_FULL_RANGE` too large. Gamepad1 left trigger (force fire) bypasses every check |
| Turret points the wrong way | `TURRET_SERVO_POSITION_AT_ZERO`, `TURRET_DEG_PER_SERVO_UNIT`, `TURRET_ENCODER_SIGN` (step 3) |
| "DEAD ZONE: rotate the robot" | target behind the turret's travel: turn the robot |
| Misses only while spinning | `TURRET_SERVO_LEAD_SEC`, `LOOKAHEAD_SEC`, `TURRET_OFFSET_*` (steps 3, 9) |
| Misses only while strafing | `LEAD_DRAG_COEFF` (step 10) |
| Misses only while driving in/out | ToF column (step 10) |
| Second ball of a burst short | flywheel recovery (step 4); `FEEDER_PULSE_SEC` |
| Everything off by the same amount | aim points / `FIELD_CENTER_IN` (step 6), pose reset (step 2) |
