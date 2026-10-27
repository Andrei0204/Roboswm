# FTC Mechanism Control for Shoot-While-Moving: Turret, Flywheel, Adjustable Hood (FTC SDK 12.0, Java, REV Control Hub)

How this was checked: I confirmed the SDK 12.0 API facts by running `javap` on the actual `org.firstinspires.ftc:Hardware:12.0.0` and `RobotCore:12.0.0` AARs that this project resolves (Maven Central: https://repo1.maven.org/maven2/org/firstinspires/ftc/Hardware/12.0.0/ and https://repo1.maven.org/maven2/org/firstinspires/ftc/RobotCore/12.0.0/, both returned HTTP 200, and the copies in the local Gradle cache were the ones inspected). Online javadoc on javadoc.io is published per version; I read the 10.3.0 pages, and where those pages and the 12.0 bytecode both cover a method, the signatures match. The only open-source DECODE (2025–26) turret, flywheel and hood codebase I could read in full is FTC 11138 "Robo Eclipse", repo https://github.com/FelixYBW/FTC11138. The two Medium write-ups on DECODE shooters returned HTTP 403, so only their search-result summaries are available. Where I cite those summaries, I say so.

---

## Flywheel velocity control: built-in setVelocity/PIDF vs. custom feedforward + P / bang-bang; voltage compensation; encoder units and resolution; getVelocity behavior and the 16-bit issue; recovery; what top DECODE teams used

### Takeaway
The DECODE code I could inspect does not use the hub's internal PIDF. It runs the flywheel in `RUN_WITHOUT_ENCODER` with a custom controller computed every loop: `kV·target + kS·sign(target) + kP·error`, with optional I. The velocity input is still `getVelocity()`. That reading is measured on the hub over a 50 ms window, so it comes in steps of 20 ticks/s and arrives as a 16-bit value. Built-in `setVelocity()` also works. REV's own DECODE starter-bot material says the SDK velocity blocks already do PID. But the built-in loop runs on the hub at a slower, undocumented rate, cannot be voltage-compensated by the user, and cannot do bang-bang. Calling `setVelocity()` also silently switches the motor into `RUN_USING_ENCODER`.

### Cited Findings
**SDK 12.0 API (exact signatures, from javap on the 12.0 artifacts)**
- `DcMotorEx` in SDK 12.0 declares: `setMotorEnable()`, `setMotorDisable()`, `isMotorEnabled()`, `setVelocity(double)`, `setVelocity(double, AngleUnit)`, `getVelocity()`, `getVelocity(AngleUnit)`, `setPIDCoefficients(RunMode, PIDCoefficients)`, `setPIDFCoefficients(RunMode, PIDFCoefficients) throws UnsupportedOperationException`, `setVelocityPIDFCoefficients(double p, double i, double d, double f)`, `setPositionPIDFCoefficients(double p)`, `getPIDCoefficients`, `getPIDFCoefficients`, `setTargetPositionTolerance(int)`, `getTargetPositionTolerance()`, `getCurrent(CurrentUnit)`, `getCurrentAlert(CurrentUnit)`, `setCurrentAlert(double, CurrentUnit)`, `isOverCurrent()`. — [FTC SDK RobotCore 12.0.0 artifact](https://repo1.maven.org/maven2/org/firstinspires/ftc/RobotCore/12.0.0/)
- Javadoc semantics: `setVelocity(double angularRate)` takes "ticks per second". `getVelocity()` returns "the current velocity of the motor, in ticks per second". `setPIDFCoefficients` is "a superset of setPIDCoefficients" that adds F. Legacy PID requires F=0. In RUN_TO_POSITION both the RUN_TO_POSITION and RUN_USING_ENCODER coefficients are used, and only "p" is meaningful for RUN_TO_POSITION. `setPIDCoefficients` and `getPIDCoefficients` are marked `@Deprecated`. — [javadoc.io DcMotorEx (10.3.0)](https://javadoc.io/static/org.firstinspires.ftc/RobotCore/10.3.0/com/qualcomm/robotcore/hardware/DcMotorEx.html)
- **Mode-switch behavior (SDK 12.0 bytecode).** `LynxDcMotorController.setMotorVelocity(int, double)` keeps the current mode only if it is RUN_TO_POSITION or RUN_USING_ENCODER. Otherwise, including RUN_WITHOUT_ENCODER, it calls `setMotorMode(RUN_USING_ENCODER)`. It then rounds the target, clips it to `[-32767, 32767]` ticks/s, and sends a `LynxSetMotorTargetVelocityCommand`. So one stray `setVelocity()` call on a custom-controlled flywheel switches it to the hub's PID. — [FTC SDK Hardware 12.0.0 artifact](https://repo1.maven.org/maven2/org/firstinspires/ftc/Hardware/12.0.0/)
- **getVelocity data path (SDK 12.0 bytecode).** `LynxGetBulkInputDataResponse` stores `short[] velocities`, which are 16-bit signed, and `int[] encoders`. `LynxModule.BulkData.getMotorVelocity(int)` returns `int`. `LynxDcMotorController.getMotorVelocity(int, AngleUnit)` divides ticks/s by `MotorConfigurationType.getTicksPerRev()`, the motor type chosen in the Robot Configuration, before converting to degrees or radians. `DcMotorImplEx.getVelocity()` passes the value through `adjustAngularRate`, which applies the motor's `Direction`. — [FTC SDK Hardware 12.0.0 artifact](https://repo1.maven.org/maven2/org/firstinspires/ftc/Hardware/12.0.0/)
- **Built-in motor types (SDK 12.0 `@MotorType` annotations).**
  - `GoBILDA5202Series`: `ticksPerRev = 2786`, `gearing = 99.5`, `maxRPM = 60`, which is 28 ticks per motor revolution.
  - `RevRoboticsUltraPlanetaryHdHexMotor`: `ticksPerRev = 560`, `gearing = 20`, `maxRPM = 300`, which is 28 per motor revolution.
  - `RevRoboticsHdHexMotor` is marked `@Deprecated`.
  - Source: [FTC SDK Hardware 12.0.0 artifact](https://repo1.maven.org/maven2/org/firstinspires/ftc/Hardware/12.0.0/)
- goBILDA 5203 Yellow Jacket, 1:1 ratio, 6000 RPM: 28 PPR encoder resolution at the output shaft, Hall-effect quadrature, 3.3–5 V encoder supply, 12 VDC, 6,000 RPM no-load, 9.2 A stall. — [goBILDA product page](https://gobilda.com/5203-series-yellow-jacket-motor-1-1-ratio-24mm-length-8mm-rex-shaft-6000-rpm-3-3-5v-encoder). A goBILDA 5202 6 mm D-shaft variant lists a tested no-load speed of 5,400 RPM against the 6,000 theoretical figure. — [goBILDA 5202 1:1 page](https://gobilda.com/5202-series-yellow-jacket-motor-1-1-ratio-24mm-length-6mm-d-shaft-6000-rpm-3-3-5v-encoder) (via search summary)
- **16-bit overflow and quantization (Road Runner's FTC library).** The library's `Encoders.kt` comments say: "encoder velocities are sent as 16-bit ints"; "by the time they reach here, they are widened into an int and possibly negated"; and "the velocity is always a multiple of 20 cps due to Expansion Hub's 50ms measurement window". Its `OverflowEncoder.inverseOverflow(input, estimate)` works as follows:
  - mask the input with `input and 0xFFFF`;
  - correct the upper bits with `real += real % 20 / 4 * 0x10000`;
  - snap to the position-derived estimate with `real += round((estimate - real) / (5 * 0x10000)) * 5 * 0x10000`;
  - the estimate is `(pos - lastPos) / dt`, run through a filter;
  - `RawEncoder.applyDirection` negates the value for motor `REVERSE` and again for encoder `REVERSE`.
  - Source: [acmerobotics/road-runner-ftc Encoders.kt](https://raw.githubusercontent.com/acmerobotics/road-runner-ftc/master/RoadRunner/src/main/java/com/acmerobotics/roadrunner/ftc/Encoders.kt)
- DigitalChickenLabs advertises that its OctoQuad MK2 firmware avoids the REV Hub velocity integer-overflow issue. This independently confirms the issue exists. — [OctoQuad listing on Tindie](https://www.tindie.com/products/digitalchickenlabs/octoquad-ftc-ed-mk2-8x-encoderpwm-imu/) (via search summary)

**Built-in PIDF tuning recipe (REV)**
- Measure maxV by running at power 1 in RUN_WITHOUT_ENCODER on a full battery and logging the peak `getVelocity()`. Then F = 32767 / maxV, P = 0.1·F, I = 0.1·P, D = 0. Example: maxV = 2600 → F ≈ 12.6, P = 1.26, I = 0.126. Position P = 5.0. Call: `motor.setVelocityPIDFCoefficients(1.26, 0.126, 0, 12.6)`. "PIDF values are only remembered for the duration of the OpMode that they are set in." An `UnsupportedOperationException` saying "setting of pidf coefficients not supported" means hub firmware needs updating. — [REV docs: FGC PID](https://docs.revrobotics.com/first-global/fgc-pid)

**Custom-control guidance**
- "this internal velocity control runs at a slower frequency and will often have less desirable results." "We advise teams to use their own, external PID rather than the in built PID because it runs at a faster refresh rate." "RUN WITHOUT ENCODERS does NOT disable the encoder from doing things such as position or velocity measurement." — [CTRL ALT FTC: FTC Motor Control](https://www.ctrlaltftc.com/practical-examples/ftc-motor-control)
- Feedforward forms: `output = Kp*(refVel - curVel) + Kv*refVel`, and `output = PID(refVel, curVel) + Kv*refVel + Ka*refAccel`. Feedforward can stand in for the integral term when windup is a concern. — [CTRL ALT FTC: Feedforward Control](https://www.ctrlaltftc.com/feedforward-control)
- **REV DECODE starter bot.**
  - Quote: "When using the velocity blocks from the FTC SDK there is already PID calculations in place."
  - Their own version sets the motor to power with `targetPower = F*target + P*(target - current)`, starting at P = F = 0.0004.
  - Tune F first, until velocity is "consistently within 10 ticks" of target. Then raise P until the flywheel "accelerates quickly" back to speed after launches without oscillation.
  - Source: [REV DECODE Programming Tips & Tricks](https://docs.revrobotics.com/ftc-kickoff-concepts/decode-2025-26/programming-tips-and-tricks)

**Voltage compensation**
- Road Runner's quickstart gets `voltageSensor = hardwareMap.voltageSensor.iterator().next();` and applies `power = feedforward.compute(vel) / voltageSensor.getVoltage()`. — [road-runner-quickstart MecanumDrive.java](https://github.com/acmerobotics/road-runner-quickstart/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/MecanumDrive.java)
- `VoltageSensor` in SDK 12.0 has a single method, `double getVoltage()`. `LynxModule.BulkData` has no battery-voltage getter. Its getters are digital state, encoder position, velocity, busy, over-current and analog voltage. So a voltage read is a separate hub transaction that bulk caching does not cover. — [FTC SDK 12.0.0 artifacts](https://repo1.maven.org/maven2/org/firstinspires/ftc/Hardware/12.0.0/)

**What a DECODE team actually ran: FTC 11138 "Penumbra"**
- **Hardware and loop.** Two flywheel motors in `RUN_WITHOUT_ENCODER` with `ZeroPowerBehavior.FLOAT`. A hand-written PIDF on velocity error applied with `setPower()`, not the SDK's onboard PID.
- **Gains.** `shooterKp = 0.01`, `Ki = 0`, `Kd = 0`, `shooterKs = 0.065`, `shooterKv = 0.000365` (power per tick/s).
- **Integral and dt.** The integral is clamped to ±5000. `dt` comes from `System.nanoTime()` and falls back to 0.02 s if dt ≤ 0 or > 0.25 s.
- **Output.** Clipped to [-1, 1] and floored at 0 through `Math.max(0, pidPower)`, so the wheel is never driven backwards to brake.
- **Encoders.** Only one motor's encoder is read: `Math.abs(bottom.getVelocity())`. Both motors get the same power.
- **Tolerance.** `shooterVelocityTolerance = 100` ticks/s.
- **Unused helper.** The file has a `setVelocity(tps)` helper that is never called.
- **Approach latch.** Within 20 in of a shoot pose, the target is held at that pose's value to cut spin-up time.
- **Not in README.** The README does not describe voltage compensation or bang-bang.
- Sources: [Shooter.java](https://github.com/FelixYBW/FTC11138/blob/HEAD/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Shooter.java), [Constants.java](https://github.com/FelixYBW/FTC11138/blob/HEAD/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Constants.java), [README](https://github.com/FelixYBW/FTC11138)
- A DECODE shooter write-up, of which I saw only the search summary because the page returned 403, notes that while moving the turret and flywheel setpoints keep changing, so the controllers never settle the way they do when the robot is stationary. It recommends tuning for transient response and replacing fixed readiness thresholds with computed ones. — [Medium: Programming a DECODE shooter](https://medium.com/@vikramaditya.nishant/programming-a-decode-shooter-4ab114dac01f); [Medium: Programming a hooded DECODE shooter](https://medium.com/@vikramaditya.nishant/programming-a-hooded-decode-shooter-463d6a71832f)

### Inferences
- **Units and range for a 5203 1:1 flywheel.**
  - Free speed is 6000 rpm × 28 / 60 = 2800 ticks/s. That is far below the 16-bit limit of 32767, so the overflow cannot affect a 5203 or REV HD Hex read on the motor's own encoder.
  - 11138's kV = 0.000365 implies about 1/0.000365 ≈ 2740 ticks/s at full power, which is consistent.
  - Overflow does matter for an 8192-CPR REV Through Bore encoder: it overflows above 32767 / 8192 ≈ 4 rev/s (240 rpm) at the encoder shaft. Use `inverseOverflow`, or a velocity derived from position, for any Through Bore encoder that turns that fast.
- **Resolution and lag.**
  - 20 ticks/s quantization ≈ 0.71 % of 2800 ticks/s ≈ 43 rpm at 28 CPR.
  - The 50 ms measurement window means the reading lags the true velocity by roughly 25 ms on average. That limits how high kP can go and makes pure bang-bang overshoot or chatter.
  - A position-difference velocity over a 10–20 ms loop at 28 CPR is even coarser: only about 28–56 ticks per window at 2800 ticks/s. So keep using `getVelocity()` (bulk-cached) for 28-CPR flywheels.
- **Recommended controller (a synthesis, not a single-source recipe).**
  - `u = (kS·sign(ω*) + kV·ω* + kA·dω*/dt) · (12.0 / V_filt) + kP·(ω* − ω)`, clipped to [0, 1], with FLOAT zero-power behavior.
  - Optional "bang-bang assist": if `ω* − ω > band` (e.g. 100–200 ticks/s, roughly 11138's tolerance scale), output 1.0 for faster recovery after a shot; inside the band use FF + P.
  - Tune kS and kV from steady-state power-vs-velocity points with P = 0. Tune kP last for recovery.
  - Read battery voltage only every ~100–250 ms and low-pass it. It is not in the bulk packet, and it sags during each shot.
- **Do not mix control modes.** Never call `setVelocity()` on a flywheel you control with `setPower()`, because SDK 12.0 switches it to RUN_USING_ENCODER.
- **Configured motor type.** Set it correctly, or avoid the `AngleUnit` overloads. A 1:1 5203 configured as "goBILDA 5202/3/4" (2786 ticks/rev) makes `getVelocity(AngleUnit.DEGREES)` wrong by 99.5×. Raw ticks/s is unaffected.
- **Two-motor flywheels.** Mount the motors mechanically opposed and set `Direction` so positive power spins both the same way. Then either read one encoder (as 11138 does) or average both, and watch the sign, because `getVelocity()` applies `Direction`.
- **Built-in option.** `setVelocityPIDFCoefficients(P, I, D, F)` with F = 32767/maxV is a valid low-effort baseline. Re-apply it every OpMode. Note that maxV measured at full battery makes F voltage-dependent: there is no compensation.

### Gaps
- I found no official REV or FIRST statement of the hub's internal velocity-PID loop rate, or of the 50 ms velocity window. The 50 ms / 20 cps figure comes from a comment in Road Runner's source, which names the Expansion Hub. I assume it applies to the Control Hub too because both use the same Lynx board, but I could not confirm that.
- No REV or FIRST document describes the 16-bit velocity overflow. The evidence is the SDK's `short[] velocities` field, Road Runner's comments, and OctoQuad's marketing.
- I could not determine which approaches the top DECODE teams (Worlds-level) used beyond FTC 11138. The two Medium posts were inaccessible (403), and no other open-source DECODE turret shooter code surfaced in search.
- No measured recovery times after a shot (ms to return within tolerance) were found for any FTC flywheel.

---

## Turret: motor vs. servo drive, PID + feedforward, wraparound and soft limits, shortest legal path, homing/zeroing, gear ratios

### Takeaway
Run the turret with a feedback position loop plus feedforward. Feedforward alone does not work for turrets. While the robot moves, add feedforward terms for the robot's yaw rate and for the rate of change of the target bearing. Enforce soft limits at the power output and pick the shortest legal target among the 360°-equivalent angles. FTC hubs cannot read the REV Through Bore Encoder's absolute output, so zeroing must come from one of four sources: a known start pose, a limit switch, an analog absolute sensor (Axon feedback wire), or a pose carried over from autonomous.

### Cited Findings
- WPILib: "Feedforward-only control is not a viable control scheme for turrets!" Velocity feedforward has to be paired with feedback. — [WPILib: Tuning a Turret Position Controller](https://docs.wpilib.org/en/2024/docs/software/advanced-controls/introduction/tuning-turret.html) (via search summary)
- Chief Delphi turret-control suggestion: `output = PD(target_angle) + feedforward_for_drivebase_angular_velocity + feedforward_for_drivebase_to_target_velocity`. — [Chief Delphi: Turret Example Code](https://www.chiefdelphi.com/t/turret-example-code/385147) (via search summary)
- **FTC 11138 turret implementation.**
  - **Hardware and resolution.** A CR servo drives the turret. Feedback is an 8192-CPR Through Bore encoder on a spare motor-encoder port. 45:102 gearing gives `turretTicksPerRevolution = 8192 × 102/45 ≈ 18,570`, about 0.02°/tick.
  - **Control law.**
    - PID: `kP_v = 0.017`, `kI_v = 0`, `kD_v = 0.001`.
    - Clamps: integral at ±30, derivative at ±320.
    - Deadband: 1°. Inside it the controller resets and outputs 0.
    - Static friction: optional `kS·sign(power)`, currently 0.
    - Output limit: `maxPower = 0.6`.
    - dt from nanoTime, with a 0.02 s fallback.
  - **Soft limits.** −175° to +90°, applied at the power level: power that would push further past a limit is zeroed. Targets are `Range.clip`'ed to [min, max]. There is no wrap logic.
  - **Zeroing.** At init: `STOP_AND_RESET_ENCODER`, then `RUN_WITHOUT_ENCODER`, then an angle offset set from a known start angle. A static field carries the angle from autonomous into teleop.
  - **Aiming geometry.** The turret pose is offset from the robot center (forward −0.15 in, left −2.17 in), and the goal bearing is solved from that point.
  - **Autonomous.** The turret pre-aims at the upcoming shoot pose and switches to live aim within 3 in.
  - Sources: [Turret.java](https://github.com/FelixYBW/FTC11138/blob/HEAD/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Turret.java), [Constants.java](https://github.com/FelixYBW/FTC11138/blob/HEAD/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Constants.java), [README](https://github.com/FelixYBW/FTC11138)
- REV Through Bore Encoder: "The FTC Control System (Control Hub and Expansion Hub) only supports incremental encoder input through the motor encoder ports at this time". Absolute pulse input is not supported. The factory zero cannot be recalibrated. The V2 (REV-11-3174) has the same FTC restriction. — [REV Through Bore Encoder docs](https://docs.revrobotics.com/through-bore-encoder); [REV TBE V2 docs](https://docs.revrobotics.com/rev-crossover-products/sensors/tbe)
- `LynxModule.BulkData` in SDK 12.0 includes `getAnalogInputVoltage(int)` and `getDigitalChannelState(int)`, so analog absolute encoders and limit switches cost nothing extra once bulk caching is on. `AnalogInput` has `getVoltage()` and `getMaxVoltage()`. — [FTC SDK RobotCore 12.0.0 artifact](https://repo1.maven.org/maven2/org/firstinspires/ftc/RobotCore/12.0.0/)
- "although getMaxVoltage() returns 3.3v, the expansion and control hub analog input ports can safely handle up to 5v". — [gm0 (RU mirror): Reading and Writing to Hardware](https://gm0-ru.readthedocs.io/ru/latest/docs/software/getting-started/common-hardware-components.html) (via search summary)
- Axon MINI and MAX MK2 have a 4th wire for analog position feedback. Axon MINI MK2 specs:
  - PWM: max range 500–2500 µs; continuous mode 1050–1950 µs; 2 µs deadband.
  - Travel: about 0.175°/µs, 350° max.
  - Speed: about 0.11 s/60° at 4.8 V to 0.07 s/60° at 8.4 V.
  - Sources: [goBILDA: Axon MINI Servo MK2](https://www.gobilda.com/axon-mini-servo-mk2/); [ServoCity: Axon MAX Servo MK2](https://www.servocity.com/axon-max-servo-mk2/) (via search summary)
- FTC auto-aim practice: one team computes the target angle from Pinpoint odometry x/y. Another runs a PID on the Limelight 3A tag offset. Pinpoint accuracy drifted late in teleop. — [Chief Delphi: FTC auto-aiming](https://www.chiefdelphi.com/t/ftc-auto-aiming/524081)
- FRC 5572 lists "whipping around its travel limit" as a state in which the turret is not aimed. Their fix sets `isAimed() = !currentlyWhipping`. — [Frc5572/FRC2026 issue #230](https://github.com/Frc5572/FRC2026/issues/230)

### Inferences
- **Hardware choice.**
  - *DC motor + encoder*, e.g. a geared Yellow Jacket: the 28-CPR encoder is in the bulk packet and you get torque headroom.
  - *CR servo + external encoder*, as 11138 does: cheap, but the servo deadband (Axon continuous mode 1050–1950 µs) adds a kS-like nonlinearity.
  - *Positional servo* (Axon 350° or goBILDA 300° at 500–2500 µs): simplest, but open-loop, limited to about 0.07–0.11 s/60° at the servo, and it accepts no velocity feedforward.
  - For shoot-on-move, a motor or CR-servo turret with closed-loop FF + PD tracks a moving setpoint best.
- **Control law.**
  - `u = kP·e + kD·(ė) + kV·ω_ref + kS·sign(ω_ref or e)`.
  - `ω_ref = d/dt(turret target angle) = (d/dt field bearing to goal) − ω_robot`.
  - `d/dt bearing = (relative velocity perpendicular to the goal line) / distance`.
  - Get `ω_robot` from the odometry heading rate (Pinpoint or Pedro).
  - Prefer the derivative of the measurement over the derivative of the error so setpoint steps don't spike D. Give I a small clamp, as 11138 does, or drop it and use kS instead.
- **Shortest legal path with cable limits [min, max]**, where the span may exceed 360°:
  1. Compute the desired robot-relative angle θ.
  2. Generate the candidates θ + 360k that lie inside [min, max].
  3. Pick the one closest to the current angle.
  4. If there is none (θ falls in the dead zone), clamp to the nearer limit and report not-aimed.
  5. If the best candidate is more than about 180° away, the turret is "whipping"; block feeding until it arrives.
  - Never normalize the measured turret angle into [−180, 180). Keep it continuous so the limits are meaningful.
- **Zeroing options.**
  - Mechanical hard stop or jig at init, then `STOP_AND_RESET_ENCODER`.
  - A digital limit or Hall switch: home slowly at low power, latch the offset on the edge.
  - An analog absolute signal (Axon feedback or a potentiometer) through a 1:1 or known ratio. This is ambiguous if the sensor turns more than once per turret revolution.
  - Carry the angle from auto to teleop through a static field, as 11138 does.
- **Gear-ratio conversions.**
  - Turret angle (deg) = `ticks / (CPR_encoder × encoderRevsPerTurretRev) × 360`.
  - For a motor-mounted 28-CPR encoder behind gearbox G and belt ratio R: ticks per turret rev = `28 × G × R`.
  - Check the overflow limit for 8192-CPR encoders on fast turrets. With 11138's 102/45 ratio, the encoder overflows at turret speeds above about (32767/8192)/(102/45) ≈ 1.76 rev/s ≈ 635°/s.

### Gaps
- No published FTC turret gains, or measured settling times, other than 11138's constants.
- No primary Axon documentation for the feedback-wire voltage range or angle mapping. The goBILDA and ServoCity listings say only that it is analog position feedback.
- I found no guidance specific to DECODE on the cable-wrap span top teams use.

---

## Hood: servo position mapping, calibration, servo speed and shot readiness, `Servo.scaleRange`, `ServoImplEx.setPwmRange`

### Takeaway
Map hood angle to servo position with a calibrated linear or piecewise-linear table. Shot tables can also be indexed directly in servo units, as 11138 does. If the servo is a goBILDA or Axon unit, widen the PWM range to 500–2500 µs; the SDK default is 600–2400 µs. Do not use `Servo.getPosition()` as a readiness check, because it returns the last commanded value. Estimate arrival time from servo speed, or read the Axon analog feedback.

### Cited Findings
- **`Servo` javadoc.**
  - `setPosition` takes [0.0, 1.0].
  - `getPosition()` returns "the position to which the servo was last commanded to move" and "does NOT read a position from the servo through any electrical means".
  - `scaleRange(min, max)` "Scales the available movement range of the servo to be a subset of its maximum range". Its arguments are relative to the full range, so `scaleRange(0.0, 1.0)` resets it. Example: with `scaleRange(0.2, 0.8)`, `setPosition(0)` maps to 0.2 and `setPosition(1)` to 0.8.
  - Servos are guaranteed FORWARD at the start of an OpMode.
  - Source: [javadoc.io Servo (10.3.0)](https://javadoc.io/static/org.firstinspires.ftc/RobotCore/10.3.0/com/qualcomm/robotcore/hardware/Servo.html)
- **SDK 12.0 `ServoImplEx` (javap).** Methods: `setPwmRange(PwmControl.PwmRange)`, `getPwmRange()`, `setPwmEnable()`, `setPwmDisable()`, `isPwmEnabled()`, `setPulseWidth(double)`, `getPulseWidth()`. `PwmControl.PwmRange` defaults: `usPulseLowerDefault = 600.0`, `usPulseUpperDefault = 2400.0`, `usFrameDefault = 20000.0`. Constructors: `PwmRange(lower, upper)` and `PwmRange(lower, upper, usFrame)`. — [FTC SDK RobotCore 12.0.0 artifact](https://repo1.maven.org/maven2/org/firstinspires/ftc/RobotCore/12.0.0/); constructor descriptions at [javadoc.io PwmControl.PwmRange (10.3.0)](https://javadoc.io/static/org.firstinspires.ftc/RobotCore/10.3.0/com/qualcomm/robotcore/hardware/PwmControl.PwmRange.html)
- SDK 12.0 `LynxServoController.setServoPosition` converts position to pulse with `Range.scale(pos, 0, 1, usPulseLower, usPulseUpper)`. It skips the hub write when the position equals the last commanded value (`LastKnown.updateValue`). — [FTC SDK Hardware 12.0.0 artifact](https://repo1.maven.org/maven2/org/firstinspires/ftc/Hardware/12.0.0/)
- goBILDA 2000 Series Dual Mode servo, 25-3 Speed: about 0.11 s/60° at 4.8 V, 0.09 s at 6.0 V, 0.07 s at 7.4 V (no load). Default-mode PWM 500–2500 µs with 300° travel. Continuous mode 1000–2000 µs, set with the programmer. 4 µs deadband. The 25-4 Super Speed is 0.055 s/60° at 4.8 V with the same 500–2500 µs range. — [Core Electronics: 2000 Series 25-3 Speed](https://core-electronics.com.au/2000-series-dual-mode-servo-25-3-speed.html); [Core Electronics: 25-4 Super Speed](https://core-electronics.com.au/2000-series-dual-mode-servo-25-4-super-speed.html) (reseller listings, via search summary)
- goBILDA: "500µs-2500µs is the max PWM range of the servo commander and allows a goBILDA® servo to achieve a full 300 degrees of travel". If a servo buzzes at an extreme, it is hitting the gear-train limit and the PWM range should be reduced. — [goBILDA Servo Commander guide v1.2](https://www.gobilda.com/content/user_manuals/servo_commander_startup_guide.v1.2.pdf) (via search summary)
- Axon MINI MK2: 500–2500 µs max range, about 0.175°/µs, 350°, about 0.11 s/60° at 4.8 V. — [goBILDA: Axon MINI Servo MK2](https://www.gobilda.com/axon-mini-servo-mk2/) (via search summary)
- **FTC 11138 hood.**
  - Plain `Servo`, limits 0.0–0.81 in servo units, `shooterHoodTolerance = 0.015`.
  - Hood and flywheel targets come from measured 2-D tables: a 3×2 far-zone grid and a 3×4 close-zone grid, 18 points, interpolated bilinearly with the Smile library and clamped to the table hull. Lookups use the turret position, not the robot center.
  - A `hoodActive` latch keeps the hood uncommanded until the shooter is first used, so it does not snap to its minimum at init.
  - `atTarget()` compares `adjustableHood.getPosition()` with the target.
  - Sources: [Shooter.java](https://github.com/FelixYBW/FTC11138/blob/HEAD/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Shooter.java), [README](https://github.com/FelixYBW/FTC11138)

### Inferences
- **11138's hood readiness check does nothing physical.** Their `hoodReady` effectively tests that the commanded position equals the target, which is true one loop after any command, because `getPosition()` returns the commanded value. A real check needs one of:
  - a time model: `t_ready = t_cmd + |Δservo_deg| / (60° / t_60) + margin`, using the spec at the actual servo-rail voltage and derated under load;
  - Axon analog feedback read from the bulk packet.
- **Speed and travel at the default PWM range.**
  - Example: a 0.09 s/60° servo moving 20° takes about 30 ms plus PWM-frame and settling delay (≥ 20 ms frame).
  - With the SDK default 600–2400 µs, a 300° goBILDA servo uses only (2400 − 600)/(2500 − 500) = 90 % of its travel, about 270°. Call `((ServoImplEx) servo).setPwmRange(new PwmControl.PwmRange(500, 2500))` to get the full range. Back off if it buzzes at the ends.
- **Calibration procedure.**
  1. Command several servo positions.
  2. Measure the actual hood angle at each (inclinometer or CAD).
  3. Fit `pos = a·angle + b`, or interpolate piecewise.
  4. Clip to the mechanical limits.
  - `scaleRange(posAtMinAngle, posAtMaxAngle)` makes [0, 1] cover only the safe arc. Remember that `scaleRange` arguments are absolute in the full range. Alternatively, keep the shot table in raw servo units, as 11138 does, and skip angle modeling entirely.
- **Linkages.** If the hood is driven through a linkage, the angle-to-position relation is nonlinear. Use a piecewise table rather than a single linear fit.

### Gaps
- I did not confirm the Control Hub's servo-port supply voltage; the REV specifications page returned 404. Servo speeds are spec'd at 4.8–8.4 V, so actual speed on hub power is unverified.
- No primary Axon spec sheet was read. Axon figures come from the goBILDA and ServoCity reseller listings.
- `ServoImplEx.setPulseWidth/getPulseWidth` exist in SDK 12.0's bytecode. I did not determine which SDK version added them, so treat them as version-dependent.

---

## Loop timing on the Control Hub: bulk caching, write caching, typical loop times, dt measurement, and the effect on shoot-on-move latency

### Takeaway
Use `LynxModule.BulkCachingMode.MANUAL` with `clearBulkCache()` once at the top of each loop, or AUTO if you prefer safety. One bulk read returns all encoder positions, velocities, digital and analog inputs. Motor current, I2C (IMU, Pinpoint) and battery voltage are not in the bulk packet. The SDK already skips writes that are exactly identical. Thresholded write caching saves more. Measure dt with `System.nanoTime()` and guard against outlier values.

### Cited Findings
- **Bulk caching modes (gm0).**
  - Modes: OFF (default), AUTO ("a new bulk read is done when a hardware read is repeated"), MANUAL (the cache "is only reset once manually reset", otherwise "stale values will be returned").
  - Code: `hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);` at init, and `hub.clearBulkCache();` for each hub at the top of every loop.
  - Coverage: bulk reads cover "all sensor values (except I2C)", and "reading ten sensors takes as much time as reading one sensor".
  - gm0 recommends AUTO unless you know what you're doing. Calling the same read twice per loop in AUTO triggers two bulk reads.
  - Source: [gm0: Bulk Reads](https://gm0.org/en/latest/docs/software/tutorials/bulk-reads.html)
- SDK 12.0 API: `LynxModule.setBulkCachingMode(BulkCachingMode)`, `getBulkCachingMode()`, `clearBulkCache()`, `getBulkData()`. Enum values: OFF, MANUAL, AUTO. `BulkData` getters: `getDigitalChannelState`, `getMotorCurrentPosition`, `getMotorVelocity`, `isMotorBusy`, `isMotorOverCurrent`, `getAnalogInputVoltage(int[, VoltageUnit])`. There is no motor current and no battery voltage. — [FTC SDK Hardware 12.0.0 artifact](https://repo1.maven.org/maven2/org/firstinspires/ftc/Hardware/12.0.0/)
- FTC 11138 `RobotOpMode`: MANUAL mode on `hardwareMap.getAll(LynxModule.class)`, `clearBulkCache()` once per `loop()` and once per `init_loop()`. A code comment says motor `getCurrent()` "is NOT part of the bulk packet and still costs its own transaction". Loop time is tracked with `System.nanoTime()` and an EMA (`avg*0.9 + new*0.1`) shown as ms and Hz. The README says the control loop runs about 3× faster than the 15 Hz camera. — [RobotOpMode.java](https://github.com/FelixYBW/FTC11138/blob/HEAD/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/RobotOpMode.java); [README](https://github.com/FelixYBW/FTC11138)
- Road Runner's quickstart uses `BulkCachingMode.AUTO` for all hubs. — [road-runner-quickstart MecanumDrive.java](https://github.com/acmerobotics/road-runner-quickstart/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/MecanumDrive.java)
- Older measurements (ftc_app / Expansion Hub era): a motor power command took about 3 ms and an IMU read about 7 ms. "Each API call is a bus transaction." There is no background hardware cycle, so the update rate depends on how many transactions the user code issues. — [OpenFTC RevExtensions2 issue #9](https://github.com/openftc/revextensions2/issues/9) (via search summary; dated, may not reflect current Control Hub)
- SDK 12.0 skips a hub write when the new value exactly equals the last one: `LastKnown.updateValue` in `LynxDcMotorController.internalSetMotorPower` and `LynxServoController.setServoPosition`. — [FTC SDK Hardware 12.0.0 artifact](https://repo1.maven.org/maven2/org/firstinspires/ftc/Hardware/12.0.0/)
- CTRL ALT FTC recommends a finite state machine over threads, because LynxCommands are blocking. — [CTRL ALT FTC: FTC Motor Control](https://www.ctrlaltftc.com/practical-examples/ftc-motor-control)
- REV's DECODE material uses an `ElapsedTime` timer (`autoLaunchTimer`, reset at the start of the launch sequence) for time-based gating. — [REV DECODE Programming Tips & Tricks](https://docs.revrobotics.com/ftc-kickoff-concepts/decode-2025-26/programming-tips-and-tricks)

### Inferences
- **Per-loop write budget.** A loop with about 3 motors and 3 servos writing every cycle costs one transaction per changed value. Wrap writes in a cache that skips them when `|new − last| < ε`, for example 0.002–0.005 for power and 0.001 for servo position, and force a refresh every N loops. The SDK's own cache only catches bit-identical values, which a continuously recomputed controller output never repeats.
- **Things to read once per loop or less.** Pinpoint and IMU over I2C, motor current, and battery voltage are not bulk-cached. Read them at most once per loop, and voltage at a slower rate.
- **Latency stack for shoot-on-move.** Total delay ≈ loop dt (≈ 10–25 ms) + velocity measurement lag (≈ 25 ms average from the 50 ms window) + servo PWM frame (20 ms) + mechanism travel + feed-to-exit time.
  - Predict the robot pose and velocity at the release time, `t_now + latency`, rather than at `t_now`.
  - Clamp dt as 11138 does, falling back to 0.02 s when dt ≤ 0 or > 0.25 s, to survive pauses such as garbage collection or a telemetry burst.
- **Rough loop rate.** 11138's "about 3× the 15 Hz camera" implies roughly 45 Hz (≈ 22 ms) for a full DECODE robot with Limelight, Pinpoint and Pedro. This is an inference from a loose statement, not a measured figure.

### Gaps
- I found no authoritative current (SDK 10–12, Control Hub) per-transaction timing or typical loop-time figures. The 3 ms and 7 ms numbers are from the older Expansion Hub era.
- I did not confirm whether the Control Hub's internal Lynx board shows the same per-transaction latency as an Expansion Hub over RS485 or USB.

---

## Feeding/indexing: gating the feeder on flywheel, hood and turret tolerance; estimating feed latency

### Takeaway
Feed only when all three are in tolerance: flywheel speed, hood (by a real or time-based estimate), and turret error. Derive the tolerances from shot geometry rather than picking round numbers. When the robot moves, the setpoints change every loop, so use errors relative to the moving setpoint, add short hysteresis or debounce, and never count "passing through" the setpoint while the turret is slewing.

### Cited Findings
- REV starter bot: the original gate required velocity ≥ target − 50 ticks with no upper bound. The tuned gate is target − 50 ≤ v ≤ target + 100 ticks/s, with the window set from telemetry. In autonomous an `ElapsedTime` buffer is added, starting around 2000 ms, for "more consistent launches". Excess fluctuation from too much P "can cause early feeding". — [REV DECODE Programming Tips & Tricks](https://docs.revrobotics.com/ftc-kickoff-concepts/decode-2025-26/programming-tips-and-tricks)
- **FTC 11138 gating.**
  - `atTarget()` = shooter on, target ≠ 0, |velocity error| ≤ 100 ticks/s, and |hood command − target| ≤ 0.015. The same predicate drives auto shots, driver fire control and the status LEDs.
  - Gate autos additionally require turret alignment ≤ 2° and robot position tolerance, raced against a timeout.
  - Turret `atTarget(tol)` compares |target − angle|.
  - Sources: [README](https://github.com/FelixYBW/FTC11138); [Turret.java](https://github.com/FelixYBW/FTC11138/blob/HEAD/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Turret.java)
- 11138's feeder is a CR feed servo slaved to the spindexer. It runs outward while the carousel indexes, with a 300 ms coast, and inward while intaking or firing. — [README](https://github.com/FelixYBW/FTC11138)
- **FRC 5572 issue.**
  - Feed only when `isOkayToShoot() && isAimed()`.
  - Choose the aim tolerance from geometry: how far off the turret can be at the longest shot and still hit the opening.
  - Concern that `isAimed()` flickering could cause partial feeds, so consider hysteresis and log first.
  - "With shooting on the move ... the setpoint changes every loop, so a fixed tolerance may never be met while moving". Log turret error during shots.
  - Passing through the setpoint in transit should not count as aimed.
  - The merged fix set `isAimed() = !currentlyWhipping`. The issue closed on Oct 14, 2026.
  - Source: [Frc5572/FRC2026 issue #230](https://github.com/Frc5572/FRC2026/issues/230)
- A DECODE write-up (search summary only) recommends replacing fixed readiness thresholds with calculated ones while moving. — [Medium: Programming a DECODE shooter](https://medium.com/@vikramaditya.nishant/programming-a-decode-shooter-4ab114dac01f)

### Inferences
- **Suggested gate.** `ready = |ω* − ω| < tolV(dist) && hoodSettled(now) && |θ* − θ| < tolθ(dist) && !turretWhipping && |θ̇ − θ̇*| < tolRate`, held true for about 2–3 consecutive loops (≈ 30–50 ms).
  - `tolθ(dist) ≈ atan((goalHalfWidth − ballRadius) / dist)`.
  - `tolV` comes from the shot-table sensitivity: the change in ticks/s that moves the landing point by the allowed margin.
- **Measuring feed latency empirically.**
  1. Log the time the feed is commanded.
  2. Detect ball exit by the flywheel velocity dip, a drop of more than X ticks/s within a loop or two. The dip is visible in `getVelocity()`, but it is smeared by the 50 ms window.
  3. Average across many shots. Optionally add a beam-break sensor (digital input, free in the bulk packet) at the exit for a cleaner timestamp.
- **Using the latency.** Feed the measured feed latency plus the projectile time of flight into the shoot-on-move lead computation. Keep the flywheel target latched near shoot poses (as 11138 does within 20 in) to minimize spin-up time.

### Gaps
- No published FTC measurements of feed-to-exit latency or projectile time of flight for DECODE were found.
- No source gave recommended numeric turret-angle tolerances for DECODE beyond 11138's 2° in gate autos.

---

## Tuning utilities: FTC Dashboard and Panels, @Config / @Configurable live tuning, and the Gradle dependencies required

### Takeaway
Neither FTC Dashboard (`com.acmerobotics.dashboard`) nor Panels (`com.bylazar`) is in this project. Both need an extra Maven repository and dependency. The installed `com.pedropathing:tuning:1.0.2` is a Pedro autotune web server with its own `@Tuner` and `@DisplayName` annotations. It is not a general live-config tool, so `@Config` and `@Configurable` will not compile without adding one of the two libraries.

### Cited Findings
- **This project's dependencies** (local inspection of `build.dependencies.gradle`, a file inside this project at `/home/andrei0204/Coding/Robo/FtcRobotController-12.0/build.dependencies.gradle`):
  - repositories: `mavenCentral()`, `google()`, `https://repo.dairy.foundation/releases/`;
  - dependencies: FTC SDK 12.0.0 modules, `com.pedropathing:revhub:3.0.2`, `com.pedropathing:tuning:1.0.2`.
  - `revhub:3.0.2` depends on `com.pedropathing:core:3.0.2`.
  - `tuning:1.0.2` depends on `revhub:3.0.0`, `dev.frozenmilk.sinister:Sloth:0.3.2`, `com.aventrix.jnanoid:jnanoid:2.0.0`, `org.nanohttpd:nanohttpd-websocket:2.3.1` and `kotlin-stdlib:2.3.21`, according to the POM in the local Gradle cache.
  - `tuning-1.0.2` is on Maven Central (HTTP 200 at https://repo1.maven.org/maven2/com/pedropathing/tuning/1.0.2/tuning-1.0.2.pom).
  - The local Gradle cache contains no `com.acmerobotics.*` or `com.bylazar.*` artifacts.
- **What `com.pedropathing:tuning:1.0.2` contains** (from its sources jar): package `com.pedropathing.tuning.autotune` with `TuningOpMode`, `WebServer` (NanoHTTPD on port 10158), `WebSocketServer` (port 12649), `@Tuner` (method annotation, `String name()`), `@DisplayName` (field annotation), `Procedure`, `Inputs` and `TunerScanner` (a Sloth `Scanner`). — local inspection of the sources jar; artifact at [Maven Central: com.pedropathing:tuning:1.0.2](https://repo1.maven.org/maven2/com/pedropathing/tuning/1.0.2/tuning-1.0.2.pom)
- **FTC Dashboard install.** Add `maven { url = 'https://maven.brott.dev/' }` and `implementation 'com.acmerobotics.dashboard:dashboard:0.6.0'`. With non-standard SDK dependencies, add `exclude group: 'org.firstinspires.ftc'`. Control Hub URL: `192.168.43.1:8080/dash`. — [FTC Dashboard: Getting Started](https://acmerobotics.github.io/ftc-dashboard/gettingstarted)
- **Panels** (by Lazar, the dashboard Pedro Pathing uses).
  - Repository `maven { url = "https://mymaven.bylazar.com/releases" }` and dependency `implementation 'com.bylazar:fullpanels:1.0.12'`, as used by DECODE team 11138 together with `com.acmerobotics.dashboard:dashboard:0.5.1`, `com.pedropathing:ftc:2.1.2`, `com.pedropathing:telemetry:1.0.0` and `com.pedropathing:ivy:1.0.0` on SDK 11.1.0. — [FTC11138 build.dependencies.gradle](https://github.com/FelixYBW/FTC11138/blob/HEAD/build.dependencies.gradle)
  - Latest `fullpanels` release is 1.0.19, with `lastUpdated` 2026-10-19. — [Panels maven-metadata.xml](https://mymaven.bylazar.com/releases/com/bylazar/fullpanels/maven-metadata.xml)
  - Live tuning uses `import com.bylazar.configurables.annotations.Configurable;` with `@Configurable` on a class of `public static` fields. 11138 puts all shooter, turret and hood gains in `util/Constants.java` this way. — [FTC11138 Constants.java](https://github.com/FelixYBW/FTC11138/blob/HEAD/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Constants.java)
  - A third-party skill page says the Panels dashboard is at `192.168.43.1:8001` and that `public static` fields on `@Configurable` classes are adjustable live. — [skills.sh: ncssm-robotics panels](https://skills.sh/ncssm-robotics/ftc-claude/panels) (third-party; unverified against official Panels docs)

### Inferences
- **Option 1: Panels.** To get live-tunable gains, add the Panels repository and a `com.bylazar:fullpanels` version, then annotate a constants class with `@Configurable`. This is the natural fit for a Pedro project.
- **Option 2: FTC Dashboard.** Add `maven.brott.dev` and `dashboard:0.6.0`, then use `@Config` on a class with `public static` fields.
- **Both together.** 11138 shows the two can coexist in one build, on SDK 11.1 with Pedro 2.x.
- **Zero-dependency fallback.** Keep gains as `public static` fields in a `Constants` class, nudge them with gamepad d-pad bindings in a tuning OpMode, and print them with `telemetry`. This needs no Gradle change.

### Gaps
- I could not reach the official Panels documentation site (404 on the guessed URL), so I could not verify the Panels port or confirm compatibility of `fullpanels` 1.0.19 with Pedro `core`/`revhub` 3.0.x and FTC SDK 12.0.
- I could not verify that FTC Dashboard 0.6.0 is compatible with SDK 12.0. The getting-started page lists no supported SDK versions, and the `maven.brott.dev` metadata request returned nothing.
- I found no documentation for the Pedro `tuning:1.0.2` autotune UI beyond its class and annotation names and ports.
