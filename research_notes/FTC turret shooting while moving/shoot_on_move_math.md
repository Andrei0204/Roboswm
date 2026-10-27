# Shoot-on-the-move math and physics for an FTC turret + flywheel + adjustable-hood robot (Pedro Pathing odometry)

Notation and sign conventions used in all sections (my own; chosen to match Pedro Pathing and WPILib):
- Field frame F: x, y in the floor plane, z up. Heading θ is measured CCW-positive from +x. ω = dθ/dt (rad/s, CCW +).
- Robot frame B: +x forward, +y left. R(θ) = [[cosθ, −sinθ],[sinθ, cosθ]] maps B to F.
- Turret angle φ is measured in B, CCW-positive from robot +x. Field yaw of the shot is ψ = θ + φ.
- Hood: α is the actual ball launch elevation above horizontal. The hood servo setting is a different number (see Q5).
- Exit speed s is the ball speed relative to the robot (m/s). Flywheel speed Ω is in rad/s or ticks/s.
- In 2-D, ẑ × (a, b) = (−b, a). Units are SI unless stated. Pedro Pathing defaults to inches, so either convert or use g = 386.09 in/s².
- Worked numbers labelled "(my sim)" come from my own RK2 quadratic-drag simulation. Its inputs: 12.7 cm ball, 71 g, C_d = 0.5, ρ = 1.2 kg/m³, target 0.6 m above the exit point, fixed 45° launch, goal at d = 2.5 m. Treat them as order-of-magnitude illustrations, not measured data.

## 1. Kinematics: the ball's field-frame velocity, the virtual-target method vs. the launch-velocity-vector method, and when they agree

### Takeaway
The ball leaves with field velocity u_field = u_rel + v_exit, where v_exit = v + ω × r is the velocity of the exit point. With no drag, an iterated virtual target g_v = g − v_exit·T, combined with a stationary lookup at |g_v − p|, is exact. Solving u_rel = u_static − v_exit is also exact, and stays exact even with drag, but it needs a calibrated launcher model to turn (s, α) into (RPM, hood). The two methods choose different but equally valid trajectories. Radial motion always changes the required speed and hood, and tangential motion changes them too, because d_v > d.

### Cited Findings
- **The virtual goal and the moving frame.** Several teams describe the virtual goal as treating the robot as stationary and the goal as moving, then choosing turret, hood and shooter speed for where the goal will be after the time of flight (FRC6302, Mar 2022) — [CD: Advice on shooting while moving](https://www.chiefdelphi.com/t/advice-on-shooting-while-moving/405472). In Jan 2026 posters described the same thing as shooting at the hub offset by (−t·vx, −t·vy) — [CD: Shoot on the move from the code perspective](https://www.chiefdelphi.com/t/shoot-on-the-move-from-the-code-perspective/511815).
- **The "cancel robot velocity" vector method (jacob6838, Apr 2022).** Solve the static shot (V, φ_v, φ_h) from a lookup table or optimizer. Then solve for shot speed v_s, hood θ_v and turret θ_h so that `v_s_x − v_x = V_x`, `v_s_y − v_y = V_y`, `v_s_z = V_z`, with v_s_x = v_s·cosθ_v·cosθ_h and so on. The method was verified in a pygame sim — [CD: Advice on shooting while moving](https://www.chiefdelphi.com/t/advice-on-shooting-while-moving/405472).
- **A closed-form variant (kiwi0401, 2022).** Solve the time of flight, then form `vXYZ = {x/t − xVel, y/t − yVel, z/t + g*t}`. Hood = atan2(vz, √(vx²+vy²)), turret = atan2(vy, vx), ball speed = |vXYZ|. The code also swapped axes (`xVel = getYVelocity()`), a frame bug worth avoiding — [same thread](https://www.chiefdelphi.com/t/advice-on-shooting-while-moving/405472).
- **Changing speed or hood changes the time in the air.** "Changing flywheel speed or hood angle … also changes time in air, which breaks the earlier calculations" (susanclay, 2022). That circular dependency is the reason the TOF has to be iterated — [same thread](https://www.chiefdelphi.com/t/advice-on-shooting-while-moving/405472).
- **6328 (FRC 2026) production code.** It moves the launcher forward by v·t_eff rather than moving the goal backward, which is equivalent. It then recomputes the distance from the look-ahead launcher position to the target and takes hood, flywheel and TOF from interpolating maps at that distance — [6328 LaunchCalculator.java](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/subsystems/launcher/LaunchCalculator.java).
- **254 (2022) "Shooting on the fly".** The team computed optimal shooter RPM and hood angle from "a linear regression on the ball's vertical and horizontal velocity vectors" and used a turret feedforward from robot rotation rate and angular velocity relative to the goal. That is a velocity-vector approach — [Team 254 2022 Tech Binder](https://media.team254.com/2022/10/8f2407a5-Team_254_Tech_Binder_2022.pdf).
- **Velocity of the off-center launcher.** 6328 computes it as vx' = vx − ω(r_x sinθ + r_y cosθ) and vy' = vy + ω(r_x cosθ − r_y sinθ), which is v + ω ẑ × R(θ)r — [6328 GeomUtil.transformVelocity](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/util/geometry/GeomUtil.java).

### Inferences
#### Derivation of the ball's field velocity
- Exit-point position: p_s = p + R(θ)·r_B, where r_B is the turret pivot or ball exit point in the robot frame.
- Exit-point velocity: v_s = v + ω ẑ × (R(θ) r_B) = (v_x − ω·r_Fy, v_y + ω·r_Fx).
- If the ball exit point sits a distance e from the turret pivot, add (ω + φ̇) ẑ × e_F. This is usually small: e = 5 cm at 4 rad/s gives 0.2 m/s.
- The robot has no vertical velocity. With u_rel = s·[cosα cosψ, cosα sinψ, sinα] and ψ = θ + φ, the ball's field velocity is u_field = u_rel + (v_s, 0).
#### Drag-free hit condition
- Horizontal: g_h = p_s + (u_rel,h + v_s)·T. Vertical: z_g − z_0 = s sinα·T − ½gT².
- This rearranges to u_rel,h·T = (g − v_s T) − p_s = g_v − p_s.
- The relative launch is therefore exactly a stationary shot at the virtual goal g_v = g − v_s·T. The vertical equation is unchanged, so the time of flight must equal that stationary shot's own TOF.
- This gives the fixed point T = T_table(|g − v_s T − p_s|) (Q2).
- My sim confirms it: with drag set to zero, the iterated virtual target leaves residual misses of 1 mm or less for tangential, radial and diagonal 1.5–2 m/s cases at d = 2.5 m.
#### Vector method
- Take the stationary solution for the real distance d = |g − p_s|: u* = s*(d)·[cosα*(d)·ê, sinα*(d)], where ê = (g − p_s)/d.
- Command u_rel = u* − (v_s, 0):
  - ψ = atan2(u_rel,y, u_rel,x), then φ = ψ − θ
  - α = atan2(u*_z, |u_rel,h|)
  - s = |u_rel|
- No TOF iteration is needed. The field-frame initial conditions are identical to a tested stationary shot, so the method stays exact with quadratic drag.
- What remains is second order:
  - spin rate and spin axis follow the relative exit, so Magnus differs slightly
  - error in the inverse launcher model (s, α) → (Ω, hood)
#### Both methods use the same 1-parameter family
- For a given motion, any T works if u_rel,h = (g_v(T) − p_s)/T and u_z = (Δz + ½gT²)/T.
- **Virtual target + table** picks the T at which the relative shot is a tested table shot. The launcher only runs at tested (hood, RPM) pairs.
- **Vector method** picks T = T*(d). The field trajectory is a tested trajectory, but the launcher runs at untested (hood, RPM) combinations.
- **Worked example (my sim, d = 2.5 m, static 45° and 6.00 m/s, so u*_h = u*_z = 4.25 m/s):**

| Robot motion at 1.5 m/s | Vector method: s, α, yaw | Virtual target, fixed 45° hood (drag, naive t) |
|---|---|---|
| Tangential | s = 6.19 m/s, α = 43.3°, yaw offset −19.5° | s = 6.19 m/s, yaw −21.9°, d_v = 2.70 m, T = 0.670 s |
| Radial, toward goal | s = 5.06 m/s, α = 57.1° | s = 5.32 m/s at 45°, d_v = 1.76 m |
| Radial, away from goal | s = 7.14 m/s, α = 36.5° | s = 7.19 m/s at 45°, d_v = 3.74 m |

- **Radial vs. tangential.** Radial motion mostly changes speed and elevation. Tangential motion mostly changes yaw but also raises the required speed, because d_v = √(d² + (v_t T)²) > d.
  - **Common bug:** rotating the turret for tangential motion while keeping the static distance. At 2.5 m and 1.5 m/s that leaves a ~3% speed deficit, about a 6% range error (R ∝ s² without drag), roughly 15 cm short.
- **Entry-angle side effect.** The field-frame descent angle differs from the stationary table shot. Approaching flattens the arrival and receding steepens it. That matters when the goal has a lip or funnel; Team 5000 added a funnel-clearance margin scaled by predicted/actual distance — [CD 511815](https://www.chiefdelphi.com/t/shoot-on-the-move-from-the-code-perspective/511815).
- **Recommendation for FTC.** Use the virtual target with LUTs and an effective-TOF drag correction (6328-style). It never commands untested hood/RPM pairs and needs no exit-speed calibration. Use the vector method only if you calibrate exit speed against RPM and exit angle against hood setting, for example with slow-motion video.

### Gaps
- I found no source that formally proves the equivalence or quantifies the Magnus-related error of the vector method. The derivation above is mine.
- 254's "linear regression on velocity vectors" is described only in one bullet. No equations were published in the binder.

## 2. Time of flight: lookup table vs. ballistic model, fixed-point iteration, convergence

### Takeaway
Use a measured TOF(d) table, smoothed and made monotonic, or a fitted linear-drag model. Iterate t ← TOF(|g − p_s − v_s·t_eff(t)|). The map is a contraction when |dTOF/dd|·|v_radial| < 1, which is about 0.2–0.4 at FTC speeds. Run 5–20 iterations with a 1 ms tolerance. Flag the shot invalid if it fails to converge or d_v leaves the table range. With drag, scale the lead by the effective TOF t_eff = (1 − e^(−ct))/c, since the robot-imparted velocity decays in flight.

### Cited Findings
- **1706 (FRC 2022) — single pass, measured table.** They used a LinearInterpolationTable of (distance in, TOF s): (105, 0.82), (135, 0.82), (165, 0.85), (195, 0.85), (250, 1.05). The team said "a straight trig approach gave poor results", so they switched to the table. They made one correction pass (compute newDist to the virtual goal, then RPM and hood from tables) with no iteration — [CD: Shoot While Move Code (1706)](https://www.chiefdelphi.com/t/shoot-while-move-code-1706/410494).
- **5000 (FRC 2026) — closed-form start, 3 iterations.** The initial TOF is `dist / (vel * Math.cos(angle))`. Compute the stationary shot, predict the target, recompute. They used 3 iterations and said 1–2 can suffice. They used g = 386 in/s² — [CD 511815](https://www.chiefdelphi.com/t/shoot-on-the-move-from-the-code-perspective/511815).
- **Oblarg on the recursion.** He called time-of-flight recursion "easy and robust", said it works with a static lookup table, and said it can account for air friction — [CD 511815](https://www.chiefdelphi.com/t/shoot-on-the-move-from-the-code-perspective/511815).
- **6328 (FRC 2026) — 20 iterations with a drag-corrected lead.**
  - The loop runs a fixed 20 iterations of `timeOfFlight = timeOfFlightMap.get(lookaheadDistance)`.
  - Lead: `effectiveTOF = (1 - exp(-timeOfFlight*linearDragTimeConstant))/linearDragTimeConstant`, with the tunable `linearDragTimeConstant` = 0.375 (1/s); the launcher offset is v·effectiveTOF.
  - TOF table: 1.63 m → 1.017 s, 2.40 → 0.967, 3.25 → 1.19, 4.15 → 1.18, 4.875 → 1.25. It is not monotonic.
  - The shot is valid only if the look-ahead distance is within [0.6, 6.0] m and the robot is outside the "bad boxes".
  - Source: [6328 LaunchCalculator.java](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/subsystems/launcher/LaunchCalculator.java).
- **Closed-form drag-free TOF (kiwi0401, 2022).** For given speed v and target offset (x, y, z):
  - Straight shot: `sqrt(-4gz + 2v² - 2sqrt(-4g²x² - 4g²y² - 4gv²z + v⁴))/(2g)`
  - Arc shot: `sqrt(2)*sqrt(-2gz + v² + sqrt(...))/(2g)`
  - Source: [CD 405472](https://www.chiefdelphi.com/t/advice-on-shooting-while-moving/405472).

### Inferences
#### Models for TOF
- **Without drag**, with exit speed s and elevation α: T = d/(s cosα). This is 5000's formula; it ignores drag.
- **With linear drag** (rate c ≈ k·|u|, where k = ρC_dA/2m):
  - x(t) = (u_h/c)(1 − e^(−ct)), so T = −ln(1 − c·d/u_h)/c, valid only when c·d < u_h
  - z(t) = ((u_z + g/c)/c)(1 − e^(−ct)) − (g/c)·t
- **Best practice:** measure TOF directly and fit a smooth monotone curve (linear or quadratic in d). 6328's raw table is non-monotonic (1.017 s at 1.63 m vs 0.967 s at 2.40 m), which makes the local slope noisy.
- **Magnitudes (my sim, 45° launch, Δz = +0.6 m):**

| d | TOF with drag | TOF without drag | Exit speed with drag | Exit speed without drag |
|---|---|---|---|---|
| 1.0 m | 0.346 s | 0.350 s | 4.99 m/s | 4.85 m/s |
| 2.5 m | 0.637 s | 0.622 s | 6.00 m/s | 5.68 m/s |
| 3.5 m | 0.795 s | 0.769 s | 6.96 m/s | 6.44 m/s |

  - A 55° launch gives 0.415–0.981 s over the same range.
  - The slope dT/dd is about 0.18 s/m at 45° and about 0.23 s/m at 55°.
#### Fixed-point iteration
- **Iteration:** F(t) = T(|g − p_s − v_s·t_eff(t)|).
- **Contraction condition:** F'(t) = −T'(d_v)·(ê_v·v_s)·t_eff'(t), where ê_v is the unit vector from p_s to g_v. Then |F'| ≤ |T'|·|v_radial| < 1 is sufficient (Banach); for example 0.18 s/m × 1.5 m/s ≈ 0.27.
  - **Approaching** the goal: F' < 0, so the iterates alternate.
  - **Receding:** F' > 0, so they converge monotonically.
  - **Purely tangential:** ê·v ≈ 0 at first, so convergence is fastest.
- **Observed in my sim (naive t, d = 2.5 m, 1.5 m/s):**
  - Approaching: 0.637 → 0.445 → 0.510 → 0.489 → 0.496 → 0.494 s (ratio ≈ −0.34)
  - Receding: 0.637 → 0.788 → 0.821 → 0.828 → 0.829 s
  - Tangential: 0.637 → 0.667 → 0.670 s
  - Error falls by roughly 3× per iteration, so 5 iterations reach < 1 ms.
- **Failure condition:** |T'|·|v_radial| ≥ 1. With T' = 0.23 s/m that needs |v| ≥ 4.3 m/s, which is not reachable in FTC. Steep or noisy tables, table-edge clamping and very high lob shots can still break it locally.
- **Mitigations if it does not converge:**
  - iterate at most N = 10–20 times; stop when |Δt| < 1 ms
  - if oscillating, damp: t ← t + λ(F(t) − t) with λ = 0.5
  - or bracket-and-bisect g(t) = F(t) − t on [T_min, T_max]
  - if still unconverged, or d_v is outside the table, mark the shot invalid and do not feed
- **One pass is not enough for radial motion (my sim, 2.5 m, 1.5 m/s, drag):**

| Motion | One pass (TOF from the real d) | Iterated, naive t | Iterated, t_eff with c = 0.35 |
|---|---|---|---|
| Toward goal | 0.42 m short | 0.14 m short | 0.07 m |
| Away from goal | 0.02 m | 0.24 m long | 0.06 m |
| Tangential | 0.04 m | 0.08 m | 0.03 m |

  - Without drag, one pass misses 0.26 m approaching and 0.21 m receding, while the iterated solution is exact.
  - The best-fit c differed by direction (about 0.25 tangential, about 0.5 radial). Linear drag is only an approximation, so tune c empirically, as 6328 does with a tunable.
#### Reference per-loop algorithm (50–100 Hz, Java)
1. `follower.update()`. Read pose (x, y, θ) and field velocity (vx, vy, ω); convert to metres. Optionally low-pass or alpha-beta filter v and ω (Q7).
2. **Predict to release time.** L = loop/sensor delay + feed delay.
   - p ← p + v·L + ½a·L², θ ← θ + ω·L, v ← v + a·L.
   - 6328 instead uses pose.exp(twist·0.03 s).
3. **Exit point and its velocity.** p_s = p + R(θ)r_B; v_s = v + ω ẑ × R(θ)r_B.
4. **Iterate the TOF.**
   - t = TOF(|g − p_s|)
   - repeat ≤ 10×: t_eff = (1 − e^(−ct))/c; g_v = g − v_s·t_eff; t_new = TOF(|g_v − p_s|); stop when |t_new − t| < 1e-3
5. **Aim and set the launcher.**
   - d_v = |g_v − p_s|; ψ = atan2(g_v − p_s)
   - φ_cmd = wrap(ψ − θ) into the turret's range (handle the wrap/unwind)
   - hood = Hood(d_v); Ω_cmd = RPM(d_v)
6. **Turret feedforward** φ̇_ff (Q4). Optionally add a hood feedforward from the filtered dHood/dt, as 6328 does.
7. **Gate** (Q6): converged, d_v in range, |φ_err| < tol, |Ω − Ω_cmd| < tol, hood settled, outside no-shoot zones. Then feed.

### Gaps
- No source gave a formal convergence analysis. The contraction bound above is my derivation.
- 1706's TOF table and 6328's TOF table are for FRC goals (high arcs, about 0.8–1.25 s). I found no published FTC TOF tables.

## 3. Shooter-point velocity (ω × r), latency compensation, acceleration, and how big the effects are at FTC scale

### Takeaway
At FTC scale (2.5 m shot, about 0.6 s TOF) every correction matters:

| Effect | Miss if ignored |
|---|---|
| No lead at all, 1.5 m/s sideways | ≈ 0.9 m |
| ω × r offset (r = 0.15 m, ω = 3 rad/s) | ≈ 0.26 m |
| 50 ms stale heading at ω = 3 rad/s (turret pointing) | ≈ 0.38 m |
| 0.1 s feed delay during 3 m/s² acceleration | ≈ 0.17 m |

Predict the state to the ball-release instant, use the exit-point velocity, and prefer commanded/setpoint velocity or a filtered estimate with acceleration.

### Cited Findings
- **6328's look-ahead.** They predict the pose forward with a constant phase delay: `phaseDelay = 0.03` s, applied as `estimatedPose.exp(Twist2d(vx*phaseDelay, vy*phaseDelay, omega*phaseDelay))` using robot-relative velocity.
  - In teleop they use the drivetrain's field **setpoint** velocity (`getFieldSetpointVelocity`), not the measured one, transformed to the launcher point.
  - In autonomous they use the setpoint velocity without the launcher-offset transform.
  - Sources: [6328 LaunchCalculator.java](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/subsystems/launcher/LaunchCalculator.java); [6328 RobotState.java](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/RobotState.java).
- **1706's acceleration term.** They added acceleration to the lead: `virtualGoalX = target.getX() - fixedShotTime*(robotVel.vx + robotAccel.ax*kAccelCompFactor)`. The factor was "experimentally determined" (value not given).
  - It was meant to cover velocity change over about 0.1 s, citing Limelight latency and the delay between feeding and the ball leaving.
  - Without it the turret, shooter and hood lagged during acceleration; with it, shooting while moving was as fast as stationary. They reported shooting at over 3 m/s.
  - Source: [CD 1706](https://www.chiefdelphi.com/t/shoot-while-move-code-1706/410494).
- **calcmogul on prediction.** Predicting the robot's next-timestep pose with a drivetrain feedforward model and aiming from that pose "implicitly takes the drivetrain movement into account"; "The turret feedback + feedforward won't need modification for this because it's just changing the goal" — [CD: High refresh rate vision processing #32](https://www.chiefdelphi.com/t/high-refresh-rate-vision-processing/387949/32).
- **Pedro Pathing 3.x velocity API.** The user's project depends on `com.pedropathing:core/revhub 3.0.2`.
  - `Follower.velocity()` returns `com.pedropathing.math.Velocity(vx, vy, omega)`. `Velocity.toTwist(heading)` rotates (vx, vy) by −heading into the robot frame, so `Velocity` is field-frame and `Twist` is robot-frame — [Pedro Velocity.java](https://github.com/Pedro-Pathing/PedroPathing/blob/main/core/src/main/java/com/pedropathing/math/Velocity.java); [Pedro MotionState.java](https://github.com/Pedro-Pathing/PedroPathing/blob/main/core/src/main/java/com/pedropathing/localization/MotionState.java).
  - `PinpointLocalizer.update()` builds that `Velocity` from `pinpoint.getVelX/getVelY(globalDistanceUnit)` and `getHeadingVelocity(RADIANS)` — [Pedro PinpointLocalizer.java](https://github.com/Pedro-Pathing/PedroPathing/blob/main/revhub/src/main/java/com/pedropathing/revhub/localizers/PinpointLocalizer.java).
  - I checked the class and method names (`velocity`, `com/pedropathing/math/Velocity`) against the strings in the locally cached core-3.0.2.jar.

### Inferences
#### Worked magnitudes
Assumptions: d = 2.5 m, TOF T = 0.64 s, effective lead time t_eff ≈ 0.57 s (c = 0.35). Small-angle lateral miss ≈ d·δψ.

| Error source (if uncompensated) | Calculation | Miss at goal |
|---|---|---|
| No lead, v_t = 1.5 m/s (my sim) | — | 0.88 m lateral |
| No lead, v_t = 1.0 m/s | scaled | ≈ 0.59 m |
| No lead, v_t = 2.0 m/s (my sim) | — | 1.17 m |
| No lead, radial ±1.5 m/s (my sim) | — | 0.76 m long (toward) / 0.83 m short (away) |
| ω × r ignored, r = 0.15 m, ω = 3 rad/s | 0.45 m/s × 0.57 s | ≈ 0.26 m |
| ω × r ignored, ω = 5 rad/s | 0.75 m/s × 0.57 s | ≈ 0.43 m |
| ω × r ignored, r = 0.10 m, ω = 3 rad/s | — | ≈ 0.17 m |
| Stale heading in the turret command, ω = 3 rad/s, 50 ms | δψ = ωL = 0.15 rad (8.6°) | ≈ 0.38 m (0.75 m at 100 ms) |
| Stale heading, ω = 1 rad/s, 50 ms | — | ≈ 0.13 m |
| Translation latency, v = 1.5 m/s, L = 50 ms | position error v·L = 7.5 cm | ≈ 7.5 cm lateral if the motion is tangential |
| Feed/indexer delay of 150 ms not predicted | robot moves 0.22 m before release | up to ≈ 0.2 m |
| Acceleration ignored, a = 3 m/s², L = 0.1 s | position ½aL² = 1.5 cm (small); velocity aL = 0.3 m/s | ≈ 0.17 m |

- **Heading latency dominates when the robot spins.** The robot-frame turret command is φ = ψ − θ, so stale θ maps one-to-one into pointing error. When spinning, the heading/ω prediction matters more than translation prediction.
- **Predict the velocity too, not only the position.** The velocity error (aL) is the one that matters.
- **Which latencies to add up:**
  - L_sensor: Pinpoint/odometry data age, plus half a loop
  - L_loop: 10–20 ms at 50–100 Hz
  - L_feed: time from the "feed" command until the ball loses contact with the flywheel
- **What the turret and hood need.** They need the predicted state at release and the feedforward of Q4. Actuator settling is handled by gating (Q6), not by prediction.
- **Measuring L_feed:** log the feed command time, detect release (flywheel current/velocity dip, or a break-beam at the exit), and take the average. A flywheel-speed dip timestamp is a cheap detector.
- **Pedro specifics:**
  - Convert inches to metres, or work in inches with g = 386.09 in/s².
  - The heading from Pedro is CCW-positive radians (consistent with the Velocity/Twist rotation above).
  - Verify the Velocity frame empirically: at heading 90°, drive robot-forward, and vy should be positive and vx ≈ 0.
  - Acceleration is not provided by the Velocity class. Either difference the filtered velocity, or use the path follower's commanded/setpoint velocity (6328-style).

### Gaps
- I found no published measurements of Pinpoint/Control Hub end-to-end pose latency or of typical FTC feed-to-release delay. The numbers above are illustrative assumptions.
- I did not confirm from goBILDA documentation that `getVelX/getVelY` are field-frame. The inference rests on Pedro's code treating them as field-frame `Velocity`.
- 1706's `kAccelCompFactor` value was not published.

## 4. Turret feedforward: line-of-sight rate from translation plus counter-rotation of robot ω

### Takeaway
Turret command φ = wrap(ψ_aim − θ), with feedforward rate φ̇_ff = ψ̇_aim − ω, where ψ̇_aim = (r_y·v_s,x − r_x·v_s,y)/|r|² = −v_⊥/d, using r = g_v − p_s. Combine it with position feedback; feedforward alone is not viable.

### Cited Findings
- **254 (2022):** "Compute turret feedforward based on robot rotation rate and angular velocity relative to the goal, so the turret is always lined up for an ideal shot" — [Team 254 2022 Tech Binder](https://media.team254.com/2022/10/8f2407a5-Team_254_Tech_Binder_2022.pdf).
- **6328 (2026)** aims the whole drivetrain. They differentiate the solved aim angle each loop and smooth it with a 0.1 s moving average: `driveAngleFilter = LinearFilter.movingAverage((int)(0.1/loopPeriodSecs))`, `driveVelocity = filter((driveAngle − lastDriveAngle)/loopPeriodSecs)`. They do the same for the hood angle velocity — [6328 LaunchCalculator.java](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/subsystems/launcher/LaunchCalculator.java).
- **WPILib** notes that feedforward-only control is not viable for turrets and pairs feedforward with feedback — [WPILib: Tuning a Turret Position Controller](https://docs.wpilib.org/en/2024/docs/software/advanced-controls/introduction/tuning-turret.html).
- **Lagging turrets.** A Chief Delphi post argues that a turret that lags behind robot velocity gives no advantage over aiming the drivetrain — [CD: Turret Motion Profile](https://www.chiefdelphi.com/t/turret-motion-profile/517998) (from a search summary; the page was not fetched).

### Inferences
#### Derivation
- ψ_aim = atan2(r_y, r_x) with r = g_v − p_s, so ψ̇ = (r_x ṙ_y − r_y ṙ_x)/|r|².
- For a stationary goal, ṙ = −v_s − d(v_s t_eff)/dt ≈ −v_s (neglecting the slow variation of the virtual offset). So ψ̇_aim = (r_y v_s,x − r_x v_s,y)/|r|² = −(v_s · n̂)/|r|, with n̂ = ẑ × r̂ (the left-perpendicular of the line of sight).
- Turret (robot-frame) rate: φ̇_ff = ψ̇_aim − ω.
- **Sign check:** robot at the origin, goal at +x, robot moving +y (n̂ = ẑ × x̂ = +ŷ, so v_⊥ = +v) ⇒ ψ̇ = −v/d < 0. The bearing to the goal swings clockwise. ✓
- **Example:** d = 2.5 m, v_⊥ = +1.5 m/s, ω = +3 rad/s ⇒ ψ̇ = −0.6 rad/s and φ̇_ff = −3.6 rad/s. The turret must counter-rotate faster than the chassis spins.
- **Practical alternative (6328-style):** numerically differentiate φ_cmd (unwrapped) each loop and low-pass it (0.05–0.1 s). This automatically includes the d(g_v)/dt terms, the acceleration terms and the offset geometry, at the cost of a small lag.
- **Motor command:** u = kS·sgn(φ̇_ff) + kV·φ̇_ff + kA·φ̈_ff + kP·(φ_cmd − φ) (+ kD·(φ̇_ff − φ̇)).
- **Why feedforward matters (illustrative):** with P-only control, a type-1 loop tracking a ramp has steady error ≈ φ̇/K. With K = 15 s⁻¹ and φ̇ = 3.6 rad/s, the lag is 0.24 rad ≈ 14°, about 0.6 m at 2.5 m.
- **Wrap handling:** compute φ_cmd continuously (unwrap). Choose the equivalent angle within the turret's mechanical limits that is closest to the current angle. If an unwind is needed, block firing during it.

### Gaps
- No source provided measured turret tracking-error data at FTC scale. The P-only lag figure is a textbook illustration with assumed gains.

## 5. Projectile physics for light FTC balls (drag, Magnus), why teams use lookup tables, building and interpolating tables, measuring TOF

### Takeaway
For a 5 in, roughly 71 g ball, quadratic drag is about 35% of g at 8 m/s, and backspin Magnus lift can be about 10–15% of g. Spin and exit angle also depend on compression, friction and wear, so measured stationary tables (distance → hood, RPM, TOF) capture all of it and are the practical standard. Add a single tunable drag constant for the moving-shot lead. Note that the current (2026–27) FTC game BIOBUZZ uses smaller 3.6 in and 2.8 in balls.

### Cited Findings
- **Game pieces.**
  - The DECODE (2025–26) manual glossary defined an ARTIFACT as a "5 in. (12.70 cm) nominal Gopher ResisDent™ polypropylene ball". That text came from a search snippet of the [FTC game manual glossary](https://ftc-resources.firstinspires.org/ftc/game/manual-16).
  - The same URL now serves the BIOBUZZ glossary (V1): NECTAR is "approximately 3.6 in. (9.1 cm) Gopher ResisDent™ polyethylene balls" and POLLEN is "2.8 in. (7.1 cm) Gopher ResisDent™ polyethylene balls". LAUNCHING is defined as a scoring element "shot into the air…", and a FLOWER is a structure into whose top POLLEN and NECTAR can be placed — [FTC manual Section 16 Glossary](https://ftc-resources.firstinspires.org/ftc/game/manual-16).
  - The two descriptions conflict because the page content changed with the new season.
- **Ball mass (reference ball).** A 5 in Gopher Screamin' Yellow ResisDent ball (FRC 2017 "fuel") weighs 0.156 lb (≈ 71 g) — [AndyMark](https://www.andymark.com/product-p/am-frcball.htm). It is not confirmed to be the same ball as the DECODE ARTIFACT.
- **Drag equation.** D = ½·C_d·ρ·V²·A, and a sphere's C_d "varies widely with Reynolds number" — [NASA Glenn: Drag of a Sphere](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/drag-of-a-sphere).
- **Magnus force.** F_M = ½·C_L·ρ·A·v² when spin is perpendicular to velocity, with C_L depending mainly on the spin parameter S = ωr/v — [Py4Phy baseball notebook](https://nbviewer.jupyter.org/github/Py4Phy/PHY432-resources/blob/main/11_ODE_applications/baseball_solution.ipynb).
- **C_L values.** For smooth spheres at subcritical Re, C_L rises roughly linearly with S to about 0.3–0.4 near S ≈ 0.5, then plateaus — [Firgelli Magnus calculator](https://www.firgelliauto.com/blogs/engineering-calculators/magnus-force-calculator). This is a commercial page and less authoritative.
- **Inverse Magnus.** Near the drag-crisis regime, negative lift (inverse Magnus) can occur at low spin — [Hokkaido Univ. LES paper](https://eprints.lib.hokudai.ac.jp/repo/huscap/all/53053/JoV15-3_261-268.pdf).
- **FRC references and the empirical view.**
  - Ether's Chief Delphi paper models drag and Magnus lift/dive with equations of motion and C code (2-D only) — [CD paper: Ballistic trajectory with air friction drag and Magnus](https://www.chiefdelphi.com/media/papers/2725).
  - Others report that the Magnus effect is very real and that backspin helped range and accuracy (from a search summary) — [CD: Ball Flight Model](https://www.chiefdelphi.com/t/ball-flight-model/132456).
  - FRC6302: "you shouldn't need to take into account magnus effect … it should be automatically accounted for in the data points you get from testing" — [CD 405472](https://www.chiefdelphi.com/t/advice-on-shooting-while-moving/405472).
- **Tables in practice.**
  - 6328 (2026) uses `InterpolatingTreeMap`/`InterpolatingDoubleTreeMap` for hood (10 points, 1.12–5.01 m, 10°–38°), flywheel (145–195 rad/s), TOF (5 points) and kicker speed, all keyed by launcher-to-target distance, with min/max distance validity — [6328 LaunchCalculator.java](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/subsystems/launcher/LaunchCalculator.java).
  - 1706 used linear-interpolation tables and dashboard trim offsets (`SetShotAdjust`, `SetHoodAdjust`) — [CD 1706](https://www.chiefdelphi.com/t/shoot-while-move-code-1706/410494).
  - kiwi0401 held one fixed flywheel speed (better accuracy, stiffer PID) and varied only hood and turret. He added an empirical long-range hood term `hoodAngle += x²·0.2` for drag beyond 2 m — [CD 405472](https://www.chiefdelphi.com/t/advice-on-shooting-while-moving/405472).

### Inferences
#### Equations of motion
- m·dv/dt = −m g ẑ − ½ρC_dA|v|v + ½ρC_LA|v|²(ŝ × v̂), where ŝ is the spin-axis unit vector (right-hand rule). Backspin gives upward lift.
- With k = ρC_dA/(2m), the drag deceleration is k|v|².
- **Drag sizes, assuming C_d ≈ 0.5 (my assumption, typical of a subcritical sphere, Re = vD/ν ≈ 6 × 0.127/1.5e-5 ≈ 5×10⁴):**
  - 12.7 cm, 71 g: k ≈ 0.054 1/m, so 3.4 m/s² at 8 m/s (≈ 35% of g)
  - 9.1 cm (NECTAR): k ≈ 0.039–0.065 1/m for 50–30 g (assumed masses)
  - 7.1 cm (POLLEN): k ≈ 0.059 1/m for an assumed 20 g
- **Effect of drag on required exit speed (my sim):** about +2–3% at 1 m, +4–7% at 2.5 m and +6–10% at 3.5 m versus vacuum, for all these balls. TOF changes only 1–4%.
- **Equivalent linear-drag constant** c ≈ k·|u| ≈ 0.25–0.4 1/s, consistent with 6328's tuned 0.375 for their foam ball.
- **Magnus size.** A single flywheel against a fixed hood rolls the ball, so the centre speed is about half the wheel surface speed and the spin is ω_b ≈ v/r, giving S ≈ 1 (strong backspin). With C_L 0.25–0.35 at 6 m/s on a 71 g, 5 in ball, F_M ≈ 0.07–0.10 N ≈ 10–14% of weight. Lift is larger for higher A/m (lighter or larger balls).
- **Spin varies with mechanics.** Spin depends on hood friction, compression and wear. This is the strongest argument for empirical tables and for re-checking tables after hood or wheel wear. Dual-wheel (top/bottom differential) shooters let you set spin deliberately.
- **Hood setting ≠ launch elevation.** The ball leaves at an angle set by contact geometry and compression, which varies with speed. Tables keyed by servo position absorb this. Physics-based solvers need a measured hood→α map.
#### Building the lookup table (stationary, FTC)
1. Fix the reference point: distance from the **ball exit point / turret axis** to the goal aim point (6328 uses the launcher position, not the robot centre).
2. Pick 6–10 distances spanning the scoring range (e.g. 0.75 m to 3.5 m, every 0.25–0.5 m), denser where curvature is high.
3. **Choose the hood first:**
   - lower hood (flatter, shorter TOF) at long range
   - higher hood near the goal so the ball clears the lip and enters steeply
   - or hold the hood fixed and vary RPM
4. Tune RPM for centre hits at each point (≥ 5 shots per point), then record (d, hood, RPM, TOF, hit rate).
5. **Interpolate:** piecewise-linear interpolation (Java TreeMap floor/ceiling entries) clamped at the ends, or least-squares polynomials. Prefer a smooth monotone fit for TOF, because noise in TOF directly becomes jitter in the lead.
6. Validate at mid-points between table entries and at the table edges.
7. Re-check after any change to flywheel compression, wheel wear or a new batch of game pieces.
#### Measuring TOF
- Phone slow-motion video at 240 fps gives ≈ 4.2 ms resolution. Count frames from the ball leaving the flywheel/hood to crossing the goal plane; a 0.6 s flight spans about 144 frames, so ±1 frame ≈ ±0.7%.
- Alternatively, timestamp release (break-beam at the exit or flywheel-speed dip) and arrival (break-beam or microphone at the goal) on the hub.
- Or fit a drag model to measured landing points and compute TOF from it.
- Log TOF alongside each table point. 6328 and 1706 both use TOF tables, but neither describes how they measured them.

### Gaps
- No source gave official masses for DECODE ARTIFACTs or BIOBUZZ NECTAR/POLLEN. Masses above are assumptions or a reference product.
- I found no measured C_d or C_L for Gopher ResisDent balls. C_d ≈ 0.5 is a textbook subcritical value I could not tie to a specific fetched source.
- I could not confirm the BIOBUZZ goal geometry, or whether its main scoring is a launched shot into a fixed goal. Only the glossary (V1) was readable.
- No source described how 6328 or 1706 measured TOF.

## 6. Flywheel speed drop after each shot, its effect on accuracy, and shot-readiness gating

### Takeaway
Each shot takes energy ≈ (5/6)·m·v² from the flywheel (translation plus backspin). The fractional speed drop is ΔΩ/Ω ≈ ΔE/(2E_fly). A ~3–4% drop on a light FTC wheel means ~7% range error (≈ 15–20 cm at 2.5 m). Fix it with inertia, P + F velocity control, and gating on |Ω − Ω_cmd|, turret error, hood settle and solver validity.

### Cited Findings
- **REV DECODE guide (FTC):**
  - Use only P and F: targetPower = F·target + P·(target − current), starting at F = P = 0.0004.
  - Tune F until velocity is "consistently within 10 ticks of your target velocity".
  - Feed only when velocity is within a window (example: ≥ target − 50 ticks and ≤ target + 100 ticks), plus a 2 s settle timer at first.
  - It notes that the first ball can launch stronger than later ones.
  - Source: [REV FTC DECODE Programming Tips & Tricks](https://docs.revrobotics.com/ftc-kickoff-concepts/decode-2025-26/programming-tips-and-tricks).
- **Chief Delphi on recovery:** slow recovery usually means more P is needed; too much P causes micro-oscillation; D is "completely unnecessary in a 1st order system like a flywheel" — [CD: Velocity PID help](https://www.chiefdelphi.com/t/velocity-pid-help/513118) (from a search summary).
- **VEX forum:** a well-tuned feedforward "seemed to almost double the fire rate" and improved accuracy because RPM was nearly always in range after a shot — [VEX Forum: Hybrid Velocity Control Reveal](https://www.vexforum.com/t/hybrid-velocity-control-reveal/33243) (from a search summary).
- **Single fixed speed:** kiwi0401 kept one flywheel speed to allow "a much stronger PID on the flywheel" — [CD 405472](https://www.chiefdelphi.com/t/advice-on-shooting-while-moving/405472).
- **Turret tolerance:** only shoot when the turret is within about 3–5° of its setpoint — [CD: Turret Motion Profile](https://www.chiefdelphi.com/t/turret-motion-profile/517998) (from a search summary).
- **6328 validity:** the shot is valid only when the look-ahead distance is within [minDistance = 0.6, maxDistance = 6.0] m and the robot is outside predefined "bad boxes". They also provide a driver flywheel-speed offset trim (`incrementFlywheelSpeedOffset`) — [6328 LaunchCalculator.java](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/subsystems/launcher/LaunchCalculator.java).

### Inferences
#### Energy per shot
- ΔE = ½mv² + ½I_bω_b². For a thin-shell ball, I_b = (2/3)mr². If the ball rolls on a fixed hood, ω_b = v/r, so ΔE = (5/6)mv². Slip and compression losses add to this.
- Flywheel energy is E = ½JΩ², so ΔΩ/Ω ≈ 1 − √(1 − ΔE/E) ≈ ΔE/(2E).
#### Example (illustrative inputs)
- Ball 71 g at 6 m/s ⇒ ΔE ≈ 2.1 J.
- Wheel radius 48 mm with surface speed ≈ 2 × 6 = 12 m/s ⇒ Ω ≈ 250 rad/s (2,390 rpm).
- With J = 0.001 kg·m², E ≈ 31 J ⇒ about a 3.5% speed drop (ideal, no losses).
- Exit speed ∝ Ω, and range ∝ s² (vacuum), so the range is ≈ 7% short, about 17 cm at 2.5 m. A second ball fired before recovery compounds this.
- Recovery time t_rec ≈ J·ΔΩ/τ_net, where τ_net is the motor torque at that speed minus friction.
- Doubling J halves the drop but doubles spin-up and recovery energy.
#### Mitigations
- Increase flywheel inertia (add a mass wheel).
- Run the velocity loop at the highest rate available (hub-side PIDF at 20 Hz+, or a custom P+F loop with bulk reads).
- Optional feedforward "kick": a short voltage boost synchronised to the feed.
- Measure Ω at the moment of release and re-solve the hood for the actual speed: invert the table with the speed fixed, using ∂hood/∂Ω from neighbouring table entries.
- Space shots by at least t_rec.
#### Deriving gate thresholds from the goal tolerance
- Let the allowed miss e_allow = (W_goal − D_ball)/2 − margin.
- Yaw tolerance: |φ_err| < e_allow/d. For example, e_allow = 10 cm at 2.5 m gives 2.3°.
- Speed tolerance (vacuum, near 45°): δs/s < e_allow/(2d), so 10 cm at 2.5 m gives 2%. Convert to ticks/s for the gate.
- Hood tolerance: δα < e_allow/|∂R/∂α|. Estimate ∂R/∂α from neighbouring table entries or the drag model. Near the maximum-range angle, range is insensitive to α (∂R/∂α ≈ 0); at steep or shallow angles it is not.
#### Full fire gate
Fire only when all of these hold:
- the solver converged
- d_v is in the table range
- turret, hood and flywheel are within tolerance and their errors have stayed below tolerance for N consecutive loops (debounce, e.g. 2–3 loops)
- |a| is below a limit (prediction is least reliable during hard acceleration)
- the robot is not in a no-shoot zone and the turret is not unwinding

### Gaps
- I found no measured FTC flywheel inertia, speed-drop or recovery data. The numbers above are illustrative calculations.
- Goal opening dimensions for DECODE and BIOBUZZ were not retrieved, so e_allow is symbolic.

## 7. Error budget: how pose error, velocity noise and latency become miss distance, and velocity filtering trade-offs

### Takeaway
Small-signal miss ≈ δp_⊥ + d·δψ + t_eff·δv + v·δL + (range error from δs, δα). Errors combine roughly as a root-sum-square. At FTC scale the biggest terms are uncompensated heading latency during rotation and velocity lag from heavy filtering during acceleration. Use light filtering (τ ≈ 20–50 ms) with an acceleration or setpoint term rather than a heavy low-pass.

### Cited Findings
- **The math is not the hard part.** Jared_Russell (254) said error stack-up across vision, odometry, latency, control and the shooter was harder than the math: "the math was the easier part of shoot-on-the-move". kornblau added that shooting during or after defensive contact is harder still — [CD 405472](https://www.chiefdelphi.com/t/advice-on-shooting-while-moving/405472).
- **Odometry drift.** susanclay noted that odometry degrades quickly in teleop and planned to supplement it with vision — [CD 405472](https://www.chiefdelphi.com/t/advice-on-shooting-while-moving/405472).
- **6328's filtering choices.** They avoid noisy measured velocity by using the drivetrain setpoint velocity, and smooth derived rates (drive-angle and hood velocity feedforwards) with 0.1 s moving averages — [6328 LaunchCalculator.java](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/subsystems/launcher/LaunchCalculator.java); [6328 RobotState.java](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/RobotState.java).
- **254's drive constraints.** They put velocity and acceleration constraints on the drivebase "for more controlled movements, improving shooting on the fly" — [Team 254 2022 Tech Binder](https://media.team254.com/2022/10/8f2407a5-Team_254_Tech_Binder_2022.pdf).
- **Pose history for vision.** Keep about 1 s of pose (and turret angle) history and look up the pose at the vision frame's capture time (from a search summary) — [CD 387949](https://www.chiefdelphi.com/t/high-refresh-rate-vision-processing/387949/8).

### Inferences
#### Sensitivities (d = 2.5 m, t_eff ≈ 0.57 s, v = 1.5 m/s)

| Error | Miss |
|---|---|
| Translational pose error δp = 3 cm | ≈ 3 cm |
| Heading error of 1° (only if the turret command uses robot heading, which it does) | 4.4 cm |
| Velocity noise σ_v = 0.05 m/s per axis | ≈ 3 cm |
| Velocity bias of 0.1 m/s | ≈ 6 cm |
| Latency error of 20 ms at 1.5 m/s and ω = 3 rad/s (translation; rotation) | ≈ 3 cm; ≈ 15 cm |
| Flywheel speed 2% low | ≈ 4% range ≈ 10 cm |
| Uncorrected drag in the lead (naive t) at 1.5 m/s radial (my sim) | 14–24 cm |

#### Filtering trade-off
- A first-order low-pass with time constant τ lags velocity by about τ during acceleration a. The bias a·τ creates a miss of a·τ·t_eff.
  - Example: a = 2 m/s², τ = 0.1 s ⇒ 0.2 m/s ⇒ ≈ 11 cm.
- Noise attenuation scales roughly with √(T_s/(2τ)) for a loop period T_s.
- At a 10–20 ms loop, τ ≈ 20–50 ms usually balances these.
- **Better options than a heavy low-pass:**
  - an alpha-beta (constant-acceleration) filter or small Kalman filter on (p, v, a)
  - blending in the path follower's commanded velocity (6328 uses the setpoint)
  - predicting with acceleration (1706)
- Filter the derived turret/hood feedforward rates more heavily (6328: 0.1 s) than the velocity used for the lead.
#### Operational mitigations
- Cap or soften robot acceleration and ω while the shoot button is held (254 constrained the drivebase).
- Gate on |a| and |ω| limits.
- Use vision or AprilTag resets to bound pose drift.
- Use pose history at the camera capture timestamp for latency-correct vision fusion.

### Gaps
- I found no quantitative noise specifications for Pinpoint velocity, and no FTC-specific measured error budgets. The numbers above are derived from assumed σ values and my simulation.
- I found no first-hand source for Eeshwar's "Shooting on the Fly" blog (Parts 1–2, referenced in CD 511815, Jan 2026) or for the Orbit 1690 software session (Part 2). Both are likely relevant tuning references but were not located.
