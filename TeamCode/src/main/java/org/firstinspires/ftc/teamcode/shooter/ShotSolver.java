package org.firstinspires.ftc.teamcode.shooter;

/**
 * Shoot-on-the-move solver. It is pure math with no hardware or Pedro types, so it can be
 * tested off the robot.
 *
 * <p>Method ("virtual goal" with an iterated time of flight, as in FRC 6328's 2026
 * LaunchCalculator and FRC 971's aiming library):
 * <ol>
 *   <li>Predict the robot state LOOKAHEAD_SEC ahead to cover sensor, loop and mechanism lag:
 *       p' = p + v*t + a*t^2/2, v' = v + a*t, heading' = heading + omega*t.</li>
 *   <li>Move to the turret pivot: s = p' + R(heading')*offset. The ball inherits the pivot's
 *       velocity, which includes the rotation term: v_s = v' + omega x R(heading')*offset.</li>
 *   <li>Solve the time of flight T with a fixed-point iteration: shift the goal against the
 *       shooter velocity, g_v = g - v_s * lead(T), look T up again at |g_v - s|, and repeat.
 *       lead(T) = T without drag, or (1 - e^(-cT))/c with the drag coefficient c.</li>
 *   <li>A stationary shot at the virtual goal plus the inherited velocity lands on the real
 *       goal. So the turret points at g_v, and flywheel and hood come from the stationary table
 *       for this ball type at |g_v - s|. That distance also captures the extra or reduced speed
 *       needed when the robot moves toward or away from the goal.</li>
 *   <li>Turret feedforward = rate of change of the robot-relative aim angle, from a central
 *       difference of steps 1-4 along the predicted motion (FRC 604's approach).</li>
 *   <li>Gate: distance inside the table, the ball able to enter the outward-facing CELL opening,
 *       and robot motion within limits. The allowed turret error comes from the opening's real
 *       clearance for this ball at this approach angle (FRC 254's "miss at the goal" idea).</li>
 * </ol>
 *
 * <p>All inputs are in the Pedro field frame: inches, seconds, radians, CCW positive.
 */
public final class ShotSolver {
    private static final double TWO_PI = 2.0 * Math.PI;
    /** Time step of the feedforward central difference (s). */
    private static final double FEEDFORWARD_STEP_SEC = 0.01;

    private final InterpolatingTable[] rpmTables = new InterpolatingTable[BallType.values().length];
    private final InterpolatingTable[] hoodTables = new InterpolatingTable[BallType.values().length];
    private final InterpolatingTable[] tofTables = new InterpolatingTable[BallType.values().length];
    private final ShotSolution solution = new ShotSolution();
    private final Core core = new Core();

    /** Aim trim added to the field aim angle (rad). Positive turns the shot counter-clockwise. */
    public double aimTrimRad = 0.0;
    /** Distance trim added before the flywheel/hood lookup (in). Positive shoots farther. */
    public double distanceTrimIn = 0.0;

    public ShotSolver() {
        reloadTables();
    }

    /** Rebuilds the lookup tables from the SHOT_TABLE_* arrays in {@link ShooterConstants}. */
    public void reloadTables() {
        for (BallType ball : BallType.values()) {
            double[][] rows = ShooterConstants.shotTable(ball);
            int i = ball.ordinal();
            rpmTables[i] = InterpolatingTable.fromColumns(rows, 0, 1);
            hoodTables[i] = InterpolatingTable.fromColumns(rows, 0, 2);
            tofTables[i] = InterpolatingTable.fromColumns(rows, 0, 3);
        }
    }

    public double tableMinDistance(BallType ball) {
        return tofTables[ball.ordinal()].minX();
    }

    public double tableMaxDistance(BallType ball) {
        return tofTables[ball.ordinal()].maxX();
    }

    public double rpmAt(double distance, BallType ball) {
        return rpmTables[ball.ordinal()].get(distance);
    }

    public double hoodAt(double distance, BallType ball) {
        return hoodTables[ball.ordinal()].get(distance);
    }

    public double timeOfFlightAt(double distance, BallType ball) {
        return tofTables[ball.ordinal()].get(distance);
    }

    /**
     * Computes the shot. The returned object is reused on every call.
     *
     * @param x            robot x (in)
     * @param y            robot y (in)
     * @param heading      robot heading (rad)
     * @param vx           robot field velocity x (in/s)
     * @param vy           robot field velocity y (in/s)
     * @param omega        robot angular velocity (rad/s, CCW positive)
     * @param ax           robot field acceleration x (in/s^2), 0 if unused
     * @param ay           robot field acceleration y (in/s^2), 0 if unused
     * @param targetX      aim point x (in)
     * @param targetY      aim point y (in)
     * @param inwardAngle  field direction a ball must travel to enter the target (rad)
     * @param ball         the ball that will be fired next
     */
    public ShotSolution solve(double x, double y, double heading,
                              double vx, double vy, double omega,
                              double ax, double ay,
                              double targetX, double targetY, double inwardAngle,
                              BallType ball) {
        ShotSolution s = solution;
        s.problems = 0;
        s.ballType = ball;
        InterpolatingTable tof = tofTables[ball.ordinal()];
        boolean useAccel = ShooterConstants.USE_ACCELERATION;
        double axUsed = useAccel ? ax : 0.0;
        double ayUsed = useAccel ? ay : 0.0;
        double lookahead = Math.max(0.0, ShooterConstants.LOOKAHEAD_SEC);

        // 1-3. Predicted pivot state and the time-of-flight iteration.
        Core c = core;
        c.solve(tof, x, y, heading, vx, vy, omega, axUsed, ayUsed, lookahead, targetX, targetY);
        s.predictedHeading = c.heading;
        s.turretX = c.sx;
        s.turretY = c.sy;
        s.turretVx = c.svx;
        s.turretVy = c.svy;
        s.realDistance = c.realDistance;
        s.iterations = c.iterations;
        s.timeOfFlight = c.tof;
        s.leadTime = c.lead;
        s.virtualX = c.gx;
        s.virtualY = c.gy;
        s.virtualDistance = c.virtualDistance;
        if (!c.converged) s.problems |= ShotSolution.NOT_CONVERGED;
        double dx = targetX - c.sx;
        double dy = targetY - c.sy;

        // 4. Aim, flywheel and hood.
        s.aimFieldAngle = c.aim + aimTrimRad;
        s.turretAngle = normalizeSigned(s.aimFieldAngle - c.heading);
        double lookup = s.virtualDistance + distanceTrimIn;
        s.flywheelRpm = rpmTables[ball.ordinal()].get(lookup);
        s.hoodPosition = hoodTables[ball.ordinal()].get(lookup);
        if (lookup < tof.minX()) s.problems |= ShotSolution.TOO_CLOSE;
        if (lookup > tof.maxX()) s.problems |= ShotSolution.TOO_FAR;

        // 5. Turret feedforward: central difference of the robot-relative aim along the predicted
        //    motion (FRC 604's approach). Unlike the simple v/d line-of-sight formula, this
        //    includes every way the virtual goal moves: the time of flight changing with distance,
        //    the pivot's centripetal acceleration (omega^2 * offset) and the robot's acceleration.
        double h = FEEDFORWARD_STEP_SEC;
        c.solve(tof, x, y, heading, vx, vy, omega, axUsed, ayUsed, lookahead + h, targetX, targetY);
        double ahead = c.aim - c.heading;
        c.solve(tof, x, y, heading, vx, vy, omega, axUsed, ayUsed, Math.max(0.0, lookahead - h), targetX, targetY);
        double behind = c.aim - c.heading;
        double span = h + Math.min(h, lookahead);
        double rate = span > 0.0 ? normalizeSigned(ahead - behind) / span : 0.0;
        double maxRate = ShooterConstants.TURRET_MAX_FEEDFORWARD_RATE;
        s.turretRate = Math.max(-maxRate, Math.min(maxRate, rate));

        // 6. Approach check: the ball's horizontal path runs from the pivot to the REAL aim point
        //    (shooter velocity times T plus the launch vector toward g_v), and it must enter the
        //    tilted opening from outside, close to the opening's inward direction.
        if (s.realDistance > 1e-6) {
            double cosApproach = (dx * Math.cos(inwardAngle) + dy * Math.sin(inwardAngle)) / s.realDistance;
            s.approachAngle = Math.acos(Math.max(-1.0, Math.min(1.0, cosApproach)));
        } else {
            s.approachAngle = Math.PI;
        }
        if (s.approachAngle > Math.toRadians(ShooterConstants.MAX_APPROACH_ANGLE_DEG)) {
            s.problems |= ShotSolution.BAD_APPROACH;
        }

        // 7. Allowed turret error: the ball must clear the opening, whose width seen along the
        //    ball path is 20 in * cos(approach). A pointing error e misses by about e * |g_v - s|.
        double clearance = Math.cos(Math.min(s.approachAngle, Math.PI / 2.0)) * BioBuzzField.CELL_OPENING_WIDTH_IN;
        s.lateralAllowance = Math.max(0.0, (clearance - ball.diameterIn) / 2.0 - ShooterConstants.AIM_MARGIN_IN);
        s.allowedAimError = Math.atan2(s.lateralAllowance, Math.max(s.virtualDistance, 1.0));

        // 8. Motion limits: latency, velocity noise and the constant-acceleration prediction all
        //    cause errors that grow with speed, turn rate and acceleration.
        if (Math.hypot(vx, vy) > ShooterConstants.MAX_SHOT_SPEED_IN_PER_SEC) s.problems |= ShotSolution.TOO_FAST;
        if (Math.abs(omega) > ShooterConstants.MAX_SHOT_OMEGA_RAD_PER_SEC) s.problems |= ShotSolution.TURNING_TOO_FAST;
        if (Math.hypot(ax, ay) > ShooterConstants.MAX_SHOT_ACCEL_IN_PER_SEC2) s.problems |= ShotSolution.ACCELERATING;

        s.valid = s.problems == 0;
        return s;
    }

    /** Predicted turret-pivot state and the time-of-flight fixed point for one prediction time. */
    private static final class Core {
        double heading;
        double sx;
        double sy;
        double svx;
        double svy;
        double realDistance;
        double tof;
        double lead;
        double gx;
        double gy;
        double virtualDistance;
        double aim;
        int iterations;
        boolean converged;

        /** @param t prediction time (s): the robot keeps its acceleration and omega until then */
        void solve(InterpolatingTable tofTable, double x, double y, double heading0,
                   double vx, double vy, double omega, double ax, double ay,
                   double t, double targetX, double targetY) {
            // 1. Robot state t seconds ahead.
            double px = x + vx * t + 0.5 * ax * t * t;
            double py = y + vy * t + 0.5 * ay * t * t;
            double pvx = vx + ax * t;
            double pvy = vy + ay * t;
            heading = heading0 + omega * t;

            // 2. Turret pivot position and velocity (rigid body: v + omega x r).
            double cos = Math.cos(heading);
            double sin = Math.sin(heading);
            double ox = ShooterConstants.TURRET_OFFSET_X_IN * cos - ShooterConstants.TURRET_OFFSET_Y_IN * sin;
            double oy = ShooterConstants.TURRET_OFFSET_X_IN * sin + ShooterConstants.TURRET_OFFSET_Y_IN * cos;
            sx = px + ox;
            sy = py + oy;
            svx = pvx - omega * oy;
            svy = pvy + omega * ox;
            realDistance = Math.hypot(targetX - sx, targetY - sy);

            // 3. Fixed-point iteration on the time of flight.
            double time = tofTable.get(realDistance);
            converged = false;
            int maxIterations = Math.max(1, ShooterConstants.SOLVER_MAX_ITERATIONS);
            iterations = 0;
            while (iterations < maxIterations) {
                iterations++;
                double l = leadTime(time);
                double next = tofTable.get(Math.hypot(targetX - svx * l - sx, targetY - svy * l - sy));
                double change = next - time;
                // Plain iteration contracts at FTC speeds; damp the second half in case it oscillates.
                time = iterations > maxIterations / 2 ? time + 0.5 * change : next;
                if (Math.abs(change) < ShooterConstants.SOLVER_TOLERANCE_SEC) {
                    converged = true;
                    break;
                }
            }
            tof = time;
            lead = leadTime(time);
            gx = targetX - svx * lead;
            gy = targetY - svy * lead;
            double rx = gx - sx;
            double ry = gy - sy;
            virtualDistance = Math.hypot(rx, ry);
            aim = virtualDistance > 1e-6 ? Math.atan2(ry, rx) : heading;
        }
    }

    /** Time the robot velocity acts on the ball, optionally reduced by drag. */
    static double leadTime(double tof) {
        double c = ShooterConstants.LEAD_DRAG_COEFF;
        if (Math.abs(c) < 1e-9) return tof;
        return (1.0 - Math.exp(-c * tof)) / c;
    }

    /** Wraps an angle to (-pi, pi]. */
    public static double normalizeSigned(double angle) {
        double a = angle % TWO_PI;
        if (a <= -Math.PI) a += TWO_PI;
        if (a > Math.PI) a -= TWO_PI;
        return a;
    }
}
