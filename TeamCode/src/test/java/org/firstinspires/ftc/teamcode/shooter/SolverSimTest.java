package org.firstinspires.ftc.teamcode.shooter;

import java.util.Random;

/**
 * Off-robot checks of the shoot-on-the-move math: plain Java, no JUnit or robot needed. It uses
 * your current shot tables, so re-run it after editing them. From the project root:
 *
 *   javac --release 8 -d build/sotm-test TeamCode/src/main/java/org/firstinspires/ftc/teamcode/shooter/{ShooterConstants,InterpolatingTable,ShotSolution,ShotSolver,BioBuzzField,Alliance,HiveCell,BallType,MotionEstimator,TurretMath,TurretController}.java TeamCode/src/test/java/org/firstinspires/ftc/teamcode/shooter/*.java
 *   java -cp build/sotm-test org.firstinspires.ftc.teamcode.shooter.SolverSimTest
 *
 * The ball model is drag-free or linear-drag projectile motion that is consistent with the shot
 * table, so it checks the geometry, signs and iteration, not the table values themselves.
 */
public class SolverSimTest {
    static int failures = 0;
    static final BallType P = BallType.POLLEN;

    static void check(boolean ok, String msg) {
        if (!ok) { failures++; System.out.println("FAIL: " + msg); }
    }

    /**
     * Ball model with linear horizontal drag k (k = 0: no drag). A stationary shot programmed for
     * distance D flies T(D) and covers D. Linear drag is separable, so the inherited shooter
     * velocity adds v_s * (1 - e^(-kT)) / k and the vertical motion (and T) is unchanged.
     */
    static double[] land(ShotSolver solver, ShotSolution s, double k, double sx, double sy, double svx, double svy, BallType b) {
        double D = s.virtualDistance;
        double T = solver.timeOfFlightAt(D, b);
        double drift = k == 0 ? T : (1 - Math.exp(-k * T)) / k;
        double ux = Math.cos(s.aimFieldAngle), uy = Math.sin(s.aimFieldAngle);
        return new double[]{sx + D * ux + svx * drift, sy + D * uy + svy * drift};
    }

    static void noLimits() {
        ShooterConstants.MAX_SHOT_SPEED_IN_PER_SEC = 1e9;
        ShooterConstants.MAX_SHOT_OMEGA_RAD_PER_SEC = 1e9;
        ShooterConstants.MAX_SHOT_ACCEL_IN_PER_SEC2 = 1e9;
    }

    public static void main(String[] args) {
        // Pin the constants that the checks below assert exact values for.
        BioBuzzField.FIELD_CENTER_IN = 72.0;
        BioBuzzField.HIVE_SIDE_OFFSET_IN = 12.75;
        BioBuzzField.CELL_OPENING_AXIAL_OFFSET_IN = 17.3;
        BioBuzzField.CELL_OPENING_WIDTH_IN = 20.0;
        ShooterConstants.AIM_MARGIN_IN = 2.0;
        ShooterConstants.MAX_APPROACH_ANGLE_DEG = 45.0;
        ShooterConstants.LEAD_DRAG_COEFF = 0.0;
        ShooterConstants.TURRET_MAX_FEEDFORWARD_RATE = 10.0;
        ShooterConstants.LOOKAHEAD_SEC = 0.05;
        ShooterConstants.USE_ACCELERATION = true;
        ShooterConstants.SOLVER_TOLERANCE_SEC = 1e-5;
        ShooterConstants.SOLVER_MAX_ITERATIONS = 20;
        noLimits();

        // ---- InterpolatingTable
        InterpolatingTable t = new InterpolatingTable(new double[]{0, 10, 20}, new double[]{0, 100, 300});
        check(Math.abs(t.get(5) - 50) < 1e-12 && Math.abs(t.get(15) - 200) < 1e-12, "table interp");
        check(t.get(-5) == 0 && t.get(25) == 300 && t.get(10) == 100, "table clamp/exact");
        boolean threw = false;
        try { new InterpolatingTable(new double[]{0, 0}, new double[]{1, 2}); } catch (IllegalArgumentException e) { threw = true; }
        check(threw, "table rejects non-increasing x");

        // ---- normalizeSigned
        check(Math.abs(ShotSolver.normalizeSigned(3 * Math.PI) - Math.PI) < 1e-12, "normalize 3pi");
        check(Math.abs(ShotSolver.normalizeSigned(-Math.PI) - Math.PI) < 1e-12, "normalize -pi -> pi");
        check(Math.abs(ShotSolver.normalizeSigned(-0.5) + 0.5) < 1e-12, "normalize -0.5");

        // ---- TurretMath
        double r = Math.toRadians(1);
        double sp = TurretMath.chooseSetpoint(179 * r, 170 * r, -180 * r, 180 * r);
        check(Math.abs(sp - 179 * r) < 1e-9, "turret: plain target " + Math.toDegrees(sp));
        sp = TurretMath.chooseSetpoint(-179 * r, 179 * r, -180 * r, 180 * r);
        check(Math.abs(sp + 179 * r) < 1e-9 && TurretMath.reaches(sp, -179 * r), "turret: must unwind past limit " + Math.toDegrees(sp));
        sp = TurretMath.chooseSetpoint(-170 * r, 0, -90 * r, 200 * r);
        check(Math.abs(sp - 190 * r) < 1e-9, "turret: uses extended travel " + Math.toDegrees(sp));
        sp = TurretMath.chooseSetpoint(135 * r, 0, -90 * r, 90 * r);
        check(Math.abs(sp - 90 * r) < 1e-9 && !TurretMath.reaches(sp, 135 * r), "turret: clamps unreachable " + Math.toDegrees(sp));
        sp = TurretMath.chooseSetpoint(10 * r, 360 * r, -400 * r, 400 * r);
        check(Math.abs(sp - 370 * r) < 1e-9, "turret: continuity with previous setpoint " + Math.toDegrees(sp));
        // Wrap continuity: sweep the target through +-pi repeatedly; the setpoint must stay inside
        // the limits and only jump (by 2pi) when a limit forces an unwind.
        double prev = 0; int jumps = 0; boolean inside = true;
        for (int i = 0; i <= 7200; i++) {
            double target = ShotSolver.normalizeSigned(Math.toRadians(i * 0.5));
            double next = TurretMath.chooseSetpoint(target, prev, -200 * r, 200 * r);
            inside &= next >= -200 * r - 1e-9 && next <= 200 * r + 1e-9 && TurretMath.reaches(next, target);
            if (Math.abs(next - prev) > 10 * r) jumps++;
            prev = next;
        }
        check(inside, "wrap sweep stays inside limits and on target");
        // Unwinds at 200.5 deg + 360k for 0..3600 deg: k = 0..9.
        check(jumps == 10, "wrap sweep unwinds once per 360 deg after the first 200 deg (jumps=" + jumps + ")");

        ShotSolver solver = new ShotSolver();

        // ---- Pure rotation sign (report test): offset (5, 0), heading 0, omega 2 -> v_s = (0, +10)
        ShooterConstants.TURRET_OFFSET_X_IN = 5; ShooterConstants.TURRET_OFFSET_Y_IN = 0;
        ShooterConstants.LOOKAHEAD_SEC = 0.0;
        ShotSolution s = solver.solve(40, 10, 0, 0, 0, 2, 0, 0, 59.25, 54.7, Math.PI / 2, P);
        check(Math.abs(s.turretVx) < 1e-9 && Math.abs(s.turretVy - 10) < 1e-9, "omega x r sign: " + s.turretVx + "," + s.turretVy);

        // ---- Feedforward sign (report test): pivot at origin, goal (48, 0), v_s = (0, 30) -> LOS rate about -0.625 rad/s
        ShooterConstants.TURRET_OFFSET_X_IN = 0;
        s = solver.solve(0, 0, 0, 0, 30, 0, 0, 0, 48, 0, 0, P);
        // The turret aims at the virtual goal (48, -30T): rate = r_x * (-30) / d^2, plus a small term
        // from T growing with distance. The real-goal v/d value (-0.625) does not apply.
        double expected = 48.0 * -30.0 / (s.virtualDistance * s.virtualDistance);
        check(s.turretRate < 0 && Math.abs(s.turretRate - expected) < 0.03, "feedforward sign/size " + s.turretRate + " vs " + expected);
        System.out.printf("feedforward sign test: %.4f rad/s (pure LOS at 48 in would be -0.625; virtual distance %.1f in)%n", s.turretRate, s.virtualDistance);
        ShooterConstants.LOOKAHEAD_SEC = 0.05;

        // ---- Stationary shot
        double gx = BioBuzzField.aimX(Alliance.RED, HiveCell.AUDIENCE), gy = BioBuzzField.aimY(Alliance.RED, HiveCell.AUDIENCE);
        check(Math.abs(gx - 59.25) < 1e-9 && Math.abs(gy - 54.7) < 1e-9, "red audience aim point " + gx + "," + gy);
        s = solver.solve(40, 10, Math.toRadians(30), 0, 0, 0, 0, 0, gx, gy, BioBuzzField.inwardAngle(HiveCell.AUDIENCE), P);
        double expAim = Math.atan2(gy - 10, gx - 40);
        check(Math.abs(s.aimFieldAngle - expAim) < 1e-9, "stationary aim");
        check(Math.abs(s.turretAngle - ShotSolver.normalizeSigned(expAim - Math.toRadians(30))) < 1e-9, "stationary turret angle");
        check(Math.abs(s.virtualDistance - s.realDistance) < 1e-9 && Math.abs(s.turretRate) < 1e-9, "stationary virtual==real, zero ff");
        check(s.valid, "stationary shot valid: " + s.describeProblems());
        System.out.printf("stationary: dist %.2f in, turret %.2f deg, rpm %.0f, hood %.3f, approach %.1f deg, miss allowance %.2f in, aim tol %.2f deg%n",
                s.realDistance, Math.toDegrees(s.turretAngle), s.flywheelRpm, s.hoodPosition, Math.toDegrees(s.approachAngle),
                s.lateralAllowance, Math.toDegrees(s.allowedAimError));

        // ---- CELL-clearance tolerance: head-on POLLEN (20 - 2.8)/2 - 2 = 6.6 in, NECTAR 6.2 in
        s = solver.solve(gx, gy - 50, Math.PI / 2, 0, 0, 0, 0, 0, gx, gy, Math.PI / 2, BallType.POLLEN);
        check(Math.abs(s.lateralAllowance - 6.6) < 1e-6 && Math.abs(s.approachAngle) < 1e-6, "pollen head-on allowance " + s.lateralAllowance);
        s = solver.solve(gx, gy - 50, Math.PI / 2, 0, 0, 0, 0, 0, gx, gy, Math.PI / 2, BallType.NECTAR);
        check(Math.abs(s.lateralAllowance - 6.2) < 1e-6, "nectar head-on allowance " + s.lateralAllowance);
        check(s.ballType == BallType.NECTAR, "solution records ball type");

        // ---- Wrong side / oblique
        s = solver.solve(40, 100, 0, 0, 0, 0, 0, 0, gx, gy, BioBuzzField.inwardAngle(HiveCell.AUDIENCE), P);
        check((s.problems & ShotSolution.BAD_APPROACH) != 0, "wrong side flagged");
        s = solver.solve(gx - 60, gy - 10, 0, 0, 0, 0, 0, 0, gx, gy, BioBuzzField.inwardAngle(HiveCell.AUDIENCE), P);
        check((s.problems & ShotSolution.BAD_APPROACH) != 0, "oblique approach flagged (" + Math.toDegrees(s.approachAngle) + " deg)");

        // ---- Field symmetry
        double bx = BioBuzzField.aimX(Alliance.BLUE, HiveCell.REAR), by = BioBuzzField.aimY(Alliance.BLUE, HiveCell.REAR);
        check(Math.abs(bx - (144 - gx)) < 1e-9 && Math.abs(by - (144 - gy)) < 1e-9, "blue rear = rotated red audience");
        check(BioBuzzField.startUpCell(Alliance.BLUE) == HiveCell.REAR && BioBuzzField.startUpCell(Alliance.RED) == HiveCell.AUDIENCE, "start up cells");
        double[] rp = BioBuzzField.resetPose(Alliance.BLUE);
        check(Math.abs(rp[0] - 135) < 1e-9 && Math.abs(rp[1] - 135) < 1e-9 && Math.abs(rp[2] - Math.PI) < 1e-9, "blue reset pose");
        // Blue solver on the 180-deg-rotated red state gives the same turret command (report test)
        ShooterConstants.TURRET_OFFSET_X_IN = -4; ShooterConstants.TURRET_OFFSET_Y_IN = 2;
        Random rnd = new Random(1);
        double worstSym = 0;
        for (int i = 0; i < 2000; i++) {
            double x = 20 + rnd.nextDouble() * 80, y = 5 + rnd.nextDouble() * 35, h = rnd.nextDouble() * 6.28;
            double vx = (rnd.nextDouble() * 2 - 1) * 50, vy = (rnd.nextDouble() * 2 - 1) * 50, w = (rnd.nextDouble() * 2 - 1) * 3;
            double ax = (rnd.nextDouble() * 2 - 1) * 80, ay = (rnd.nextDouble() * 2 - 1) * 80;
            ShotSolution a = solver.solve(x, y, h, vx, vy, w, ax, ay, gx, gy, BioBuzzField.inwardAngle(HiveCell.AUDIENCE), P);
            double ta = a.turretAngle, ra = a.turretRate, da = a.virtualDistance, ha = a.hoodPosition; int pa = a.problems;
            ShotSolution b = solver.solve(144 - x, 144 - y, h + Math.PI, -vx, -vy, w, -ax, -ay, bx, by, BioBuzzField.inwardAngle(HiveCell.REAR), P);
            worstSym = Math.max(worstSym, Math.abs(ShotSolver.normalizeSigned(b.turretAngle - ta)) + Math.abs(b.turretRate - ra)
                    + Math.abs(b.virtualDistance - da) + Math.abs(b.hoodPosition - ha));
            check(b.problems == pa, "symmetric problem flags");
        }
        check(worstSym < 1e-6, "alliance symmetry error " + worstSym);

        // ---- Moving shots: random states, compare miss with and without compensation
        for (double k : new double[]{0.0, 0.375}) {
            ShooterConstants.LEAD_DRAG_COEFF = k;
            for (BallType ball : BallType.values()) {
                double worst = 0, sumNaive = 0, worstNaive = 0;
                int n = 0, maxIt = 0, notConv = 0;
                for (int i = 0; i < 20000; i++) {
                    double x = 20 + rnd.nextDouble() * 80;          // audience half, in front of the opening
                    double y = 5 + rnd.nextDouble() * 45;
                    double h = rnd.nextDouble() * 2 * Math.PI;
                    double speed = rnd.nextDouble() * 60, dir = rnd.nextDouble() * 2 * Math.PI;
                    double vx = speed * Math.cos(dir), vy = speed * Math.sin(dir);
                    double w = (rnd.nextDouble() * 2 - 1) * 3.0;
                    double acc = rnd.nextDouble() * 100, adir = rnd.nextDouble() * 2 * Math.PI;
                    double ax = acc * Math.cos(adir), ay = acc * Math.sin(adir);
                    s = solver.solve(x, y, h, vx, vy, w, ax, ay, gx, gy, BioBuzzField.inwardAngle(HiveCell.AUDIENCE), ball);
                    check(!Double.isNaN(s.turretAngle + s.turretRate + s.flywheelRpm + s.hoodPosition + s.allowedAimError), "NaN in solution");
                    if ((s.problems & (ShotSolution.TOO_CLOSE | ShotSolution.TOO_FAR)) != 0) continue;
                    if ((s.problems & ShotSolution.NOT_CONVERGED) != 0) notConv++;
                    maxIt = Math.max(maxIt, s.iterations);
                    // True state at release (constant a and omega over the lookahead).
                    double tau = ShooterConstants.LOOKAHEAD_SEC;
                    double px = x + vx * tau + 0.5 * ax * tau * tau, py = y + vy * tau + 0.5 * ay * tau * tau;
                    double pvx = vx + ax * tau, pvy = vy + ay * tau, ph = h + w * tau;
                    double ox = -4 * Math.cos(ph) - 2 * Math.sin(ph), oy = -4 * Math.sin(ph) + 2 * Math.cos(ph);
                    double sx = px + ox, sy = py + oy, svx = pvx - w * oy, svy = pvy + w * ox;
                    double fieldAim = ph + s.turretAngle;
                    check(Math.abs(ShotSolver.normalizeSigned(fieldAim - s.aimFieldAngle)) < 1e-9, "turret->field aim");
                    double[] p = land(solver, s, k, sx, sy, svx, svy, ball);
                    double miss = Math.hypot(p[0] - gx, p[1] - gy);
                    worst = Math.max(worst, miss);
                    double Dn = Math.hypot(gx - sx, gy - sy);
                    double Tn = solver.timeOfFlightAt(Dn, ball);
                    double driftN = k == 0 ? Tn : (1 - Math.exp(-k * Tn)) / k;
                    double naive = Math.hypot(svx * driftN, svy * driftN);
                    sumNaive += naive; worstNaive = Math.max(worstNaive, naive);
                    n++;
                }
                System.out.printf("%s drag k=%.3f: %d shots, worst miss WITH compensation %.4f in; naive mean %.1f in, worst %.1f in; max iterations %d, not converged %d%n",
                        ball, k, n, worst, sumNaive / n, worstNaive, maxIt, notConv);
                check(worst < 0.05, "compensated miss too big: " + worst);
                check(notConv == 0, "solver failed to converge " + notConv + " times");
            }
        }
        ShooterConstants.LEAD_DRAG_COEFF = 0.0;

        // ---- Turret feedforward vs finite difference of the turret command along the true motion
        double worstRateErr = 0; int ffN = 0;
        for (int i = 0; i < 5000; i++) {
            double x = 20 + rnd.nextDouble() * 80, y = 5 + rnd.nextDouble() * 40, h = rnd.nextDouble() * 6.28;
            double vx = (rnd.nextDouble() * 2 - 1) * 40, vy = (rnd.nextDouble() * 2 - 1) * 40, w = (rnd.nextDouble() * 2 - 1) * 3;
            double ax = (rnd.nextDouble() * 2 - 1) * 60, ay = (rnd.nextDouble() * 2 - 1) * 60;
            double dt = 1e-4;
            ShotSolution a = solver.solve(x, y, h, vx, vy, w, ax, ay, gx, gy, Math.PI / 2, P);
            if ((a.problems & (ShotSolution.TOO_CLOSE | ShotSolution.TOO_FAR)) != 0) continue;
            if (a.virtualDistance < solver.tableMinDistance(P) + 1 || a.virtualDistance > solver.tableMaxDistance(P) - 1) continue;
            double a0 = a.turretAngle, rate = a.turretRate;
            if (Math.abs(rate) >= ShooterConstants.TURRET_MAX_FEEDFORWARD_RATE) continue;
            ShotSolution b = solver.solve(x + vx * dt + 0.5 * ax * dt * dt, y + vy * dt + 0.5 * ay * dt * dt, h + w * dt,
                    vx + ax * dt, vy + ay * dt, w, ax, ay, gx, gy, Math.PI / 2, P);
            double fd = ShotSolver.normalizeSigned(b.turretAngle - a0) / dt;
            worstRateErr = Math.max(worstRateErr, Math.abs(fd - rate));
            ffN++;
        }
        System.out.printf("turret feedforward (%d states): worst |solver rate - finite difference| = %.3f deg/s%n", ffN, Math.toDegrees(worstRateErr));
        check(ffN > 1000 && Math.toDegrees(worstRateErr) < 1.0, "feedforward rate disagrees with finite difference");

        // ---- Motion estimator: constant acceleration
        ShooterConstants.VELOCITY_FILTER_SEC = 0.0;
        ShooterConstants.ACCEL_FILTER_SEC = 0.08;
        MotionEstimator m = new MotionEstimator();
        double v = 0;
        for (int i = 0; i < 100; i++) { v += 50 * 0.02; m.update(v, 0, 0, 0.02); }
        check(Math.abs(m.ax() - 50) < 0.5 && Math.abs(m.ay()) < 1e-9, "accel estimate " + m.ax());

        // ---- Motion gating
        ShooterConstants.MAX_SHOT_SPEED_IN_PER_SEC = 60;
        s = solver.solve(40, 10, 0, 70, 0, 0, 0, 0, gx, gy, Math.PI / 2, P);
        check((s.problems & ShotSolution.TOO_FAST) != 0, "too fast flagged");

        System.out.println(failures == 0 ? "ALL CHECKS PASSED" : failures + " CHECK(S) FAILED");
        if (failures != 0) System.exit(1);
    }
}
