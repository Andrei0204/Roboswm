package org.firstinspires.ftc.teamcode.shooter;

import java.util.Random;

/**
 * Off-robot check of {@link TurretController} against a simulated Axon turret: plain Java, no
 * JUnit or robot needed. From the project root:
 *
 *   javac --release 8 -d build/sotm-test TeamCode/src/main/java/org/firstinspires/ftc/teamcode/shooter/{ShooterConstants,InterpolatingTable,ShotSolution,ShotSolver,BioBuzzField,Alliance,HiveCell,BallType,MotionEstimator,TurretMath,TurretController}.java TeamCode/src/test/java/org/firstinspires/ftc/teamcode/shooter/*.java
 *   java -cp build/sotm-test org.firstinspires.ftc.teamcode.shooter.TurretSimTest
 *
 * The simulated servo is deliberately imperfect: its real scale is 3% off the constants, it
 * follows its command with a 50 ms lag and a 400 deg/s slew limit, the gears have 0.8 deg of
 * backlash, and the encoder is quantized and reports velocity over a 50 ms window like a hub.
 * A count reset only shows up after one stale (bulk-cached) read, as on a real hub.
 */
public class TurretSimTest {
    static int failures = 0;

    static void check(boolean ok, String msg) {
        if (!ok) { failures++; System.out.println("FAIL: " + msg); }
    }

    // ---- simulated hardware
    static final double SCALE_ERROR = 1.03;
    static final double SERVO_TAU = 0.05;
    static final double SERVO_SLEW_DEG = 400.0;
    static final double BACKLASH_DEG = 0.8;
    static double servoDeg;      // servo output, in turret degrees
    static double turretDeg;     // true turret angle
    static double commandDeg;    // where the servo is trying to go (turret degrees)
    static boolean powered;
    static final double[] tickHistory = new double[3];   // ticks 0, 20, 40 ms ago (50 Hz loop)
    static int resetOffset;       // raw count at the last encoder reset
    static boolean staleRead;     // the next read still returns the pre-reset count
    static int staleCount;
    static boolean ignoreResets;  // simulate a reset that never happens

    static double degPerTick() {
        return 360.0 / (ShooterConstants.TURRET_ENCODER_TICKS_PER_REV * ShooterConstants.TURRET_ENCODER_REVS_PER_TURRET_REV);
    }

    static int rawTicks() {
        return (int) Math.round(ShooterConstants.TURRET_ENCODER_SIGN * turretDeg / degPerTick());
    }

    static int encoderTicks() {
        return rawTicks() - resetOffset;
    }

    /** What the hub does on STOP_AND_RESET_ENCODER. */
    static void resetEncoder() {
        if (ignoreResets) return;
        staleCount = encoderTicks();
        staleRead = true;
        resetOffset = rawTicks();
        homeTurretDeg = turretDeg;
    }

    static void writeServo(double position) {
        // The real servo's map differs from the constants by SCALE_ERROR.
        commandDeg = (position - ShooterConstants.TURRET_SERVO_POSITION_AT_ZERO)
                * ShooterConstants.TURRET_DEG_PER_SERVO_UNIT * SCALE_ERROR;
        powered = true;
    }

    static void stepPhysics(double dt) {
        int sub = 20;
        double h = dt / sub;
        for (int i = 0; i < sub; i++) {
            if (powered) {
                double v = (commandDeg - servoDeg) / SERVO_TAU;
                v = Math.max(-SERVO_SLEW_DEG, Math.min(SERVO_SLEW_DEG, v));
                servoDeg += v * h;
            }
            double gap = servoDeg - turretDeg;
            if (gap > BACKLASH_DEG / 2) turretDeg = servoDeg - BACKLASH_DEG / 2;
            if (gap < -BACKLASH_DEG / 2) turretDeg = servoDeg + BACKLASH_DEG / 2;
        }
    }

    static TurretController ctl;
    static double homeTurretDeg;   // true turret angle that homing called 0
    static double time;

    /**
     * Runs one 50 Hz loop with the given target. Returns the true aiming error (deg) at the
     * instant the encoder was read, which is the instant the controller's aim check refers to.
     */
    static double loop(double targetDeg, double targetRateDeg) {
        return step(false, targetDeg, targetRateDeg);
    }

    /** One 50 Hz INIT loop: the controller may only move the servo while homing. */
    static void initLoop() {
        step(true, 0, 0);
    }

    static void readEncoder() {
        int ticks = staleRead ? staleCount : encoderTicks();
        staleRead = false;
        double velocity = (ticks - tickHistory[2]) / 0.04;
        tickHistory[2] = tickHistory[1];
        tickHistory[1] = tickHistory[0];
        tickHistory[0] = ticks;
        ctl.setMeasurement(ticks, velocity);
    }

    static double step(boolean init, double targetDeg, double targetRateDeg) {
        double dt = 0.02;
        // Hub velocity: ticks over the last ~40-60 ms window.
        readEncoder();
        boolean wasHoming = ctl.isHoming();
        double position = init
                ? ctl.initUpdate(dt)
                : ctl.update(Math.toRadians(targetDeg), Math.toRadians(targetRateDeg), dt);
        if (ctl.takeResetRequest()) resetEncoder();
        double error = targetDeg - (turretDeg - homeTurretDeg);
        if (init && !wasHoming) {
            check(Double.isNaN(position), "servo moved in INIT without a homing request");
        } else {
            check(position >= 0.0 && position <= 1.0, "servo position out of range: " + position);
            writeServo(position);
        }
        stepPhysics(dt);
        time += dt;
        return error;
    }

    public static void main(String[] args) {
        // The constants as shipped for the 60:24 + 26:76 Axon turret.
        ShooterConstants.TURRET_SERVO_POSITION_AT_ZERO = 0.5;
        ShooterConstants.TURRET_DEG_PER_SERVO_UNIT = 350.0 * ShooterConstants.TURRET_GEAR_RATIO;
        ShooterConstants.TURRET_MIN_DEG = -145.0;
        ShooterConstants.TURRET_MAX_DEG = 145.0;
        ShooterConstants.TURRET_ENCODER_TICKS_PER_REV = 8192.0;
        ShooterConstants.TURRET_ENCODER_REVS_PER_TURRET_REV = 1.0;
        ShooterConstants.TURRET_ENCODER_SIGN = -1;     // also exercise a reversed encoder
        TurretController.validateConstants();

        Random rnd = new Random(3);
        for (int trial = 0; trial < 20; trial++) {
            TurretController.forgetHomingForTest();   // app restart: the count is not trusted
            resetOffset = 0;
            staleRead = false;
            ignoreResets = false;
            ctl = new TurretController();
            time = 0;
            powered = false;
            servoDeg = turretDeg = (rnd.nextDouble() * 2 - 1) * 120.0;   // unpowered, anywhere
            homeTurretDeg = Double.NaN;
            int t0 = encoderTicks();
            tickHistory[0] = tickHistory[1] = tickHistory[2] = t0;

            // 0. INIT without the button: nothing moves, and there is no zero.
            double startDeg = turretDeg;
            for (int i = 0; i < 25; i++) initLoop();
            check(!powered && turretDeg == startDeg && !ctl.hasReference(), "turret moved or homed in INIT without the button");

            // 0b. Started without a zero: the turret points open-loop but never counts as aimed.
            TurretController saved = ctl;
            double savedServo = servoDeg, savedTurret = turretDeg, savedCommand = commandDeg;
            boolean savedPowered = powered;
            for (int i = 0; i < 50; i++) {
                loop(40, 0);
                check(!ctl.isAimed(Math.toRadians(30)), "un-homed turret counted as aimed");
            }
            check(Math.abs(turretDeg - 40 * SCALE_ERROR) < 2.0, "open-loop pointing off: " + turretDeg);
            ctl = saved;
            servoDeg = savedServo; turretDeg = savedTurret; commandDeg = savedCommand; powered = savedPowered;
            tickHistory[0] = tickHistory[1] = tickHistory[2] = encoderTicks();

            // 1. The INIT button homes; aiming is blocked until it ends.
            ctl.startHoming();
            time = 0;
            int loops = 0;
            while (ctl.isHoming() && loops < 200) {
                initLoop();
                check(!ctl.isAimed(Math.toRadians(5)), "aimed while homing");
                loops++;
            }
            check(!ctl.isHoming() && !ctl.homeTimedOut(), "homing did not settle (trial " + trial + ")");
            check(time <= ShooterConstants.TURRET_HOME_TIMEOUT_SEC + 0.05, "homing took " + time + " s");
            // Homing zero must be the servo's zero within backlash: the 3% scale error does not
            // matter at position 0.5.
            check(Math.abs(homeTurretDeg) <= BACKLASH_DEG / 2 + 0.2, "home zero off by " + homeTurretDeg + " deg");

            // 2. Static target: the encoder correction removes the 3% scale error (1.8 deg at 60).
            double err = 0;
            for (int i = 0; i < 75; i++) err = loop(60, 0);
            // Backlash (0.8 deg) bounds how exactly the turret can sit.
            check(Math.abs(err) < 0.5, "static error at 60 deg: " + err);
            double staticErr = err;
            check(ctl.isAimed(Math.toRadians(1.0)), "not aimed after settling at 60 deg");

            // 3. Sweep at 60 deg/s from 60 to -60 deg: lead cancels most of the servo lag.
            double worstSweep = 0;
            for (int i = 0; i <= 100; i++) {
                double target = 60 - 60 * (i * 0.02);
                err = loop(target, -60);
                if (i > 15) worstSweep = Math.max(worstSweep, Math.abs(err));
                // Never report aimed when the true error is larger than the tolerance asked
                // (plus encoder quantization and the encoder's velocity window lag).
                double tolDeg = 1.0;
                if (ctl.isAimed(Math.toRadians(tolDeg))) {
                    check(Math.abs(err) <= tolDeg + 0.1, "aimed but true error " + err);
                }
            }
            if (trial == 0) {
                System.out.printf("trial 0: static error %.2f deg, 60 deg/s sweep error up to %.2f deg (3%% servo scale error, 0.8 deg backlash)%n",
                        staticErr, worstSweep);
            }
            // Well inside the aim tolerance, which is at least 4.7 deg at 80 in.
            check(worstSweep < 3.0, "sweep tracking error " + worstSweep);

            // 4. Target in the dead zone behind the turret: unreachable, never aimed, held at a limit.
            for (int i = 0; i < 50; i++) loop(170, 0);
            check(!ctl.isReachable() && !ctl.isAimed(Math.toRadians(30)), "dead-zone target treated as reachable");
            check(Math.abs(Math.toDegrees(ctl.setpoint()) - 145.0) < 1e-6, "dead-zone setpoint not at limit: " + Math.toDegrees(ctl.setpoint()));

            // 5. Next OpMode (AUTO -> TeleOp): a new controller trusts the count the hub kept,
            //    with nothing stored.
            ctl = new TurretController();
            readEncoder();
            check(ctl.checkZero(Math.toRadians(15)) && ctl.hasReference(), "count from the previous OpMode not trusted");
            for (int i = 0; i < 75; i++) err = loop(30, 0);
            check(Math.abs(err) < 0.5 && ctl.isAimed(Math.toRadians(1.0)), "in the next OpMode, error at 30: " + err);

            // 5b. A count that puts the turret far outside its travel (hub lost its count) is dropped.
            int savedOffset = resetOffset;
            resetOffset = rawTicks() - (int) Math.round(ShooterConstants.TURRET_ENCODER_SIGN * 200.0 / degPerTick());
            ctl = new TurretController();
            readEncoder();
            check(!ctl.checkZero(Math.toRadians(15)) && !ctl.hasReference(), "impossible count (200 deg) trusted");
            for (int i = 0; i < 25; i++) loop(30, 0);
            check(!ctl.isAimed(Math.toRadians(30)), "aimed after the count was dropped");
            resetOffset = savedOffset;
            tickHistory[0] = tickHistory[1] = tickHistory[2] = encoderTicks();

            // 5c. A reset that never takes effect is caught and leaves the turret un-homed.
            ignoreResets = true;
            resetOffset = rawTicks() - (int) Math.round(ShooterConstants.TURRET_ENCODER_SIGN * 50.0 / degPerTick());
            ctl.startHoming();
            loops = 0;
            while (ctl.isHoming() && loops < 200) { initLoop(); loops++; }
            check(ctl.resetFailed() && !ctl.hasReference(), "failed reset not caught");
            ignoreResets = false;

            // 6. Home again in INIT, then aim.
            ctl.startHoming();
            loops = 0;
            while (ctl.isHoming() && loops < 200) { initLoop(); loops++; }
            check(!ctl.isHoming() && !ctl.homeTimedOut() && !ctl.resetFailed() && ctl.hasReference(), "re-home did not settle");
            double holdWorst = 0;
            for (int i = 0; i < 100; i++) {
                err = loop(-90, 0);
                if (i >= 75) holdWorst = Math.max(holdWorst, Math.abs(err));   // last 0.5 s: no hunting
            }
            check(holdWorst < 0.5 && ctl.isAimed(Math.toRadians(1.0)), "after re-home, error at -90 over the last 0.5 s: " + holdWorst);
        }
        System.out.println(failures == 0 ? "ALL TURRET CHECKS PASSED" : failures + " TURRET CHECK(S) FAILED");
        if (failures != 0) System.exit(1);
    }
}
