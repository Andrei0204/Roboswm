package org.firstinspires.ftc.teamcode.aiming;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;
import org.firstinspires.ftc.teamcode.config.ShooterConstants;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.BioBuzzField;
import org.firstinspires.ftc.teamcode.field.HiveCell;
import org.firstinspires.ftc.teamcode.field.HiveState;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

/**
 * Shoot-on-the-move aiming: turret + flywheel + hood, aimed at the up CELL of our BIOBUZZ HIVE
 * using Pedro Pathing's pose and velocity. It decides when a shot is ready ({@link #isReady()});
 * feeding the ball is the {@code Feeder}'s job, and {@code RobotSys} ties the two together.
 *
 * <p>Call {@link #update()} once per loop, after {@code follower.update()}. It reads the
 * mechanism sensors, solves the shot and drives the turret, flywheel and hood.
 *
 * <p>Readiness ("gated" firing, as in FRC 254 / 6328): a valid solution (in range, correct
 * side of the HIVE, robot motion within limits), flywheel inside its speed window, hood settled,
 * turret error under the angle that keeps the ball inside the CELL opening with AIM_MARGIN_IN to
 * spare and moving at the commanded rate (both measured by the turret encoder), the turret target
 * inside its reachable range, the turret homed, and no recent HIVE TIP, all held for
 * READY_DEBOUNCE_LOOPS consecutive loops.
 */
public final class ShootOnTheMove {
    /** A turret zero is dropped if the encoder count puts the turret this far outside its travel. */
    private static final double STORED_ZERO_MARGIN_DEG = 15.0;

    private final Follower follower;
    private final Turret turret;
    private final Flywheel flywheel;
    private final Hood hood;
    private final ShotSolver solver = new ShotSolver();
    private final MotionEstimator motion = new MotionEstimator();
    private final HiveState hive;
    private final List<VoltageSensor> voltageSensors;

    private Alliance alliance;
    private ShotSolution solution;
    private boolean armed;
    private double overrideRpm = Double.NaN;
    private double overrideHood = Double.NaN;

    private long lastNanos;
    private double voltage = ShooterConstants.NOMINAL_VOLTAGE;
    private double nextVoltageRead = Double.NEGATIVE_INFINITY;
    private boolean ready;
    private boolean storedZeroRejected;
    private int readyLoops;
    private BallType nextBall = BallType.POLLEN;

    public ShootOnTheMove(HardwareMap hardwareMap, Follower follower, Alliance alliance) {
        this.follower = follower;
        this.alliance = alliance;
        turret = new Turret(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        hood = new Hood(hardwareMap);
        hive = new HiveState(alliance);
        voltageSensors = hardwareMap.getAll(VoltageSensor.class);
        // A homing from an earlier OpMode lives on in the encoder count; drop it if impossible.
        turret.readSensors();
        storedZeroRejected = !turret.checkZero(Math.toRadians(STORED_ZERO_MARGIN_DEG));
    }

    /**
     * Call every INIT loop. The only thing that can move here is turret homing, and only after
     * {@link #homeTurret()} was called (the home button); homing finishes after START if START
     * comes first.
     */
    public void initLoop() {
        long nanos = System.nanoTime();
        double dt = lastNanos == 0L ? 0.0 : (nanos - lastNanos) * 1e-9;
        lastNanos = nanos;
        turret.readSensors();
        turret.initUpdate(dt);
    }

    /** Moves the turret and hood servos to their start positions (the INIT "servos to start" button). */
    public void servosToStart() {
        turret.moveToStartPosition();
        hood.setPosition(ShooterConstants.HOOD_START_POSITION, 0.0);
    }

    /** Starts turret homing (the home button): the turret moves to its zero, in INIT or after. */
    public void homeTurret() {
        storedZeroRejected = false;
        turret.startHoming();
    }

    /** One-line turret status for INIT telemetry. */
    public String turretStatus() {
        if (turret.isHoming()) return "HOMING...";
        if (!turret.hasReference()) {
            if (turret.resetFailed()) return "encoder reset did not read back as 0: press Y in INIT";
            return storedZeroRejected
                    ? "encoder count impossible (hub reset?): press Y to home"
                    : "NOT HOMED since power-on, will not shoot: press Y to home";
        }
        return String.format(java.util.Locale.US, "homed%s, encoder angle %.1f deg",
                turret.homeTimedOut() ? " (TIMED OUT: press Y again)" : "", Math.toDegrees(turret.angle()));
    }

    /** Runs one aiming cycle. Call once per loop, after {@code follower.update()}. */
    public void update() {
        long nanos = System.nanoTime();
        double now = nanos * 1e-9;
        double dt = lastNanos == 0L ? 0.0 : (nanos - lastNanos) * 1e-9;
        lastNanos = nanos;

        readVoltage(now);
        turret.readSensors();
        flywheel.readSensors();

        Pose pose = follower.pose();
        Velocity velocity = follower.velocity();
        motion.update(velocity.vx, velocity.vy, velocity.omega, dt);

        HiveCell cell = hive.upCell();
        solution = solver.solve(pose.x(), pose.y(), pose.heading(),
                motion.vx(), motion.vy(), motion.omega(),
                motion.ax(), motion.ay(),
                BioBuzzField.aimX(alliance, cell), BioBuzzField.aimY(alliance, cell),
                BioBuzzField.inwardAngle(cell), nextBall);

        double voltageScale = ShooterConstants.NOMINAL_VOLTAGE / voltage;
        turret.update(solution.turretAngle, solution.turretRate, dt);

        boolean override = !Double.isNaN(overrideRpm);
        double rpm = override ? overrideRpm : solution.flywheelRpm;
        flywheel.setTargetRpm(armed ? rpm : ShooterConstants.FLYWHEEL_IDLE_RPM);
        flywheel.update(voltageScale);
        hood.setPosition(override ? overrideHood : solution.hoodPosition, dt);

        boolean mechanismsReady = armed
                && flywheel.atSpeed()
                && hood.isSettled()
                && turret.isAimed(solution.allowedAimError);
        // In calibration (override) mode the table may not cover this distance yet, so only the
        // mechanisms are checked.
        boolean readyNow = mechanismsReady && (override || solution.valid) && !hive.isPaused(now);
        readyLoops = readyNow ? readyLoops + 1 : 0;
        ready = readyLoops >= Math.max(1, ShooterConstants.READY_DEBOUNCE_LOOPS);
    }

    private void readVoltage(double now) {
        if (now < nextVoltageRead) return;
        nextVoltageRead = now + ShooterConstants.VOLTAGE_READ_PERIOD_SEC;
        double lowest = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : voltageSensors) {
            double v = sensor.getVoltage();
            if (v > 0.0) lowest = Math.min(lowest, v);
        }
        // Clamp so a bad reading can never scale outputs wildly.
        if (lowest != Double.POSITIVE_INFINITY) voltage = Math.max(9.0, Math.min(14.0, lowest));
    }

    /** Stops the flywheel and disarms. */
    public void stop() {
        armed = false;
        flywheel.stop();
    }

    // ------------------------------------------------------------------ controls
    public void setArmed(boolean armed) {
        this.armed = armed;
    }

    public void toggleArmed() {
        armed = !armed;
    }

    /** Changes alliance and resets the HIVE to its match-start state. */
    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
        hive.reset(alliance);
    }

    public void reportHiveTip() {
        hive.reportTip(System.nanoTime() * 1e-9);
    }

    public void undoHiveTip() {
        hive.undoTip();
    }

    public void setUpCell(HiveCell cell) {
        hive.setUpCell(cell);
    }

    /**
     * Ball that will be fired next, which selects its shot table. Set it from a color sensor at
     * the feeder, or from the operator.
     */
    public void setNextBall(BallType ball) {
        nextBall = ball;
    }

    public BallType nextBall() {
        return nextBall;
    }

    /**
     * True when firing is blocked by where or how the robot is driving, which the driver can fix:
     * wrong side or angle to the CELL, too fast, turning or accelerating, or the CELL sitting in
     * the turret's dead zone (rotate the robot).
     */
    public boolean isBlockedByDriving() {
        int driving = ShotSolution.BAD_APPROACH | ShotSolution.TOO_FAST
                | ShotSolution.TURNING_TOO_FAST | ShotSolution.ACCELERATING;
        return solution != null && ((solution.problems & driving) != 0 || !turret.isReachable());
    }

    /** Calibration only: fixed flywheel rpm and hood position instead of the shot table. */
    public void setCalibrationOverride(double rpm, double hoodPosition) {
        overrideRpm = rpm;
        overrideHood = hoodPosition;
    }

    public void clearCalibrationOverride() {
        overrideRpm = Double.NaN;
        overrideHood = Double.NaN;
    }
    /** Clears the velocity/acceleration filters, e.g. after follower.setPose(). */
    public void resetMotion() {
        motion.reset();
    }

    // ------------------------------------------------------------------ state
    public ShotSolver solver() {
        return solver;
    }

    public ShotSolution solution() {
        return solution;
    }

    public Alliance alliance() {
        return alliance;
    }

    public HiveCell upCell() {
        return hive.upCell();
    }

    public boolean isArmed() {
        return armed;
    }

    public boolean isReady() {
        return ready;
    }

    public double turretAngle() {
        return turret.angle();
    }

    public Flywheel flywheel() {
        return flywheel;
    }

    public Hood hood() {
        return hood;
    }

    public Turret turret() {
        return turret;
    }

    public void addTelemetry(Telemetry telemetry) {
        ShotSolution s = solution;
        telemetry.addData("Alliance / up CELL / ball", "%s / %s (tips %d) / %s",
                alliance, hive.upCell(), hive.tips(), nextBall);
        telemetry.addData("Armed / READY", "%b / %b", armed, ready);
        if (s == null) return;
        telemetry.addData("Problems", s.describeProblems());
        telemetry.addData("Distance real / virtual", "%.1f / %.1f in", s.realDistance, s.virtualDistance);
        telemetry.addData("TOF / lead / iterations", "%.3f s / %.3f s / %d", s.timeOfFlight, s.leadTime, s.iterations);
        telemetry.addData("Approach angle", "%.1f deg", Math.toDegrees(s.approachAngle));
        if (turret.isHoming() || !turret.hasReference()) {
            telemetry.addData("Turret", turretStatus());
        } else {
            telemetry.addData("Turret target / set / encoder", "%.1f / %.1f / %.1f deg%s%s",
                    Math.toDegrees(s.turretAngle), Math.toDegrees(turret.setpoint()), Math.toDegrees(turret.angle()),
                    turret.isReachable() ? "" : "  (DEAD ZONE: rotate the robot)",
                    turret.homeTimedOut() ? "  (homing timed out: home again in INIT)" : "");
        }
        telemetry.addData("Turret servo / correction / rate", "%.3f / %.2f deg / %.0f deg/s",
                turret.servoPosition(), Math.toDegrees(turret.correction()), Math.toDegrees(turret.rate()));
        telemetry.addData("Turret error / allowed", "%.2f / %.2f deg (miss allowance %.1f in)",
                Math.toDegrees(turret.error()), Math.toDegrees(s.allowedAimError), s.lateralAllowance);
        telemetry.addData("Turret ff rate", "%.1f deg/s", Math.toDegrees(s.turretRate));
        telemetry.addData("Flywheel target / actual", "%.0f / %.0f rpm (power %.2f)",
                flywheel.targetRpm(), flywheel.measuredRpm(), flywheel.power());
        telemetry.addData("Hood command / settled", "%.3f / %b", hood.commandedPosition(), hood.isSettled());
        telemetry.addData("Velocity (filtered)", "%.1f, %.1f in/s  %.2f rad/s", motion.vx(), motion.vy(), motion.omega());
        telemetry.addData("Accel (filtered)", "%.1f, %.1f in/s^2", motion.ax(), motion.ay());
        telemetry.addData("Battery", "%.2f V", voltage);
    }
}
