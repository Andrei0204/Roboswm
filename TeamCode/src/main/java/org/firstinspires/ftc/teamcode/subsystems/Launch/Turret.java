package org.firstinspires.ftc.teamcode.subsystems.Launch;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static com.pedropathing.ivy.commands.Commands.infinite;

/**
 * Turret position control built on a trapezoidal motion profile that is
 * tracked with a feedforward + PD loop, plus a separate "hold" controller
 * once the turret is within tolerance of its target.
 *
 * Why not plain PID?
 *  - A step-input PID either overshoots (if it's fast/aggressive enough to
 *    be useful) or is slow (if damped enough to avoid overshoot).
 *  - A motion profile ramps velocity up, cruises, then ramps back down to
 *    exactly zero at the target. The controller is always chasing a moving
 *    setpoint that is already decelerating into the target, which is what
 *    removes overshoot while still letting you run near max speed for most
 *    of the move.
 *  - Once you're basically there, position holding is a different problem
 *    (small corrections around a fixed point) than getting there fast, so
 *    it gets its own gains instead of sharing with the move controller.
 */
@Configurable
public class Turret {
    private static final double TICKS_PER_REVOLUTION = 103.8 * 4.75;
    private static final double TICKS_PER_DEGREE = TICKS_PER_REVOLUTION / 360.0;

    // ---------------- Motion profile constraints ----------------
    // Tune these first, to what the motor/mechanism can actually do.
    public static double MAX_VELOCITY_DEG_PER_SEC = 2000;
    public static double MAX_ACCEL_DEG_PER_SEC2 = 7000;

    // ---------------- Move (profile-tracking) gains ----------------
    public static double kV = 0.0007; // power per deg/s of profile velocity; start near 1/MAX_VELOCITY_DEG_PER_SEC
    public static double kA = 0.0;         // power per deg/s^2; leave at 0 unless you've measured real inertia
    public static double kS = 0.06;        // static-friction feedforward: min power to break stiction
    public static double kP = 0.014;       // corrects error against the profile (not against the final target)
    public static double kD = 0.003;

    // ---------------- Hold gains (used once settled at the target) ----------------
    public static double holdKP = 0.09;
    public static double holdKD = 0.0011;

    // ---------------- Tolerances ----------------
    public static double POSITION_TOLERANCE_DEG = 6;
    public static double REACQUIRE_TOLERANCE_DEG = 6;

    // ---------------- Optional target smoothing ----------------
    // If the incoming target itself is noisy/jittery (e.g. raw vision angle),
    // you can slew-limit how fast the target the controller chases is allowed
    // to move. Left huge by default (pass-through) -- the velocity law above
    // already prevents any physical discontinuity on its own.
    public static double MAX_TARGET_SLEW_DEG_PER_SEC = 100000;

    public boolean forceOff = false;

    private final CRServo turretMotor;

    private double rawTargetDegrees = 0;  // last value passed to setTargetDegrees()
    private double targetDegrees = 0;     // slew-limited target actually chased
    private Mode mode = Mode.OFF;

    private double commandedVelocity = 0; // deg/s, persists loop-to-loop
    private double lastPositionError = 0;
    private double lastTime = 0;
    private boolean settled = false;

    public Turret(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(CRServo.class, "TUR");
    }

    public void start() {

        lastTime = 0; // lazily re-initialized on the first control loop, see runPositionControl()
    }

    public double getRawAngleDegrees() {
        return turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    public void off() {
        mode = Mode.OFF;
        commandedVelocity = 0;
        settled = false;
    }

    public double getTargetDegrees() {
        return rawTargetDegrees;
    }

    /** True once the turret is within POSITION_TOLERANCE_DEG and holding. */
    public boolean atTarget() {
        return settled;
    }

    public void setTargetDegrees(double targetDegrees) {
        if (Double.isNaN(targetDegrees)) return;
        this.rawTargetDegrees = targetDegrees;
        mode = Mode.POSITION;
        // Nothing else to do here -- no profile to regenerate. update() picks up
        // the new target on its very next iteration, which is what makes rapid
        // re-targeting safe.
    }

    public Mode getMode() {
        return mode;
    }

    public void setMode(Mode mode) {
        this.mode = mode;
    }

    /**
     * No longer needed: gains are read directly from the static fields every
     * loop, so dashboard tuning applies immediately. Kept as a no-op so
     * existing call sites elsewhere don't break the build.
     */
    @Deprecated
    public void updatepid() {
    }

    public Command periodic() {
        return infinite(this::update);
    }

    /** Also callable directly (e.g. from a teleop loop or a test OpMode) without the command scheduler. */
    public void tester() {
        update();
    }

    private void update() {
        if (forceOff) {
            turretMotor.setPower(0);
            return;
        }

        switch (mode) {
            case POSITION:
                runPositionControl();
                break;
            case OFF:
            default:
                turretMotor.setPower(0);
                commandedVelocity = 0;
                break;
        }
    }

    private void runPositionControl() {
        double t = now();
        if (lastTime == 0) {
            // First cycle since start()/a mode switch -- just timestamp it, don't
            // compute a dt against garbage, since System.nanoTime()'s origin is arbitrary.
            lastTime = t;
            return;
        }
        double dt = clamp(t - lastTime, 1e-3, 0.1); // clamp guards against a lag spike blowing up the slew limit

        double currentAngle = getRawAngleDegrees();

        // Optional slew-limit on the target itself (disabled by default, see field comment above).
        double targetStep = clamp(rawTargetDegrees - targetDegrees,
                -MAX_TARGET_SLEW_DEG_PER_SEC * dt, MAX_TARGET_SLEW_DEG_PER_SEC * dt);
        targetDegrees += targetStep;

        double positionError = targetDegrees - currentAngle;

        if (!settled && Math.abs(positionError) <= POSITION_TOLERANCE_DEG) {
            settled = true;
        } else if (settled && Math.abs(positionError) > REACQUIRE_TOLERANCE_DEG) {
            settled = false;
        }

        double power;
        if (settled) {
            double dError = (positionError - lastPositionError) / dt;
            power = holdKP * positionError + holdKD * dError;
            commandedVelocity = 0;
        } else {
            double distanceRemaining = Math.abs(positionError);
            double direction = Math.signum(positionError);

            // Fastest we could be going right now and still stop exactly at the target.
            double stoppingLimitedVel = Math.sqrt(2 * MAX_ACCEL_DEG_PER_SEC2 * distanceRemaining);
            double desiredVelocity = direction * Math.min(MAX_VELOCITY_DEG_PER_SEC, stoppingLimitedVel);

            // Slew-limit against the PREVIOUS COMMAND (not a stored plan), so it's always
            // physically continuous no matter how much the target jumps between loops.
            double maxDeltaV = MAX_ACCEL_DEG_PER_SEC2 * dt;
            commandedVelocity = clamp(desiredVelocity, commandedVelocity - maxDeltaV, commandedVelocity + maxDeltaV);

            double dError = (positionError - lastPositionError) / dt;
            double ff = kV * commandedVelocity + Math.signum(commandedVelocity) * kS;
            double fb = kP * positionError + kD * dError;
            power = ff + fb;
        }

        if(clamp(power, -1.0, 1.0)==clamp(power, -1.0, 1.0))
            turretMotor.setPower(clamp(power, -1.0, 1.0));
        lastPositionError = positionError;
        lastTime = t;
    }

    private static double now() {
        return System.nanoTime() / 1e9;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    public enum Mode {
        POSITION,
        OFF,
    }
}

