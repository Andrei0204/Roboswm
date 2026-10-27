package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.config.ShooterConstants;

/**
 * Control logic for the servo + external-encoder turret. Pure math with no hardware, so it can
 * be tested off the robot. {@link Turret} feeds it encoder readings and writes the servo
 * position it returns.
 *
 * <p>Angles are robot-relative radians, counter-clockwise positive, 0 = shooting straight
 * forward.
 *
 * <p>Each update:
 * <ul>
 *   <li>picks the reachable equivalent of the target inside the soft limits and the servo's
 *       reach (a servo turret covers under 360 deg, so the gap behind it is unreachable);</li>
 *   <li>commands setpoint + rate * TURRET_SERVO_LEAD_SEC + trim. The lead cancels the servo's
 *       tracking lag. The trim comes from the encoder error near the target: a slow integral
 *       that removes static error (backlash, load sag, servo-map error) plus a proportional term
 *       with gain below 1 that cuts tracking error;</li>
 *   <li>counts as aimed only when the ENCODER angle is within tolerance and the encoder rate
 *       matches the commanded rate.</li>
 * </ul>
 *
 * <p>The zero lives in the encoder count itself: count 0 = turret at 0 deg. Homing runs only when
 * the drive team asks for it during INIT ({@link #startHoming} + {@link #initUpdate}). The servo
 * goes to its zero position, and once the encoder shows the turret is still, {@link Turret}
 * resets the encoder count to 0. The hub keeps counting between OpModes (nothing else resets
 * the port), so a homing in AUTO's INIT is still valid in TeleOp with nothing stored.
 *
 * <p>The only state kept across OpModes is {@link #homedSinceAppStart}: after a power cycle the
 * count restarts at 0 wherever the turret happens to be, so until homing has run in this app
 * session the turret still points open-loop from the servo map but never counts as aimed, and
 * nothing is fired.
 */
public final class TurretController {
    /** Readings to wait after a reset before trusting the count (the first may be bulk-cached). */
    private static final int FRESH_READS_AFTER_RESET = 2;
    /** After the reset the held-still turret must read within this of 0 deg. */
    private static final double RESET_CHECK_DEG = 1.0;

    /**
     * True once homing reset the encoder in this app session. Static, so it survives from AUTO to
     * TeleOp; it clears when the Robot Controller app restarts (which a power cycle does).
     */
    private static boolean homedSinceAppStart;

    private int ticks;
    private double angle;
    private double rate;

    private boolean homing;
    private boolean homeTimedOut;
    private double homeElapsed;
    private double stillFor;
    private boolean resetRequested;
    private int freshReadsNeeded;
    private boolean resetFailed;

    /** Desired angle without lead or correction (rad). */
    private double setpoint;
    private boolean hasSetpoint;
    private boolean reachable = true;
    private double commandedRate;
    /** Learned trim added to the servo command (rad). */
    private double correction;

    /** Checks the turret constants; throws with a readable message if they cannot work. */
    public static void validateConstants() {
        if (Math.abs(ShooterConstants.TURRET_DEG_PER_SERVO_UNIT) < 1.0) {
            throw new IllegalStateException("TURRET_DEG_PER_SERVO_UNIT is not set");
        }
        if (!(TurretMath.minAngle() < TurretMath.maxAngle())) {
            throw new IllegalStateException("TURRET_MIN_DEG..TURRET_MAX_DEG does not overlap the servo's travel");
        }
        if (!(ShooterConstants.TURRET_ENCODER_TICKS_PER_REV * ShooterConstants.TURRET_ENCODER_REVS_PER_TURRET_REV > 0.0)) {
            throw new IllegalStateException("TURRET_ENCODER_TICKS_PER_REV and TURRET_ENCODER_REVS_PER_TURRET_REV must be positive");
        }
        if (Math.abs(ShooterConstants.TURRET_ENCODER_SIGN) != 1) {
            throw new IllegalStateException("TURRET_ENCODER_SIGN must be 1 or -1");
        }
        if (!(ShooterConstants.TURRET_ENCODER_KP >= 0.0 && ShooterConstants.TURRET_ENCODER_KP < 1.0)) {
            throw new IllegalStateException("TURRET_ENCODER_KP must be in [0, 1) to stay stable");
        }
    }

    /**
     * New encoder reading (call once per loop, before {@link #update}).
     *
     * @param encoderTicks       raw encoder count
     * @param encoderTicksPerSec raw encoder velocity
     */
    public void setMeasurement(int encoderTicks, double encoderTicksPerSec) {
        double k = TurretMath.radiansPerTick();
        ticks = encoderTicks;
        angle = ticks * k;
        rate = encoderTicksPerSec * k;
        if (freshReadsNeeded > 0 && --freshReadsNeeded == 0) {
            // First trusted reading after the reset: the held-still turret must now read ~0.
            homing = false;
            resetFailed = Math.abs(angle) > Math.toRadians(RESET_CHECK_DEG);
            homedSinceAppStart = !resetFailed;
        }
    }

    /**
     * At OpMode start (after {@link #setMeasurement}): if the count puts the turret more than
     * {@code marginRad} outside its travel, the zero cannot be right (e.g. the hub lost its
     * count), so it is dropped and the turret must be homed again.
     *
     * @return false if a zero existed and was dropped
     */
    public boolean checkZero(double marginRad) {
        if (!homedSinceAppStart) return true;
        if (angle < TurretMath.minAngle() - marginRad || angle > TurretMath.maxAngle() + marginRad) {
            homedSinceAppStart = false;
            return false;
        }
        return true;
    }

    /** Starts homing: the servo goes to its zero and the encoder count is reset there. */
    public void startHoming() {
        homing = true;
        homeTimedOut = false;
        homeElapsed = 0.0;
        stillFor = 0.0;
        resetRequested = false;
        freshReadsNeeded = 0;
        resetFailed = false;
        homedSinceAppStart = false;
        correction = 0.0;
        hasSetpoint = false;
    }

    /**
     * {@link Turret} calls this after every update: true means "reset the encoder count to 0
     * now". The next readings are then checked before the zero is trusted.
     */
    public boolean takeResetRequest() {
        if (!resetRequested) return false;
        resetRequested = false;
        freshReadsNeeded = FRESH_READS_AFTER_RESET;
        return true;
    }

    /**
     * @param targetAngle desired robot-relative angle (rad, any wrap)
     * @param targetRate  desired angular rate relative to the robot (rad/s)
     * @param dt          seconds since the previous call
     * @return the servo position to write, in [0, 1]
     */
    public double update(double targetAngle, double targetRate, double dt) {
        dt = Math.max(0.0, dt);
        if (homing) return home(dt);

        double min = TurretMath.minAngle();
        double max = TurretMath.maxAngle();
        double reference = hasSetpoint ? setpoint : (hasReference() ? angle : 0.0);
        setpoint = TurretMath.chooseSetpoint(targetAngle, reference, min, max);
        hasSetpoint = true;
        reachable = TurretMath.reaches(setpoint, targetAngle);
        commandedRate = reachable ? targetRate : 0.0;

        // Encoder corrections apply only with a zero and near the target, never during a big
        // swing. Without a zero the servo map alone points the turret.
        double error = setpoint - angle;
        double limit = Math.toRadians(ShooterConstants.TURRET_MAX_CORRECTION_DEG);
        boolean near = hasReference() && reachable
                && Math.abs(error) < Math.toRadians(ShooterConstants.TURRET_CORRECTION_WINDOW_DEG);
        // Integral: learned only while aiming nearly still, so it never absorbs the tracking lag
        // that the lead already handles.
        double gain = ShooterConstants.TURRET_CORRECTION_GAIN;
        if (gain > 0.0 && near
                && Math.abs(commandedRate) < Math.toRadians(ShooterConstants.TURRET_CORRECTION_MAX_RATE_DEG_PER_SEC)) {
            correction = Math.max(-limit, Math.min(limit, correction + gain * error * dt));
        }
        // Proportional: also while tracking, outside the backlash deadband.
        double proportional = 0.0;
        if (near && Math.abs(error) > Math.toRadians(ShooterConstants.TURRET_P_DEADBAND_DEG)) {
            proportional = ShooterConstants.TURRET_ENCODER_KP * error;
        }
        double trim = Math.max(-limit, Math.min(limit, correction + proportional));

        // The intended angle (with lead) stays inside the soft limits; the trim only compensates
        // the servo's own error on the way there.
        double intended = Math.max(min, Math.min(max, setpoint + commandedRate * ShooterConstants.TURRET_SERVO_LEAD_SEC));
        return clampPosition(TurretMath.positionForAngle(intended + trim));
    }

    /**
     * INIT-loop step: runs homing if it was started, otherwise leaves the turret alone.
     *
     * @return the servo position to write, or NaN to write nothing (no motion in INIT)
     */
    public double initUpdate(double dt) {
        return homing ? home(Math.max(0.0, dt)) : Double.NaN;
    }

    private double home(double dt) {
        homeElapsed += dt;
        stillFor = Math.abs(rate) <= Math.toRadians(ShooterConstants.TURRET_HOME_STILL_DEG_PER_SEC) ? stillFor + dt : 0.0;
        boolean settled = homeElapsed >= ShooterConstants.TURRET_HOME_MIN_SEC
                && stillFor >= ShooterConstants.TURRET_HOME_STILL_SEC;
        boolean timedOut = homeElapsed >= ShooterConstants.TURRET_HOME_TIMEOUT_SEC;
        // Once still (or out of time), ask for the reset; homing ends when fresh readings confirm it.
        if ((settled || timedOut) && !resetRequested && freshReadsNeeded == 0) {
            resetRequested = true;
            homeTimedOut = !settled;
        }
        return clampPosition(TurretMath.positionForAngle(0.0));
    }

    private static double clampPosition(double position) {
        return Math.max(0.0, Math.min(1.0, position));
    }

    /**
     * True when the target is reachable and the encoder shows the turret within
     * {@code tolerance} of it, moving at the commanded rate (not swinging through).
     */
    public boolean isAimed(double tolerance) {
        return hasReference() && hasSetpoint && reachable
                && Math.abs(setpoint - angle) <= tolerance
                && Math.abs(rate - commandedRate) <= Math.toRadians(ShooterConstants.TURRET_RATE_TOLERANCE_DEG_PER_SEC);
    }

    public boolean isReachable() {
        return reachable;
    }

    public boolean isHoming() {
        return homing;
    }

    /** True when the encoder count is a trusted turret angle (homed in this app session). */
    public boolean hasReference() {
        return homedSinceAppStart && !homing;
    }

    /** True if the last homing's reset did not read back as 0 (the zero was not set). */
    public boolean resetFailed() {
        return resetFailed;
    }

    /** True if the last homing ended on its timeout instead of seeing the turret still. */
    public boolean homeTimedOut() {
        return homeTimedOut;
    }

    public double setpoint() {
        return setpoint;
    }

    /** Encoder angle (rad); NaN until homed. */
    public double angle() {
        return hasReference() ? angle : Double.NaN;
    }

    /** Encoder rate (rad/s). */
    public double rate() {
        return rate;
    }

    public double error() {
        return hasReference() && hasSetpoint ? setpoint - angle : 0.0;
    }

    /** Learned servo trim (rad). */
    public double correction() {
        return correction;
    }
}
