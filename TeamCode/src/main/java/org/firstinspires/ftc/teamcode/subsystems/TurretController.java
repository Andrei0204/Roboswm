package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.config.ShooterConstants;

/**
 * Control logic for the continuous-rotation-servo + external-encoder turret. Pure math with no
 * hardware, so it can be tested off the robot. {@link Turret} feeds it encoder readings and
 * writes the servo power it returns.
 *
 * <p>Angles are robot-relative radians, counter-clockwise positive, 0 = shooting straight
 * forward.
 *
 * <p>Each update:
 * <ul>
 *   <li>picks the reachable equivalent of the target inside the soft limits (a turret limited by
 *       its cables covers under 360 deg, so the gap behind it is unreachable);</li>
 *   <li>closes the loop on the encoder: PID with a NEAR and a FAR gain set: power = KP * error + KI * integral + KD * (target rate - rate). The servo has no position of its own, so without a homed encoder it
 *       is not powered at all;</li>
 *   <li>cuts power that would push past a soft limit;</li>
 *   <li>counts as a    imed only when the ENCODER angle is within tolerance and the encoder rate
 *       matches the commanded rate.</li>
 * </ul>
 *
 * <p>The zero lives in the encoder count itself: count 0 = turret at 0 deg. The hub resets the
 * count whenever an OpMode ends, so every OpMode simply assumes the turret starts at 0 deg (point
 * it straight forward before INIT). If it was not, the drive team presses the reset button during
 * INIT ({@link #startHoming} + {@link #initUpdate}): once the encoder shows the turret is still,
 * {@link Turret} resets the encoder count to 0 there.
 */
public final class TurretController {
    /** Readings to wait after a reset before trusting the count (the first may be bulk-cached). */
    private static final int FRESH_READS_AFTER_RESET = 2;
    /** After the reset the held-still turret must read within this of 0 deg. */
    private static final double RESET_CHECK_DEG = 1.0;

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

    /** Desired angle inside the soft limits (rad). */
    private double setpoint;
    private boolean hasSetpoint;
    private boolean reachable = true;
    private double commandedRate;
    private double power;
    private double integral;

    /** Checks the turret constants; throws with a readable message if they cannot work. */
    public static void validateConstants() {
        if (!(TurretMath.minAngle() < TurretMath.maxAngle())) {
            throw new IllegalStateException("TURRET_MIN_DEG must be below TURRET_MAX_DEG");
        }
        if (!(ShooterConstants.TURRET_ENCODER_TICKS_PER_REV * ShooterConstants.TURRET_ENCODER_REVS_PER_TURRET_REV > 0.0)) {
            throw new IllegalStateException("TURRET_ENCODER_TICKS_PER_REV and TURRET_ENCODER_REVS_PER_TURRET_REV must be positive");
        }
        if (Math.abs(ShooterConstants.TURRET_ENCODER_SIGN) != 1) {
            throw new IllegalStateException("TURRET_ENCODER_SIGN must be 1 or -1");
        }
        if (!(ShooterConstants.TURRET_MAX_POWER > 0.0 && ShooterConstants.TURRET_MAX_POWER <= 1.0)) {
            throw new IllegalStateException("TURRET_MAX_POWER must be in (0, 1]");
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
        }
    }

    /** Starts the encoder reset: once the turret is still, the count is reset to 0 there. */
    public void startHoming() {
        homing = true;
        homeTimedOut = false;
        homeElapsed = 0.0;
        stillFor = 0.0;
        resetRequested = false;
        freshReadsNeeded = 0;
        resetFailed = false;
        hasSetpoint = false;
        integral = 0.0;
        power = 0.0;
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
     * @return the servo power to write, in [-1, 1] (positive turns the turret counter-clockwise)
     */
    public double update(double targetAngle, double targetRate, double dt) {
        dt = Math.max(0.0, dt);
        if (homing) return home(dt);
        if (!hasReference()) {
            hasSetpoint = false;
                reachable = true;
            commandedRate = 0.0;
            integral = 0.0;
            return power = 0.0;
        }

        double min = TurretMath.minAngle();
        double max = TurretMath.maxAngle();
        setpoint = TurretMath.chooseSetpoint(targetAngle, hasSetpoint ? setpoint : angle, min, max);
        hasSetpoint = true;
        reachable = TurretMath.reaches(setpoint, targetAngle);
        commandedRate = reachable ? targetRate : 0.0;

        // PID on the angle error with two gain sets: aggressive NEAR the target (accuracy),
        // gentler FAR from it. Backlash itself is removed by the opposing servo bias in Turret.
        double error = setpoint - angle;
        boolean near = Math.abs(error) < Math.toRadians(ShooterConstants.TURRET_NEAR_DEG);
        double kp = near ? ShooterConstants.TURRET_NEAR_KP : ShooterConstants.TURRET_FAR_KP;
        double ki = near ? ShooterConstants.TURRET_NEAR_KI : ShooterConstants.TURRET_FAR_KI;
        double kd = near ? ShooterConstants.TURRET_NEAR_KD : ShooterConstants.TURRET_FAR_KD;
        double iMax = Math.max(0.0, ShooterConstants.TURRET_KI_MAX_POWER);
        if (near && ki != 0.0) {
            integral += error * dt;
            integral = Math.max(-iMax / Math.abs(ki), Math.min(iMax / Math.abs(ki), integral));
        } else {
            integral = 0.0;
        }
        // D acts on the derivative of the error: (target rate - real rate).
        double p = kp * error + ki * integral + kd * (commandedRate - rate);
        // Never push further out through a soft limit.
        if ((angle >= max && p > 0.0) || (angle <= min && p < 0.0)) p = 0.0;
        double maxPower = ShooterConstants.TURRET_MAX_POWER;
        return power = Math.max(-maxPower, Math.min(maxPower, p));
    }

    /**
     * INIT-loop step: runs homing if it was started, otherwise leaves the turret alone.
     *
     * @return the servo power to write, or NaN to write nothing (no motion in INIT)
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
        return power = 0.0;
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

    /** True when the encoder count is a usable turret angle (always, except while a reset runs). */
    public boolean hasReference() {
        return !homing;
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

    /** Last servo power computed, in [-1, 1]. */
    public double power() {
        return power;
    }
}
