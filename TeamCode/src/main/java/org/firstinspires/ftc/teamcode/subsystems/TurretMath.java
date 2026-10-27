package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.aiming.ShotSolver;
import org.firstinspires.ftc.teamcode.config.ShooterConstants;

/** Turret angle wrapping, servo mapping and encoder scaling. Pure math, testable off the robot. */
public final class TurretMath {
    private static final double TWO_PI = 2.0 * Math.PI;

    private TurretMath() {}

    /**
     * Picks the equivalent of {@code target} (target + 2*pi*k) that lies inside [min, max] and is
     * closest to {@code reference} (the previous setpoint, so the turret never jumps between
     * equivalent angles without a reason). If no equivalent fits, which can only happen when the
     * travel is under 360 deg, it returns the limit closest to an equivalent.
     *
     * @return the setpoint in the turret's continuous angle (rad)
     */
    public static double chooseSetpoint(double target, double reference, double min, double max) {
        double base = target + TWO_PI * Math.round((reference - target) / TWO_PI);
        double best = Double.NaN;
        double bestViolation = Double.POSITIVE_INFINITY;
        double bestDistance = Double.POSITIVE_INFINITY;
        for (int k = -2; k <= 2; k++) {
            double candidate = base + TWO_PI * k;
            double violation = candidate < min ? min - candidate : (candidate > max ? candidate - max : 0.0);
            double clamped = Math.max(min, Math.min(max, candidate));
            double distance = Math.abs(clamped - reference);
            if (violation < bestViolation - 1e-12
                    || (Math.abs(violation - bestViolation) <= 1e-12 && distance < bestDistance)) {
                bestViolation = violation;
                bestDistance = distance;
                best = clamped;
            }
        }
        return best;
    }

    /** True when {@code setpoint} is an exact equivalent of {@code target} (the target is reachable). */
    public static boolean reaches(double setpoint, double target) {
        return Math.abs(ShotSolver.normalizeSigned(setpoint - target)) < 1e-6;
    }

    /** Servo position for a turret angle (rad). */
    public static double positionForAngle(double angle) {
        return ShooterConstants.TURRET_SERVO_POSITION_AT_ZERO
                + Math.toDegrees(angle) / ShooterConstants.TURRET_DEG_PER_SERVO_UNIT;
    }

    /** Turret angle (rad) for a servo position. */
    public static double angleForPosition(double position) {
        return Math.toRadians((position - ShooterConstants.TURRET_SERVO_POSITION_AT_ZERO)
                * ShooterConstants.TURRET_DEG_PER_SERVO_UNIT);
    }

    /** Lowest reachable angle (rad): the soft limit or the servo's end of travel. */
    public static double minAngle() {
        double servoMin = Math.min(angleForPosition(0.0), angleForPosition(1.0));
        return Math.max(Math.toRadians(ShooterConstants.TURRET_MIN_DEG), servoMin);
    }

    /** Highest reachable angle (rad): the soft limit or the servo's end of travel. */
    public static double maxAngle() {
        double servoMax = Math.max(angleForPosition(0.0), angleForPosition(1.0));
        return Math.min(Math.toRadians(ShooterConstants.TURRET_MAX_DEG), servoMax);
    }

    /** Turret radians per encoder tick, including the encoder sign. */
    public static double radiansPerTick() {
        return ShooterConstants.TURRET_ENCODER_SIGN * 2.0 * Math.PI
                / (ShooterConstants.TURRET_ENCODER_TICKS_PER_REV * ShooterConstants.TURRET_ENCODER_REVS_PER_TURRET_REV);
    }
}
