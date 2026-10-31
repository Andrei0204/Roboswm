package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.aiming.ShotSolver;
import org.firstinspires.ftc.teamcode.config.ShooterConstants;

/** Turret angle wrapping, soft limits and encoder scaling. Pure math, testable off the robot. */
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

    /** Lowest reachable angle (rad): the soft limit. */
    public static double minAngle() {
        return Math.toRadians(ShooterConstants.TURRET_MIN_DEG);
    }

    /** Highest reachable angle (rad): the soft limit. */
    public static double maxAngle() {
        return Math.toRadians(ShooterConstants.TURRET_MAX_DEG);
    }

    /** Turret radians per encoder tick, including the encoder sign. */
    public static double radiansPerTick() {
        return ShooterConstants.TURRET_ENCODER_SIGN * 2.0 * Math.PI
                / (ShooterConstants.TURRET_ENCODER_TICKS_PER_REV * ShooterConstants.TURRET_ENCODER_REVS_PER_TURRET_REV);
    }
}
