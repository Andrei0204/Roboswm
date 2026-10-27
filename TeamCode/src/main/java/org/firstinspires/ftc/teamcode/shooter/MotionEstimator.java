package org.firstinspires.ftc.teamcode.shooter;

/**
 * Smooths Pedro's field velocity and derives field acceleration. Pedro v3 has no acceleration
 * API, and it passes Pinpoint/OTOS velocity through unfiltered (two- and three-wheel localizers
 * use a raw one-loop difference), so acceleration needs filtering. Every filter adds lag: 0.1 s
 * of lag at 2 m/s^2 is about 11 cm of miss. Keep the time constants short.
 */
public final class MotionEstimator {
    private boolean initialized;
    private double vx;
    private double vy;
    private double omega;
    private double ax;
    private double ay;

    public void reset() {
        initialized = false;
        vx = vy = omega = ax = ay = 0.0;
    }

    /**
     * @param rawVx    field velocity x from Pedro (in/s)
     * @param rawVy    field velocity y from Pedro (in/s)
     * @param rawOmega angular velocity from Pedro (rad/s)
     * @param dt       seconds since the previous call
     */
    public void update(double rawVx, double rawVy, double rawOmega, double dt) {
        if (!initialized || !(dt > 0.0) || dt > 0.5) {
            // First sample, or a long stall (e.g. a blocking call): restart without a fake spike.
            vx = rawVx;
            vy = rawVy;
            omega = rawOmega;
            ax = 0.0;
            ay = 0.0;
            initialized = true;
            return;
        }
        double kv = smoothing(dt, ShooterConstants.VELOCITY_FILTER_SEC);
        double newVx = vx + kv * (rawVx - vx);
        double newVy = vy + kv * (rawVy - vy);
        omega += kv * (rawOmega - omega);

        double rawAx = (newVx - vx) / dt;
        double rawAy = (newVy - vy) / dt;
        double ka = smoothing(dt, ShooterConstants.ACCEL_FILTER_SEC);
        ax += ka * (rawAx - ax);
        ay += ka * (rawAy - ay);
        double max = ShooterConstants.MAX_ACCEL_ESTIMATE;
        double mag = Math.hypot(ax, ay);
        if (mag > max && mag > 0.0) {
            ax *= max / mag;
            ay *= max / mag;
        }
        vx = newVx;
        vy = newVy;
    }

    /** First-order low-pass gain for a time constant (0 = no filtering). */
    private static double smoothing(double dt, double timeConstant) {
        return timeConstant <= 0.0 ? 1.0 : dt / (timeConstant + dt);
    }

    public double vx() {
        return vx;
    }

    public double vy() {
        return vy;
    }

    public double omega() {
        return omega;
    }

    public double ax() {
        return ax;
    }

    public double ay() {
        return ay;
    }
}
