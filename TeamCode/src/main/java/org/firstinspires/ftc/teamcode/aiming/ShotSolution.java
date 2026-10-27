package org.firstinspires.ftc.teamcode.aiming;

/**
 * Output of {@link ShotSolver}. One instance is reused every loop, so copy any field you need
 * to keep.
 */
public final class ShotSolution {
    // Reasons a shot is not allowed (bit flags in {@link #problems}).
    public static final int TOO_CLOSE = 1;
    public static final int TOO_FAR = 1 << 1;
    public static final int NOT_CONVERGED = 1 << 2;
    /** The robot is on the wrong side of the HIVE, or too far to the side of the CELL opening. */
    public static final int BAD_APPROACH = 1 << 3;
    public static final int TOO_FAST = 1 << 4;
    public static final int TURNING_TOO_FAST = 1 << 5;
    public static final int ACCELERATING = 1 << 6;

    /** True when no problem flag is set. Mechanism readiness is checked separately. */
    public boolean valid;
    public int problems;
    /** Ball the solution was computed for. */
    public BallType ballType = BallType.POLLEN;

    /** Predicted robot heading at the lookahead time (rad). */
    public double predictedHeading;
    /** Turret pivot position on the field at the lookahead time (in). */
    public double turretX;
    public double turretY;
    /** Field velocity of the turret pivot (in/s), including the omega x offset term. */
    public double turretVx;
    public double turretVy;

    /** Horizontal distance from the turret pivot to the real aim point (in). */
    public double realDistance;
    /** Virtual goal (aim point shifted against the shooter velocity) and its distance (in). */
    public double virtualX;
    public double virtualY;
    public double virtualDistance;
    /** Time of flight looked up at the virtual distance (s), and the lead time used for the shift (s). */
    public double timeOfFlight;
    public double leadTime;
    public int iterations;

    /** Field angle the shooter must point along (rad, CCW from +x), including aim trim. */
    public double aimFieldAngle;
    /** Turret angle relative to the robot, normalized to (-pi, pi] (wrapping happens in Turret). */
    public double turretAngle;
    /** Turret angular-rate feedforward relative to the robot (rad/s). */
    public double turretRate;
    /** Sideways miss the ball can take and still clear the opening with AIM_MARGIN_IN to spare (in). */
    public double lateralAllowance;
    /** Largest turret pointing error that keeps the miss within {@link #lateralAllowance} (rad). */
    public double allowedAimError;

    public double flywheelRpm;
    public double hoodPosition;

    /** Angle between the ball's horizontal path and the CELL's inward direction (rad). */
    public double approachAngle;

    public String describeProblems() {
        if (problems == 0) return "none";
        StringBuilder sb = new StringBuilder();
        if ((problems & TOO_CLOSE) != 0) sb.append("TOO_CLOSE ");
        if ((problems & TOO_FAR) != 0) sb.append("TOO_FAR ");
        if ((problems & NOT_CONVERGED) != 0) sb.append("NOT_CONVERGED ");
        if ((problems & BAD_APPROACH) != 0) sb.append("WRONG_SIDE_OR_ANGLE ");
        if ((problems & TOO_FAST) != 0) sb.append("TOO_FAST ");
        if ((problems & TURNING_TOO_FAST) != 0) sb.append("TURNING_TOO_FAST ");
        if ((problems & ACCELERATING) != 0) sb.append("ACCELERATING ");
        return sb.toString().trim();
    }
}
