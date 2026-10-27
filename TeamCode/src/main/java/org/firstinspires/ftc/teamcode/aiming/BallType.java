package org.firstinspires.ftc.teamcode.aiming;

/**
 * BIOBUZZ game pieces that can be launched into a CELL (Competition Manual section 9.8). They
 * differ in size and mass, so each has its own shot table.
 */
public enum BallType {
    /** Yellow polyethylene ball, about 2.8 in. */
    POLLEN(2.8),
    /** Alliance-colored ball, about 3.6 in, about 1.33-1.67x the mass of POLLEN. */
    NECTAR(3.6);

    /** Nominal diameter (in). MEASURE several balls with calipers. */
    public final double diameterIn;

    BallType(double diameterIn) {
        this.diameterIn = diameterIn;
    }

    public BallType other() {
        return this == POLLEN ? NECTAR : POLLEN;
    }
}
