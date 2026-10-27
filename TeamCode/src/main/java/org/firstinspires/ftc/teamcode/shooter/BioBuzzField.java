package org.firstinspires.ftc.teamcode.shooter;

/**
 * BIOBUZZ (FTC 2026-2027) field geometry, in the Pedro field frame.
 *
 * <p>Pedro frame used by all of this code (Pedro's documented convention, inches):
 * <ul>
 *   <li>origin at the field corner on the RED alliance wall, audience side;</li>
 *   <li>+x from the red wall toward the blue wall, +y from the audience toward the rear wall;</li>
 *   <li>heading in radians, counter-clockwise, 0 = +x.</li>
 * </ul>
 * This is Pedro's FTC-to-Pedro conversion (x = y_ftc + 72, y = 72 - x_ftc) applied to the FTC
 * frame (+Y away from the red wall, +X toward the audience, because red is on the audience's
 * left this season). Your Pedro start poses must use this same frame.
 *
 * <p>The HIVE numbers are derived from the dimensioned figures in Competition Manual section 9.6
 * (TU05, 2026-10-15). FIRST publishes no field coordinates for the CELL openings, so MEASURE
 * them in the official Onshape field CAD and on a real field before competing.
 *
 * <p>Geometry summary: each alliance has one seesaw HIVE beside the field center, with a CELL on
 * the audience side and one on the rear side. Only the upward CELL may be scored (G417), and it
 * swaps after every TIP. A CELL's opening is at its outer end, faces AWAY from the field center
 * and is tilted up about 30 deg, so a ball must travel toward the field center to enter it. You
 * shoot from the half of the field that the up CELL faces.
 */
public final class BioBuzzField {
    private BioBuzzField() {}

    /** Pedro's field size convention. */
    public static double FIELD_SIZE_IN = 144.0;
    /**
     * Field center in Pedro coordinates. 72 if your start poses use Pedro's 144 in convention;
     * about 70.75 if you measure start poses from the walls of a real 141.5 in field.
     */
    public static double FIELD_CENTER_IN = 72.0;
    /** Field center to each HIVE centerline, toward that alliance's wall (centerlines are 25.5 in apart). MEASURE. */
    public static double HIVE_SIDE_OFFSET_IN = 12.75;
    /** Field center to a CELL opening center along the audience-rear axis (manual figures give 16.5-18.1). MEASURE. */
    public static double CELL_OPENING_AXIAL_OFFSET_IN = 17.3;
    /** CELL opening center height above the tiles. Informational: the shot table is calibrated against it. */
    public static double CELL_OPENING_HEIGHT_IN = 58.4;
    /** Width of the pentagonal CELL opening (14 in tall, 12 in deep). Aim at its lateral center. */
    public static double CELL_OPENING_WIDTH_IN = 20.0;

    /** CELL that is up at the start of the match (manual: red audience CELL, blue rear CELL). */
    public static HiveCell RED_START_UP_CELL = HiveCell.AUDIENCE;
    public static HiveCell BLUE_START_UP_CELL = HiveCell.REAR;

    /**
     * Pose the driver can reset to during TeleOp: the robot pushed into the red-alliance /
     * audience corner, back against the red wall. Blue uses the same pose rotated 180 deg about
     * the field center (the field is rotationally symmetric, not mirrored). MEASURE: x is half the
     * robot length and y half the robot width for an 18 in square robot.
     */
    public static double RED_RESET_X_IN = 9.0;
    public static double RED_RESET_Y_IN = 9.0;
    public static double RED_RESET_HEADING_DEG = 0.0;

    /** Aim point x (Pedro, inches) of an alliance's CELL opening center. */
    public static double aimX(Alliance alliance, HiveCell cell) {
        return alliance == Alliance.RED
                ? FIELD_CENTER_IN - HIVE_SIDE_OFFSET_IN
                : FIELD_CENTER_IN + HIVE_SIDE_OFFSET_IN;
    }

    /** Aim point y (Pedro, inches) of an alliance's CELL opening center. */
    public static double aimY(Alliance alliance, HiveCell cell) {
        return cell == HiveCell.AUDIENCE
                ? FIELD_CENTER_IN - CELL_OPENING_AXIAL_OFFSET_IN
                : FIELD_CENTER_IN + CELL_OPENING_AXIAL_OFFSET_IN;
    }

    /**
     * Field direction (radians) a ball must travel, horizontally, to go INTO the CELL. The
     * audience CELL opens toward the audience (-y), so balls must travel +y, and the reverse for
     * the rear CELL.
     */
    public static double inwardAngle(HiveCell cell) {
        return cell == HiveCell.AUDIENCE ? Math.PI / 2.0 : -Math.PI / 2.0;
    }

    public static HiveCell startUpCell(Alliance alliance) {
        return alliance == Alliance.RED ? RED_START_UP_CELL : BLUE_START_UP_CELL;
    }

    /**
     * Rotation to pass to Pedro's ManualDrive.fieldCentric so that "stick forward" drives away
     * from the driver's own alliance wall: red drivers face +x, blue drivers face -x.
     */
    public static double driverHeadingOffset(Alliance alliance) {
        return alliance == Alliance.RED ? 0.0 : Math.PI;
    }

    /** Reset pose {x, y, heading(rad)} for an alliance. */
    public static double[] resetPose(Alliance alliance) {
        double heading = Math.toRadians(RED_RESET_HEADING_DEG);
        if (alliance == Alliance.RED) {
            return new double[]{RED_RESET_X_IN, RED_RESET_Y_IN, heading};
        }
        return new double[]{
                2.0 * FIELD_CENTER_IN - RED_RESET_X_IN,
                2.0 * FIELD_CENTER_IN - RED_RESET_Y_IN,
                heading + Math.PI};
    }

    /**
     * AprilTag IDs (36h11) of the four-tag cluster under each CELL, from the FTC SDK 12.0 game
     * database. The tags move with the HIVE, so they are not for localization, but a tag's roll
     * tells you whether that CELL is up.
     */
    public static int[] cellTagIds(Alliance alliance, HiveCell cell) {
        int first;
        if (alliance == Alliance.RED) {
            first = cell == HiveCell.REAR ? 30 : 34;
        } else {
            first = cell == HiveCell.AUDIENCE ? 38 : 42;
        }
        return new int[]{first, first + 1, first + 2, first + 3};
    }
}
