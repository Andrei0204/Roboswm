package org.firstinspires.ftc.teamcode.shooter;

/**
 * Hands the robot pose, alliance and up CELL from one OpMode to the next (AUTO to TeleOp). The
 * turret needs nothing here: homing zeroes its encoder count, and the hub keeps that count.
 * Static fields survive between OpModes until the Robot Controller app restarts.
 */
public final class PoseStorage {
    private PoseStorage() {}

    public static boolean hasData = false;
    public static double x;
    public static double y;
    public static double heading;
    public static Alliance alliance = Alliance.RED;
    /** CELL that was up when the previous OpMode ended. */
    public static HiveCell upCell = null;


    public static void save(double x, double y, double heading, Alliance alliance, HiveCell upCell) {
        PoseStorage.x = x;
        PoseStorage.y = y;
        PoseStorage.heading = heading;
        PoseStorage.alliance = alliance;
        PoseStorage.upCell = upCell;
        PoseStorage.hasData = true;
    }

    public static void clear() {
        hasData = false;
        upCell = null;
    }
}
