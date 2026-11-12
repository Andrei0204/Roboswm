package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;


@Configurable
public class ShootingCalc {
    public static Pose RED_AUD = new Pose(57, 55);
    public static Pose RED_REAR = new Pose(57, 90);
    public static Pose RED_START = new Pose(6.9, 6.9, 0);
    public static Pose BLUE_AUD = new Pose(87, 55);
    public static Pose BLUE_REAR = new Pose(87, 90);
    public static Pose BLUE_START = new Pose(137.1, 6.9, Math.PI);

    private static boolean same(Pose a, Pose b) {
        return Math.abs(a.x() - b.x()) < 1e-6 && Math.abs(a.y() - b.y()) < 1e-6;
    }

    public static Pose other(Alliance alliance, Pose goal) {
        if (alliance == Alliance.RED)
            return same(goal, RED_AUD) ? RED_REAR : RED_AUD;
        return same(goal, BLUE_AUD) ? BLUE_REAR : BLUE_AUD;
    }

}
