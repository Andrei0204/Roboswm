package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

import com.pedropathing.math.Pose;


@Config
public class ShootingCalc {
    public static Pose RED_AUD = new Pose(61, 55);
    public static Pose RED_REAR = new Pose(61, 79);
    public static Pose RED_START = new Pose(6.9, 6.9, 0);
    public static Pose BLUE_AUD = new Pose(84, 55);
    public static Pose BLUE_REAR = new Pose(84, 80);
    public static Pose BLUE_START = new Pose(136, 131, Math.PI);

    private static boolean same(Pose a, Pose b) {
        return Math.abs(a.x() - b.x()) < 1e-6 && Math.abs(a.y() - b.y()) < 1e-6;
    }

    public static Pose other(Alliance alliance, Pose goal) {
        if (alliance == Alliance.RED)
            return same(goal, RED_AUD) ? RED_REAR : RED_AUD;
        return same(goal, BLUE_AUD) ? BLUE_REAR : BLUE_AUD;
    }

}
