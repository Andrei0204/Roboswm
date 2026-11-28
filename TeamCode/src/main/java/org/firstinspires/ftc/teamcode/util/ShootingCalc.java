package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

import com.pedropathing.math.Pose;


@Config
public class ShootingCalc {
    public static Pose RED_AUD = new Pose(60, 56);
    public static Pose RED_REAR = new Pose(60, 76);
    public static Pose RED_START = new Pose(6.9, 6.9, 0);
    public static Pose RED_START_new = new Pose(38, 58.8, 0);
    public static Pose BLUE_AUD = new Pose(84, 65);
    public static Pose BLUE_REAR = new Pose(84, 70);
    public static Pose BLUE_START = new Pose(137, 127, Math.PI);
    public static Pose BLUE_START_new = new Pose(105, 74, Math.PI);

    private static boolean same(Pose a, Pose b) {
        return Math.abs(a.x() - b.x()) < 1e-6 && Math.abs(a.y() - b.y()) < 1e-6;
    }

    public static Pose other(Alliance alliance, Pose goal) {
        if (alliance == Alliance.RED)
            return same(goal, RED_AUD) ? RED_REAR : RED_AUD;
        return same(goal, BLUE_AUD) ? BLUE_REAR : BLUE_AUD;
    }

}
