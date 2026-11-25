package org.firstinspires.ftc.teamcode.subsystems;


import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.Launch.Gate;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;
import org.firstinspires.ftc.teamcode.util.ShootingCalc;
import org.firstinspires.ftc.teamcode.util.ShotTable;

@Config
public class Auto extends CommandOpMode {

    public static double flywheelspeed = 0, anglepivot = 0.24, anglenorm = 0, anglenou = 1;
    Alliance alliance;
    Pose Goal;
    Pose robopose= null;
    Pose turpose=null;
    public static boolean useShotTable = true;
    // Turret pivot offset from the robot center, robot frame (x forward, y left), inches. Tune from your CAD.
    public static double TURRET_OFFSET_X = -2.7, TURRET_OFFSET_Y = 0;
    // Turret hold: when true, auto-aim is ignored and the turret stays at turretHoldAngle.
    public static boolean turretHold = false, useCurrentAngleForHold = true;
    public static double turretHoldAngle = 0;
    public RoboModules r;
    double distance;
    public IntakeSystem is;


    ElapsedTime timp;

    public Auto(Alliance alliance,HardwareMap hardwaremap) {
        r = new RoboModules(hardwaremap);
        is = r.is;

        this.alliance = alliance;
        if (this.alliance == Alliance.RED)
            Goal = ShootingCalc.RED_AUD;
        else
            Goal = ShootingCalc.BLUE_AUD;
        turretHold = false;
    }


    public void telemetry(Telemetry telemetry) {
        telemetry.addData("flywheelspeed", flywheelspeed);
        telemetry.addData("lRPM", r.l.getRPM());
        telemetry.addData("eroor", flywheelspeed - r.l.getRPM());
        telemetry.addData("errortur", Math.toDegrees(anglenorm) - r.t.getRawAngleDegrees());
        telemetry.addData("Pose",robopose);
        telemetry.addData("TurPose",turpose);
        telemetry.addData("turangle", r.t.getRawAngleDegrees());
        telemetry.addData("turlookup", Math.toDegrees(anglenorm) );
        telemetry.addData("distanta",distance);
        telemetry.update();
    }
    public void update(Pose pose){
        is.updatePid();
        if (pose == null) return;
        robopose=pose;
        turpose = getTurretPose(pose);
    }
    public void changegoal(){
        Goal=ShootingCalc.other(alliance,Goal);
    }

    public Command launch() {
        return sequential(
                r.g.setcom(Gate.Mode.OPEN),
                is.run(1),
                waitMs(40),
                is.run(2),
                waitMs(500),
                r.g.setcom(Gate.Mode.CLOSE),
                is.run(0)
        )
                .requiring(is)
                .setPriority(10)
                .setConflictBehavior(ConflictBehavior.CANCEL);
    }
    /**
     * Sets the flywheel RPM and hood angle by hand and turns the ShotTable lookup OFF,
     * so shootcalc() stops overwriting them. Use useTableShot() to go back to the table.
     * Example: auto.setShot(2450, 0.27)
     */
    public Command setShot(double rpm, double hoodAngle) {
        return instant(() -> {
            useShotTable = false;
            flywheelspeed = rpm;
            anglepivot = hoodAngle;
        });
    }

    /** Switches back to ShotTable-driven RPM + hood (distance to goal). */
    public Command useTableShot() {
        return instant(() -> useShotTable = true);
    }

    public Command updatespeed() {
        return
                infinite(() ->
                        r.l.updateshooting(flywheelspeed)
                );
    }
    /**
     * Toggles turret hold. When turned on, goal-tracking is ignored and the turret is held at
     * turretHoldAngle (degrees, robot frame). If useCurrentAngleForHold is true, the angle the
     * turret is at when toggled on is captured and held instead. Toggle again to resume tracking.
     */
    public Command toggleTurretHold() {
        return instant(() -> {
            turretHold = !turretHold;
            if (turretHold && useCurrentAngleForHold)
                turretHoldAngle = r.t.getRawAngleDegrees();
        });
    }

    /**
     * Toggles turret hold, setting the hold angle (degrees, robot frame) when it turns on.
     * Calling it again turns hold off and resumes goal tracking.
     */
    public Command toggleTurretHold(double angleDeg) {
        return instant(() -> {
            turretHold = !turretHold;
            if (turretHold) turretHoldAngle = angleDeg;
        });
    }

    public Command updateTur() {
        return
                infinite(() -> {
                    if (turretHold) {
                        r.t.setTargetDegrees(turretHoldAngle);
                        return;
                    }
                    if (robopose == null) return;
                    turpose = getTurretPose(robopose);
                    turret();
                });
    }

    public void shootcalc() {
        if (!useShotTable || robopose == null || turpose == null || Goal == null) return;
        distance = Math.hypot(Goal.x() - turpose.x(), Goal.y() - turpose.y());
        ShotTable.Shot shot = ShotTable.get(Math.abs(distance));
        flywheelspeed = shot.rpm;
        anglepivot = shot.hood;
    }


    Pose getTurretPose(Pose robot) {
        double h = robot.heading();
        double cos = Math.cos(h), sin = Math.sin(h);
        return new Pose(
                robot.x() + TURRET_OFFSET_X * cos - TURRET_OFFSET_Y * sin,
                robot.y() + TURRET_OFFSET_X * sin + TURRET_OFFSET_Y * cos,
                h);
    }

    void turret() {
        if (turretHold) {
            r.t.setTargetDegrees(turretHoldAngle);
            return;
        }
        if (robopose == null || turpose == null) return;
        // Aim from the turret's own position, not from the robot center.
        double dx = Goal.x() - turpose.x();
        double dy = Goal.y() - turpose.y();
        // Field-frame bearing from the turret to the goal.
        double theta = Math.atan2(dy, dx);

        anglenou = theta;
        // Turret angle relative to the robot: 0 = robot forward, CCW positive.
        anglenorm = theta - robopose.heading();

        // Wrap to [-180, 180] degrees.
        while (anglenorm > Math.PI) anglenorm -= 2 * Math.PI;
        while (anglenorm < -Math.PI) anglenorm += 2 * Math.PI;

        r.t.setTargetDegrees(Math.toDegrees(anglenorm));
    }



}
