package org.firstinspires.ftc.teamcode.subsystems;


import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.repeat;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.pedropathing.follower.Follower;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.Launch.Gate;

import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;
import org.firstinspires.ftc.teamcode.util.ShootingCalc;
import org.firstinspires.ftc.teamcode.util.ShotTable;

@Configurable
public class Tele extends CommandOpMode {

    public static double flywheelspeed = 0, anglepivot = 0.24, anglenorm = 0, anglenou = 1;
    Alliance alliance;
    Pose Goal;
    Pose robopose= null;
    Pose turpose=null;
    public static boolean useShotTable = true;
    // Turret pivot offset from the robot center, robot frame (x forward, y left), inches. Tune from your CAD.
    public static double TURRET_OFFSET_X = -3, TURRET_OFFSET_Y = 0;
    RoboModules r;
    IntakeSystem is;
    private TelemetryManager telemetryM;
    Follower follower;

    ElapsedTime timp;

    public Tele(Alliance alliance) {
        this.alliance = alliance;

    }

    @Override
    public void init() {

        timp = new ElapsedTime();
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.localizer.reset();
        follower.setPose(alliance == Alliance.RED ? ShootingCalc.RED_START : ShootingCalc.BLUE_START);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        if (alliance == Alliance.RED)
            Goal = ShootingCalc.RED_AUD;
        else
            Goal = ShootingCalc.BLUE_AUD;
    }

    @Override
    public void init_loop() {

    }


    @Override
    public void start() {
        robopose =follower.pose();
        r = new RoboModules(hardwareMap);
        is = r.is;
        r.p.set(0.24);


        Command activateShooter = infinite(() -> {
                    r.p.setPos(anglepivot);
                    shootcalc();
                }
        );
        Scheduler.schedule(
                r.t.periodic(),
                r.l.periodic(), r.p.periodic(),
                activateShooter,
                updatespeed()
        );
    }

    @Override
    public void loop() {
        robopose =follower.pose();
        turpose = getTurretPose(robopose);


        follower.update();
        telemetryM.update();
        Scheduler.execute();
        teleopdrive();
        turret();

        telemetry();
        if (gamepad1.dpadDownWasPressed()) {
            follower.setPose(alliance == Alliance.RED ? ShootingCalc.RED_START : ShootingCalc.BLUE_START);
        }
        if (gamepad1.right_trigger > 0.2)
            Scheduler.schedule(launch());

        if (gamepad1.rightBumperWasPressed())
            Scheduler.schedule(toggleIntake());
        if(gamepad1.bWasPressed())
            Goal=ShootingCalc.other(alliance,Goal);
    }

    void telemetry() {
        telemetry.addData("flywheelspeed", flywheelspeed);
        telemetry.addData("lRPM", r.l.getRPM());
        telemetry.addData("eroor", flywheelspeed - r.l.getRPM());
        telemetry.addData("errortur", Math.toDegrees(anglenorm) - r.t.getRawAngleDegrees());
        telemetry.addData("Pose",robopose);
        telemetry.addData("TurPose",turpose);
        telemetry.update();
    }

    public Command launch() {
        return sequential(
                r.g.setcom(Gate.Mode.OPEN),
                is.run(1),
                waitMs(600),
                r.g.setcom(Gate.Mode.CLOSE),
                is.run(0)
        )
                .requiring(is)
                .setPriority(10)
                .setConflictBehavior(ConflictBehavior.CANCEL);
    }public Command toggleIntake() {
        return is.run(is.getIntakeOn() == 0 ? 1 : 0)
                .requiring(is)
                .setPriority(0)
                .setBlockedBehavior(BlockedBehavior.CANCEL);
    }

    public Command updatespeed() {
        return
                infinite(() ->
                        r.l.updateshooting(flywheelspeed)
                );
    }

    /** Sets flywheel RPM + hood from the distance to the goal when useShotTable is on. */
    public void shootcalc() {
        if (!useShotTable || robopose == null || Goal == null) return;
        double distance = Math.hypot(Goal.x() - turpose.x(), Goal.y() - turpose.y());
        ShotTable.Shot shot = ShotTable.get(distance);
        flywheelspeed = shot.rpm;
        anglepivot = shot.hood;
    }

    /**
     * Turret pivot position on the field = robot pose + the turret offset rotated by the robot heading.
     * Offset is in the robot frame (x forward, y left), same units as the field (inches).
     */
    Pose getTurretPose(Pose robot) {
        double h = robot.heading();
        double cos = Math.cos(h), sin = Math.sin(h);
        return new Pose(
                robot.x() + TURRET_OFFSET_X * cos - TURRET_OFFSET_Y * sin,
                robot.y() + TURRET_OFFSET_X * sin + TURRET_OFFSET_Y * cos,
                h);
    }

    void turret() {

        // Aim from the turret's own position, not from the robot center.
        double dx = turpose.x() - Goal.x();
        double dy = turpose.y() - Goal.y();
        double theta = Math.atan2(dy, dx);

        anglenou = Math.PI / 2 - theta;
        anglenorm = anglenou + robopose.heading() - Math.PI / 2;

        // Wrap to [-180, 180] degrees.
        while (anglenorm > Math.PI) anglenorm -= 2 * Math.PI;
        while (anglenorm < -Math.PI) anglenorm += 2 * Math.PI;

        r.t.setTargetDegrees(Math.toDegrees(anglenorm));
    }


    public void teleopdrive() {
        follower.manual(-gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                Math.abs(gamepad1.right_stick_x)<0.2 ? 0 : gamepad1.right_stick_x
        );
    }


}
