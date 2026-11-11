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

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Launch.Gate;
import org.firstinspires.ftc.teamcode.subsystems.Launch.Turret;

import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;
import org.firstinspires.ftc.teamcode.util.ShootingCalc;

@Configurable
public class Tele extends CommandOpMode {

    public static double flywheelspeed = 0, anglepivot = 0, anglenorm = 0;
    Alliance alliance;
    Pose Goal;

    RoboModules r;
    double launched = 0;
    private TelemetryManager telemetryM;
    Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this

    public static double anglenou = 1;
    ElapsedTime timp;

    public Tele(Alliance alliance) {
        this.alliance = alliance;

    }

    @Override
    public void init() {

        ShootingCalc.reset();
        timp = new ElapsedTime();
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.localizer.reset();

        follower.setPose(new Pose(64.3, 7.3, Math.toRadians(90)));

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        if (alliance == Alliance.RED)
            Goal = ShootingCalc.GOAL_POS_RED;
        else
            Goal = ShootingCalc.GOAL_POS_BLUE;

    }

    @Override
    public void init_loop() {

    }


    @Override
    public void start() {
        r = new RoboModules(hardwareMap);
        r.p.set(0.51);
        is = new intakesys(r);
//        camera.start();
        Command activateShooter = infinite(() -> {
                    r.p.setAngle(anglepivot);
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

        follower.update();
        telemetryM.update();
        Scheduler.execute();
        teleopdrive();
        turret();

        if (gamepad1.right_trigger_pressed && launched == 0) {
            is.setIntakeOn(0);
            Scheduler.schedule(is.update());
            Scheduler.schedule(launch());
            launched = 1;
        }


        telemetry();
        if (gamepad1.dpadDownWasPressed()) {
            if (alliance == Alliance.RED)
                follower.setPose(new Pose(125, 7, 0));
            else
                follower.setPose(new Pose(14.2, 8, Math.PI));
            ShootingCalc.reset();//TODO TODO TODO!!!!

        }
    }

    void telemetry() {
        telemetry.addData("flywheelspeed", flywheelspeed);
        telemetry.addData("lspeed", r.l.GetRPS());
        telemetry.addData("eroor", r.l.conversion(flywheelspeed) - r.l.GetRPS());
        telemetry.addData("errortur", Math.toDegrees(anglenorm) - r.t.getRawAngleDegrees());
        telemetry.update();
    }

    public Command Shootcom() {
        return launch();
    }

    public Command launch() {
        return sequential(
                r.g.setcom(Gate.Mode.OPEN),
                sequential(
                        is.setIn(6),
                        is.update()
                ),
                waitMs(600),
                r.g.setcom(Gate.Mode.CLOSE),
                is.setIn(0),
                is.update(),
                instant(() -> {
                    launched = 0;
                })
        );
    }

    public Command updatespeed() {
        return
                infinite(() ->
                        r.l.updateshooting(flywheelspeed)
                );
    }

    public void shootcalc() {

    }

    void turret() {
        Vector RobotTogoal = new Vector(new Pose(Goal.getX() - follower.getPose().getX(), Goal.getY() - follower.getPose().getY()));
        anglenou = Math.PI / 2 - RobotTogoal.getTheta();
        anglenorm = anglenou + follower.getPose().getHeading() - Math.PI / 2;

        while (anglenorm > Math.PI) anglenorm -= 2 * Math.PI;
        while (anglenorm < -Math.PI) anglenorm += 2 * Math.PI;
        if (Math.toDegrees(anglenorm) > 180) {
            anglenorm = Math.toRadians(180);
        } else if (Math.toDegrees(anglenorm) < -180)
            anglenorm = Math.toRadians(-180);
        r.t.setTargetDegrees(Math.toDegrees(anglenorm));

    }


    public void teleopdrive() {
        follower.manual(-gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                Math.abs(gamepad1.right_stick_x)<0.2 ? 0 : gamepad1.right_stick_x
        );
    }


}
