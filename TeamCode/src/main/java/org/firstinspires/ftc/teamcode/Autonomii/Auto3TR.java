package org.firstinspires.ftc.teamcode.Autonomii;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.hold;

import static org.firstinspires.ftc.teamcode.subsystems.Intake.Tavalug.Mode.COLLECT_FLOARE;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Auto;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Flower;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Tavalug;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;

@Autonomous
public class Auto3TR extends CommandOpMode {
    Auto auto = null;
    Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    // Poses
    private final Pose startPose = poseFactory.of(56.4, 4.3, 180);
    private final Pose firstflower = poseFactory.of(17, 43, 180);

    private final Pose holdf = poseFactory.of(15.5, 43, 180);

    private final Pose path1Control1 = poseFactory.of(30, 43, 180);
    private final Pose secondLaunchC1 = poseFactory.of(23, 95, 90);
    private final Pose secondLaunch = poseFactory.of(47, 114, 105);  // 4 y 1 x

    private final Pose holdf2 = poseFactory.of(47, 124, 90); // x 1 y 1

    private final Pose point5 = poseFactory.of(47, 120, 90);  //trebuia sa fie ca holdf2
    private final Pose salutpath = poseFactory.of(30.7696, 71.1079, 90);

    private final Pose holdintake = poseFactory.of(11.0, 71.0, 180);
    private final Pose point8 = poseFactory.of(44.2799, 6.2242, 111.3878);
    private final Pose point9 = poseFactory.of(10.9354, 3.8, -177.0489); // era x = 15
    private final Pose ultimaaruncare = poseFactory.of(56.3258, 4.7999, 178.5553);

    private final Pose parcare = poseFactory.of(15.5603, 91.7781, -64.8881);
    // Path methods


    private Path gotoflower() {
        return line(startPose, firstflower).linear(startPose, firstflower);
    }

    private Path secondlaunchBack() {
        return line(firstflower, path1Control1).linear(firstflower, path1Control1);
    }

    private Path secondlaunch() {
        return curve(path1Control1, secondLaunchC1,secondLaunch).constant(secondLaunch);
    }

    public Path salutpath() {
        return line(point5, salutpath).linear(holdf2,point5);
    }

    public Path path8() {
        return line(holdintake, point8).reverseTangent();
    }


    public Path path9() {
        return line(point8, point9).tangent();
    }

    public Path ultimaaruncare() {
        return line(point9, ultimaaruncare).reverseTangent();
    }

    public Path parcare() {
        return line(ultimaaruncare, parcare).reverseTangent();
    }




    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.localizer.reset();
        follower.update();
        follower.setPose(startPose);
        auto = new Auto(Alliance.RED, hardwareMap);
        auto.update(follower.pose());

    }

    @Override
    public void init_loop() {
        if (gamepad1.y) auto.r.t.resetEncoder();
        auto.update(follower.pose());
        Scheduler.execute();
        follower.update();
    }

    @Override
    public void start() {
        Command activateShooter = infinite(() -> {
                    auto.r.p.setPos(auto.anglepivot);
                    auto.shootcalc();
                }
        );
        Scheduler.schedule(
                auto.r.t.periodic(),
                auto.r.l.periodic(), auto.r.p.periodic(),
                activateShooter,
                auto.updatespeed(),
                auto.updateTur(),
                autoroutine()
        );
    }

    @Override
    public void loop() {
        auto.update(follower.pose());
        Scheduler.execute();
        follower.update();
        auto.telemetry(telemetry);
        telemetry.addData("T", follower.parametricCompletion());
        telemetry.update();


    }

    @Override
    public void stop() {
        OpModeStorage.autonomousEndPose = follower.pose(); //saves your position in that file
    }

    private Command autoroutine() {
        return sequential(
                waitMs(1800),
                auto.launch(),
                parallel(
                        waitUntil(() -> follower.pose().x() < 40).then(
                                sequential(
                                        auto.r.f.setModeC(Flower.Mode.DOWN),
                                        auto.is.run(2)
                                )
                        ),
                        follow(follower, gotoflower())

                ),
                deadline(
                        waitMs(1500),
                        hold(follower, holdf)
                ),
                auto.toggleTurretHold(178),
                follow(follower, secondlaunchBack()),
                auto.r.f.setModeC(Flower.Mode.UP),
                auto.is.run(0),
                //instant(()->auto.changegoal()),
                follow(follower, secondlaunch()),
                auto.r.f.setModeC(Flower.Mode.DOWN),
                waitMs(500),
                auto.launch(),
                auto.is.run(2),
                auto.toggleTurretHold(178),
                auto.toggleTurretHold(190),
                deadline(
                        waitMs(2000),
                        hold(follower,holdf2)
                ),
                waitMs(500),
                auto.launch(),
                auto.r.tav.setModeC(COLLECT_FLOARE),
                parallel(
                        waitUntil(() -> follower.pose().x() < 40).then(
                                sequential(
                                        auto.r.f.setModeC(Flower.Mode.UP),
                                        auto.is.run(2)
                                )
                        ),
                        follow(follower, salutpath())
                ),
                deadline(
                        waitMs(1500),
                        hold(follower, holdintake)
                ),
                auto.toggleTurretHold(0),
                //instant(()->auto.changegoal()),
                follow(follower, path8()),
                waitMs(500),
                auto.launch(),
                parallel(
                        waitUntil(() -> follower.pose().x() < 25).then(
                                        auto.is.run(2)
                        ),
                        follow(follower, path9())
                ),
                follow(follower, ultimaaruncare()),
                waitMs(500),
                auto.launch(),
                follow(follower, parcare())
//                waitMs(1000)


        );
    }


}
