package org.firstinspires.ftc.teamcode.Autonomii;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.parallel;
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
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;

@Autonomous
public class ParkBlue extends CommandOpMode {
    Auto auto = null;
    Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    // Poses
    private final Pose startPose = poseFactory.of(140, 58, 180);
    private final Pose firstflower = poseFactory.of(137, 28, 180);

    private final Pose holdf = poseFactory.of(128.5, 99, 0);

    private final Pose path1Control1 = poseFactory.of(114, 101, 0);
    private final Pose secondLaunchC1 = poseFactory.of(121, 49, -90);
    private final Pose secondLaunch = poseFactory.of(99, 30, -75);  // 4 y 1 x

    private final Pose holdf2 = poseFactory.of(97, 20, -90); // x 1 y 1

    private final Pose point5 = poseFactory.of(97, 20, -90);
    private final Pose salutpath = poseFactory.of(115.2304, 72.8921, -128.9755);

    private final Pose holdintake = poseFactory.of(133, 73, 0);
    private final Pose point8 = poseFactory.of(99.7201, 137.7758, -68.6122);
    private final Pose point9 = poseFactory.of(128.0646, 140.2, 2.9511);
    private final Pose ultimaaruncare = poseFactory.of(87.6742, 139.2001, -1.4447);

    private final Pose parcare = poseFactory.of(128.4397, 52.2219, 115.1119);
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
        return line(point5, salutpath).tangent();
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
        auto = new Auto(Alliance.BLUE, hardwareMap);
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
                        follow(follower, gotoflower())

        );
    }


}
