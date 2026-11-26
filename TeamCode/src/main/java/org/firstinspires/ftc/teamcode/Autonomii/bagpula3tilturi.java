package org.firstinspires.ftc.teamcode.Autonomii;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.hold;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.api.Paths;
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
public class bagpula3tilturi extends CommandOpMode {
    Auto auto = null;
    Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    // Poses
    private final Pose startPose = poseFactory.of(58.4, 4.3, 180);
    private final Pose path1 = poseFactory.of(15, 4.75, 180);
    private final Pose point2 = poseFactory.of(50, 115, 70);
    private final Pose point2Control1 = poseFactory.of(25, 87.75, 0);
    private final Pose point3 = poseFactory.of(50, 123, 90);
    private final Pose iesitfloare = poseFactory.of(50.1693, 115.9553, 90);
    private final Pose colectaliat = poseFactory.of(25.7143, 114.9577, 180);
    private final Pose aruncare2 = poseFactory.of(49.5316, 5.9663, 180);
    private final Pose dutefloare = poseFactory.of(28.2252, 46.4874, 131.4218);

    private final Pose iafloare222 = poseFactory.of(18.2826, 46.3457, 180);

    private final Pose ultimaaruncare = poseFactory.of(44.5741, 12.1768, 90);

    private final Pose parcare = poseFactory.of(14.6284, 92.2666, 110.0989);

    public Path parcarea() {
        return Paths.line(ultimaaruncare, parcare).tangent();
    }




    public Path path1() {
        return Paths.line(startPose, path1).linear(startPose, path1).with(Constants.foresightConfig.maxPathSpeed.at(5.0));
    }

    public Path path2() {
        return Paths.curve(path1, point2Control1, point2).linear( path1,point2,0.3);
    }

    public Path path3() {
        return Paths.line(point2, point3).linear(point2, point3);
    }


    public Path iesfloare() {
        return Paths.line(point3, iesitfloare).reverseTangent();
    }

    public Path colectal() {
        return Paths.line(iesitfloare, colectaliat).tangent();
    }


    public Path aruncaasta() {
        return Paths.line(colectaliat, aruncare2).linear(colectaliat, aruncare2);
    }

    public Path dutefloare2() {
        return Paths.line(aruncare2, dutefloare).tangent();
    }

    public Path iafloare() {
        return Paths.line(dutefloare, iafloare222).tangent();
    }

    public Path iesifloareul() {
        return Paths.line(iafloare222, dutefloare).reverseTangent();
    }

    public Path aruncaulrima() {
        return Paths.line(dutefloare, ultimaaruncare).linear(iafloare222, ultimaaruncare);
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
        auto.enable();
        Scheduler.schedule(autoroutine());
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
                auto.intake(1),
                follow(follower, path1()),
                instant(() -> auto.changegoal()),
                waitMs(200),
                follow(follower, path2()).with(waitUntil(() -> follower.pose().y() > 20).then(auto.intake(0))),
                waitMs(800),
                auto.launch(),
                auto.r.f.setModeC(Flower.Mode.DOWN),
                auto.intake(1),
                waitMs(100),
                follow(follower,path3()),
                waitMs(1000),
                auto.launch(),
                instant(() -> auto.changegoal()),
                follow(follower,iesfloare()),
                auto.r.f.setModeC(Flower.Mode.UP),
                auto.intake(1),
                follow(follower,colectal()),
                follow(follower,aruncaasta()),
                waitMs(1000),
                auto.launch(),
                follow(follower,dutefloare2()),
                auto.r.f.setModeC(Flower.Mode.DOWN),
                auto.intake(1),
                follow(follower,iafloare()),
                waitMs(1000),
                follow(follower,iesifloareul()),
                auto.r.f.setModeC(Flower.Mode.UP),
                auto.intake(0),
                follow(follower,aruncaulrima()),
                waitMs(600),
                auto.launch(),
                instant(() -> auto.changegoal()),
                follow(follower,parcarea())

                );
    }


}
