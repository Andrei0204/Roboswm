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
public class tiltBlue2 extends CommandOpMode {
    Auto auto = null;
    Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    // Poses
    double addon = 89 - 85.6;
    private final Pose startPose = poseFactory.of(89, 139.7, 0);
    private final Pose path1 = poseFactory.of(129 + addon, 139.25, 0);
    private final Pose point2 = poseFactory.of(94 + addon, 27, -110);
    private final Pose point2Control1 = poseFactory.of(119 + addon, 56.25, 180);
    private final Pose point3 = poseFactory.of(94 + addon, 21, -90);
    private final Pose iesitfloare = poseFactory.of(93.8307 + addon, 28.0447, 110);
    private final Pose plecatadoua = poseFactory.of(116.833 + addon, 90, 66.3198);
    private final Pose iafloare = poseFactory.of(125.1705 + addon, 97.8972, 0);
    private final Pose aruncafloare = poseFactory.of(95 + addon, 25.3762, 73.0271);
    private final Pose aruncafloarep1 = poseFactory.of(120 + addon, 50, 73.0271);
    private final Pose parck = poseFactory.of(128.311 + addon, 50, -132.8898);


    public Path path1() {
        return Paths.line(startPose, path1).linear(startPose, path1).with(Constants.foresightConfig.maxPathSpeed.at(5.0));
    }

    public Path path2() {
        return Paths.curve(path1, point2Control1, point2).linear(path1, point2, 0.3);
    }

    public Path path3() {
        return Paths.line(point2, point3).linear(point2, point3);
    }

    public Path iesfloare() {
        return Paths.line(point3, iesitfloare).reverseTangent();
    }

    public Path floareb() {
        return Paths.line(iesitfloare, plecatadoua).tangent();
    }

    public Path colectfloare() {
        return Paths.line(plecatadoua, iafloare).linear(plecatadoua, iafloare, 0.5);
    }

    public Path iesifloare2() {
        return Paths.line(iafloare, plecatadoua).reverseTangent();
    }


    public Path salutaruncam() {
        return Paths.curve(iesitfloare, aruncafloarep1, aruncafloare).constant(iesitfloare);
    }

    public Path parcarea() {
        return Paths.line(point3, parck).reverseTangent();
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
        telemetry.addData("goal", auto.Goal);
        telemetry.update();


    }

    @Override
    public void stop() {
        OpModeStorage.autonomousEndPose = follower.pose(); //saves your position in that file
    }

    private Command autoroutine() {
        return sequential(
                waitMs(2500),
                auto.launch(),
                auto.intake(1),
                follow(follower, path1()).with(auto.intake(0)),
                instant(() -> auto.changegoal()),
                waitMs(200),
                follow(follower, path2()).with(waitUntil(() -> follower.pose().y() < 124).then(auto.intake(0))),
                waitMs(800),
                auto.launch(),
                auto.r.f.setModeC(Flower.Mode.DOWN),
                auto.intake(1),
                waitMs(100),
                follow(follower, path3()),
                waitMs(1500),
                auto.launch(),
                follow(follower, iesfloare()),
                follow(follower, floareb()).with(waitUntil(() -> follower.pose().y() > 44).then(auto.r.f.setModeC(Flower.Mode.UP))),
                auto.r.f.setModeC(Flower.Mode.DOWN),
                auto.intake(1),
                follow(follower, colectfloare()),
                waitMs(1500),
                follow(follower, iesifloare2()),
                auto.r.f.setModeC(Flower.Mode.UP),
                follow(follower, salutaruncam()),
                waitMs(1600),
                auto.launch(),
                instant(() -> auto.changegoal()),
                follow(follower, parcarea())
                //               follow(follower,iesfloare()),
        );
    }


}
