package org.firstinspires.ftc.teamcode.Autonomii;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
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
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;

@Autonomous
public class parckblue extends CommandOpMode {
    Auto auto = null;
    Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    // Poses
    private final Pose start = poseFactory.of(9, 62, 0);
    private final Pose path1 = poseFactory.of(8.58, 94.8037, 0);

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.localizer.reset();
        follower.update();
        follower.setPose(start);
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
                follow(follower, path1())
        );
    }


}
