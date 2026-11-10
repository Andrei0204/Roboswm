package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static org.firstinspires.ftc.teamcode.auto.AutoBlocks.sequence;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.robot.RobotSys;
import org.firstinspires.ftc.teamcode.subsystems.IntakeServo;

/** Same square path as before, but the robot shoots after every leg. */
@Autonomous
public class treihiveuriparcare extends LinearOpMode {

    // x, y, heading (deg) - RED alliance
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(36.1622, 8.6704, 180);
    private final Pose floare = poseFactory.of(14.8893, 46.7661, 180);
    private final Pose hive2Start = poseFactory.of(14.8893, 46.7661, 180);
    private final Pose hive2 = poseFactory.of(39.5414, 126.8685, 45);
    private final Pose point3Start = poseFactory.of(39.5414, 126.8685, 45);
    private final Pose point3 = poseFactory.of(54.8657, 120.0652, 180);
    private final Pose point4Start = poseFactory.of(54.8657, 120.0652, 180);
    private final Pose point4 = poseFactory.of(9.6468, 89.0164, 180);
    private final Pose point4Control1 = poseFactory.of(29.392, 86.7399, 0);
    private final Pose point5Start = poseFactory.of(9.6468, 89.0164, 180);
    private final Pose point5 = poseFactory.of(38.9553, 17.0739, 90);
    private final Pose point6Start = poseFactory.of(38.9553, 17.0739, 180);
    private final Pose point6 = poseFactory.of(13.5314, 94.9902, 0);

    /** How long to hold fire at each stop, and the max wait for the shot checks to pass. */
    private static final double SHOOT_MS = 2500;
    private static final double READY_TIMEOUT_MS = 1500;
    private static final double LEG_TIMEOUT_MS = 6000;

    @Override
    public void runOpMode() {
        RobotSys robot = new RobotSys(hardwareMap, Alliance.RED);

        robot.setPose(start.x(), start.y(), start.heading());

        while (opModeInInit()) {
            if (gamepad1.yWasPressed()) robot.homeTurret();
            if (gamepad1.aWasPressed()) robot.servosToStart();
            robot.initLoop();
            telemetry.addData("Turret (gamepad1 Y = home)", robot.turretStatus());
            if (!robot.shooter().turret().hasReference() || robot.shooter().turret().isHoming()) {
                telemetry.addLine("!!! TURRET NOT HOMED - press gamepad1 Y before START !!!");
            }
            telemetry.update();
            idle();
        }
        if (isStopRequested()) return;

        AutoBlocks auto = new AutoBlocks(robot);   // resets the scheduler
        Command routine = buildRoutine(auto);
        auto.start(routine);

        while (opModeIsActive()) {
            auto.run();   // commands, then robot.update() and robot.shoot()
            robot.addTelemetry(telemetry);
            auto.addAutoDebug(telemetry);
            telemetry.addData("Routine", auto.isFinished(routine) ? "FINISHED" : "running");
            telemetry.update();
        }
        robot.stop();
    }

    /** Drive to each point and shoot there. */
    private Command buildRoutine(AutoBlocks a) {
        return sequence(
                a.arm(),
                shootHere(a),
                a.hiveTipped(),
                a.intakeFeedOn(),
                a.follow(floare(), LEG_TIMEOUT_MS),
                waitMs(1000),
                a.intakeFeedOff(),
                a.follow(hive2(), LEG_TIMEOUT_MS),
                shootHere(a),
                a.intakeFeed(2000),
                a.follow(path3(), LEG_TIMEOUT_MS),
                shootHere(a),
                sequence(
                        parallel(
                                a.follow(path4(), LEG_TIMEOUT_MS),
                                a.intakeFeedOn()
                        ),
                        waitMs(2000)
                ),
                a.hiveTipped(),
                a.follow(path5(), LEG_TIMEOUT_MS),
                shootHere(a),
                a.follow(path6(), LEG_TIMEOUT_MS),







//                shootHere(a),
//                parallel(
//                        a.follow(colectflowershoot(), LEG_TIMEOUT_MS),
//                        deploy(a)
//                ),
//                shootHere(a),
//                a.follow(park(), LEG_TIMEOUT_MS),
              a.disarm(),
              a.stopAll());
    }

    /** The shoot command: wait until ready, then fire. */
    private Command shootHere(AutoBlocks a) {
        return a.shootStrict(SHOOT_MS, 3000, 4000);
    }


    private Command deploy(AutoBlocks a) {
        return   sequence(
                waitMs(400),          // suntem si noi smekeri ciumegi
                a.intakeMode(IntakeServo.Mode.COLLECT_FLOARE)
        );
    }
    public Path floare() {
        return Paths.line(start, floare).linear(start, floare);
    }

    public Path hive2() {
        return Paths.line(hive2Start, hive2).linear(hive2Start, hive2);
    }

    public Path path3() {
        return Paths.line(point3Start, point3).linear(point3Start, point3);
    }

    public Path path4() {
        return Paths.curve(point4Start, point4Control1, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return Paths.line(point5Start, point5).linear(point5Start, point5);
    }

    public Path path6() {
        return Paths.line(point6Start, point6).linear(point6Start, point6);
    }
}
