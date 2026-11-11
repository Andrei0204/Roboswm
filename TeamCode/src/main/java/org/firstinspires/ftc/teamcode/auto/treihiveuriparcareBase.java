package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static org.firstinspires.ftc.teamcode.auto.AutoBlocks.sequence;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.robot.RobotSys;

/**
 * Shared routine for treihiveuriparcarered / treihiveuriparcareblue. Every pose is written for
 * RED and goes through {@link AutoBlocks#pose}, which rotates it 180 deg about the field center
 * for BLUE, so the alliance, the start pose, the paths and the turret aim all agree.
 *
 * <p>No controller needed in INIT: the alliance and the start pose are fixed by the subclass.
 * Put the robot on the START pose and the turret straight forward (0 deg) before INIT.
 */
public abstract class treihiveuriparcareBase extends LinearOpMode {

    protected abstract Alliance alliance();

    // Poses written for RED (x, y, heading in deg). BLUE is rotated automatically by rot().
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(40, 6.2, 180);
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
    private final Pose point6 = poseFactory.of(13.5314, 94.9902, 180);


    /** A RED pose, rotated 180 deg about the field center when the alliance is BLUE. */
    private Pose rot(Pose red) {
        return AutoBlocks.pose(alliance(), red.x(), red.y(), Math.toDegrees(red.heading()));
    }

    private static final double SHOOT_MS = 2500;
    private static final double LEG_TIMEOUT_MS = 6000;

    @Override
    public void runOpMode() {
        Alliance alliance = alliance();
        RobotSys robot = new RobotSys(hardwareMap, alliance);

        Pose startPose = rot(start);
        robot.setPose(startPose.x(), startPose.y(), startPose.heading());

        while (opModeInInit()) {
            robot.initLoop();
            Pose pose = robot.pose();
            telemetry.addData("Alliance", robot.alliance());
            telemetry.addData("Start pose", "x %.1f  y %.1f  h %.0f deg",
                    pose.x(), pose.y(), Math.toDegrees(pose.heading()));
            telemetry.addData("Turret", robot.turretStatus());
            telemetry.addLine("Turret straight forward + robot on START before INIT");
            telemetry.update();
            idle();
        }
        if (isStopRequested()) return;

        AutoBlocks auto = new AutoBlocks(robot);   // resets the scheduler
        Command routine = buildRoutine(auto);
        auto.start(routine);

        while (opModeIsActive()) {
            auto.run();
            robot.addTelemetry(telemetry);
            auto.addAutoDebug(telemetry);
            telemetry.addData("Routine", auto.isFinished(routine) ? "FINISHED" : "running");
            telemetry.update();
        }
        robot.stop();
    }

    private Command buildRoutine(AutoBlocks a) {
        Pose start = rot(this.start);
        Pose floare = rot(this.floare);
        Pose hive2Start = rot(this.hive2Start);
        Pose hive2 = rot(this.hive2);
        Pose point3Start = rot(this.point3Start);
        Pose point3 = rot(this.point3);
        Pose point4Start = rot(this.point4Start);
        Pose point4Control1 = rot(this.point4Control1);
        Pose point4 = rot(this.point4);
        Pose point5Start = rot(this.point5Start);
        Pose point5 = rot(this.point5);
        Pose point6Start = rot(this.point6Start);
        Pose point6 = rot(this.point6);

        Path pFloare = a.line(start, floare);
        Path pHive2 = a.line(hive2Start, hive2);
        Path path3 = a.line(point3Start, point3);
        Path path4 = a.curve(point4Start, point4Control1, point4);
        Path path5 = a.line(point5Start, point5);
        Path path6 = a.line(point6Start, point6);

        return sequence(
                a.arm(),
                shootHere(a),
                a.hiveTipped(),
                a.intakeFeedOn(),
                a.follow(pFloare, LEG_TIMEOUT_MS),
                waitMs(1000),
                a.intakeFeedOff(),
                a.follow(pHive2, LEG_TIMEOUT_MS),
                shootHere(a),
                a.intakeFeed(2000),
                a.follow(path3, LEG_TIMEOUT_MS),
                shootHere(a),
                sequence(
                        parallel(
                                a.follow(path4, LEG_TIMEOUT_MS),
                                a.intakeFeedOn()
                        ),
                        waitMs(2000)
                ),
                a.hiveTipped(),
                a.follow(path5, LEG_TIMEOUT_MS),
                shootHere(a),
                a.follow(path6, LEG_TIMEOUT_MS),
                a.disarm(),
                a.stopAll());
    }

    private Command shootHere(AutoBlocks a) {
        return a.shootStrict(SHOOT_MS, 3000, 4000);
    }
}
