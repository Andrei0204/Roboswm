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
public class douahiveuriparcare extends LinearOpMode {

    // x, y, heading (deg) - RED alliance
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(43.7639, 8.622, 90);
    private final Pose intakegarden = poseFactory.of(10.5183, 8.1237, 180);
    private final Pose shoothiveback1Start = poseFactory.of(10.5183, 8.1237, 180);
    private final Pose shoothiveback1 = poseFactory.of(41.5223, 126.6917, -90);
    private final Pose colectflowershootStart = poseFactory.of(41.5223, 126.6917, -90);
    private final Pose colectflowershoot = poseFactory.of(56.7383, 128.6941, 130);
    private final Pose colectflowershootControl1 = poseFactory.of(51.3265, 111.9123, 0);
    private final Pose parkStart = poseFactory.of(56.7383, 128.6941, 130);
    private final Pose park = poseFactory.of(16.6557, 107.3155, 0);



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
                a.intakeOn(),
                a.follow(intakegarden(), LEG_TIMEOUT_MS),
                a.follow(shoothiveback1(), LEG_TIMEOUT_MS),
                shootHere(a),
                parallel(
                        a.follow(colectflowershoot(), LEG_TIMEOUT_MS),
                        deploy(a)
                ),
                shootHere(a),
                a.follow(park(), LEG_TIMEOUT_MS),
                a.disarm(),
                a.stopAll());
    }

    /** The shoot command: wait until ready, then fire. */
    private Command shootHere(AutoBlocks a) {
        return sequence(
                a.waitReady(READY_TIMEOUT_MS),
                a.shoot(SHOOT_MS));
    }


    private Command deploy(AutoBlocks a) {
        return   sequence(
                waitMs(400),          // suntem si noi smekeri ciumegi
                a.intakeMode(IntakeServo.Mode.COLLECT_FLOARE)
                );
    }
    public Path intakegarden() {
        return Paths.line(start, intakegarden).constant(intakegarden);
    }

    public Path shoothiveback1() {
        return Paths.line(shoothiveback1Start, shoothiveback1).linear(shoothiveback1Start, shoothiveback1);
    }

    public Path colectflowershoot() {
        return Paths.curve(colectflowershootStart, colectflowershootControl1, colectflowershoot).linear(colectflowershootStart, colectflowershoot);
    }

    public Path park() {
        return Paths.line(parkStart, park).linear(parkStart, park);
    }
}
