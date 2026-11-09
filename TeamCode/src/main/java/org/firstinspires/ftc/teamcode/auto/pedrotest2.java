package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.auto.AutoBlocks.sequence;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.robot.RobotSys;

/** Same square path as before, but the robot shoots after every leg. */
@Autonomous(name = "AutoPathmuedinamo", group = "Autonomous")
public class pedrotest2 extends LinearOpMode {

    // x, y, heading (deg) - RED alliance
    private static final double[] START = {7.7243, 7.4648, -90};
    private static final double[] aruncare1 = {30.8679, 14.082, 90};
    private static final double[] POINT2 = {7.4243, 7.0048, -90};
    private static final double[] POINT3 = {30.8679, 14.082, 90};
    private static final double[] POINT4 = {118.4712, 24.5502, 90};

    /** How long to hold fire at each stop, and the max wait for the shot checks to pass. */
    private static final double SHOOT_MS = 2500;
    private static final double READY_TIMEOUT_MS = 1500;
    private static final double LEG_TIMEOUT_MS = 6000;

    @Override
    public void runOpMode() {
        RobotSys robot = new RobotSys(hardwareMap, Alliance.RED);

        Pose start = AutoBlocks.pose(Alliance.RED, START[0], START[1], START[2]);
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
        Pose start = a.pose(START[0], START[1], START[2]);
        Pose p1 = a.pose(aruncare1[0], aruncare1[1], aruncare1[2]);
        Pose p2 = a.pose(POINT2[0], POINT2[1], POINT2[2]);
        Pose p3 = a.pose(POINT3[0], POINT3[1], POINT3[2]);
        Pose p4 = a.pose(POINT4[0], POINT4[1], POINT4[2]);

        return sequence(
                a.arm(),   // flywheel spins up for the whole run
                a.follow(a.lineConstantHeading(start, p1), LEG_TIMEOUT_MS),
                shootHere(a),
                a.intakeOn(),
                a.follow(a.lineConstantHeading(p1, p2), LEG_TIMEOUT_MS),
                shootHere(a),
                a.follow(a.lineConstantHeading(p2, p3), LEG_TIMEOUT_MS),
                shootHere(a),
                a.follow(a.lineConstantHeading(p3, p4), LEG_TIMEOUT_MS),
                shootHere(a),
                a.disarm(),
                a.stopAll());
    }

    /** The shoot command: wait until ready, then fire. */
    private Command shootHere(AutoBlocks a) {
        return sequence(
                a.waitReady(READY_TIMEOUT_MS),
                a.shoot(SHOOT_MS));
    }
}
