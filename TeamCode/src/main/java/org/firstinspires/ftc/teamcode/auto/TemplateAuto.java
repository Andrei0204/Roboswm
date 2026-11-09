package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.robot.RobotSys;
import org.firstinspires.ftc.teamcode.subsystems.IntakeServo;

import static org.firstinspires.ftc.teamcode.auto.AutoBlocks.sequence;
import static org.firstinspires.ftc.teamcode.auto.AutoBlocks.whileDoing;

/**
 * TEMPLATE AUTO: copy this class, rename it and edit {@link #buildRoutine}. Every block used here
 * is explained in docs/auto.md. All poses are written for RED; BLUE is rotated automatically.
 *
 * <p>INIT: gamepad1 X = blue, B = red, Y = reset the turret encoder (turret straight forward),
 * A = servos to start position. Place the robot on START_* before pressing START.
 *
 * <p>The numbers below are PLACEHOLDERS: MEASURE them on the field before running.
 */
@Autonomous(name = "Template Auto", group = "BioBuzz", preselectTeleOp = "Shoot On The Move")
public class TemplateAuto extends LinearOpMode {
    // Start pose (x in, y in, heading deg) and the spots the routine uses. MEASURE (placeholders).
    private static final double[] START = {9.0, 9.0, 0.0};
    private static final double[] SHOOT_SPOT = {40.0, 40.0, 45.0};
    private static final double[] PICKUP_START = {48.0, 36.0, 0.0};
    private static final double[] PICKUP_END = {48.0, 12.0, 0.0};
    private static final double[] PARK_SPOT = {20.0, 20.0, 0.0};

    @Override
    public void runOpMode() {
        RobotSys robot = new RobotSys(hardwareMap, Alliance.RED);

        // ---------------------------------------------------------------- INIT
        Alliance posed = null;
        while (opModeInInit()) {
            Alliance chosen = gamepad1.x ? Alliance.BLUE : gamepad1.b ? Alliance.RED : robot.alliance();
            // The start pose depends on the alliance: set it on the first loop and on every change.
            if (chosen != posed) {
                robot.setAlliance(chosen);
                Pose start = AutoBlocks.pose(chosen, START[0], START[1], START[2]);
                robot.setPose(start.x(), start.y(), start.heading());
                posed = chosen;
            }
            if (gamepad1.yWasPressed()) robot.homeTurret();
            if (gamepad1.aWasPressed()) robot.servosToStart();
            robot.initLoop();
            Pose pose = robot.pose();
            telemetry.addData("Alliance", "%s  (gamepad1 X = blue, B = red)", robot.alliance());
            telemetry.addData("Start pose", "x %.1f  y %.1f  h %.0f deg", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
            telemetry.addData("Turret (gamepad1 Y = home)", robot.turretStatus());
            telemetry.update();
            idle();
        }
        if (isStopRequested()) return;

        // ---------------------------------------------------------------- RUN
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

    /** The routine. Read it top to bottom: it runs in that order. */
    private Command buildRoutine(AutoBlocks a) {
        Pose shoot = a.pose(SHOOT_SPOT[0], SHOOT_SPOT[1], SHOOT_SPOT[2]);
        Pose pickupStart = a.pose(PICKUP_START[0], PICKUP_START[1], PICKUP_START[2]);
        Pose pickupEnd = a.pose(PICKUP_END[0], PICKUP_END[1], PICKUP_END[2]);
        Pose park = a.pose(PARK_SPOT[0], PARK_SPOT[1], PARK_SPOT[2]);
        Pose start = a.pose(START[0], START[1], START[2]);

        // Paths are built once, up front. A path starts where the robot IS; use a.driveTo(pose)
        // when you do not want to define the start.
        Path toShoot = a.line(start, shoot);
        Path toPickup = a.curve(shoot, a.pose(46.0, 44.0, 0.0), pickupStart);
        Path sweep = a.lineConstantHeading(pickupStart, pickupEnd);   // slow pass over the balls
        Path backToShoot = a.line(pickupEnd, shoot);

        return sequence(
                // 1. Spin up the flywheel and drive to the shooting spot at the same time.
                AutoBlocks.together(
                        a.arm(),
                        a.follow(toShoot, 4000)),

                // 2. Shoot the preload: wait (max 1.5 s) until every check passes, then fire 3 s.
                a.waitReady(1500),
                a.shoot(3000),

                // 3. Go to the balls and collect while sweeping across them.
                a.follow(toPickup, 4000),
                whileDoing(
                        a.follow(sweep, 3000),
                        a.collect(IntakeServo.Mode.COLLECT_ALL)),
                a.stopCollecting(),

                // 4. Back to the shooting spot, shoot again.
                a.follow(backToShoot, 4000),
                a.waitReady(1500),
                a.shoot(3000),

                // 5. Park, flywheel off, everything off.
                a.disarm(),
                a.driveTo(park, 4000),
                a.hold(),
                a.stopAll());
    }
}
