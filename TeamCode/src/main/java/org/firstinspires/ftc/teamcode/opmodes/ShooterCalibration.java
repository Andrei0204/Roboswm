package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotSys;
import org.firstinspires.ftc.teamcode.shooter.Alliance;
import org.firstinspires.ftc.teamcode.shooter.BallType;
import org.firstinspires.ftc.teamcode.shooter.ShotSolution;
import org.firstinspires.ftc.teamcode.shooter.ShooterConstants;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Builds SHOT_TABLE_POLLEN and SHOT_TABLE_NECTAR. Park the robot STILL, head-on to the up CELL,
 * let the turret aim, and adjust rpm and hood until balls drop cleanly through the middle of the
 * opening. Arriving about 20-40 deg below horizontal works best, since the opening faces 30 deg
 * up. Then record the row. Repeat every 8-12 in across the range you can shoot from (about
 * 15-80 in), for each ball type. Measure each row's time of flight with slow-motion video
 * (frames from leaving the hood to crossing the opening, divided by the frame rate) and put it
 * in the 4th column.
 *
 * <p>INIT: gamepad1 X = blue, B = red, Y = home the turret (the only INIT movement, and only on this button; needed once per app session;
 * the turret will not shoot until it is homed).
 * <p>gamepad1: sticks = drive (field-centric), left bumper = arm/disarm flywheel,
 * right trigger = fire one ball (needs flywheel at speed, hood settled, turret aimed),
 * dpad up/down = rpm +/-50, dpad left/right = hood -/+0.01 (hold left trigger for fine steps),
 * Y = load the current table values for this distance, B = switch target CELL,
 * X = switch ball type (only if SHOOTS_NECTAR), A = record a row, back = reset pose (robot in the reset corner).
 */
@TeleOp(name = "Shooter Calibration", group = "BioBuzz")
public class ShooterCalibration extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotSys robot = new RobotSys(hardwareMap, Alliance.RED);
        boolean fromAuto = robot.restorePoseFromStorage();
        if (!fromAuto) robot.resetPoseToCorner();

        while (opModeInInit()) {
            if (!fromAuto) {
                Alliance chosen = gamepad1.x ? Alliance.BLUE : gamepad1.b ? Alliance.RED : robot.alliance();
                if (chosen != robot.alliance()) {
                    robot.setAlliance(chosen);
                    robot.resetPoseToCorner();
                }
            }
            if (gamepad1.yWasPressed()) robot.homeTurret();
            robot.initLoop();
            telemetry.addData("Alliance", robot.alliance());
            telemetry.addData("Start pose", fromAuto ? "stored by previous OpMode" : "reset corner");
            telemetry.addData("Turret (gamepad1 Y = home)", robot.turretStatus());
            telemetry.update();
            idle();
        }
        if (isStopRequested()) return;

        double rpm = 3000.0;
        double hood = 0.5;
        List<String> rows = new ArrayList<>();

        while (opModeIsActive()) {
            robot.drive(-gamepad1.left_stick_y * 0.5, -gamepad1.left_stick_x * 0.5, -gamepad1.right_stick_x * 0.5);

            boolean fine = gamepad1.left_trigger > 0.5;
            double rpmStep = fine ? 10.0 : 50.0;
            double hoodStep = fine ? 0.002 : 0.01;
            // Each WasPressed() call reports a press once, so read each button only once per loop.
            if (gamepad1.leftBumperWasPressed()) robot.toggleArmed();
            if (gamepad1.dpadUpWasPressed()) rpm += rpmStep;
            if (gamepad1.dpadDownWasPressed()) rpm = Math.max(0.0, rpm - rpmStep);
            if (gamepad1.dpadLeftWasPressed()) hood -= hoodStep;
            if (gamepad1.dpadRightWasPressed()) hood += hoodStep;
            hood = Math.max(ShooterConstants.HOOD_MIN_POSITION, Math.min(ShooterConstants.HOOD_MAX_POSITION, hood));
            if (gamepad1.bWasPressed()) robot.setUpCell(robot.upCell().other());
            if (gamepad1.xWasPressed() && ShooterConstants.SHOOTS_NECTAR) robot.setNextBall(robot.nextBall().other());
            if (gamepad1.backWasPressed()) robot.resetPoseToCorner();
            ShotSolution s = robot.solution();
            BallType ball = robot.nextBall();
            if (gamepad1.yWasPressed() && s != null) {
                rpm = robot.solver().rpmAt(s.realDistance, ball);
                hood = robot.solver().hoodAt(s.realDistance, ball);
            }
            if (gamepad1.aWasPressed() && s != null) {
                rows.add(String.format(Locale.US, "%s: {%.1f, %.0f, %.3f, TOF},", ball, s.realDistance, rpm, hood));
            }

            robot.setCalibrationOverride(rpm, hood);
            robot.update();
            robot.shoot(gamepad1.right_trigger > 0.5, false);

            s = robot.solution();
            ball = robot.nextBall();
            telemetry.addData("Ball (X to switch)", ball);
            telemetry.addData("Distance pivot->CELL", "%.1f in  (record while STILL)", s.realDistance);
            telemetry.addData("Override rpm / hood", "%.0f / %.3f", rpm, hood);
            telemetry.addData("Table at this distance", "%.0f rpm / %.3f hood / %.3f s",
                    robot.solver().rpmAt(s.realDistance, ball), robot.solver().hoodAt(s.realDistance, ball),
                    robot.solver().timeOfFlightAt(s.realDistance, ball));
            telemetry.addLine("Recorded rows (replace TOF with the video-measured value):");
            for (String row : rows) telemetry.addLine(row);
            robot.addTelemetry(telemetry);
            telemetry.update();
        }
        robot.stop();
    }
}
