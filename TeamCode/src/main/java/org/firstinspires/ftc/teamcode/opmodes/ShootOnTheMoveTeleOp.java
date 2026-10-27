package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.config.ShooterConstants;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.robot.RobotSys;
import org.firstinspires.ftc.teamcode.subsystems.IntakeServo;


/**
 * Field-centric driving with an auto-aiming turret that shoots into the up CELL of our HIVE
 * while moving.
 *
 * <p>INIT: gamepad1 X = blue, B = red (skipped if AUTO already stored a pose). Without a stored
 * pose the robot must start in the alliance reset corner (see BioBuzzField.resetPose).
 * gamepad1 Y = home the turret (the turret moves to its zero; a zero homed in an earlier OpMode,
 * e.g. AUTO, is reused). gamepad1 A = move all servos to their start position (turret forward,
 * hood start, gate closed, intake servo INIT). Those two buttons are the only movement in INIT;
 * everything else stays still until START. Without a homed zero the turret points but never
 * shoots.
 *
 * <p>gamepad1 (driver): left stick = drive (field-centric), right stick X = turn,
 * right bumper = slow, right trigger = fire (fires only when ready; rumbles when the driving is
 * what blocks the shot), left trigger = FORCE fire (bypasses every check, including flywheel
 * armed and gamepad2 hold fire), left bumper = arm/disarm the flywheel, back = reset pose (robot
 * in the reset corner), A = intake on/off (it always runs during launches too), X held = intake
 * reverse (clears a jam),
 * dpad up = intake servo collect all, dpad left = collect pollen only, dpad down = back to init.
 *
 * <p>gamepad2 (operator): Y = our HIVE tipped (switch CELL), X = undo a TIP, B = next ball
 * POLLEN/NECTAR (only if SHOOTS_NECTAR), left bumper = hold fire, dpad left/right = aim trim -/+0.5 deg,
 * dpad up/down = distance trim +/-1 in, A = clear trims.
 */
@TeleOp(name = "Shoot On The Move", group = "BioBuzz")
public class ShootOnTheMoveTeleOp extends LinearOpMode {
    private static final double AIM_TRIM_STEP_DEG = 0.5;
    private static final double DISTANCE_TRIM_STEP_IN = 1.0;
    private static final int RUMBLE_MS = 150;
    /**
     * Telemetry is rebuilt and sent this often; the Driver Hub shows about 4 updates/s anyway.
     */
    private static final long TELEMETRY_PERIOD_NANOS = 100_000_000L;

    @Override
    public void runOpMode() {
        RobotSys robot = new RobotSys(hardwareMap, Alliance.RED);
        boolean fromAuto = robot.restorePoseFromStorage();
        if (!fromAuto) robot.resetPoseToCorner();

        while (opModeInInit()) {
//            if (!fromAuto) {
            Alliance chosen = gamepad1.x ? Alliance.BLUE : gamepad1.b ? Alliance.RED : robot.alliance();
            if (chosen != robot.alliance()) {
                robot.setAlliance(chosen);
                robot.resetPoseToCorner();
            }
//            }
            if (gamepad1.yWasPressed()) robot.homeTurret();
            if (gamepad1.aWasPressed()) robot.servosToStart();
            robot.initLoop();
            telemetry.addData("Alliance", "%s  %s", robot.alliance(),
                    fromAuto ? "(from previous OpMode)" : "(gamepad1 X = blue, B = red)");
            telemetry.addData("Start pose", fromAuto
                    ? "stored by previous OpMode (restart the app to clear)"
                    : "reset corner: place the robot there");
            telemetry.addData("Up CELL", robot.upCell());
            telemetry.addData("Turret (gamepad1 Y = home)", robot.turretStatus());
            telemetry.addData("Servos", "gamepad1 A = move all servos to their start position");
            telemetry.update();
            idle();
        }
        if (isStopRequested()) return;

        long nextRumbleNanos = 0L;
        long nextTelemetryNanos = 0L;
        long lastLoopNanos = 0L;
        double loopSumMs = 0.0;
        double loopWorstMs = 0.0;
        int loopCount = 0;

        while (opModeIsActive()) {
            // Loop time: full period from one loop start to the next.
            long loopStart = System.nanoTime();
            if (lastLoopNanos != 0L) {
                double ms = (loopStart - lastLoopNanos) * 1e-6;
                loopSumMs += ms;
                loopWorstMs = Math.max(loopWorstMs, ms);
                loopCount++;
            }
            lastLoopNanos = loopStart;

            // Drive: sticks are in the driver's frame (+forward away from our wall, +left).
            robot.drive(-gamepad1.left_stick_y, -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x);

            // Each WasPressed() call reports a press once, so read each button only once per loop.
            if (gamepad1.leftBumperWasPressed()) robot.toggleArmed();
            if (gamepad1.backWasPressed()) robot.resetPoseToCorner();
            if (gamepad1.aWasPressed()) robot.toggleIntake();
            robot.setIntakeReverse(gamepad1.x);
            if (gamepad1.dpadUpWasPressed()) robot.setIntakeMode(IntakeServo.Mode.COLLECT_ALL);
            if (gamepad1.dpadLeftWasPressed()) robot.setIntakeMode(IntakeServo.Mode.COLLECT_POLLEN);
            if (gamepad1.dpadDownWasPressed()) robot.setIntakeMode(IntakeServo.Mode.INIT);

            if (gamepad2.yWasPressed()) robot.reportHiveTip();
            if (gamepad2.xWasPressed()) robot.undoHiveTip();
            if (gamepad2.bWasPressed() && ShooterConstants.SHOOTS_NECTAR)
                robot.setNextBall(robot.nextBall().other());
            if (gamepad2.aWasPressed()) robot.clearTrims();
            if (gamepad2.dpadLeftWasPressed()) robot.addAimTrimDeg(-AIM_TRIM_STEP_DEG);
            if (gamepad2.dpadRightWasPressed()) robot.addAimTrimDeg(AIM_TRIM_STEP_DEG);
            if (gamepad2.dpadUpWasPressed()) robot.addDistanceTrimIn(DISTANCE_TRIM_STEP_IN);
            if (gamepad2.dpadDownWasPressed()) robot.addDistanceTrimIn(-DISTANCE_TRIM_STEP_IN);

            boolean fire = gamepad1.right_trigger > 0.5 && !gamepad2.left_bumper;
            // Force fire: feeds no matter what the checks say (even with the flywheel off).
            boolean force = gamepad1.left_trigger > 0.5;
            robot.update();
            robot.shoot(fire, force);

            // Tell the driver when a held shot is blocked by their driving (wrong side or angle to
            // the CELL, too fast): crossing the face of the CELL usually fixes it.
            long nowNanos = System.nanoTime();
            if (fire && robot.isArmed() && robot.isBlockedByDriving() && nowNanos >= nextRumbleNanos) {
                gamepad1.rumble(RUMBLE_MS);
                nextRumbleNanos = nowNanos + 2L * RUMBLE_MS * 1_000_000L;
            }

            // Building telemetry costs CPU every time, so only do it every TELEMETRY_PERIOD.
            if (loopStart >= nextTelemetryNanos) {
                nextTelemetryNanos = loopStart + TELEMETRY_PERIOD_NANOS;
                double averageMs = loopCount > 0 ? loopSumMs / loopCount : 0.0;
                telemetry.addData("Loop", "%.1f ms avg (%.0f Hz), worst %.1f ms",
                        averageMs, averageMs > 0.0 ? 1000.0 / averageMs : 0.0, loopWorstMs);
                loopSumMs = 0.0;
                loopWorstMs = 0.0;
                loopCount = 0;
                robot.addTelemetry(telemetry);
                telemetry.update();
            }
        }
        robot.stop();
    }
}
