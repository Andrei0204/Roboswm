package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.shooter.Alliance;
import org.firstinspires.ftc.teamcode.shooter.BioBuzzField;
import org.firstinspires.ftc.teamcode.shooter.PoseStorage;
import org.firstinspires.ftc.teamcode.shooter.ShootOnTheMove;
import org.firstinspires.ftc.teamcode.shooter.ShooterConstants;

import java.util.List;

/**
 * Field-centric driving with an auto-aiming turret that shoots into the up CELL of our HIVE
 * while moving.
 *
 * <p>INIT: gamepad1 X = blue, B = red (skipped if AUTO already stored a pose). Without a stored
 * pose the robot must start in the alliance reset corner (see BioBuzzField.resetPose).
 * gamepad1 Y = home the turret encoder (the turret moves to its zero). This is the only place
 * homing happens; a zero homed in an earlier OpMode (e.g. AUTO) is reused. Without a zero the
 * turret points but never shoots.
 *
 * <p>gamepad1 (driver): left stick = drive (field-centric), right stick X = turn,
 * right bumper = slow, right trigger = fire (fires only when ready; rumbles when the driving is
 * what blocks the shot), left trigger = FORCE fire (bypasses every check, including flywheel
 * armed and gamepad2 hold fire), left bumper = arm/disarm the flywheel, back = reset pose (robot
 * in the reset corner).
 *
 * <p>gamepad2 (operator): Y = our HIVE tipped (switch CELL), X = undo a TIP, B = next ball
 * POLLEN/NECTAR (only if SHOOTS_NECTAR), left bumper = hold fire, dpad left/right = aim trim -/+0.5 deg,
 * dpad up/down = distance trim +/-1 in, A = clear trims.
 */
@TeleOp(name = "Shoot On The Move", group = "BioBuzz")
public class ShootOnTheMoveTeleOp extends LinearOpMode {
    private static final double SLOW_SCALE = 0.4;
    private static final double AIM_TRIM_STEP_DEG = 0.5;
    private static final double DISTANCE_TRIM_STEP_IN = 1.0;
    private static final int RUMBLE_MS = 150;
    /** Telemetry is rebuilt and sent this often; the Driver Hub shows about 4 updates/s anyway. */
    private static final long TELEMETRY_PERIOD_NANOS = 100_000_000L;

    @Override
    public void runOpMode() {
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        Follower follower = Constants.create(hardwareMap);
        if (follower == null) {
            telemetry.addLine("Pedro is not configured: pedro/Constants.create() returned null.");
            telemetry.addLine("Run the Pedro tuners and fill in Constants.java first.");
            telemetry.update();
            waitForStart();
            return;
        }

        boolean fromAuto = PoseStorage.hasData;
        Alliance alliance = fromAuto ? PoseStorage.alliance : Alliance.RED;

        for (LynxModule hub : hubs) hub.clearBulkCache();
        ShootOnTheMove shooter = new ShootOnTheMove(hardwareMap, follower, alliance);
        if (fromAuto) {
            follower.setPose(new Pose(PoseStorage.x, PoseStorage.y, PoseStorage.heading));
            if (PoseStorage.upCell != null) shooter.setUpCell(PoseStorage.upCell);
        } else {
            follower.setPose(resetPose(alliance));
        }

        while (opModeInInit()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            if (!fromAuto) {
                Alliance chosen = gamepad1.x ? Alliance.BLUE : gamepad1.b ? Alliance.RED : alliance;
                if (chosen != alliance) {
                    alliance = chosen;
                    shooter.setAlliance(alliance);
                    follower.setPose(resetPose(alliance));
                }
            }
            if (gamepad1.yWasPressed()) shooter.homeTurret();
            shooter.initLoop();

            telemetry.addData("Alliance", "%s  %s", alliance,
                    fromAuto ? "(from previous OpMode)" : "(gamepad1 X = blue, B = red)");
            telemetry.addData("Start pose", fromAuto
                    ? "stored by previous OpMode (restart the app to clear)"
                    : "reset corner: place the robot there");
            telemetry.addData("Up CELL", shooter.upCell());
            telemetry.addData("Turret (gamepad1 Y = home)", shooter.turretStatus());
            telemetry.update();
            idle();
        }
        if (isStopRequested()) return;

        shooter.resetMotion();
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

            for (LynxModule hub : hubs) hub.clearBulkCache();

            // Drive: sticks are in the driver's frame (+forward away from our wall, +left), which
            // fieldCentric rotates into the robot frame.
            double scale = gamepad1.right_bumper ? SLOW_SCALE : 1.0;
            double forward = -gamepad1.left_stick_y * scale;
            double left = -gamepad1.left_stick_x * scale;
            double turn = -gamepad1.right_stick_x * scale;
            follower.manual(ManualDrive.fieldCentric(forward, left, turn,
                    follower.pose().heading(), BioBuzzField.driverHeadingOffset(alliance)));
            follower.update();

            // Each WasPressed() call reports a press once, so read each button only once per loop.
            if (gamepad1.leftBumperWasPressed()) shooter.toggleArmed();
            if (gamepad1.backWasPressed()) {
                follower.setPose(resetPose(alliance));
                shooter.resetMotion();
            }
            if (gamepad2.yWasPressed()) shooter.reportHiveTip();
            if (gamepad2.xWasPressed()) shooter.undoHiveTip();
            if (gamepad2.bWasPressed() && ShooterConstants.SHOOTS_NECTAR)
                shooter.setNextBall(shooter.nextBall().other());
            if (gamepad2.aWasPressed()) {
                shooter.solver().aimTrimRad = 0.0;
                shooter.solver().distanceTrimIn = 0.0;
            }
            if (gamepad2.dpadLeftWasPressed())
                shooter.solver().aimTrimRad -= Math.toRadians(AIM_TRIM_STEP_DEG);
            if (gamepad2.dpadRightWasPressed())
                shooter.solver().aimTrimRad += Math.toRadians(AIM_TRIM_STEP_DEG);
            if (gamepad2.dpadUpWasPressed())
                shooter.solver().distanceTrimIn += DISTANCE_TRIM_STEP_IN;
            if (gamepad2.dpadDownWasPressed())
                shooter.solver().distanceTrimIn -= DISTANCE_TRIM_STEP_IN;

            boolean fire = gamepad1.right_trigger > 0.5 && !gamepad2.left_bumper;
            // Force fire: feeds no matter what the checks say (even with the flywheel off).
            boolean force = gamepad1.left_trigger > 0.5;
            shooter.update(fire, force);
            // Tell the driver when a held shot is blocked by their driving (wrong side or angle to
            // the CELL, too fast): crossing the face of the CELL usually fixes it.
            long nowNanos = System.nanoTime();
            if (fire && shooter.isArmed() && shooter.isBlockedByDriving() && nowNanos >= nextRumbleNanos) {
                gamepad1.rumble(RUMBLE_MS);
                nextRumbleNanos = nowNanos + 2L * RUMBLE_MS * 1_000_000L;
            }

            Pose pose = follower.pose();
            PoseStorage.save(pose.x(), pose.y(), pose.heading(), alliance, shooter.upCell());

            // Building telemetry costs CPU every time, so only do it every TELEMETRY_PERIOD.
            if (loopStart >= nextTelemetryNanos) {
                nextTelemetryNanos = loopStart + TELEMETRY_PERIOD_NANOS;
                double averageMs = loopCount > 0 ? loopSumMs / loopCount : 0.0;
                telemetry.addData("Loop", "%.1f ms avg (%.0f Hz), worst %.1f ms",
                        averageMs, averageMs > 0.0 ? 1000.0 / averageMs : 0.0, loopWorstMs);
                loopSumMs = 0.0;
                loopWorstMs = 0.0;
                loopCount = 0;
                telemetry.addData("Pose", "x %.1f  y %.1f  h %.1f deg", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
                telemetry.addData("Trims", "aim %.1f deg, distance %.0f in",
                        Math.toDegrees(shooter.solver().aimTrimRad), shooter.solver().distanceTrimIn);
                shooter.addTelemetry(telemetry);
                telemetry.update();
            }
        }
        shooter.stop();
    }

    private static Pose resetPose(Alliance alliance) {
        double[] p = BioBuzzField.resetPose(alliance);
        return new Pose(p[0], p[1], p[2]);
    }
}
