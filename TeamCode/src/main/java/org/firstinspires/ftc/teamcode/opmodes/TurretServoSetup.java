package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.shooter.TurretMath;

import java.util.Locale;

/**
 * Calibrates the servo turret and its external encoder (no Pedro needed).
 *
 * <ol>
 *   <li>Jog the servo (dpad left/right; hold right bumper for fine steps) until the turret
 *       shoots straight forward, and press A.</li>
 *   <li>Jog until it points exactly 90 deg to the robot's LEFT (counter-clockwise seen from
 *       above), and press B.</li>
 *   <li>Paste the printed TURRET_SERVO_POSITION_AT_ZERO, TURRET_DEG_PER_SERVO_UNIT,
 *       TURRET_ENCODER_SIGN and TURRET_ENCODER_REVS_PER_TURRET_REV into ShooterConstants, and
 *       restart this OpMode.</li>
 *   <li>Jog to each end of safe travel (cables, frame) and use the encoder angle for
 *       TURRET_MIN_DEG / TURRET_MAX_DEG.</li>
 *   <li>Y sweeps between the limits: the encoder-measured span should match the expected span.
 *       If not, redo A/B.</li>
 * </ol>
 * dpad up = go to the configured zero, dpad down = go to the servo center (0.5).
 */
@TeleOp(name = "Turret Servo Setup", group = "BioBuzz")
public class TurretServoSetup extends LinearOpMode {
    private static final double COARSE_STEP = 0.01;
    private static final double FINE_STEP = 0.001;
    private static final double SWEEP_SETTLE_SEC = 1.5;
    private static final double SWEEP_TIMEOUT_SEC = 3.0;
    private static final double STILL_DEG_PER_SEC = 3.0;

    @Override
    public void runOpMode() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        String[] names = ShooterConstants.TURRET_SERVO_NAMES;
        Servo[] servos = new Servo[names.length];
        for (int i = 0; i < names.length; i++) {
            Servo servo = hardwareMap.get(Servo.class, names[i]);
            servo.setDirection(ShooterConstants.TURRET_SERVO_REVERSED[i] ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
            if (ShooterConstants.TURRET_SET_PWM_RANGE && servo instanceof PwmControl) {
                ((PwmControl) servo).setPwmRange(new PwmControl.PwmRange(
                        ShooterConstants.TURRET_PWM_MIN_US, ShooterConstants.TURRET_PWM_MAX_US));
            }
            servos[i] = servo;
        }
        DcMotorEx encoder = hardwareMap.get(DcMotorEx.class, ShooterConstants.TURRET_ENCODER_NAME);

        telemetry.addLine("Press START: the turret moves to the configured zero position.");
        telemetry.update();
        waitForStart();

        double position = clip(ShooterConstants.TURRET_SERVO_POSITION_AT_ZERO);
        double posA = Double.NaN, posB = Double.NaN;
        int ticksA = 0, ticksB = 0;
        // Sweep: 0 = idle, 1 = settling at the low end, 2 = moving to the high end.
        int sweep = 0;
        double sweepStart = 0.0;
        double stillFor = 0.0;
        double lastTime = getRuntime();
        int sweepTicks0 = 0;
        String sweepResult = "not run";

        while (opModeIsActive()) {
            double now = getRuntime();
            double dt = now - lastTime;
            lastTime = now;
            int ticks = encoder.getCurrentPosition();
            double ticksPerSec = encoder.getVelocity();
            double degPerTick = Math.toDegrees(TurretMath.radiansPerTick());

            // Read every press once per loop, even when it is ignored, so none fires later.
            boolean left = gamepad1.dpadLeftWasPressed();
            boolean right = gamepad1.dpadRightWasPressed();
            boolean up = gamepad1.dpadUpWasPressed();
            boolean down = gamepad1.dpadDownWasPressed();
            boolean pressA = gamepad1.aWasPressed();
            boolean pressB = gamepad1.bWasPressed();
            boolean pressY = gamepad1.yWasPressed();

            double step = gamepad1.right_bumper ? FINE_STEP : COARSE_STEP;
            if (sweep == 0) {
                if (left) position = clip(position - step);
                if (right) position = clip(position + step);
                if (up) position = clip(ShooterConstants.TURRET_SERVO_POSITION_AT_ZERO);
                if (down) position = 0.5;
                if (pressA) { posA = position; ticksA = ticks; }
                if (pressB) { posB = position; ticksB = ticks; }
            }

            double minPos = clip(TurretMath.positionForAngle(TurretMath.minAngle()));
            double maxPos = clip(TurretMath.positionForAngle(TurretMath.maxAngle()));
            if (pressY && sweep == 0) {
                sweep = 1;
                sweepStart = now;
                position = minPos;
            } else if (sweep == 1 && now - sweepStart >= SWEEP_SETTLE_SEC) {
                sweep = 2;
                sweepStart = now;
                stillFor = 0.0;
                sweepTicks0 = ticks;
                position = maxPos;
            } else if (sweep == 2) {
                stillFor = Math.abs(ticksPerSec * degPerTick) <= STILL_DEG_PER_SEC ? stillFor + dt : 0.0;
                double elapsed = now - sweepStart;
                if ((elapsed > 0.3 && stillFor >= 0.2) || elapsed > SWEEP_TIMEOUT_SEC) {
                    double measured = (ticks - sweepTicks0) * degPerTick;
                    double expected = Math.toDegrees(TurretMath.maxAngle() - TurretMath.minAngle());
                    sweepResult = String.format(Locale.US, "encoder %.1f deg vs expected %.1f deg (%.1f%%), %.2f s",
                            measured, expected, 100.0 * measured / expected, elapsed - stillFor);
                    sweep = 0;
                }
            }

            for (Servo servo : servos) servo.setPosition(position);

            telemetry.addData("Servo position", "%.4f", position);
            telemetry.addData("Servo-map angle (current constants)", "%.1f deg",
                    Math.toDegrees(TurretMath.angleForPosition(position)));
            telemetry.addData("Encoder", "%d ticks, %.0f deg/s", ticks, ticksPerSec * degPerTick);
            if (!Double.isNaN(posA)) {
                telemetry.addData("Encoder angle from A (current constants)", "%.1f deg", (ticks - ticksA) * degPerTick);
            }
            telemetry.addData("Reachable range (current constants)", "%.1f .. %.1f deg",
                    Math.toDegrees(TurretMath.minAngle()), Math.toDegrees(TurretMath.maxAngle()));
            telemetry.addData("A: straight forward", Double.isNaN(posA) ? "press A"
                    : String.format(Locale.US, "pos %.4f, %d ticks", posA, ticksA));
            telemetry.addData("B: 90 deg left", Double.isNaN(posB) ? "press B"
                    : String.format(Locale.US, "pos %.4f, %d ticks", posB, ticksB));
            if (!Double.isNaN(posA) && !Double.isNaN(posB) && Math.abs(posB - posA) > 1e-3 && ticksB != ticksA) {
                int dTicks = ticksB - ticksA;
                double ticksPerTurretRev = Math.abs(dTicks) * 4.0;
                telemetry.addLine("Paste into ShooterConstants:");
                telemetry.addLine(String.format(Locale.US, "TURRET_SERVO_POSITION_AT_ZERO = %.4f;", posA));
                telemetry.addLine(String.format(Locale.US, "TURRET_DEG_PER_SERVO_UNIT = %.2f;", 90.0 / (posB - posA)));
                telemetry.addLine(String.format(Locale.US, "TURRET_ENCODER_SIGN = %d;", dTicks > 0 ? 1 : -1));
                telemetry.addLine(String.format(Locale.US, "TURRET_ENCODER_REVS_PER_TURRET_REV = %.3f;",
                        ticksPerTurretRev / ShooterConstants.TURRET_ENCODER_TICKS_PER_REV));
                telemetry.addData("Encoder mount check", "1.000 = turret shaft, 2.923 = 24T/26T shaft, 1.169 = servo shaft");
                telemetry.addData("Servo scale check", "%.1f deg/unit expected from 350 deg x %.3f",
                        350.0 * ShooterConstants.TURRET_GEAR_RATIO, ShooterConstants.TURRET_GEAR_RATIO);
            }
            telemetry.addData("Y sweep", sweep == 0 ? sweepResult : "running...");
            telemetry.update();
        }
    }

    private static double clip(double position) {
        return Math.max(0.0, Math.min(1.0, position));
    }
}
