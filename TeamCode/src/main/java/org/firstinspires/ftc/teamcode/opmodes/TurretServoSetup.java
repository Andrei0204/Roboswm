package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Locale;
import org.firstinspires.ftc.teamcode.config.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.TurretMath;

/**
 * Calibrates the continuous-rotation servo turret and its external encoder (no Pedro needed).
 *
 * <p>The encoder count resets when an OpMode ends, so this OpMode assumes 0 ticks = 0 deg at the
 * start (turret straight forward). Gamepad1 X resets the encoder to 0 at any time, in INIT or
 * after START.
 *
 * <ol>
 *   <li>Hold dpad left / right to turn the turret (right bumper = slow). Dpad LEFT must turn it
 *       counter-clockwise seen from above; if it turns clockwise, flip that servo in
 *       TURRET_SERVO_REVERSED.</li>
 *   <li>Turn it to shoot straight forward and press A.</li>
 *   <li>Turn it exactly 90 deg to the robot's LEFT (counter-clockwise) and press B.</li>
 *   <li>Paste the printed TURRET_ENCODER_SIGN and TURRET_ENCODER_REVS_PER_TURRET_REV into
 *       ShooterConstants, and restart this OpMode.</li>
 *   <li>Turn to each end of safe travel (cables, frame): press A at forward first, then read
 *       "Encoder angle from A" for TURRET_MIN_DEG / TURRET_MAX_DEG.</li>
 * </ol>
 */
@TeleOp(name = "Turret Servo Setup", group = "BioBuzz")
public class TurretServoSetup extends LinearOpMode {
    private static final double JOG_POWER = 0.4;
    private static final double FINE_JOG_POWER = 0.12;

    @Override
    public void runOpMode() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        String[] names = ShooterConstants.TURRET_SERVO_NAMES;
        Servo[] servos = new Servo[names.length];
        for (int i = 0; i < names.length; i++) {
            servos[i] = hardwareMap.get(Servo.class, names[i]);
            servos[i].setDirection(ShooterConstants.TURRET_SERVO_REVERSED[i]
                    ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        }
        DcMotorEx encoder = hardwareMap.get(DcMotorEx.class, ShooterConstants.TURRET_ENCODER_NAME);

        // INIT: X zeroes the encoder where the turret is now. The servos stay still.
        while (opModeInInit()) {
            if (gamepad1.xWasPressed()) resetEncoder(encoder);
            telemetry.addLine("Turret straight forward = 0 ticks. Press X to reset the encoder here.");
            telemetry.addData("Encoder", "%d ticks", encoder.getCurrentPosition());
            telemetry.addLine("Press START, then hold dpad left / right to turn the turret.");
            telemetry.update();
            idle();
        }
        if (isStopRequested()) return;

        int ticksA = 0, ticksB = 0;
        boolean haveA = false, haveB = false;

        while (opModeIsActive()) {
            int ticks = encoder.getCurrentPosition();
            double ticksPerSec = encoder.getVelocity();
            double degPerTick = Math.toDegrees(TurretMath.radiansPerTick());

            if (gamepad1.xWasPressed()) {
                resetEncoder(encoder);
                haveA = false;
                haveB = false;
                continue;
            }
            if (gamepad1.aWasPressed()) { ticksA = ticks; haveA = true; }
            if (gamepad1.bWasPressed()) { ticksB = ticks; haveB = true; }

            double jog = gamepad1.right_bumper ? FINE_JOG_POWER : JOG_POWER;
            double power = gamepad1.dpad_left ? jog : (gamepad1.dpad_right ? -jog : 0.0);
            for (Servo servo : servos) servo.setPosition(0.5 + 0.5 * power);

            telemetry.addData("Power (dpad left = + = counter-clockwise)", "%.2f", power);
            telemetry.addData("Encoder (X = reset to 0)", "%d ticks, %.0f deg/s", ticks, ticksPerSec * degPerTick);
            telemetry.addData("Encoder angle (0 ticks = 0 deg)", "%.1f deg", ticks * degPerTick);
            if (haveA) {
                telemetry.addData("Encoder angle from A (current constants)", "%.1f deg", (ticks - ticksA) * degPerTick);
            }
            telemetry.addData("Soft limits (current constants)", "%.1f .. %.1f deg",
                    Math.toDegrees(TurretMath.minAngle()), Math.toDegrees(TurretMath.maxAngle()));
            telemetry.addData("A: straight forward", haveA ? String.format(Locale.US, "%d ticks", ticksA) : "press A");
            telemetry.addData("B: 90 deg left", haveB ? String.format(Locale.US, "%d ticks", ticksB) : "press B");
            if (haveA && haveB && ticksB != ticksA) {
                int dTicks = ticksB - ticksA;
                double ticksPerTurretRev = Math.abs(dTicks) * 4.0;
                telemetry.addLine("Paste into ShooterConstants:");
                telemetry.addLine(String.format(Locale.US, "TURRET_ENCODER_SIGN = %d;", dTicks > 0 ? 1 : -1));
                telemetry.addLine(String.format(Locale.US, "TURRET_ENCODER_REVS_PER_TURRET_REV = %.3f;",
                        ticksPerTurretRev / ShooterConstants.TURRET_ENCODER_TICKS_PER_REV));
                telemetry.addData("Encoder mount check", "1.000 = turret shaft, 2.923 = 24T/26T shaft, 1.169 = servo shaft");
            }
            telemetry.update();
        }
        for (Servo servo : servos) servo.setPosition(0.5);
    }

    private static void resetEncoder(DcMotorEx encoder) {
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
