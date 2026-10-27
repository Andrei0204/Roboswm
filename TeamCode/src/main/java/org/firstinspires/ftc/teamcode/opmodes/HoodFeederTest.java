package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.shooter.Feeder;
import org.firstinspires.ftc.teamcode.shooter.ShooterConstants;

import java.util.Locale;

/**
 * Finds the hood limits and checks the feeder (no Pedro needed). See TUNING.md, step 5.
 *
 * <p>HOOD: dpad left/right = position -/+0.01 (hold right bumper for 0.002). The current limits
 * are NOT applied here, so move in small steps and stop before the hood touches a hard stop.
 * X = record this as one end, Y = record the other end; paste the printed lines. A = swing
 * between the two recorded ends: film it and time one full swing for HOOD_SECONDS_PER_FULL_RANGE.
 *
 * <p>FEEDER (no balls loaded): hold right trigger = feed direction, hold left trigger = reverse.
 * Balls must move toward the flywheel with the right trigger; if not, make FEEDER_FIRE_POWER
 * negative.
 */
@TeleOp(name = "Hood & Feeder Test", group = "BioBuzz")
public class HoodFeederTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Servo hood = hardwareMap.get(Servo.class, ShooterConstants.HOOD_SERVO_NAME);
        if (ShooterConstants.HOOD_SET_PWM_RANGE && hood instanceof PwmControl) {
            ((PwmControl) hood).setPwmRange(new PwmControl.PwmRange(
                    ShooterConstants.HOOD_PWM_MIN_US, ShooterConstants.HOOD_PWM_MAX_US));
        }
        Feeder feeder = new Feeder(hardwareMap);

        telemetry.addLine("Press START: the hood moves to the middle of its current limits.");
        telemetry.update();
        waitForStart();

        double position = 0.5 * (ShooterConstants.HOOD_MIN_POSITION + ShooterConstants.HOOD_MAX_POSITION);
        double endX = Double.NaN, endY = Double.NaN;
        boolean swingToY = true;
        double swingStart = Double.NaN;

        while (opModeIsActive()) {
            double step = gamepad1.right_bumper ? 0.002 : 0.01;
            if (gamepad1.dpadLeftWasPressed()) position = Math.max(0.0, position - step);
            if (gamepad1.dpadRightWasPressed()) position = Math.min(1.0, position + step);
            if (gamepad1.xWasPressed()) endX = position;
            if (gamepad1.yWasPressed()) endY = position;
            // Read A even when it cannot act yet, so an early press does not swing the hood later.
            if (gamepad1.aWasPressed() && !Double.isNaN(endX) && !Double.isNaN(endY)) {
                position = swingToY ? endY : endX;
                swingToY = !swingToY;
                swingStart = getRuntime();
            }
            hood.setPosition(position);

            double feed = gamepad1.right_trigger > 0.5 ? ShooterConstants.FEEDER_FIRE_POWER
                    : gamepad1.left_trigger > 0.5 ? -ShooterConstants.FEEDER_FIRE_POWER : Double.NaN;
            if (Double.isNaN(feed)) {
                feeder.set(false);
            } else {
                feeder.setRawPower(feed);
            }

            telemetry.addData("Hood position", "%.3f", position);
            telemetry.addData("X end", Double.isNaN(endX) ? "press X" : String.format(Locale.US, "%.3f", endX));
            telemetry.addData("Y end", Double.isNaN(endY) ? "press Y" : String.format(Locale.US, "%.3f", endY));
            if (!Double.isNaN(endX) && !Double.isNaN(endY)) {
                telemetry.addLine("Paste into ShooterConstants:");
                telemetry.addLine(String.format(Locale.US, "HOOD_MIN_POSITION = %.3f;", Math.min(endX, endY)));
                telemetry.addLine(String.format(Locale.US, "HOOD_MAX_POSITION = %.3f;", Math.max(endX, endY)));
                telemetry.addData("A: swing", Double.isNaN(swingStart) ? "film it and time one swing"
                        : String.format(Locale.US, "commanded %.1f s ago", getRuntime() - swingStart));
            }
            telemetry.addData("Feeder", Double.isNaN(feed) ? "idle (RT = feed, LT = reverse)"
                    : String.format(Locale.US, "power %.2f", feed));
            telemetry.update();
        }
        feeder.stop();
    }
}
