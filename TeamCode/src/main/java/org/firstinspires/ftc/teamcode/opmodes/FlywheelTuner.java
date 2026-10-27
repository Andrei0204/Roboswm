package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.shooter.Feeder;
import org.firstinspires.ftc.teamcode.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.shooter.ShooterConstants;

import java.util.List;
import java.util.Locale;

/**
 * Tunes the flywheel (no Pedro needed). See TUNING.md, step 4.
 *
 * <p>OPEN LOOP (finds FLYWHEEL_KS and FLYWHEEL_KV): dpad up/down = power +/-0.05 (hold right
 * bumper for 0.01). Set a low power (about 0.3), wait until the rpm is steady, press X. Set a high
 * power (about 0.8), wait, press Y. Paste the printed lines. The fit is normalized to
 * NOMINAL_VOLTAGE, so the battery level does not matter.
 *
 * <p>CLOSED LOOP (checks FLYWHEEL_KP, FLYWHEEL_BANG_BANG_RPM and the tolerances): dpad up/down =
 * target +/-100 rpm (right bumper: 25). Right trigger feeds one ball. The rpm dip and the time to
 * get back inside the "at speed" window are shown for each shot.
 *
 * <p>A = switch mode, B = stop/run.
 */
@TeleOp(name = "Flywheel Tuner", group = "BioBuzz")
public class FlywheelTuner extends LinearOpMode {
    @Override
    public void runOpMode() {
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        Flywheel flywheel = new Flywheel(hardwareMap);
        Feeder feeder = new Feeder(hardwareMap);
        List<VoltageSensor> voltageSensors = hardwareMap.getAll(VoltageSensor.class);

        telemetry.addLine("Flywheel Tuner: START, then follow the on-screen help.");
        telemetry.addLine("Keep hands clear of the flywheel.");
        telemetry.update();
        waitForStart();

        boolean closedLoop = false;
        boolean running = true;
        double power = 0.3;
        double targetRpm = 3000.0;
        double voltage = ShooterConstants.NOMINAL_VOLTAGE;
        double nextVoltageRead = 0.0;
        double p1 = Double.NaN, rpm1 = 0, p2 = Double.NaN, rpm2 = 0;
        boolean feeding = false;
        double feedEnd = 0.0;
        double shotTime = Double.NaN, minRpm = 0.0;
        boolean dipped = false;
        String lastShot = "fire with the right trigger";

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            double now = getRuntime();
            if (now >= nextVoltageRead) {
                nextVoltageRead = now + ShooterConstants.VOLTAGE_READ_PERIOD_SEC;
                double lowest = Double.POSITIVE_INFINITY;
                for (VoltageSensor sensor : voltageSensors) {
                    double v = sensor.getVoltage();
                    if (v > 0.0) lowest = Math.min(lowest, v);
                }
                if (lowest != Double.POSITIVE_INFINITY) voltage = lowest;
            }
            flywheel.readSensors();
            double rpm = flywheel.measuredRpm();

            // Read every press once per loop, even in the mode that ignores it, so none fires later.
            boolean up = gamepad1.dpadUpWasPressed();
            boolean down = gamepad1.dpadDownWasPressed();
            boolean pressX = gamepad1.xWasPressed();
            boolean pressY = gamepad1.yWasPressed();
            boolean fired = gamepad1.rightTriggerWasPressed();
            if (gamepad1.aWasPressed()) closedLoop = !closedLoop;
            if (gamepad1.bWasPressed()) running = !running;
            boolean fine = gamepad1.right_bumper;
            if (!closedLoop) {
                double step = fine ? 0.01 : 0.05;
                if (up) power = Math.min(1.0, power + step);
                if (down) power = Math.max(0.0, power - step);
                if (pressX) { p1 = power * voltage / ShooterConstants.NOMINAL_VOLTAGE; rpm1 = rpm; }
                if (pressY) { p2 = power * voltage / ShooterConstants.NOMINAL_VOLTAGE; rpm2 = rpm; }
                flywheel.setOpenLoopPower(running ? power : 0.0);
            } else {
                double step = fine ? 25.0 : 100.0;
                if (up) targetRpm += step;
                if (down) targetRpm = Math.max(0.0, targetRpm - step);
                flywheel.setTargetRpm(running ? targetRpm : 0.0);
                flywheel.update(ShooterConstants.NOMINAL_VOLTAGE / Math.max(9.0, voltage));
            }

            // Feed one ball (or feed while held if FEEDER_PULSE_SEC <= 0) and time the recovery.
            double pulse = ShooterConstants.FEEDER_PULSE_SEC;
            if (pulse > 0.0) {
                if (feeding && now >= feedEnd) feeding = false;
                if (fired && !feeding) {
                    feeding = true;
                    feedEnd = now + pulse;
                }
            } else {
                feeding = gamepad1.right_trigger > 0.5;
            }
            if (fired) {
                shotTime = now;
                minRpm = rpm;
                dipped = false;
                lastShot = "measuring...";
            }
            feeder.set(feeding);
            if (!Double.isNaN(shotTime)) {
                minRpm = Math.min(minRpm, rpm);
                double target = flywheel.targetRpm();
                if (target > 0.0 && rpm < target - ShooterConstants.FLYWHEEL_TOLERANCE_BELOW_RPM) dipped = true;
                if (dipped && flywheel.atSpeed()) {
                    lastShot = String.format(Locale.US, "dip to %.0f rpm (%.0f below target), back at speed after %.0f ms",
                            minRpm, target - minRpm, (now - shotTime) * 1000.0);
                    shotTime = Double.NaN;
                } else if (now - shotTime > 2.0) {
                    lastShot = dipped
                            ? String.format(Locale.US, "dip to %.0f rpm, NOT back at speed after 2 s: raise KP/KV", minRpm)
                            : "no dip seen (no ball fed, or closed loop off)";
                    shotTime = Double.NaN;
                }
            }


            telemetry.addData("Mode (A)", closedLoop ? "CLOSED LOOP" : "OPEN LOOP");
            telemetry.addData("Running (B)", running);
            telemetry.addData("Battery", "%.2f V", voltage);
            telemetry.addData("Measured", "%.0f rpm, power %.3f", rpm, flywheel.power());
            if (!closedLoop) {
                telemetry.addData("Open-loop power", "%.2f (dpad)", power);
                telemetry.addData("X: low point", Double.isNaN(p1) ? "press X at ~0.3 when steady"
                        : String.format(Locale.US, "%.3f (at 12 V) -> %.0f rpm", p1, rpm1));
                telemetry.addData("Y: high point", Double.isNaN(p2) ? "press Y at ~0.8 when steady"
                        : String.format(Locale.US, "%.3f (at 12 V) -> %.0f rpm", p2, rpm2));
                if (!Double.isNaN(p1) && !Double.isNaN(p2) && Math.abs(rpm2 - rpm1) > 100.0) {
                    double kV = (p2 - p1) / (rpm2 - rpm1);
                    double kS = p1 - kV * rpm1;
                    telemetry.addLine("Paste into ShooterConstants:");
                    telemetry.addLine(String.format(Locale.US, "FLYWHEEL_KV = %.8f;", kV));
                    telemetry.addLine(String.format(Locale.US, "FLYWHEEL_KS = %.4f;", kS));
                    telemetry.addData("Top speed at 12 V", "about %.0f rpm (keep shot rpm under ~85%% of this)",
                            (1.0 - kS) / kV);
                }
            } else {
                telemetry.addData("Target (dpad)", "%.0f rpm, at speed: %b", targetRpm, flywheel.atSpeed());
                telemetry.addData("Window", "-%.0f / +%.0f rpm",
                        ShooterConstants.FLYWHEEL_TOLERANCE_BELOW_RPM, ShooterConstants.FLYWHEEL_TOLERANCE_ABOVE_RPM);
                telemetry.addData("Last shot", lastShot);
            }
            telemetry.update();
        }
        flywheel.stop();
        feeder.stop();
    }
}
