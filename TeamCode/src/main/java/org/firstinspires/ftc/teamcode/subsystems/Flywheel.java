package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.ShooterConstants;

/**
 * Flywheel velocity control: feedforward (kS + kV * rpm, voltage compensated) + P + I on a
 * smoothed rpm measurement, with full
 * power when far below target so it recovers quickly after a shot.
 *
 * <p>The motors run in RUN_WITHOUT_ENCODER with our own controller. Never call setVelocity() on
 * them: it silently switches the motor to RUN_USING_ENCODER and fights this controller.
 */
public final class Flywheel {
    private final DcMotorEx[] motors;
    private double targetRpm;
    private double measuredRpm;
    private double lastPower = Double.NaN;
    private double integral;
    private double lastUpdateTime = Double.NaN;
    private double lastTarget;
    private boolean filterPrimed;
    /** Last time a ball was being fed (s), for the recoil boost hold. */
    private double lastFeedTime = Double.NEGATIVE_INFINITY;

    public Flywheel(HardwareMap hardwareMap) {
        String[] names = ShooterConstants.FLYWHEEL_MOTOR_NAMES;
        boolean[] reversed = ShooterConstants.FLYWHEEL_MOTOR_REVERSED;
        if (names.length == 0 || names.length != reversed.length) {
            throw new IllegalStateException("FLYWHEEL_MOTOR_NAMES and FLYWHEEL_MOTOR_REVERSED must have the same, non-zero length");
        }
        motors = new DcMotorEx[names.length];
        for (int i = 0; i < names.length; i++) {
            DcMotorEx motor = hardwareMap.get(DcMotorEx.class, names[i]);
            motor.setDirection(reversed[i] ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motors[i] = motor;
        }
    }

    /** Reads the encoder velocity. Call once per loop after clearing the bulk cache. */
    public void readSensors() {
        double raw = motors[0].getVelocity() / ShooterConstants.FLYWHEEL_TICKS_PER_REV * 60.0;
        double alpha = Math.max(0.01, Math.min(1.0, ShooterConstants.FLYWHEEL_RPM_FILTER));
        measuredRpm = filterPrimed ? measuredRpm + alpha * (raw - measuredRpm) : raw;
        filterPrimed = true;
    }

    public void setTargetRpm(double rpm) {
        targetRpm = Math.max(0.0, rpm);
    }

    /** Call every loop with whether a ball is being fed now, to enable the recoil compensation. */
    public void setFeeding(boolean feeding) {
        if (feeding) lastFeedTime = System.nanoTime() * 1e-9;
    }

    /** @param voltageScale nominal voltage / battery voltage */
    public void update(double voltageScale) {
        double now = System.nanoTime() * 1e-9;
        double dt = Double.isNaN(lastUpdateTime) ? 0.0 : Math.min(now - lastUpdateTime, 0.1);
        lastUpdateTime = now;
        // A new set point (or off) starts the integral from scratch.
        if (targetRpm <= 0.0 || Math.abs(targetRpm - lastTarget) > ShooterConstants.FLYWHEEL_I_ZONE_RPM) integral = 0.0;
        lastTarget = targetRpm;
        double power;
        if (targetRpm <= 0.0) {
            power = 0.0;
        } else {
            boolean recoil = System.nanoTime() * 1e-9 - lastFeedTime <= ShooterConstants.FLYWHEEL_RECOIL_HOLD_SEC;
            double drive = targetRpm + (recoil ? Math.max(0.0, ShooterConstants.FLYWHEEL_RECOIL_BOOST_RPM) : 0.0);
            double error = drive - measuredRpm;
            if (error > ShooterConstants.FLYWHEEL_BANG_BANG_RPM) {
                power = 1.0; // the integral is frozen here so it keeps what it learned
            } else {
                double kI = Math.abs(ShooterConstants.FLYWHEEL_KI);
                if (kI > 0.0 && Math.abs(error) <= ShooterConstants.FLYWHEEL_I_ZONE_RPM) {
                    double limit = Math.abs(ShooterConstants.FLYWHEEL_I_CLAMP) / kI;
                    integral = Math.max(-limit, Math.min(limit, integral + error * dt));
                } else if (kI == 0.0) {
                    integral = 0.0;
                }
                power = (ShooterConstants.FLYWHEEL_KS + ShooterConstants.FLYWHEEL_KV * drive) * voltageScale
                        + ShooterConstants.FLYWHEEL_KP * error
                        + Math.abs(ShooterConstants.FLYWHEEL_KI) * integral;
            }
        }
        writePower(power);
    }

    /** Tuning only: runs the flywheel at a fixed power, bypassing the controller. */
    public void setOpenLoopPower(double power) {
        targetRpm = 0.0;
        writePower(power);
    }

    private void writePower(double power) {
        // Never dr`ive the flywheel backwards to slow it down; let it coast.
        power = Math.max(0.0, Math.min(1.0, power)); //modificat
        // Skip changes too small to matter (each write is a hub call), but always send a change
        // to exactly 0 (stop) or 1 (full power).
        boolean endpoint = (power == 0.0 || power == 1.0) && power != lastPower;
        if (!Double.isNaN(lastPower) && !endpoint
                && Math.abs(power - lastPower) < ShooterConstants.FLYWHEEL_WRITE_THRESHOLD) return;
        lastPower = power;
        for (DcMotorEx motor : motors) {
            motor.setPower(power);
        }
    }

    public void stop() {
        targetRpm = 0.0;
        update(1.0);
    }

    public boolean atSpeed() {
        return targetRpm > 0.0
                && measuredRpm >= targetRpm - ShooterConstants.FLYWHEEL_TOLERANCE_BELOW_RPM
                && measuredRpm <= targetRpm + ShooterConstants.FLYWHEEL_TOLERANCE_ABOVE_RPM;
    }

    public double targetRpm() {
        return targetRpm;
    }

    public double measuredRpm() {
        return measuredRpm;
    }

    public double power() {
        return Double.isNaN(lastPower) ? 0.0 : lastPower;
    }
}
