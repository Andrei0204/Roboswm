package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Flywheel velocity control: feedforward (kS + kV * rpm, voltage compensated) + P, with full
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
        measuredRpm = motors[0].getVelocity() / ShooterConstants.FLYWHEEL_TICKS_PER_REV * 60.0;
    }

    public void setTargetRpm(double rpm) {
        targetRpm = Math.max(0.0, rpm);
    }

    /** @param voltageScale nominal voltage / battery voltage */
    public void update(double voltageScale) {
        double power;
        if (targetRpm <= 0.0) {
            power = 0.0;
        } else {
            double error = targetRpm - measuredRpm;
            if (error > ShooterConstants.FLYWHEEL_BANG_BANG_RPM) {
                power = 1.0;
            } else {
                power = (ShooterConstants.FLYWHEEL_KS + ShooterConstants.FLYWHEEL_KV * targetRpm) * voltageScale
                        + ShooterConstants.FLYWHEEL_KP * error;
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
        // Never drive the flywheel backwards to slow it down; let it coast.
        power = Math.max(0.0, Math.min(1.0, power));
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
