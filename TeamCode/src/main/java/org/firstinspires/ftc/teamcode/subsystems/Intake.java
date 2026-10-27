package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.ShooterConstants;

/**
 * Intake motor that collects balls into the feeder.
 *
 * <p>It always runs while a launch is happening, and also whenever it was switched on by hand
 * ({@link #toggleManual()}). Holding reverse ({@link #setReverse}) overrides both, to clear a jam.
 *
 * <p>Nothing is commanded until the first {@link #update}, so the intake does not move in INIT.
 */
public final class Intake {
    private final DcMotorEx motor;
    private boolean manualOn;
    private boolean reverse;
    private double lastPower = Double.NaN;

    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, ShooterConstants.INTAKE_MOTOR_NAME);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /** Call once per loop. @param launching the gate is open or a feed pulse is running */
    public void update(boolean launching) {
        double power;
        if (reverse) {
            power = -ShooterConstants.INTAKE_POWER;
        } else if (manualOn || launching) {
            power = ShooterConstants.INTAKE_POWER;
        } else {
            power = 0.0;
        }
        write(power);
    }

    private void write(double power) {
        if (!Double.isNaN(lastPower) && power == lastPower) return;
        lastPower = power;
        motor.setPower(power);
    }

    public void toggleManual() {
        manualOn = !manualOn;
    }

    public void setManual(boolean on) {
        manualOn = on;
    }

    /** Hold true to run the intake backwards (clears a jam). */
    public void setReverse(boolean reverse) {
        this.reverse = reverse;
    }

    public boolean isManualOn() {
        return manualOn;
    }

    public boolean isRunning() {
        return !Double.isNaN(lastPower) && lastPower != 0.0;
    }

    public void stop() {
        manualOn = false;
        reverse = false;
        lastPower = 0.0;
        motor.setPower(0.0);
    }
}
