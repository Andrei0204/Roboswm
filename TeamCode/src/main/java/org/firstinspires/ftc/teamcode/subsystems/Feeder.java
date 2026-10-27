package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.ShooterConstants;

/**
 * Transfer that pushes balls into the flywheel: a motor or a CR servo.
 *
 * <p>{@link #update} feeds in pulses of FEEDER_PULSE_SEC (one ball each); a new pulse starts only
 * when the previous one has ended and a feed is still requested. A started pulse always finishes,
 * so a ball is never left half-fed. With FEEDER_PULSE_SEC <= 0 it feeds for as long as requested.
 */
public final class Feeder {
    private final DcMotorSimple actuator;
    private double lastPower = Double.NaN;
    private boolean feeding;
    private double feedEnd;
    private int feeds;

    public Feeder(HardwareMap hardwareMap) {
        actuator = hardwareMap.get(DcMotorSimple.class, ShooterConstants.FEEDER_NAME);
        if (actuator instanceof DcMotor) {
            DcMotor motor = (DcMotor) actuator;
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    /**
     * Call once per loop.
     *
     * @param requested a feed is wanted now (the caller decides: ready to shoot, or forced)
     * @return true if a new feed started this loop
     */
    public boolean update(boolean requested) {
        double now = System.nanoTime() * 1e-9;
        boolean started = false;
        double pulse = ShooterConstants.FEEDER_PULSE_SEC;
        if (pulse > 0.0) {
            if (feeding && now >= feedEnd) feeding = false;
            if (!feeding && requested) {
                feeding = true;
                feedEnd = now + pulse;
                started = true;
            }
        } else {
            started = requested && !feeding;
            feeding = requested;
        }
        if (started) feeds++;
        set(feeding);
        return started;
    }

    public boolean isFeeding() {
        return feeding;
    }

    /** Number of feeds (pulses) started since this object was created. */
    public int feeds() {
        return feeds;
    }

    /** Direct on/off, bypassing the pulse logic (tuning OpModes). */
    public void set(boolean feeding) {
        double power = feeding ? ShooterConstants.FEEDER_FIRE_POWER : ShooterConstants.FEEDER_IDLE_POWER;
        if (!Double.isNaN(lastPower) && power == lastPower) return;
        lastPower = power;
        actuator.setPower(power);
    }

    /** Tuning only: any power, e.g. reverse to check direction or clear a jam. */
    public void setRawPower(double power) {
        if (!Double.isNaN(lastPower) && power == lastPower) return;
        lastPower = power;
        actuator.setPower(power);
    }

    public void stop() {
        feeding = false;
        lastPower = 0.0;
        actuator.setPower(0.0);
    }
}
