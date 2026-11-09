package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.ShooterConstants;

/**
 * Transfer that pushes balls into the flywheel: a motor or a CR servo.
 *
 * <p>{@link #update} feeds in pulses (one ball each) lasting FEEDER_PULSE_SEC seconds
 * (timer only); a new pulse starts only
 * when the previous one has ended and a feed is still requested. A started pulse always finishes,
 * so a ball is never left half-fed. After each pulse the feeder rests the gap from FEEDER_GAP_TABLE (by HIVE distance; FEEDER_GAP_SEC fallback) before the
 * next one, for normal and forced fire alike. With FEEDER_PULSE_SEC <= 0 it feeds for as long as requested.
 */
public final class Feeder {
    private final DcMotorSimple actuator;
    /** Non-null when the feeder is a motor, so its velocity can be read for the PID. */
    private final DcMotorEx motorEx;
    /** Where the feeder velocity is read from: the feeder's own port or a separate encoder port. */
    private final DcMotorEx encoder;
    private double integral;
    private double lastPidTime = Double.NaN;
    private double lastPower = Double.NaN;
    private boolean feeding;
    private double feedEnd;
    /** Earliest time the next pulse may start (end of the previous pulse + the gap). */
    private double nextStart;
    private int feeds;

    public Feeder(HardwareMap hardwareMap) {
        actuator = hardwareMap.get(DcMotorSimple.class, ShooterConstants.FEEDER_NAME);
        if (actuator instanceof DcMotor) {
            DcMotor motor = (DcMotor) actuator;
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        motorEx = actuator instanceof DcMotorEx ? (DcMotorEx) actuator : null;
        String encName = ShooterConstants.FEEDER_ENCODER_NAME;
        if (encName != null && !encName.isEmpty()) {
            encoder = hardwareMap.get(DcMotorEx.class, encName);
            // Only the encoder is used from this port; it must not drive anything.
            encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            encoder.setPower(0.0);
        } else {
            encoder = motorEx;
        }
    }

    /**
     * Call once per loop.
     *
     * @param requested a feed is wanted now (the caller decides: ready to shoot, or forced)
     * @return true if a new feed started this loop
     */
    public boolean update(boolean requested) {
        return update(requested, Double.NaN);
    }

    /**
     * Same, with the distance to the HIVE (in) choosing the gap after the pulse from
     * FEEDER_GAP_TABLE (NaN = FEEDER_GAP_SEC).
     */
    public boolean update(boolean requested, double distanceIn) {
        double now = System.nanoTime() * 1e-9;
        boolean started = false;
        double pulse = ShooterConstants.FEEDER_PULSE_SEC;
        if (pulse > 0.0) {
            if (feeding && now >= feedEnd) {
                feeding = false;
                nextStart = feedEnd + gapSec(distanceIn);
            }
            if (!feeding && requested && now >= nextStart) {
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

    /** Gap after a pulse (s) for this HIVE distance (in). */
    public static double gapSec(double distanceIn) {
        double[][] t = ShooterConstants.FEEDER_GAP_TABLE;
        double gap = ShooterConstants.FEEDER_GAP_SEC;
        if (t != null && t.length >= 1 && !Double.isNaN(distanceIn)) {
            if (distanceIn <= t[0][0]) {
                gap = t[0][1];
            } else if (distanceIn >= t[t.length - 1][0]) {
                gap = t[t.length - 1][1];
            } else {
                for (int i = 1; i < t.length; i++) {
                    if (distanceIn <= t[i][0]) {
                        double f = (distanceIn - t[i - 1][0]) / (t[i][0] - t[i - 1][0]);
                        gap = t[i - 1][1] + f * (t[i][1] - t[i - 1][1]);
                        break;
                    }
                }
            }
        }
        return Math.max(0.0, gap);
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
        double power;
        if (feeding && ShooterConstants.FEEDER_PID_ENABLED && encoder != null) {
            power = pidPower();
        } else {
            resetPid();
            power = feeding ? ShooterConstants.FEEDER_FIRE_POWER : ShooterConstants.FEEDER_IDLE_POWER;
        }
        if (!Double.isNaN(lastPower) && Math.abs(power - lastPower) < 1e-3) return;
        lastPower = power;
        actuator.setPower(power);
    }

    /**
     * Velocity control while feeding: feedforward + P + I on the encoder velocity, so the feed
     * speed is the same for every ball. Works in the signed domain: the direction is the sign of
     * FEEDER_FIRE_POWER (the encoder must count the same way the motor is driven).
     */
    private double pidPower() {
        double now = System.nanoTime() * 1e-9;
        double dt = Double.isNaN(lastPidTime) ? 0.0 : Math.min(now - lastPidTime, 0.1);
        lastPidTime = now;
        double dir = ShooterConstants.FEEDER_FIRE_POWER < 0 ? -1.0 : 1.0;
        double target = dir * Math.abs(ShooterConstants.FEEDER_TARGET_TPS);
        double measured = encoder.getVelocity() * (ShooterConstants.FEEDER_ENCODER_REVERSED ? -1.0 : 1.0);
        double error = target - measured;
        double kI = ShooterConstants.FEEDER_KI;
        double clamp = Math.abs(ShooterConstants.FEEDER_I_CLAMP);
        integral += error * dt;
        if (kI != 0.0) {
            integral = Math.max(-clamp / Math.abs(kI), Math.min(clamp / Math.abs(kI), integral));
        } else {
            integral = 0.0;
        }
        double power = ShooterConstants.FEEDER_KV * target
                + ShooterConstants.FEEDER_KP * error
                + kI * integral;
        double limit = Math.min(1.0, Math.abs(ShooterConstants.FEEDER_FIRE_POWER));
        return Math.max(-limit, Math.min(limit, power));
    }

    /** Feeder speed in ticks/s from the encoder (NaN if there is none), sign as used by the PID. */
    public double velocityTps() {
        return encoder == null ? Double.NaN
                : encoder.getVelocity() * (ShooterConstants.FEEDER_ENCODER_REVERSED ? -1.0 : 1.0);
    }

    /** Feeder encoder position in ticks (NaN if there is none). */
    public double positionTicks() {
        return encoder == null ? Double.NaN
                : encoder.getCurrentPosition() * (ShooterConstants.FEEDER_ENCODER_REVERSED ? -1.0 : 1.0);
    }

    /** Feeder revolutions from the encoder (0 if there is none), sign as used by the PID. */
    public double revolutions() {
        double ticks = positionTicks();
        return Double.isNaN(ticks) ? 0.0 : ticks / Math.max(1e-6, ShooterConstants.FEEDER_TICKS_PER_REV);
    }

    private void resetPid() {
        integral = 0.0;
        lastPidTime = Double.NaN;
    }

    /** Tuning only: any power, e.g. reverse to check direction or clear a jam. */
    public void setRawPower(double power) {
        if (!Double.isNaN(lastPower) && power == lastPower) return;
        lastPower = power;
        actuator.setPower(power);
    }

    public void stop() {
        resetPid();
        feeding = false;
        nextStart = 0.0;
        lastPower = 0.0;
        actuator.setPower(0.0);
    }
}
