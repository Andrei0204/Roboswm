package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.config.ShooterConstants;

/**
 * Adjustable hood on a positional servo.
 *
 * <p>Servo.getPosition() only returns the last command, so it cannot tell whether the hood has
 * arrived. This class models the servo moving at a constant speed (HOOD_SECONDS_PER_FULL_RANGE)
 * and reports "settled" once the model has arrived and HOOD_SETTLE_SEC has passed.
 */
public final class Hood {
    /** Ignore changes smaller than this to avoid flooding the hub with servo writes. */
    private static final double WRITE_DEADBAND = 0.001;

    private final Servo servo;
    private double commanded = Double.NaN;
    private double modeled;
    private double settledFor;
    /** At startup the real position is unknown: assume a full-range move is needed. */
    private double startupWait = Double.NaN;

    public Hood(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, ShooterConstants.HOOD_SERVO_NAME);
        if (ShooterConstants.HOOD_SET_PWM_RANGE && servo instanceof PwmControl) {
            ((PwmControl) servo).setPwmRange(new PwmControl.PwmRange(
                    ShooterConstants.HOOD_PWM_MIN_US, ShooterConstants.HOOD_PWM_MAX_US));
        }
    }

    /**
     * @param position desired servo position (clamped to the hood limits)
     * @param dt       seconds since the previous call
     */
    public void setPosition(double position, double dt) {
        double target = Math.max(ShooterConstants.HOOD_MIN_POSITION,
                Math.min(ShooterConstants.HOOD_MAX_POSITION, position));
        if (Double.isNaN(target)) return;
        if (Double.isNaN(commanded)) {
            servo.setPosition(target);
            commanded = target;
            modeled = target;
            settledFor = 0.0;
            startupWait = ShooterConstants.HOOD_SECONDS_PER_FULL_RANGE;
            return;
        }
        if (Math.abs(target - commanded) >= WRITE_DEADBAND) {
            servo.setPosition(target);
            commanded = target;
        }
        double step = ShooterConstants.HOOD_SECONDS_PER_FULL_RANGE > 0.0
                ? Math.max(0.0, dt) / ShooterConstants.HOOD_SECONDS_PER_FULL_RANGE
                : Double.POSITIVE_INFINITY;
        double gap = commanded - modeled;
        modeled = Math.abs(gap) <= step ? commanded : modeled + Math.signum(gap) * step;
        if (startupWait > 0.0) startupWait -= Math.max(0.0, dt);
        settledFor = modeled == commanded ? settledFor + Math.max(0.0, dt) : 0.0;
    }

    public boolean isSettled() {
        return !Double.isNaN(commanded)
                && !(startupWait > 0.0)
                && settledFor >= ShooterConstants.HOOD_SETTLE_SEC;
    }

    public double commandedPosition() {
        return commanded;
    }
}
