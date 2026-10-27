package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.config.ShooterConstants;

/**
 * The intake's 3-position servo, separate from the intake motor ({@link Intake}).
 *
 * <p>Each {@link Mode} holds its own servo position. The values are public and non-final so they
 * can be tuned (e.g. {@code IntakeServo.Mode.COLLECT_ALL.position = 0.5}). MEASURE them with
 * "Hood & Feeder Test".
 *
 * <p>The servo is not commanded until the first {@link #update}, so it does not move in INIT.
 */
public final class IntakeServo {
    /** The three positions. */
    public enum Mode {
        /** Stowed, as at the start of the match. */
        INIT(0.10),
        /** Collects every ball (POLLEN and NECTAR). */
        COLLECT_ALL(0.50),
        /** Collects POLLEN only. */
        COLLECT_POLLEN(0.75);

        /** Servo position for this mode. */
        public double position;

        Mode(double position) {
            this.position = position;
        }
    }

    private final Servo servo;
    private Mode mode = Mode.INIT;
    private double lastPosition = Double.NaN;

    public IntakeServo(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, ShooterConstants.INTAKE_SERVO_NAME);
    }

    /** Selects the position; the servo moves on the next {@link #update}. */
    public void setMode(Mode mode) {
        this.mode = mode;
    }

    public Mode mode() {
        return mode;
    }

    /** Call once per loop. The first call after START commands the servo. */
    public void update() {
        double position = mode.position;
        if (!Double.isNaN(lastPosition) && position == lastPosition) return;
        lastPosition = position;
        servo.setPosition(position);
    }
}
