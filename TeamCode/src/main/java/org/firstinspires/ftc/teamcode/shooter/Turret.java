package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Turret hardware: positional servo(s) in servo mode (an Axon MAX geared 60:24 then 26:76)
 * moving the turret, and an external encoder on a motor port measuring it. The control logic
 * (homing, lead, encoder correction, aim check) lives in {@link TurretController}.
 *
 * <p>The encoder's direction is a software sign, never setDirection(). Its count is reset only by
 * homing (in INIT): that makes count 0 = turret 0 deg, and the hub keeps the count from AUTO to
 * TeleOp. The reset briefly stops any motor on the same port, so use a free port or one whose
 * motor is idle in INIT.
 */
public final class Turret {

    private final Servo[] servos;
    private final DcMotorEx encoder;
    private final TurretController controller = new TurretController();
    private double lastPosition = Double.NaN;
    /**
     * Smallest position change the servo can show. The hub sets servo pulses in whole
     * microseconds, so this is 1 us of the pulse range (0.0005 for 500-2500 us, about 0.15 deg of
     * turret). Smaller changes are not sent, which saves a hub write.
     */
    private final double writeDeadband;

    public Turret(HardwareMap hardwareMap) {
        TurretController.validateConstants();
        String[] names = ShooterConstants.TURRET_SERVO_NAMES;
        boolean[] reversed = ShooterConstants.TURRET_SERVO_REVERSED;
        if (names.length == 0 || names.length != reversed.length) {
            throw new IllegalStateException("TURRET_SERVO_NAMES and TURRET_SERVO_REVERSED must have the same, non-zero length");
        }
        servos = new Servo[names.length];
        for (int i = 0; i < names.length; i++) {
            Servo servo = hardwareMap.get(Servo.class, names[i]);
            servo.setDirection(reversed[i] ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
            if (ShooterConstants.TURRET_SET_PWM_RANGE && servo instanceof PwmControl) {
                ((PwmControl) servo).setPwmRange(new PwmControl.PwmRange(
                        ShooterConstants.TURRET_PWM_MIN_US, ShooterConstants.TURRET_PWM_MAX_US));
            }
            servos[i] = servo;
        }
        encoder = hardwareMap.get(DcMotorEx.class, ShooterConstants.TURRET_ENCODER_NAME);
        PwmControl.PwmRange range = servos[0] instanceof PwmControl
                ? ((PwmControl) servos[0]).getPwmRange()
                : PwmControl.PwmRange.defaultRange;
        writeDeadband = 1.0 / Math.max(1.0, range.usPulseUpper - range.usPulseLower);
    }

    /** Reads the encoder (bulk cached). Call once per loop after clearing the bulk cache. */
    public void readSensors() {
        controller.setMeasurement(encoder.getCurrentPosition(), encoder.getVelocity());
    }

    /**
     * @param targetAngle desired robot-relative angle (rad, any wrap)
     * @param targetRate  desired angular rate relative to the robot (rad/s)
     * @param dt          seconds since the previous call
     */
    public void update(double targetAngle, double targetRate, double dt) {
        writePosition(controller.update(targetAngle, targetRate, dt));
        if (controller.takeResetRequest()) resetEncoderCount();
    }

    /** INIT-loop step: moves the servo only while homing runs. */
    public void initUpdate(double dt) {
        double position = controller.initUpdate(dt);
        if (!Double.isNaN(position)) writePosition(position);
        if (controller.takeResetRequest()) resetEncoderCount();
    }

    /** Sets the encoder count to 0 at the homed turret position, keeping the port's run mode. */
    private void resetEncoderCount() {
        DcMotor.RunMode mode = encoder.getMode();
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(mode == DcMotor.RunMode.STOP_AND_RESET_ENCODER ? DcMotor.RunMode.RUN_WITHOUT_ENCODER : mode);
    }

    private void writePosition(double position) {
        if (Double.isNaN(lastPosition) || Math.abs(position - lastPosition) >= writeDeadband) {
            for (Servo servo : servos) servo.setPosition(position);
            lastPosition = position;
        }
    }

    public void startHoming() {
        controller.startHoming();
    }

    /** At OpMode start, after {@link #readSensors}: drops an impossible zero. */
    public boolean checkZero(double marginRad) {
        return controller.checkZero(marginRad);
    }

    public boolean hasReference() {
        return controller.hasReference();
    }

    public boolean resetFailed() {
        return controller.resetFailed();
    }

    public boolean isAimed(double tolerance) {
        return controller.isAimed(tolerance);
    }

    public boolean isReachable() {
        return controller.isReachable();
    }

    public boolean isHoming() {
        return controller.isHoming();
    }

    public boolean homeTimedOut() {
        return controller.homeTimedOut();
    }

    public double setpoint() {
        return controller.setpoint();
    }

    public double angle() {
        return controller.angle();
    }

    public double rate() {
        return controller.rate();
    }

    public double error() {
        return controller.error();
    }

    public double correction() {
        return controller.correction();
    }

    /** Last servo position written (NaN before the first update). */
    public double servoPosition() {
        return lastPosition;
    }
}
