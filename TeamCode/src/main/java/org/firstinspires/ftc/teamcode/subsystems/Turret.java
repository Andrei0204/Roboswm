package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.config.ShooterConstants;

/**
 * Turret hardware: continuous-rotation servo(s), configured and used as plain servos: position 0.5
 * is stopped, and 0.5 + 0.5 * power spins one way or the other. They move the turret, and an
 * external encoder on a motor port measuring it. The control logic (homing, encoder feedback,
 * aim check) lives in {@link TurretController}.
 *
 * <p>The encoder's direction is a software sign, never setDirection(). Count 0 = turret 0 deg. The hub
 * resets the count when an OpMode ends, so each OpMode assumes the turret starts at 0 deg; the
 * reset button (homing, in INIT) zeroes it again wherever the turret is. The reset briefly stops any motor on the same port, so use a free port or one whose
 * motor is idle in INIT.
 */
public final class Turret {

    private final Servo[] servos;
    private final DcMotorEx encoder;
    private final TurretController controller = new TurretController();
    private final double[] lastServoPower;
    /** Net commanded power (before the per-servo bias). */
    private double lastPower = Double.NaN;

    public Turret(HardwareMap hardwareMap) {
        TurretController.validateConstants();
        String[] names = ShooterConstants.TURRET_SERVO_NAMES;
        boolean[] reversed = ShooterConstants.TURRET_SERVO_REVERSED;
        if (names.length == 0 || names.length != reversed.length) {
            throw new IllegalStateException("TURRET_SERVO_NAMES and TURRET_SERVO_REVERSED must have the same, non-zero length");
        }
        servos = new Servo[names.length];
        lastServoPower = new double[names.length];
        java.util.Arrays.fill(lastServoPower, Double.NaN);
        for (int i = 0; i < names.length; i++) {
            Servo servo = hardwareMap.get(Servo.class, names[i]);
            servo.setDirection(reversed[i] ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
            servos[i] = servo;
        }
        encoder = hardwareMap.get(DcMotorEx.class, ShooterConstants.TURRET_ENCODER_NAME);
        writePower(0.0);
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
        writePower(controller.update(targetAngle, targetRate, dt));
        if (controller.takeResetRequest()) resetEncoderCount();
    }

    /** INIT-loop step: holds the servo at 0 power while homing runs, otherwise leaves it alone. */
    public void initUpdate(double dt) {
        double power = controller.initUpdate(dt);
        if (!Double.isNaN(power)) writePower(power);
        if (controller.takeResetRequest()) resetEncoderCount();
    }

    /** Stops the servo. */
    public void stop() {
        writePower(0.0);
    }

    /** Sets the encoder count to 0 at the homed turret position, keeping the port's run mode. */
    private void resetEncoderCount() {
        DcMotor.RunMode mode = encoder.getMode();
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(mode == DcMotor.RunMode.STOP_AND_RESET_ENCODER ? DcMotor.RunMode.RUN_WITHOUT_ENCODER : mode);
    }

    private void writePower(double power) {
        // Anti-backlash: with 2+ servos, servo 0 gets +bias and servo 1 -bias (alternating), so
        // they always load the gear teeth against each other. The net push is still `power`.
        double bias = 0.0;
        if (servos.length >= 2) {
            double fade = Math.max(1e-6, ShooterConstants.TURRET_BIAS_FADE_POWER);
            bias = ShooterConstants.TURRET_BIAS_POWER * Math.max(0.0, 1.0 - Math.abs(power) / fade);
        }
        for (int i = 0; i < servos.length; i++) {
            double p = power;
            if (bias != 0.0) p += (i % 2 == 0 ? bias : -bias);
            p = Math.max(-1.0, Math.min(1.0, p));
            // The servo does nothing between neutral (0.5) and +/-TURRET_SERVO_DEADBAND_POWER. Map
            // (0, 1] onto (deadband, 1] so every non-zero command really moves it.
            if (Math.abs(p) < ShooterConstants.TURRET_SERVO_ZERO_BELOW) {
                p = 0.0;
            } else {
                double dz = Math.max(0.0, Math.min(0.95, Math.abs(ShooterConstants.TURRET_SERVO_DEADBAND_POWER)));
                p = Math.signum(p) * (dz + (1.0 - dz) * Math.min(1.0, Math.abs(p)));
            }
            if (Double.isNaN(lastServoPower[i]) || p != lastServoPower[i]) {
                servos[i].setPosition(0.5 + 0.5 * p);
                lastServoPower[i] = p;
            }
        }
        lastPower = power;
    }

    public void startHoming() {
        controller.startHoming();
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

    /** Last servo power written, in [-1, 1]. */
    public double servoPower() {
        return lastPower;
    }
}
