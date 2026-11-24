package org.firstinspires.ftc.teamcode.subsystems.Launch;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Turret angle control for a continuous-rotation servo + external encoder.
 *
 * power = kS * sign(error)            static / deadband compensation
 *       + kV * targetVelocity         feedforward, so it keeps up with a moving target (robot turning)
 *       + kP * error
 *       + kI * integral               only within I_ZONE_DEG, reset when error changes sign
 *       + kD * (targetVel - turretVel) damping from the encoder's own velocity (no derivative kick)
 *
 * Inside DEADBAND_DEG the output is 0, so it parks instead of buzzing around the target.
 *
 * Why kS matters: a CR servo does nothing below roughly 0.03-0.06 power. Without kS, small
 * errors ask for less than that and the turret stops short of the target.
 */
@Config
public class Turret {
    public static double kP = 0.004 ;
    public static double kI = 0.003;
    public static double kD = 0.0004;

    // Smallest power that actually moves the turret. Tune: raise until a 1-2 deg move happens.
    public static double kS = 0.03;
    // Power per deg/s of target motion. Tune while spinning the robot in place with auto-aim on.
    public static double kV = 0.00025;

    // Close enough: output 0 here so the servo parks instead of hunting.
    public static double DEADBAND_DEG = 0.3;
    // Integral only builds within this error.
    public static double I_ZONE_DEG = 5;
    public static double I_MAX_POWER = 0.15;
    // Smoothing of the target velocity estimate (0..1, higher = less smoothing).
    public static double TARGET_VEL_ALPHA = 0.3;

    public static double MAX_POWER = 1.0;

    public static double TOLERANCE_DEG = 5;

    // Allowed turret range. The turret is never commanded outside it.
    public static double MIN_DEG = -170;
    public static double MAX_DEG = 200;

    // Encoder ticks per turret degree = 8192 ticks/rev * 2.923 encoder revs per turret rev / 360.
    public static double TICKS_PER_DEG = 8192 * 2.923 / 360.0;

    private final CRServo servo;
    private final DcMotorEx encoder;

    private double target = 0;
    private boolean enabled = false; // stays unpowered until a target is set
    private double integral = 0;
    private double lastError = 0;
    private long lastTime = 0;
    private double lastTarget = Double.NaN;
    private double targetVel = 0; // deg/s, filtered

    public Turret(HardwareMap hardwareMap) {
        servo = hardwareMap.get(CRServo.class, "turp");
        // Not reset here on purpose: the angle carries over from auto to teleop.
        encoder = hardwareMap.get(DcMotorEx.class, "FR");
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetEncoder() {
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getRawAngleDegrees() {
        return encoder.getCurrentPosition() / TICKS_PER_DEG;
    }

    public void setTargetDegrees(double degrees) {
        if (Double.isNaN(degrees)) return;
        target = limit(degrees);
        enabled = true;
    }

    /**
     * Fits an angle into [MIN_DEG, MAX_DEG]. First tries the same direction shifted by
     * a full turn (e.g. 120 -> -240), since that points the turret the same way.
     * If no equivalent fits (the 80..90 gap), it stops at the nearest limit.
     */
    private static double limit(double degrees) {
        double a = degrees;
        while (a > MAX_DEG) a -= 360;
        while (a < MIN_DEG) a += 360;
        if (a <= MAX_DEG) return a;

        // Unreachable direction: go to whichever limit is closer (angle-wise).
        double toMax = Math.abs(wrap180(a - MAX_DEG));
        double toMin = Math.abs(wrap180(a - MIN_DEG));
        return toMax <= toMin ? MAX_DEG : MIN_DEG;
    }

    private static double wrap180(double d) {
        while (d > 180) d -= 360;
        while (d < -180) d += 360;
        return d;
    }

    public double getTargetDegrees() {
        return target;
    }

    /** Turret speed in deg/s, from the encoder. */
    public double getVelocityDegPerSec() {
        return encoder.getVelocity() / TICKS_PER_DEG;
    }

    public boolean atTarget() {
        return enabled && Math.abs(target - getRawAngleDegrees()) <= TOLERANCE_DEG;
    }

    public Command periodic() {
        return infinite(this::update);
    }

    public void tester() {
        update();
    }

    private void update() {
        if (!enabled) {
            servo.setPower(0);
            return;
        }

        long now = System.nanoTime();
        double dt = lastTime == 0 ? 0.02 : clamp((now - lastTime) / 1e9, 1e-3, 0.1);
        lastTime = now;

        // How fast the target is moving (e.g. robot turning while auto-aiming).
        double rawTargetVel = Double.isNaN(lastTarget) ? 0 : (target - lastTarget) / dt;
        if (Math.abs(target - lastTarget) > 90) rawTargetVel = 0; // a jump (new goal / wrap), not motion
        targetVel += TARGET_VEL_ALPHA * (rawTargetVel - targetVel);
        lastTarget = target;

        double error = target - getRawAngleDegrees();
        double velocityError = targetVel - getVelocityDegPerSec();

        // Integral: only near the target, and dropped when we cross it (stops slow overshoot/hunting).
        if (Math.signum(error) != Math.signum(lastError)) integral = 0;
        if (Math.abs(error) < I_ZONE_DEG && kI > 0) {
            double maxIntegral = I_MAX_POWER / kI;
            integral = clamp(integral + error * dt, -maxIntegral, maxIntegral);
        }
        lastError = error;

        double power;
        if (Math.abs(error) < DEADBAND_DEG && Math.abs(targetVel) < 1) {
            // Parked on target and the target isn't moving: stop.
            power = 0;
        } else {
            power = kP * error
                    + kI * integral
                    + kD * velocityError
                    + kV * targetVel
                    + kS * Math.signum(error);
        }
        servo.setPower(clamp(power, -MAX_POWER, MAX_POWER));
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
