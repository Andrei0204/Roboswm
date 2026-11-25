package org.firstinspires.ftc.teamcode.subsystems.Launch;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Flywheel speed control: feedforward + PID, voltage compensated.
 *
 *  - Feedforward (kV * target + kS) gives roughly the right power for a target on its own,
 *    so the PID only fixes small errors and can be tuned tight.
 *  - Everything is scaled by NOMINAL_VOLTAGE / battery voltage, so the same power gives the
 *    same speed on a full or a tired battery.
 *  - The integral only builds up near the target (I_ZONE_RPM), so it can't wind up during
 *    spin-up and overshoot, and it is kept (not wiped) through small overshoots.
 */
@Config
public class Launcher {
    // Feedforward. kV = power per RPM at NOMINAL_VOLTAGE. Tune kV FIRST with kP = kI = 0 (see tuning notes).
    public static double kV =0.0002;
    public static double kS = 0.0;

    // Feedback on top of the feedforward.
    public static double kP = 0.0015;
    public static double kI = 0.0001;
    public static double kD = 0.0000;

    // Integral only accumulates within this many RPM of the target.
    public static double I_ZONE_RPM = 200;
    // Most power the integral term may add or remove.
    public static double I_MAX_POWER = 0.3;

    public static double NOMINAL_VOLTAGE = 12;

    public static double TOLERANCE_RPM = 120;

    // More than BRAKE_OVER_RPM above target -> BRAKE_POWER. Closer than that the PID handles it
    // (it can lower power to 0, but never reverses).
    public static double BRAKE_OVER_RPM = 170;
    public static double BRAKE_POWER = -0.2;

    // Bang-bang: more than BANG_BANG_RPM below target -> full power (fast spin-up and
    // recovery after a shot). Closer than that, feedforward + PID take over.
    // Keep it at or below I_ZONE_RPM so the integral is already working at the hand-off.
    public static boolean BANG_BANG = true;
    public static double BANG_BANG_RPM = 100;
    public static double BANG_BANG_POWER = 1.0;

    private final DcMotorEx motor;
    private final VoltageSensor battery;

    private double target = 0;
    private double integral = 0;
    private double lastError = 0;
    private long lastTime = 0;

    private double voltage = 12;
    private long lastVoltageRead = 0;

    public Launcher(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "launcherp");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        battery = hardwareMap.voltageSensor.iterator().hasNext()
                ? hardwareMap.voltageSensor.iterator().next() : null;
    }

    public double getRPM() {
        return motor.getVelocity() / 28.0 * 60.0;
    }

    public void setTargetRPM(double rpm) {
        if (Double.isNaN(rpm)) return;
        target = rpm;
    }

    public void updateshooting(double rpm) {
        setTargetRPM(rpm);
    }

    public void off() {
        setTargetRPM(0);
    }

    public double getTargetRPM() {
        return target;
    }

    public boolean atSpeed() {
        return target != 0 && Math.abs(target - getRPM()) <= TOLERANCE_RPM;
    }

    public Command periodic() {
        return infinite(this::update);
    }

    public void tester() {
        update();
    }

    private void update() {
        if (target == 0) {
            // Let it coast instead of driving the flywheel backwards.
            motor.setPower(0);
            integral = 0;
            lastError = 0;
            lastTime = 0;
            return;
        }

        long now = System.nanoTime();
        double dt = lastTime == 0 ? 0.02 : Math.min((now - lastTime) / 1e9, 0.1);
        lastTime = now;

        double error = target - getRPM();
        double derivative = (error - lastError) / dt;
        lastError = error;

        double power;
        if (error < -BRAKE_OVER_RPM) {
            // Way too fast (e.g. target just dropped a lot): brake.
            power = BRAKE_POWER;
            integral = 0;
        } else if (BANG_BANG && error > BANG_BANG_RPM) {
            // Way too slow (spin-up, or RPM dropped after a shot): full send.
            power = BANG_BANG_POWER;
        } else {
            if (Math.abs(error) < I_ZONE_RPM && kI > 0) {
                double maxIntegral = I_MAX_POWER / kI;
                integral = clamp(integral + error * dt, -maxIntegral, maxIntegral);
            }

            double ff = kV * target + kS;
            double fb = kP * error + kI * integral + kD * derivative;
            power = (ff + fb) * NOMINAL_VOLTAGE / readVoltage();
            power = Math.max(0, power); // never reverse while near the target
        }
        motor.setPower(clamp(power, -1, 1));
    }

    /** Battery voltage, re-read at most every 250 ms (each read costs loop time). */
    private double readVoltage() {
        if (battery == null) return NOMINAL_VOLTAGE;
        long now = System.nanoTime();
        if (now - lastVoltageRead > 250_000_000L) {
            double v = battery.getVoltage();
            if (v > 6) voltage = v; // ignore bogus readings
            lastVoltageRead = now;
        }
        return voltage;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
