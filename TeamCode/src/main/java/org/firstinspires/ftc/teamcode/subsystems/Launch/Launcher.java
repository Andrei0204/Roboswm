package org.firstinspires.ftc.teamcode.subsystems.Launch;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Tele;

/**
 * Flywheel launcher velocity control.
 *
 * Goals: fast response to a changing RPS target (spin-up AND spin-down), and
 * tight steady-state tracking (~5 RPS band on a ~6000 RPM-class motor).
 *
 * How it gets there:
 *  - Feedforward (kV * target + kS) supplies most of the power for a given
 *    target RPS on its own, worked out from the motor's free speed. Because
 *    it doesn't depend on measured error, it reacts to a target CHANGE
 *    instantly instead of needing error to build up first, the way a bare
 *    PID does.
 *  - A PID trim on top only has to correct real-world inaccuracy (voltage
 *    sag, friction, the RPS dip when a ball is fired) -- small corrections,
 *    so it can be tuned tight without going unstable, which is what
 *    actually gets you inside a 5 RPS band.
 *  - If the error is large -- a big target jump, or a ball was just fired
 *    and RPS dropped a lot -- it skips the PID and drives full power in the
 *    needed direction, including a brief reverse "brake" when the target
 *    drops a lot, since an un-driven flywheel coasts down far slower than
 *    it spins up.
 */
@Configurable
public class Launcher {

    // ---------------- Feedforward / feedback gains ----------------
    public static double kV = 0.0202857143 / 60.0; // power per RPM -- start near 1/(free-speed RPS at the flywheel), then tune
    public static double kS = 0.05;       // static friction/loss feedforwardF
    public static double kP = 0.05 / 60.0;
    public static double kI = 0.004 / 60.0;
    public static double kD = 0.0 / 60.0;

    // ---------------- Bang-bang / braking thresholds ----------------
    public static double SPIN_UP_ERROR_RPM = 120; // this far below target -> full power instead of PID
    public static double BRAKE_ERROR_RPM = 360;  // this far above target -> actively brake instead of coasting
    public static double BRAKE_POWER = 0.6;     // magnitude of reverse power used while braking -- keep modest

    public static double TOLERANCE_RPM = 120;     // "at speed" band

    // Flip this ONE flag if positive power does NOT correspond to a positive getRPM() reading
    // on the bench -- see the note in chat about verifying this before trusting it.
    public static boolean INVERT_POWER = false;

    private final DcMotorEx LAUNCHER;
    public boolean forceOff = false;
    private double targetRPM = 0;
    private Mode mode = Mode.OFF;
    
    private double integralSum = 0;
    private double lastError = 0;
    private double lastTime = 0;

    public Launcher(HardwareMap hardwareMap) {
        LAUNCHER = hardwareMap.get(DcMotorEx.class, "launcherp");
        LAUNCHER.setDirection(DcMotorSimple.Direction.REVERSE);
        LAUNCHER.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LAUNCHER.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getRPM() {
        return LAUNCHER.getVelocity() / 28.0 * 60.0;
    }

    public void off() {
        targetRPM = 0;
        mode = Mode.OFF;
        integralSum = 0;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    /** True once measured RPM is within TOLERANCE_RPM of target. */
    public boolean atSpeed() {
        return mode == Mode.POSITION && Math.abs(targetRPM - getRPM()) <= TOLERANCE_RPM;
    }

    public void setTargetRPM(double targetRPM) {
        if (Double.isNaN(targetRPM)) return;
        if (targetRPM != 0) {
            mode = Mode.POSITION;
            this.targetRPM = targetRPM;
        } else {
            this.targetRPM = 0;
            mode = Mode.OFF;
        }
    }

    public void updateshooting(double x) {
        if (Double.isNaN(x)) return;
        setTargetRPM(x);
    }


    public Mode getMode() {
        return mode;
    }

    public Command periodic() {
        return infinite(this::update);
    }

    /** Also callable directly (e.g. from a teleop loop or a test OpMode) without the command scheduler. */
    public void tester() {
        update();
    }

    private void update() {
        if (forceOff) {
            LAUNCHER.setPower(0);
            return;
        }

        switch (mode) {
            case POSITION:
                runVelocityControl();
                break;
            case OFF:
            default:
                LAUNCHER.setPower(0);

                integralSum = 0;
                break;
        }
    }

    private void runVelocityControl() {
        double t = now();
        double dt = lastTime == 0 ? 0.02 : clamp(t - lastTime, 1e-3, 0.1);
        lastTime = t;

        double currentRPM = getRPM();
        double error = targetRPM - currentRPM;

        double power;
        if (error > SPIN_UP_ERROR_RPM) {
            // Way below target -- full send, no point waiting on the PID to ramp up.
            power = 1.0;
            integralSum = 0;
        } else if (error < -BRAKE_ERROR_RPM) {
            // Way above target (it just dropped a lot) -- coasting down is too slow, brake actively.
            power = -BRAKE_POWER;
            integralSum = 0;
        } else {
            double maxIntegral = kI > 1e-9 ? 0.3 / kI : 0;
            integralSum = clamp(integralSum + error * dt, -maxIntegral, maxIntegral);
            double derivative = (error - lastError) / dt;

            double ff = kV * targetRPM + kS;
            double fb = kP * error + kI * integralSum + kD * derivative;
            power = ff + fb;
        }

        if (INVERT_POWER) power = -power;
        power = clamp(power, -1.0, 1.0);

        LAUNCHER.setPower(power);
        lastError = error;
    }

    private static double now() {
        return System.nanoTime() / 1e9;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    public enum Mode {
        POSITION,
        OFF,
    }
}