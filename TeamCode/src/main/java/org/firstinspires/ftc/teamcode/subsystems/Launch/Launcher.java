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
    public static double kV = 0.0202857143; // power per RPS -- start near 1/(free-speed RPS at the flywheel), then tune
    public static double kS = 0.05;       // static friction/loss feedforwardF
    public static double kP = 0.05;
    public static double kI = 0.004;
    public static double kD = 0.0;

    // ---------------- Bang-bang / braking thresholds ----------------
    public static double SPIN_UP_ERROR_RPS = 2; // this far below target -> full power instead of PID
    public static double BRAKE_ERROR_RPS = 6;   // this far above target -> actively brake instead of coasting
    public static double BRAKE_POWER = 0.6;     // magnitude of reverse power used while braking -- keep modest

    public static double TOLERANCE_RPS = 2;      // "at speed" band

    // Flip this ONE flag if positive power does NOT correspond to a positive GetRPS() reading
    // on the bench -- see the note in chat about verifying this before trusting it.
    public static boolean INVERT_POWER = true;

    private final DcMotorEx LAUNCHER;
    private final DcMotorEx LAUNCHER2;

    public boolean forceOff = false;
    private double targetRPS = 0;
    private Mode mode = Mode.OFF;

    public static double m = 9.5, n = 250;
    public static double mf = 12, nf = 300;

    private double integralSum = 0;
    private double lastError = 0;
    private double lastTime = 0;

    public Launcher(HardwareMap hardwareMap) {
        LAUNCHER = hardwareMap.get(DcMotorEx.class, "LAU");
        LAUNCHER.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LAUNCHER.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LAUNCHER2 = hardwareMap.get(DcMotorEx.class, "LAU2");
        LAUNCHER2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LAUNCHER2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LAUNCHER2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public double GetRPS() {
        return LAUNCHER.getVelocity() / 28.0 * (20.0 / 30.0);
    }

    public void off() {
        targetRPS = 0;
        mode = Mode.OFF;
        integralSum = 0;
    }

    public double getTargetRPS() {
        return targetRPS;
    }

    /** True once measured RPS is within TOLERANCE_RPS of target. */
    public boolean atSpeed() {
        return mode == Mode.POSITION && Math.abs(targetRPS - GetRPS()) <= TOLERANCE_RPS;
    }

    public void setTargetRPS(double targetRPS) {
        if (Double.isNaN(targetRPS)) return;
        if (targetRPS != 0) {
            mode = Mode.POSITION;
            this.targetRPS = targetRPS;
        } else {
            this.targetRPS = 0;
            mode = Mode.OFF;
        }
    }

    public void updateshooting(double x) {
        if (Double.isNaN(x)) return;
        setTargetRPS(x != 0 ? conversion(x) : 0);
    }

    public double conversion(double x) {
        if (Tele.farshooting == 0) return (m * x + n) / 60;
        return (mf * x + nf) / 60;
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
            LAUNCHER2.setPower(0);
            return;
        }

        switch (mode) {
            case POSITION:
                runVelocityControl();
                break;
            case OFF:
            default:
                LAUNCHER.setPower(0);
                LAUNCHER2.setPower(0);
                integralSum = 0;
                break;
        }
    }

    private void runVelocityControl() {
        double t = now();
        double dt = lastTime == 0 ? 0.02 : clamp(t - lastTime, 1e-3, 0.1);
        lastTime = t;

        double currentRPS = GetRPS();
        double error = targetRPS - currentRPS;

        double power;
        if (error > SPIN_UP_ERROR_RPS) {
            // Way below target -- full send, no point waiting on the PID to ramp up.
            power = 1.0;
            integralSum = 0;
        } else if (error < -BRAKE_ERROR_RPS) {
            // Way above target (it just dropped a lot) -- coasting down is too slow, brake actively.
            power = -BRAKE_POWER;
            integralSum = 0;
        } else {
            double maxIntegral = kI > 1e-9 ? 0.3 / kI : 0;
            integralSum = clamp(integralSum + error * dt, -maxIntegral, maxIntegral);
            double derivative = (error - lastError) / dt;

            double ff = kV * targetRPS + kS;
            double fb = kP * error + kI * integralSum + kD * derivative;
            power = ff + fb;
        }

        if (INVERT_POWER) power = -power;
        power = clamp(power, -1.0, 1.0);

        LAUNCHER.setPower(power);
        LAUNCHER2.setPower(power);
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