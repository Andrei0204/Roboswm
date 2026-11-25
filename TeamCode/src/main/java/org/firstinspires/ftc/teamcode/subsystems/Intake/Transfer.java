package org.firstinspires.ftc.teamcode.subsystems.Intake;

import com.acmerobotics.dashboard.config.Config;

import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class Transfer {
    private DcMotorEx i;
    // Encoder only: the transfer encoder is plugged into the "BR" port (shared with the drivetrain motor).
    private final DcMotorEx enc;
    public static double off = 0;
    public static double idle = 0.5;
    public static double in = 1;
    public static double out = -1;

    // Shooting velocity PID (ticks/s). Only active between shoot() and the next set()/spin*().
    public static double kP = 0.0008, kI = 0, kD = 0.00002, kF = 0.0004;
    public static double targetVelocity = 1700;     // ticks/s while shooting
    public static double maxIntegral = 0.3;         // clamp on the I term (power units)
    public static double minPower = 0, maxPower = 1;
    public static boolean encoderReversed = false;

    private boolean pidActive = false;
    private double integral = 0, lastError = 0, lastTime = 0;
    private boolean firstRun = true;


    public Transfer(HardwareMap hardwareMap) {
        i = hardwareMap.get(DcMotorEx.class, "transferp");
        enc = hardwareMap.get(DcMotorEx.class, "BR");
        set(0);
    }

    /** Plain open-loop power. Also turns the shooting PID off. */
    public void set(double power) {
        pidActive = false;
        i.setPower(power);
    }

    /** Turns the shooting PID on (idempotent). Call update() every loop while it is on. */
    public void shoot() {
        if (pidActive) return;
        pidActive = true;
        resetPid();
    }

    public boolean isShooting() {
        return pidActive;
    }

    private void resetPid() {
        integral = 0;
        lastError = 0;
        firstRun = true;
    }

    /** Current transfer velocity from the BR-port encoder, ticks/s. */
    public double getVelocity() {
        double v = enc.getVelocity();
        return encoderReversed ? -v : v;
    }

    /** Run the PID. Does nothing unless shoot() was called, so it is safe to call every loop. */
    public void update() {
        if (!pidActive) return;
        double now = System.nanoTime() / 1e9;
        double error = targetVelocity - getVelocity();
        double dt = firstRun ? 0 : now - lastTime;

        double d = 0;
        if (!firstRun && dt > 1e-4) {
            integral += error * dt * kI;
            integral = Math.max(-maxIntegral, Math.min(maxIntegral, integral));
            d = (error - lastError) / dt;
        }
        firstRun = false;
        lastError = error;
        lastTime = now;

        double power = kF * targetVelocity + kP * error + integral + kD * d;
        power = Math.max(minPower, Math.min(maxPower, power));
        i.setPower(power);
    }

    public void spinIn() {
        set(in);
    }

    public void spinOut() {
        set(out);
    }

    public void spinOff() {
        set(off);
    }

    public void spinIdle() {
        set(idle);
    }

    public CommandBuilder off() {
        return Commands.instant(() -> set(off));
    }

    public CommandBuilder in() {
        return Commands.instant(() -> set(in));
    }

    public CommandBuilder out() {
        return Commands.instant(() -> set(out));
    }

    public CommandBuilder idle() {
        return Commands.instant(this::spinIdle);
    }

    public CommandBuilder toggleIn() {
        return Commands.instant(() -> {
            if (i.getPower() != 0)
                spinOff();
            else
                spinIn();
        });
    }

    public CommandBuilder toggleOut() {
        return Commands.instant(() -> {
            if (i.getPower() != 0)
                spinOff();
            else
                spinOut();
        });
    }
    public  double getCurrentA(){
        return i.getCurrent(CurrentUnit.AMPS);
    }

    public String getCurrent() {
        return "Intake Motor: " + i.getCurrent(CurrentUnit.AMPS);
    }
}