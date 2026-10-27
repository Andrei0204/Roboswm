package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** Transfer that pushes balls into the flywheel: a motor or a CR servo. */
public final class Feeder {
    private final DcMotorSimple actuator;
    private double lastPower = Double.NaN;

    public Feeder(HardwareMap hardwareMap) {
        actuator = hardwareMap.get(DcMotorSimple.class, ShooterConstants.FEEDER_NAME);
        if (actuator instanceof DcMotor) {
            DcMotor motor = (DcMotor) actuator;
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void set(boolean feeding) {
        double power = feeding ? ShooterConstants.FEEDER_FIRE_POWER : ShooterConstants.FEEDER_IDLE_POWER;
        if (!Double.isNaN(lastPower) && power == lastPower) return;
        lastPower = power;
        actuator.setPower(power);
    }

    /** Tuning only: any power, e.g. reverse to check direction or clear a jam. */
    public void setRawPower(double power) {
        if (!Double.isNaN(lastPower) && power == lastPower) return;
        lastPower = power;
        actuator.setPower(power);
    }

    public void stop() {
        lastPower = 0.0;
        actuator.setPower(0.0);
    }
}
