package org.firstinspires.ftc.teamcode.subsystems.Intake;

import static com.pedropathing.ivy.commands.Commands.*;

import com.acmerobotics.dashboard.config.Config;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Flower {
    private Servo tv2;
    //TODO 0.34,0.28,0.46

    Mode mode = Mode.UP;
    public Flower(HardwareMap hardwareMap) {
        tv2 = hardwareMap.get(Servo.class, "flowers");
        setpos();
    }
    public Mode getMode() {
        return mode;
    }

    public void setMode(Mode mode) {
        this.mode = mode;
        setpos();
    }
    public void setpos(){
        tv2.setPosition(mode.position);
    }
    public Command setcom(){
        return instant(()->{
            tv2.setPosition(mode.position);
        });
    }
    public Command setModeC(Mode mode)
    {
        return instant(()->{
            this.mode=mode;
            setpos();
        });
    }

    public enum Mode {
        UP(1), //0.7 buna
        /** Collects every ball (POLLEN and NECTAR). */
        DOWN(0.57); // vedem



        public double position;

        Mode(double position) {
            this.position = position;
        }
    }
}