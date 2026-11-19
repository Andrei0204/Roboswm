package org.firstinspires.ftc.teamcode.subsystems.Launch;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.acmerobotics.dashboard.config.Config;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Gate {
    private Servo g;
    //TODO 0.295 ,0.37
    Mode mode = Mode.CLOSE;
    public Gate(HardwareMap hardwareMap) {
        g = hardwareMap.get(Servo.class, "gatep");
        setpos();
    }

    public Mode getMode() {
        return mode;
    }
    public void setMode(Mode mode) {
        this.mode = mode;setpos();
    }
    public void setpos(){
        g.setPosition(mode.position);
    }
    public Command setcom(Mode mode ){
        return instant(()->{
            this.mode=mode;
            g.setPosition(mode.position);
        });
    }

    public enum Mode {
        OPEN(0.38),
        CLOSE(0.55);
        public double position;

        Mode(double position) {
            this.position = position;
        }
    }
}