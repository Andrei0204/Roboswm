package org.firstinspires.ftc.teamcode.subsystems.Launch;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class Gate {
    private Servo g;
    //TODO 0.295 ,0.37
    Mode mode = Mode.CLOSE;
    public Gate(HardwareMap hardwareMap) {
        g = hardwareMap.get(Servo.class, "GATE");
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
        OPEN(0.25),
        CLOSE(0.3);
        public double position;

        Mode(double position) {
            this.position = position;
        }
    }
}