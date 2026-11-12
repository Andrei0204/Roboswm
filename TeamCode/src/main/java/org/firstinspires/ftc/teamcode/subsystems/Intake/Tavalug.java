package org.firstinspires.ftc.teamcode.subsystems.Intake;

import static com.pedropathing.ivy.commands.Commands.*;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class Tavalug {
    private Servo tv1;
    //TODO 0.34,0.28,0.46

    Mode mode = Mode.INIT;
    public Tavalug(HardwareMap hardwareMap) {
        tv1 = hardwareMap.get(Servo.class, "intakes");
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
        tv1.setPosition(mode.position);
    }
    public Command setcom(){
        return instant(()->{
            tv1.setPosition(mode.position);
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
        INIT(0.7), //0.7 buna
        /** Collects every ball (POLLEN and NECTAR). */
        COLLECT_ALL(0.61), // 0.57 valoare buna

        COLLECT_POLLEN(0.75),

        COLLECT_FLOARE(0.61);;
        public double position;

        Mode(double position) {
            this.position = position;
        }
    }
}