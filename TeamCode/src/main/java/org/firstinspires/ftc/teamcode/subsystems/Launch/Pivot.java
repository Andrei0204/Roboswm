package org.firstinspires.ftc.teamcode.subsystems.Launch;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class Pivot {
    private Servo p;
    public static double target = 0;
    public static double pos = -1;
    public static double angle = 0;
    Mode mode = Mode.POSITION;
    //TODO lim = 0.1,0.62

    public Pivot(HardwareMap hardwareMap) {
        p = hardwareMap.get(Servo.class, "PIV");
        p.setPosition(0);
    }



    public void setPos(double pos) {
        mode=Mode.POSITION;
        Pivot.pos    = pos;
    }
    public double getPos()
    {
        return pos;
    }
    public Command periodic() {
        return infinite(() -> {
            target = pos;
            set(target);

        });
    }
    public void set(double set){
            p.setPosition(set);
    }
    public enum Mode {
        POSITION,
    }
}