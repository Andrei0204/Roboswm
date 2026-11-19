package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Intake.BallSensor;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Flower;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Tavalug;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Launch.Gate;
import org.firstinspires.ftc.teamcode.subsystems.Launch.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Launch.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.Launch.Turret;

public class RoboModules {
    public Intake i=null;
    public Turret t=null;
    public Transfer tr=null;    
    public Launcher l=null;
    public Pivot p=null;
    public Gate g=null;
    public Tavalug tav=null;
    public IntakeSystem is=null;

    public Flower f = null;
    public BallSensor bs = null;
    public Indicator led = null;


    public RoboModules(HardwareMap hardwareMap){
        i=new Intake(hardwareMap);
        t=new Turret(hardwareMap);
        l=new Launcher(hardwareMap);
        p=new Pivot(hardwareMap);
        g=new Gate(hardwareMap);
        tr=new Transfer(hardwareMap);
        tav=new Tavalug(hardwareMap);
        tr=new Transfer(hardwareMap);
        is=new IntakeSystem(i, tr, tav);
        f=new Flower(hardwareMap);
        bs=new BallSensor(hardwareMap);
        led=new Indicator(hardwareMap);
    }
}
