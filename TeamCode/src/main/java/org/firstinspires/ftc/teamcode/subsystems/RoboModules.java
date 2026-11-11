package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Tavalug;
import org.firstinspires.ftc.teamcode.subsystems.Launch.Gate;
import org.firstinspires.ftc.teamcode.subsystems.Launch.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Launch.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.Launch.Turret;

public class RoboModules {
    public Intake i=null;
    public Turret t=null;
    public Launcher l=null;
    public Pivot p=null;
    public Gate g=null;

    public Tavalug tav=null;

    public RoboModules(HardwareMap hardwareMap){
        i=new Intake(hardwareMap);
        t=new Turret(hardwareMap);
        l=new Launcher(hardwareMap);
        p=new Pivot(hardwareMap);
        g=new Gate(hardwareMap);
        tav=new Tavalug(hardwareMap);
    }
}
