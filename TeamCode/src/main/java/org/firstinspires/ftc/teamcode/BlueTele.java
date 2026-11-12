package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Tele;
import org.firstinspires.ftc.teamcode.util.Alliance;

@TeleOp
public class BlueTele extends Tele {
    public BlueTele() {
        super(Alliance.BLUE);
    }
}