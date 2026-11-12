package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Tele;
import org.firstinspires.ftc.teamcode.util.Alliance;

@TeleOp
public class RedTele extends Tele {
    public RedTele() {
        super(Alliance.RED);
    }
}