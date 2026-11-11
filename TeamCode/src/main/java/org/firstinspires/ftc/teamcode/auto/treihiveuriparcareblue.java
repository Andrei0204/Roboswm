package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.Alliance;

@Autonomous(name = "treihiveuriparcare BLUE", group = "BioBuzz", preselectTeleOp = "Shoot On The Move")
public class treihiveuriparcareblue extends treihiveuriparcareBase {
    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }
}
