package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
@TeleOp
public class servotest extends OpMode {
    ServoImplEx s;

    public static double POSITION = 0;


    @Override
    public void init() {
        s = hardwareMap.get(ServoImplEx.class, "turp");
        s.setPwmRange(new PwmControl.PwmRange(550,2450));
    }

    @Override
    public void loop() {
        s.setPosition(POSITION);
    }


}
