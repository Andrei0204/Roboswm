package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "HoodTest")
public class servotest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Servo hoodp = hardwareMap.get(Servo.class, "hoodp");

        waitForStart();

        hoodp.setPosition(0.36);

        while (opModeIsActive()) {
            telemetry.addData("hoodp", hoodp.getPosition());
            telemetry.update();
        }
    }
}