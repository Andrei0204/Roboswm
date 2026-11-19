
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Hood Servo Config", group = "Test")
public class servotest extends LinearOpMode {

    public static double hoodPosition = 0.24;

    private Servo hoodp;

    @Override
    public void runOpMode() {

        hoodp = hardwareMap.get(Servo.class, "hoodp");

        FtcDashboard dashboard = FtcDashboard.getInstance();

        telemetry.addData("Status", "Hood initialized");
        telemetry.addData("Position", hoodPosition);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            hoodp.setPosition(hoodPosition);

            telemetry.addData("Hood Position", hoodPosition);
            telemetry.update();

            idle();
        }
    }
}
