package org.firstinspires.ftc.teamcode.Autonomii;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Auto;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;

@Autonomous
public class Auto3TR extends CommandOpMode {
    Auto auto =null;
    Follower follower;

    @Override
    public void init(){
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.localizer.reset();
        follower.update();
        follower.setPose(new Pose(56.4,4.3,Math.PI));
        auto= new Auto(Alliance.RED,hardwareMap);
        auto.update(follower.pose());
    }
    @Override
    public void init_loop(){
        auto.update(follower.pose());
        Scheduler.execute();
        follower.update();
    }
    @Override
    public void start(){
        Command activateShooter = infinite(() -> {
                    auto.r.p.setPos(auto.anglepivot);
                    auto.shootcalc();
                }
        );
        Scheduler.schedule(
                auto.r.t.periodic(),
                auto.r.l.periodic(), auto.r.p.periodic(),
                activateShooter,
                auto.updatespeed(),
                auto.updateTur()
        );
    }

    @Override
    public void loop(){

        auto.update(follower.pose());

        Scheduler.execute();
        follower.update();
        auto.telemetry(telemetry);
        telemetry.update();

    }


}
