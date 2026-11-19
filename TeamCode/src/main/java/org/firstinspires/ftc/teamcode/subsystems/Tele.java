package org.firstinspires.ftc.teamcode.subsystems;


import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.repeat;
import static com.pedropathing.ivy.groups.Groups.sequential;

import static org.firstinspires.ftc.teamcode.subsystems.Intake.Tavalug.Mode.COLLECT_FLOARE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;


import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Flower;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.Launch.Gate;

import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;
import org.firstinspires.ftc.teamcode.util.ShootingCalc;
import org.firstinspires.ftc.teamcode.util.ShotTable;

@Config
public class Tele extends CommandOpMode {

    public static double flywheelspeed = 0, anglepivot = 0.24, anglenorm = 0, anglenou = 1;
    Alliance alliance;
    Pose Goal;
    Pose robopose= null;
    Pose turpose=null;
    public static boolean useShotTable = true;
    // Turret pivot offset from the robot center, robot frame (x forward, y left), inches. Tune from your CAD.
    public static double TURRET_OFFSET_X = -3, TURRET_OFFSET_Y = 0;
    private boolean lastLeftTrigger = false;

    boolean flowerDown = false;

    public static double GOAL_FLASH_MS = 600;
    long goalFlashUntil = 0;
    Indicator.Color goalFlashColor = Indicator.Color.BLUE;
    RoboModules r;
    double distance;
    IntakeSystem is;

    Follower follower;

    ElapsedTime timp;

    public Tele(Alliance alliance) {
        this.alliance = alliance;

    }

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        timp = new ElapsedTime();
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.localizer.reset();
        follower.setPose(alliance == Alliance.RED ? ShootingCalc.RED_START : ShootingCalc.BLUE_START);

        if (alliance == Alliance.RED)
            Goal = ShootingCalc.RED_AUD;
        else
            Goal = ShootingCalc.BLUE_AUD;

        // Created in init (not start) so the turret encoder can be reset during init_loop.
        r = new RoboModules(hardwareMap);
        is = r.is;
    }

    @Override
    public void init_loop() {
        if (gamepad1.y) r.t.resetEncoder();
        telemetry.addData("turrey",r.t.getRawAngleDegrees());
        telemetry.update();
    }


    @Override
    public void start() {
//        follower.setPose(OpModeStorage.autonomousEndPose);
//        follower.update();
        robopose =follower.pose();
        r.p.set(0.24);


        Command activateShooter = infinite(() -> {
                    r.p.setPos(anglepivot);
                    shootcalc();
                }
        );
        Scheduler.schedule(
                r.t.periodic(),
                r.l.periodic(), r.p.periodic(),
                activateShooter,
                updatespeed()

        );
    }

    @Override
    public void loop() {
        robopose =follower.pose();
        turpose = getTurretPose(robopose);


        follower.update();
        Scheduler.execute();
        teleopdrive();

        turret();
        updateIndicator();

        telemetry();
        if (gamepad1.dpadDownWasPressed()) {
            follower.setPose(alliance == Alliance.RED ? ShootingCalc.RED_START : ShootingCalc.BLUE_START);
        }
        if (gamepad1.right_trigger > 0.2)
            Scheduler.schedule(launch());

        if (gamepad1.rightBumperWasPressed())
            Scheduler.schedule(toggleIntake(true));
        if (gamepad1.rightBumperWasReleased())
            Scheduler.schedule(toggleIntake(false));
        // Failsafe: hold left bumper to reverse only the intake, release to stop.


        boolean leftTrigger = gamepad1.left_trigger > 0.5;

        if (leftTrigger && !lastLeftTrigger)
            Scheduler.schedule(reverseIntake(true));

        if (!leftTrigger && lastLeftTrigger)
            Scheduler.schedule(reverseIntake(false));

        lastLeftTrigger = leftTrigger;


        if(gamepad1.bWasPressed()){
            onGoalSwitched();
            Goal=ShootingCalc.other(alliance,Goal);
            flashGoal();
        }
        if(gamepad2.bWasPressed()){
            onGoalSwitched();
            Goal=ShootingCalc.other(alliance,Goal);
            flashGoal();
        }


        if (gamepad1.leftBumperWasPressed()) {
            flowerDown = !flowerDown;

            if (flowerDown) {
                r.f.setMode(Flower.Mode.DOWN);
                r.tav.setMode(COLLECT_FLOARE);
            } else {
                r.f.setMode(Flower.Mode.UP);
                r.tav.setMode(COLLECT_FLOARE);
            }
        }
    }

    // ---- DIAG state ----
    boolean hasPrevCmd = false, switchPending = false;
    double prevCmdDeg = 0, lastWrapFrom = 0, lastWrapTo = 0;
    int wrapCount = 0, switchCount = 0;
    double switchFromCmd = 0, switchFromDist = 0, switchFromTurret = 0;

    static double wrap180(double d) {
        while (d > 180) d -= 360;
        while (d < -180) d += 360;
        return d;
    }

    String goalName() {
        if (Goal == null) return "null";
        boolean aud = Goal.x() == ShootingCalc.RED_AUD.x() && Goal.y() == ShootingCalc.RED_AUD.y()
                || Goal.x() == ShootingCalc.BLUE_AUD.x() && Goal.y() == ShootingCalc.BLUE_AUD.y();
        return aud ? "AUDIENCE" : "REAR";
    }

    void telemetry() {
        double cmdDeg = Math.toDegrees(anglenorm);
        double turDeg = r.t.getRawAngleDegrees();
        double turErr = cmdDeg - turDeg;
        double bearingDeg = Math.toDegrees(anglenou);
        double headingDeg = robopose == null ? Double.NaN : Math.toDegrees(robopose.heading());
        boolean clamped = distance < ShotTable.minDistance() || distance > ShotTable.maxDistance();

        // ===== GOAL =====
        telemetry.addLine("=== GOAL ===");
        telemetry.addData("goal", "%s %s  (%.1f, %.1f)", alliance, goalName(),
                Goal == null ? Double.NaN : Goal.x(), Goal == null ? Double.NaN : Goal.y());
        telemetry.addData("switches", switchCount);

        // ===== POSE (odometry) =====
        telemetry.addLine("=== POSE ===");
        telemetry.addData("robot x,y", "%.1f, %.1f", robopose == null ? Double.NaN : robopose.x(),
                robopose == null ? Double.NaN : robopose.y());
        telemetry.addData("heading deg", "%.1f", headingDeg);
        telemetry.addData("turret pivot x,y", "%.1f, %.1f", turpose == null ? Double.NaN : turpose.x(),
                turpose == null ? Double.NaN : turpose.y());
        telemetry.addData("auto end pose", OpModeStorage.autonomousEndPose);

        // ===== TURRET =====
        telemetry.addLine("=== TURRET ===");
        telemetry.addData("field bearing to goal deg", "%.1f", bearingDeg);
        telemetry.addData("cmd = bearing - heading (wrapped)", "%.1f", cmdDeg);
        telemetry.addData("turret actual deg (encoder)", "%.1f", turDeg);
        telemetry.addData("turret error deg", "%.1f", turErr);
        telemetry.addData("turret at target / mode", "%s / %s", r.t.atTarget(), r.t.getMode());
        telemetry.addData("WRAP>180 jumps", "%d  (last %.0f -> %.0f)", wrapCount, lastWrapFrom, lastWrapTo);
        telemetry.addData("ENCODER OUT OF +-180?", Math.abs(turDeg) > 180 ? "YES -> will unwind " + Math.round(turErr) + " deg" : "no");
        if (Math.abs(turErr) > 180) telemetry.addData("!!! TURRET MUST SPIN", "%.0f deg (wrap problem)", turErr);
        if (switchCount > 0) {
            telemetry.addData("last switch: cmd before -> now", "%.1f -> %.1f  (jump %.1f)",
                    switchFromCmd, cmdDeg, wrap180(cmdDeg - switchFromCmd));
            telemetry.addData("last switch: dist before -> now", "%.1f -> %.1f in", switchFromDist, distance);
        }

        // ===== SHOT TABLE =====
        telemetry.addLine("=== SHOT ===");
        telemetry.addData("useShotTable", useShotTable);
        telemetry.addData("distance in", "%.1f  (table %.1f..%.1f)", distance, ShotTable.minDistance(), ShotTable.maxDistance());
        telemetry.addData("TABLE CLAMPED?", clamped ? "YES - outside table, rpm/hood frozen" : "no");
        telemetry.addData("target rpm", "%.0f", flywheelspeed);
        telemetry.addData("actual rpm", "%.0f", r.l.getRPM());
        telemetry.addData("rpm error", "%.0f  atSpeed=%s", flywheelspeed - r.l.getRPM(), r.l.atSpeed());
        telemetry.addData("hood target / actual", "%.3f / %.3f", anglepivot, r.p.getPos());

        // ===== OTHER =====
        telemetry.addLine("=== OTHER ===");
        telemetry.addData("floare",r.f.getMode());
        telemetry.addData("sensor", r.bs.debug());
        telemetry.update();
    }



    /** Call right after the goal changes: flashes BLUE for audience, VIOLET for rear. */
    void flashGoal() {
        boolean aud = Goal.x() == ShootingCalc.RED_AUD.x() && Goal.y() == ShootingCalc.RED_AUD.y()
                || Goal.x() == ShootingCalc.BLUE_AUD.x() && Goal.y() == ShootingCalc.BLUE_AUD.y();
        goalFlashColor = aud ? Indicator.Color.BLUE : Indicator.Color.VIOLET;
        goalFlashUntil = System.currentTimeMillis() + (long) GOAL_FLASH_MS;
    }

    /**
     * Driver light, highest priority first:
     *  BLUE / VIOLET = just switched goal (audience / rear), briefly
     *  GREEN         = 3 pollen loaded (yellow steadily in front of the sensor)
     *  OFF           = otherwise
     */
    void updateIndicator() {
        r.bs.update();
        if (System.currentTimeMillis() < goalFlashUntil) r.led.set(goalFlashColor);
        else if (r.bs.isFull()) r.led.set(Indicator.Color.GREEN);
        else r.led.set(Indicator.Color.OFF);
    }

    public Command launch() {
        return sequential(
                r.g.setcom(Gate.Mode.OPEN),
                is.run(1),
                waitMs(40),
                is.run(2),
                waitMs(600),
                r.g.setcom(Gate.Mode.CLOSE),
                is.run(0)
        )
                .requiring(is)
                .setPriority(10)
                .setConflictBehavior(ConflictBehavior.CANCEL);
    }

    /** Hold-to-shoot: gate open + intake on while held, gate closed + intake off on release. */
    public Command toggleIntake(boolean held) {
        return sequential(
//                r.g.setcom(held ? Gate.Mode.OPEN : Gate.Mode.CLOSE),
                is.run(held ? 1 : 0)
        )
                .requiring(is)
                .setPriority(held ? 10 : 11)
                .setConflictBehavior(ConflictBehavior.CANCEL);
    }

    /** Failsafe: reverse only the intake while held, stop on release. Overrides shooting/intake commands. */
    public Command reverseIntake(boolean held) {
        return sequential(
                is.run(held ? 4 : 0)
        )
                .requiring(is)
                .setPriority(held ? 12 : 13)
                .setConflictBehavior(ConflictBehavior.CANCEL);
    }

    public Command updatespeed() {
        return
                infinite(() ->
                        r.l.updateshooting(flywheelspeed)
                );
    }

    /** Sets flywheel RPM + hood from the distance to the goal when useShotTable is on. */
    public void shootcalc() {
        if (!useShotTable || robopose == null || Goal == null) return;
        distance = Math.hypot(Goal.x() - turpose.x(), Goal.y() - turpose.y());
        ShotTable.Shot shot = ShotTable.get(Math.abs(distance));
        flywheelspeed = shot.rpm;
        anglepivot = shot.hood;
    }

    /**
     * Turret pivot position on the field = robot pose + the turret offset rotated by the robot heading.
     * Offset is in the robot frame (x forward, y left), same units as the field (inches).
     */
    Pose getTurretPose(Pose robot) {
        double h = robot.heading();
        double cos = Math.cos(h), sin = Math.sin(h);
        return new Pose(
                robot.x() + TURRET_OFFSET_X * cos - TURRET_OFFSET_Y * sin,
                robot.y() + TURRET_OFFSET_X * sin + TURRET_OFFSET_Y * cos,
                h);
    }

    void turret() {

        // Aim from the turret's own position, not from the robot center.
        double dx = Goal.x() - turpose.x();
        double dy = Goal.y() - turpose.y();
        // Field-frame bearing from the turret to the goal.
        double theta = Math.atan2(dy, dx);

        anglenou = theta;
        // Turret angle relative to the robot: 0 = robot forward, CCW positive.
        anglenorm = theta - robopose.heading();

        // Wrap to [-180, 180] degrees.
        while (anglenorm > Math.PI) anglenorm -= 2 * Math.PI;
        while (anglenorm < -Math.PI) anglenorm += 2 * Math.PI;

        double cmdDeg = Math.toDegrees(anglenorm);
        // DIAG: detect the command wrapping across +-180 (turret would then spin ~360 deg the other way).
        if (hasPrevCmd && Math.abs(cmdDeg - prevCmdDeg) > 180) {
            wrapCount++;
            lastWrapFrom = prevCmdDeg;
            lastWrapTo = cmdDeg;
        }
        prevCmdDeg = cmdDeg;
        hasPrevCmd = true;
        r.t.setTargetDegrees(cmdDeg);
    }

    /** DIAG: call right after Goal changes; remembers the turret command before/after the switch. */
    void onGoalSwitched() {
        switchCount++;
        switchFromCmd = prevCmdDeg;
        switchFromDist = distance;
        switchFromTurret = r.t.getRawAngleDegrees();
        switchPending = true; // the "after" values are captured on the next loop in turret()/telemetry
    }


    public void teleopdrive() {
        follower.manual(-gamepad1.left_stick_y,
              -  gamepad1.left_stick_x,
               Math.abs(gamepad1.right_stick_x)<0.2 ? 0 : -gamepad1.right_stick_x
        );
    }


}
