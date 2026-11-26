package org.firstinspires.ftc.teamcode.subsystems;


import static com.pedropathing.ivy.commands.Commands.*;

import static com.pedropathing.ivy.groups.Groups.sequential;

import static org.firstinspires.ftc.teamcode.subsystems.Intake.Tavalug.Mode.COLLECT_FLOARE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;


import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;

import com.pedropathing.follower.ManualDrive;
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
import org.firstinspires.ftc.teamcode.subsystems.Launch.Launcher;

import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;
import org.firstinspires.ftc.teamcode.util.ShootingCalc;
import org.firstinspires.ftc.teamcode.util.ShotTable;

@Config
public class Tele extends CommandOpMode {

    public static double flywheelspeed = 0, anglepivot = 0.32, anglenorm = 0, anglenou = 1;
    // One trigger press = gate open + continuous feed for this long (same as holding the trigger).
    // Make it long enough to empty a full load; used by ShooterTuneTele too.
    public static long LAUNCH_MS = 600;
    Alliance alliance;
    Pose Goal;

    Pose robopose = null;
    Pose turpose = null;
    public static boolean useShotTable = true;
    // Turret pivot offset from the robot center, robot frame (x forward, y left), inches. Tune from your CAD.
    public static double TURRET_OFFSET_X = -2.6, TURRET_OFFSET_Y = 0;

    boolean flowerDown = false;
    boolean lastFlowerBtn = false;
    /** Field y (in) where the goal auto-switches between audience and rear. Tunable on the dashboard. */
    public static double FAR_Y = 72;
    /** When true the goal switches by itself as the robot crosses FAR_Y (B still toggles, until the next crossing). */
    public static boolean AUTO_SIDE = true;

    /**
     * Fixed shot used while gamepad2.a manual mode is on.
     */

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
        telemetry.addData("turrey", r.t.getRawAngleDegrees());
        telemetry.update();
    }


    @Override
    public void start() {
        if (OpModeStorage.autonomousEndPose.x() != 0)
            follower.setPose(OpModeStorage.autonomousEndPose);
        follower.update();
        robopose = follower.pose();
        r.p.set(0.24);


        OpModeStorage.autonomousEndPose = new Pose(0, 0);
    }

    @Override
    public void loop() {
        robopose = follower.pose();
        turpose = getTurretPose(robopose);
        autoSwitchGoal();

        follower.update();
        Scheduler.execute(); 

        teleopdrive();
        update();
        telemetry();
        if (gamepad1.dpadDownWasPressed()) {

            follower.setPose(alliance == Alliance.RED ? ShootingCalc.RED_START : ShootingCalc.BLUE_START);
        }
        if (gamepad1.dpadUpWasPressed()) {

            follower.setPose(alliance == Alliance.RED ? ShootingCalc.RED_START_new : ShootingCalc.BLUE_START_new);
        }
        if (gamepad1.right_trigger > 0.2)
            Scheduler.schedule(launch());

        // D-pad up toggles the flower between DOWN and UP.
        // Edge detection: toggles once per press, not every loop while held.
        boolean flowerBtn = gamepad1.left_trigger_pressed;
        if (flowerBtn && !lastFlowerBtn) {
            r.f.setMode(r.f.getMode() == Flower.Mode.DOWN ? Flower.Mode.UP : Flower.Mode.DOWN);
        }
        lastFlowerBtn = flowerBtn;

        // Intake runs while right bumper is held (left bumper = reverse) OR while a shot is feeding.
        // Set directly every loop, so shooting can never block or override it.
        is.set(feeding ? 2 : gamepad1.right_bumper ? 1 : gamepad1.left_bumper ? 4 : 0);

        if (gamepad1.bWasPressed() || gamepad2.bWasPressed()) {
            onGoalSwitched();
            Goal = ShootingCalc.other(alliance, Goal);
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

    String  goalName() {
        if (Goal == null) return "null";
        boolean aud = Goal.x() == ShootingCalc.RED_AUD.x() && Goal.y() == ShootingCalc.RED_AUD.y()
                || Goal.x() == ShootingCalc.BLUE_AUD.x() && Goal.y() == ShootingCalc.BLUE_AUD.y();
        return aud ? "AUDIENCE" : "REAR";
    }

    /**
     * All per-loop subsystem updates, run directly (no scheduler/commands).
     */
    void update() {
        r.p.setPos(anglepivot);
        is.updatePid();
        shootcalc();
        r.l.updateshooting(flywheelspeed);
        turret();
        r.t.tester();
        r.l.tester();
        r.p.tester();
        r.bs.update();
        rpmLight();
    }

    void telemetry() {
        double cmdDeg = Math.toDegrees(anglenorm);
        double turDeg = r.t.getRawAngleDegrees();
        double turErr = cmdDeg - turDeg;
        double bearingDeg = Math.toDegrees(anglenou);
        double headingDeg = robopose == null ? Double.NaN : Math.toDegrees(robopose.heading());
        boolean clamped = distance < ShotTable.minDistance() || distance > ShotTable.maxDistance();

        // ===== TRANSFER =====
        telemetry.addLine("=== TRANSFER ===");
        telemetry.addData("transfer speed", "%.0f ticks/s", r.tr.getVelocity());
        telemetry.addData("transfer target", r.tr.isShooting()
                ? String.format("%.0f ticks/s", org.firstinspires.ftc.teamcode.subsystems.Intake.Transfer.targetVelocity)
                : "off (PID inactive)");

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
        telemetry.addData("turret at target", r.t.atTarget());
        telemetry.addData("side offset deg", "%+.1f  (robot x - goal x = %.1f)",
                sideOffsetDeg(), robopose.x() - Goal.x());
        telemetry.addData("WRAP>180 jumps", "%d  (last %.0f -> %.0f)", wrapCount, lastWrapFrom, lastWrapTo);
        telemetry.addData("ENCODER OUT OF +-180?", Math.abs(turDeg) > 180 ? "YES -> will unwind " + Math.round(turErr) + " deg" : "no");
        if (Math.abs(turErr) > 180)
            telemetry.addData("!!! TURRET MUST SPIN", "%.0f deg (wrap problem)", turErr);
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
        telemetry.addData("floare", r.f.getMode());
        telemetry.addData("sensor", r.bs.debug());
        telemetry.update();
    }


    /** Driver light: AMBER = shooting at the audience goal, BLUE = rear goal. */
    void rpmLight() {
        r.led.set(goalName().equals("AUDIENCE") ? Indicator.Color.AMBER : Indicator.Color.BLUE);
    }

    Command setManualShot(double rpm, double hood) {
        return instant(() -> {
            useShotTable = false;
            flywheelspeed = rpm;
            anglepivot = hood;
        });
    }

    /**
     * Driver light, highest priority first:
     * BLUE / VIOLET = just switched goal (audience / rear), briefly
     * GREEN         = 3 pollen loaded (yellow steadily in front of the sensor)
     * OFF           = otherwise
     */


    /** True while a shot is feeding balls; the intake is turned on from loop() while this is set. */
    boolean feeding = false;

    /**
     * One press: open the gate and feed continuously for LAUNCH_MS, then close.
     * A tap now does the same as holding the trigger used to (one long uninterrupted feed).
     * Only touches the gate; the intake itself is driven from loop(), so it never fights the bumper.
     * Re-scheduling while it is running is ignored (CANCEL); holding longer just starts another burst.
     */
    private Command launch() {
        return sequential(
                r.g.setcom(Gate.Mode.OPEN),
                waitMs(70),
                instant(() -> feeding = true),
                waitMs(20),
                is.setIn(1),
                waitMs(org.firstinspires.ftc.teamcode.subsystems.Tele.LAUNCH_MS),
                r.g.setcom(Gate.Mode.CLOSE),
                instant(() -> feeding = false)
        )
                .requiring(r.g)
                .setPriority(10)
                .setConflictBehavior(ConflictBehavior.CANCEL);
    }
    /**
     * Sets flywheel RPM + hood from the distance to the goal when useShotTable is on.
     */
    public void shootcalc() {
        if (!useShotTable || robopose == null || Goal == null) return;
        Pose aim = Goal;
        distance = Math.hypot(aim.x() - turpose.x(), aim.y() - turpose.y());
        // Robot past the goal's y (y > goal y on audience, y < goal y on rear): use the smallest shot in the table.
        boolean aud = goalName().equals("AUDIENCE");
        boolean farSide = aud ? robopose.y() > Goal.y() : robopose.y() < Goal.y();
        ShotTable.Shot shot = farSide ? ShotTable.get(ShotTable.minDistance()) : ShotTable.get(Math.abs(distance));
        flywheelspeed = shot.rpm;
        anglepivot = shot.hood;
    }

    Pose getTurretPose(Pose robot) {
        double h = robot.heading();
        double cos = Math.cos(h), sin = Math.sin(h);
        return new Pose(
                robot.x() + TURRET_OFFSET_X * cos - TURRET_OFFSET_Y * sin,
                robot.y() + TURRET_OFFSET_X * sin + TURRET_OFFSET_Y * cos,
                h);
    }

    // Extra turret angle when the robot is far to one side of the goal (along field x).
    // If it makes shots miss MORE on a side, flip that side's sign.
    public static double SIDE_X_THRESHOLD = 15;
    public static double LEFT_SIDE_OFFSET_DEG = 0;   // robot x < goal x - threshold
    public static double RIGHT_SIDE_OFFSET_DEG =0; // robot x > goal x + threshold

    /** The side correction currently being added to the turret angle, in degrees. */
    double sideOffsetDeg() {
        double dxGoal = robopose.x() - Goal.x();
        if (dxGoal < -SIDE_X_THRESHOLD) return LEFT_SIDE_OFFSET_DEG;
        if (dxGoal > SIDE_X_THRESHOLD) return RIGHT_SIDE_OFFSET_DEG;
        return 0;
    }

    void turret() {

        // Aim from the turret's own position, not from the robot center.
        Pose aim = Goal;
        double dx = aim.x() - turpose.x();
        double dy = aim.y() - turpose.y();
        // Field-frame bearing from the turret to the goal.
        double theta = Math.atan2(dy, dx);

        anglenou = theta;
        // Turret angle relative to the robot: 0 = robot forward, CCW positive.
        anglenorm = theta - robopose.heading();

        // Wrap to [-180, 180] degrees.
        while (anglenorm > Math.PI) anglenorm -= 2 * Math.PI;
        while (anglenorm < -Math.PI) anglenorm += 2 * Math.PI;

        double cmdDeg = Math.toDegrees(anglenorm) + sideOffsetDeg();
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
    Boolean lastAudSide = null;

    /** Switches the goal when the robot crosses FAR_Y (audience side y < FAR_Y, rear side y > FAR_Y). B can still override in between. */
    void autoSwitchGoal() {
        if (!AUTO_SIDE || robopose == null || Goal == null) return;
        boolean audSide = robopose.y() < FAR_Y;
        if (lastAudSide != null && audSide != lastAudSide
                && audSide != goalName().equals("AUDIENCE")) {
            onGoalSwitched();
            Goal = ShootingCalc.other(alliance, Goal);
        }
        lastAudSide = audSide;
    }

    void onGoalSwitched() {
        switchCount++;
        switchFromCmd = prevCmdDeg;
        switchFromDist = distance;
        switchFromTurret = r.t.getRawAngleDegrees();
        switchPending = true; // the "after" values are captured on the next loop in turret()/telemetry
    }


    public void teleopdrive() {
        follower.manual(-gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                Math.abs(gamepad1.right_stick_x) < 0.2 ? 0 : -gamepad1.right_stick_x
        );
    }


}
