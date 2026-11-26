package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.Launch.Gate;
import org.firstinspires.ftc.teamcode.subsystems.RoboModules;
import org.firstinspires.ftc.teamcode.subsystems.Tele;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;
import org.firstinspires.ftc.teamcode.util.ShootingCalc;
import org.firstinspires.ftc.teamcode.util.ShotTable;

/**
 * Shooter tuning TeleOp: you set flywheel RPM and hood by hand, drive around, shoot,
 * and the telemetry shows everything needed to build a ShotTable entry.
 *
 * INIT:  X = RED, B = BLUE (default BLUE), Y = zero turret encoder
 *
 * GAMEPAD 1
 *   Sticks ........ drive (same as Tele)
 *   Dpad up/down .. RPM  +/- step
 *   Dpad right/left Hood +/- step
 *   A ............. toggle fine / coarse step
 *   X ............. flywheel on / off
 *   Y ............. turret auto-aim on / off (off = turret holds 0 deg)
 *   B ............. swap goal (audience / rear), same as Tele
 *   Left bumper ... "Use table": load RPM + hood from ShotTable for the current distance
 *   Right bumper .. hold = intake on
 *   Right trigger . shoot (gate open + feed)
 *   Back .......... reset pose to start
 *
 * GAMEPAD 2 (aim centering: shoot, see where it lands, nudge, repeat)
 *   Dpad up/down ..... move the CURRENT goal point +Y / -Y
 *   Dpad right/left .. move the CURRENT goal point +X / -X
 *   Right/left bumper  turret trim +/- deg (applied to every goal)
 *   A ................ toggle fine / coarse aim step
 *   B ................ reset nudges + trim for the current goal
 *   Each goal (audience / rear) keeps its own nudge, so you can fix the rear one
 *   without breaking the audience one. Copy "GOAL LINE" into ShootingCalc.
 *
 * When a shot has good results, copy the "TABLE LINE" from telemetry into ShotTable.
 */
@TeleOp
public class ShooterTuneTele extends CommandOpMode {

    // Same limits Pivot.set() enforces.
    private static final double HOOD_MIN = 0.43, HOOD_MAX = 0.62          ;
    private static final double RPM_MAX = 6000;

    private static final double RPM_FINE = 10, RPM_COARSE = 50;
    private static final double HOOD_FINE = 0.005, HOOD_COARSE = 0.001;

    private Alliance alliance = Alliance.BLUE;
    private Pose goal;
    private Pose robotPose, turretPose;

    private RoboModules r;
    private IntakeSystem is;
    private Follower follower;
    private VoltageSensor battery;

    private double targetRpm = 2400;
    private double hood = 0.55;
    private boolean fine = true;
    private boolean flywheelOn = false;
    private boolean autoAim = true;

    // Aim nudges (inches) per goal + global turret trim (deg).
    private static final double AIM_FINE = 0.5, AIM_COARSE = 2.0;
    private static final double TRIM_FINE = 0.5, TRIM_COARSE = 2.0;
    private boolean rear = false;
    private boolean aimFine = true;
    private final double[] nudgeX = new double[2], nudgeY = new double[2]; // [0]=aud [1]=rear
    private double turretTrim = 0;

    private double distance = 0;
    private double turretTargetDeg = 0;

    // Snapshot of the last shot, plus the RPM sag after it.
    private final ElapsedTime sinceShot = new ElapsedTime();
    private boolean shotTaken = false;
    private double shotDistance, shotRpmTarget, shotRpmActual, shotHood;
    private double minRpmAfterShot;
    private double recoveryMs = -1;

    private final ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.localizer.reset();
        follower.setPose(ShootingCalc.BLUE_START);

        r = new RoboModules(hardwareMap);
        is = r.is;
        battery = hardwareMap.voltageSensor.iterator().hasNext()
                ? hardwareMap.voltageSensor.iterator().next() : null;
        applyGoal();
    }

    @Override
    public void init_loop() {
        if (gamepad1.x) alliance = Alliance.RED;
        if (gamepad1.b) alliance = Alliance.BLUE;
        if (gamepad1.y) r.t.resetEncoder();
        applyGoal();
        follower.setPose(alliance == Alliance.RED ? ShootingCalc.RED_START : ShootingCalc.BLUE_START);

        telemetry.addLine("X = RED, B = BLUE, Y = zero turret");
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Turret angle", "%.1f deg", r.t.getRawAngleDegrees());
        telemetry.update();
    }

    @Override
    public void start() {
        r.p.set(hood);
        loopTimer.reset();
    }

    @Override
    public void loop() {
        robotPose = follower.pose();
        turretPose = getTurretPose(robotPose);

        follower.update();
        Scheduler.execute(); // only runs launch() now

        drive();
        handleInput();
        update();
        trackShot();
        showTelemetry();
    }

    /** All per-loop subsystem updates, run directly (no scheduler/commands), same as Tele.update(). */
    private void update() {
        r.p.setPos(hood);

        r.l.updateshooting(flywheelOn ? targetRpm : 0);
        aimTurret();
        r.t.tester();
        r.l.tester();
        r.p.tester();
        r.bs.update();

        // Feeding a shot wins over the bumper; set directly so nothing can block it.
        is.set(feeding ? 2 : gamepad1.right_bumper ? 1 : 0);
    }

    @Override
    public void stop() {
        r.l.off();
        super.stop();
    }

    // ------------------------------------------------------------------ input

    private void handleInput() {
        double rpmStep = fine ? RPM_FINE : RPM_COARSE;
        double hoodStep = fine ? HOOD_FINE : HOOD_COARSE;

        if (gamepad1.dpadUpWasPressed()) targetRpm += rpmStep;
        if (gamepad1.dpadDownWasPressed()) targetRpm -= rpmStep;
        if (gamepad1.dpadRightWasPressed()) hood += hoodStep;
        if (gamepad1.dpadLeftWasPressed()) hood -= hoodStep;

        targetRpm = clamp(targetRpm, 0, RPM_MAX);
        hood = clamp(hood, HOOD_MIN, HOOD_MAX);

        if (gamepad1.aWasPressed()) fine = !fine;
        if (gamepad1.xWasPressed()) flywheelOn = !flywheelOn;
        if (gamepad1.yWasPressed()) autoAim = !autoAim;
        if (gamepad1.bWasPressed()) { rear = !rear; applyGoal(); }
        handleAimInput();

        if (gamepad1.leftBumperWasPressed()) {
            ShotTable.Shot s = ShotTable.get(Math.abs(distance));
            targetRpm = s.rpm;
            hood = clamp(s.hood, HOOD_MIN, HOOD_MAX);
        }

        if (gamepad1.backWasPressed()) {
            follower.setPose(alliance == Alliance.RED ? ShootingCalc.RED_START : ShootingCalc.BLUE_START);
        }

        if (gamepad1.right_trigger > 0.2) {
            if (!shotTaken || sinceShot.seconds() > 1.0) recordShot();
            Scheduler.schedule(launch());
        }
    }

    private void handleAimInput() {
        int g = rear ? 1 : 0;
        double step = aimFine ? AIM_FINE : AIM_COARSE;
        double trimStep = aimFine ? TRIM_FINE : TRIM_COARSE;

        if (gamepad2.dpadUpWasPressed()) nudgeY[g] += step;
        if (gamepad2.dpadDownWasPressed()) nudgeY[g] -= step;
        if (gamepad2.dpadRightWasPressed()) nudgeX[g] += step;
        if (gamepad2.dpadLeftWasPressed()) nudgeX[g] -= step;
        if (gamepad2.rightBumperWasPressed()) turretTrim += trimStep;
        if (gamepad2.leftBumperWasPressed()) turretTrim -= trimStep;
        if (gamepad2.aWasPressed()) aimFine = !aimFine;
        if (gamepad2.bWasPressed()) { nudgeX[g] = 0; nudgeY[g] = 0; turretTrim = 0; }
        applyGoal();
    }

    private void recordShot() {
        shotTaken = true;
        shotDistance = distance;
        shotRpmTarget = targetRpm;
        shotRpmActual = r.l.getRPM();
        shotHood = hood;
        minRpmAfterShot = shotRpmActual;
        recoveryMs = -1;
        sinceShot.reset();
    }

    /** Tracks how far RPM sags after a shot and how long it takes to get back in tolerance. */
    private void trackShot() {
        if (!shotTaken) return;
        double rpm = r.l.getRPM();
        if (sinceShot.seconds() < 3.0) {
            minRpmAfterShot = Math.min(minRpmAfterShot, rpm);
            if (recoveryMs < 0 && sinceShot.seconds() > 0.1 && r.l.atSpeed()) {
                recoveryMs = sinceShot.milliseconds();
            }
        }
    }

    // ------------------------------------------------------------------ commands

    /** True while a shot is feeding; update() runs the intake in feed mode while set. */
    private boolean feeding = false;

    /** One press: gate open + continuous feed for Tele.LAUNCH_MS, then close. Same as Tele.launch(). */
    private Command launch() {
        return sequential(
                r.g.setcom(Gate.Mode.OPEN),
                waitMs(70),
                instant(() -> feeding = true),
                waitMs(10),
                is.setIn(1),
                waitMs(org.firstinspires.ftc.teamcode.subsystems.Tele.LAUNCH_MS),
                r.g.setcom(Gate.Mode.CLOSE),
                instant(() -> feeding = false)
        )
                .requiring(r.g)
                .setPriority(10)
                .setConflictBehavior(ConflictBehavior.CANCEL);
    }

    // ------------------------------------------------------------------ drive / aim

    private void drive() {
        follower.manual(-gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                Math.abs(gamepad1.right_stick_x) < 0.2 ? 0 : -gamepad1.right_stick_x);
    }

    private void applyGoal() {
        Pose base = alliance == Alliance.RED
                ? (rear ? ShootingCalc.RED_REAR : ShootingCalc.RED_AUD)
                : (rear ? ShootingCalc.BLUE_REAR : ShootingCalc.BLUE_AUD);
        int g = rear ? 1 : 0;
        goal = new Pose(base.x() + nudgeX[g], base.y() + nudgeY[g]);
    }

    /** Turret pivot on the field = robot pose + turret offset rotated by heading (same as Tele). */
    private Pose getTurretPose(Pose robot) {
        double h = robot.heading();
        double cos = Math.cos(h), sin = Math.sin(h);
        return new Pose(
                robot.x() + org.firstinspires.ftc.teamcode.subsystems.Tele.TURRET_OFFSET_X * cos
                        - org.firstinspires.ftc.teamcode.subsystems.Tele.TURRET_OFFSET_Y * sin,
                robot.y() + org.firstinspires.ftc.teamcode.subsystems.Tele.TURRET_OFFSET_X * sin
                        + org.firstinspires.ftc.teamcode.subsystems.Tele.TURRET_OFFSET_Y * cos,
                h);
    }

    private void aimTurret() {
        double dx = goal.x() - turretPose.x();
        double dy = goal.y() - turretPose.y();
        distance = Math.hypot(dx, dy);

        if (!autoAim) {
            turretTargetDeg = 0;
        } else {
            double a = Math.atan2(dy, dx) - robotPose.heading();
            while (a > Math.PI) a -= 2 * Math.PI;
            while (a < -Math.PI) a += 2 * Math.PI;
            turretTargetDeg = Math.toDegrees(a) + turretTrim + sideOffsetDeg();
        }
        r.t.setTargetDegrees(turretTargetDeg);
    }

    /** Same side correction as Tele.sideOffsetDeg(), using Tele's dashboard values. */
    private double sideOffsetDeg() {
        double dxGoal = robotPose.x() - goal.x();
        if (dxGoal < -Tele.SIDE_X_THRESHOLD) return Tele.LEFT_SIDE_OFFSET_DEG;
        if (dxGoal > Tele.SIDE_X_THRESHOLD) return Tele.RIGHT_SIDE_OFFSET_DEG;
        return 0;
    }

    // ------------------------------------------------------------------ telemetry

    private void showTelemetry() {
        double rpm = r.l.getRPM();
        double turretDeg = r.t.getRawAngleDegrees();
        double turretErr = turretTargetDeg - turretDeg;
        ShotTable.Shot table = ShotTable.get(Math.abs(distance));
        int g = rear ? 1 : 0;

        telemetry.addLine("---- AIM / TURRET ----");
        telemetry.addData("Goal", "%s %s  (%.1f, %.1f)", alliance, rear ? "REAR" : "AUDIENCE", goal.x(), goal.y());
        telemetry.addData("Distance", "%.1f in", distance);
        telemetry.addData("Turret target", "%.1f deg  (%s)", turretTargetDeg, autoAim ? "AUTO-AIM" : "HOLD 0");
        telemetry.addData("Turret actual", "%.1f deg", turretDeg);
        telemetry.addData("Turret error", "%+.1f deg  %s", turretErr,
                Math.abs(turretErr) < 2 ? "CENTERED" : (turretErr > 0 ? "<< needs +" : "needs - >>"));
        telemetry.addData("Turret on target", r.t.atTarget());
        telemetry.addData("Side offset", "%+.1f deg  (robot x - goal x = %.1f)%s",
                sideOffsetDeg(), robotPose.x() - goal.x(), autoAim ? "" : "  [not applied: HOLD 0]");
        telemetry.addData("Nudge x/y | trim", "%+.1f / %+.1f in | %+.1f deg  (%s step)",
                nudgeX[g], nudgeY[g], turretTrim, aimFine ? "fine" : "coarse");
        telemetry.addData("GOAL LINE", "new Pose(%.1f, %.1f)", goal.x(), goal.y());
        telemetry.addData("Robot", "x=%.1f y=%.1f h=%.1f deg",
                robotPose.x(), robotPose.y(), Math.toDegrees(robotPose.heading()));

        telemetry.addLine("---- SHOOTER ----");
        telemetry.addData("RPM", "%.0f / %.0f  (err %+.0f)  %s", rpm, targetRpm, rpm - r.l.getTargetRPM(),
                r.l.atSpeed() ? "AT SPEED" : "spinning");
        telemetry.addData("target",  r.l.getTargetRPM());
        telemetry.addData("Real rpm",  r.l.getRPM());
        telemetry.addData("Flywheel", flywheelOn ? "ON" : "OFF");
        telemetry.addData("Hood", "%.3f  (%.2f-%.2f)", hood, HOOD_MIN, HOOD_MAX);
        telemetry.addData("Step", "%s  (%.0f rpm / %.3f hood)", fine ? "FINE" : "COARSE",
                fine ? RPM_FINE : RPM_COARSE, fine ? HOOD_FINE : HOOD_COARSE);
        telemetry.addData("Battery", battery == null ? "n/a" : String.format("%.2f V", battery.getVoltage()));
        telemetry.addData("Table here", "rpm=%.0f hood=%.3f  (you: %+.0f / %+.3f)",
                table.rpm, table.hood, targetRpm - table.rpm, hood - table.hood);

        telemetry.addLine("---- LAST SHOT ----");
        if (shotTaken) {
            telemetry.addData("Fired at", "%.1f in, %.0f rpm (target %.0f)", shotDistance, shotRpmActual, shotRpmTarget);
            telemetry.addData("Dip / recovery", "%.0f rpm (min %.0f) / %s", shotRpmActual - minRpmAfterShot,
                    minRpmAfterShot, recoveryMs < 0 ? "..." : String.format("%.0f ms", recoveryMs));
            telemetry.addData("TABLE LINE", "add(%.1f, %.0f, %.3f);", shotDistance, shotRpmTarget, shotHood);
        } else {
            telemetry.addLine("none yet (RT to shoot)");
        }

        telemetry.addData("Loop", "%.0f ms", loopTimer.milliseconds());
        loopTimer.reset();
        telemetry.update();
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
