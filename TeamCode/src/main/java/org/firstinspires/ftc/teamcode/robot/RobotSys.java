package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.aiming.BallType;
import org.firstinspires.ftc.teamcode.aiming.ShootOnTheMove;
import org.firstinspires.ftc.teamcode.aiming.ShotSolution;
import org.firstinspires.ftc.teamcode.aiming.ShotSolver;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.BioBuzzField;
import org.firstinspires.ftc.teamcode.field.HiveCell;
import org.firstinspires.ftc.teamcode.field.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.Feeder;
import org.firstinspires.ftc.teamcode.subsystems.Gate;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeServo;

/**
 * The whole robot in one place: Pedro drive/localization, the shoot-on-the-move aiming
 * (turret, flywheel, hood) and the feeder. OpModes only talk to this class.
 *
 * <p>All hubs use bulk caching AUTO: the first read of a value in a loop does one bulk read, and
 * reading the SAME value again forces another one. Every value here is read once per loop.
 *
 * <p>Nothing moves in INIT except turret homing, and only when the home button asks for it: no
 * other servo or motor is commanded until the first {@link #update} after START. INIT loop:
 * {@link #initLoop()} every loop, {@link #homeTurret()} on the home button and
 * {@link #servosToStart()} on the start-position button.
 * Main loop: {@link #drive} (TeleOp) or a Pedro path (AUTO), then {@link #update}, then
 * {@link #shoot}, once each.
 */
public final class RobotSys {
    private final Follower follower;
    private final ShootOnTheMove shooter;
    private final Feeder feeder;
    private final Gate gate;
    private final Intake intake;
    private final IntakeServo intakeServo;
    private Alliance alliance;
    /** The current feed was started by force fire, not by the checks. */
    private boolean forced;

    /**
     * Builds every subsystem. The pose is not set here: call {@link #setPose},
     * {@link #resetPoseToCorner()} or {@link #restorePoseFromStorage()}.
     */
    public RobotSys(HardwareMap hardwareMap, Alliance alliance) {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        follower = Constants.create(hardwareMap);
        if (follower == null) {
            throw new IllegalStateException("Pedro is not configured: pedro/Constants.create() returned null");
        }
        this.alliance = alliance;
        shooter = new ShootOnTheMove(hardwareMap, follower, alliance);
        feeder = new Feeder(hardwareMap);
        gate = new Gate(hardwareMap);
        intake = new Intake(hardwareMap);
        intakeServo = new IntakeServo(hardwareMap);
    }

    // ------------------------------------------------------------------ INIT
    /** Call every INIT loop. Nothing moves unless {@link #homeTurret()} was requested. */
    public void initLoop() {
        shooter.initLoop();
    }

    /**
     * INIT button: puts every servo in its start position for TeleOp: turret straight forward,
     * hood start, gate closed, intake servo INIT. The encoder is not zeroed (homing does that),
     * and the intake motor, flywheel and feeder do not move.
     */
    public void servosToStart() {
        shooter.servosToStart();
        gate.update(false);
        intakeServo.setMode(IntakeServo.Mode.INIT);
        intakeServo.update();
    }

    /** Starts turret homing (the home button): the turret moves to its zero and the encoder is zeroed. */
    public void homeTurret() {
        shooter.homeTurret();
    }

    public String turretStatus() {
        return shooter.turretStatus();
    }

    // ------------------------------------------------------------------ pose
    public void setPose(double x, double y, double heading) {
        follower.setPose(new Pose(x, y, heading));
        shooter.resetMotion();
    }

    /** Robot pushed into this alliance's reset corner (see BioBuzzField.resetPose). */
    public void resetPoseToCorner() {
        double[] p = BioBuzzField.resetPose(alliance);
        setPose(p[0], p[1], p[2]);
    }

    /**
     * Uses the pose, alliance and up CELL a previous OpMode left in {@link PoseStorage}.
     *
     * @return false if nothing was stored
     */
    public boolean restorePoseFromStorage() {
        if (!PoseStorage.hasData) return false;
        setAlliance(PoseStorage.alliance);
        setPose(PoseStorage.x, PoseStorage.y, PoseStorage.heading);
        if (PoseStorage.upCell != null) shooter.setUpCell(PoseStorage.upCell);
        return true;
    }

    public Pose pose() {
        return follower.pose();
    }

    // ------------------------------------------------------------------ main loop
    /**
     * TeleOp driving, in the driver's frame: +forward away from our alliance wall, +left, +turn
     * counter-clockwise. Call before {@link #update}.
     */
    public void drive(double forward, double left, double turn) {
        follower.manual(ManualDrive.fieldCentric(forward, left, turn,
                follower.pose().heading(), BioBuzzField.driverHeadingOffset(alliance)));
    }

    /**
     * One robot cycle, without shooting: Pedro update, aiming (turret, flywheel, hood), intake,
     * and the pose saved for the next OpMode. Call once per loop, then {@link #shoot}.
     */
    public void update() {
        follower.update();
        shooter.update();
        // "Launching" = the gate is open or a pulse is running; the intake always runs then.
        intake.update(gate.wantsOpen() || feeder.isFeeding());
        intakeServo.update();
        Pose pose = follower.pose();
        PoseStorage.save(pose.x(), pose.y(), pose.heading(), alliance, shooter.upCell());
    }

    /**
     * The shooting itself: gate and feeder. Call once per loop after {@link #update}, every loop
     * (with false, false when not firing) so the gate closes again when you let go.
     *
     * @param fire      feed when every shot check passes
     * @param forceFire feed regardless of EVERY check (even with the flywheel off)
     */
    public void shoot(boolean fire, boolean forceFire) {
        boolean normal = fire && shooter.isReady();
        boolean wanted = normal || forceFire;
        // The gate stays open while a feed is wanted or a pulse is still running; the feeder
        // only starts a pulse once the gate is open.
        gate.update(wanted || feeder.isFeeding());
        if (feeder.update(wanted && gate.isOpen())) forced = !normal;
        if (!feeder.isFeeding()) forced = false;
    }

    /** Stops the flywheel and the feeder, and closes the gate. */
    public void stop() {
        shooter.stop();
        feeder.stop();
        gate.close();
        intake.stop();
    }

    // ------------------------------------------------------------------ shooter controls
    public void toggleArmed() {
        shooter.toggleArmed();
    }

    public void setArmed(boolean armed) {
        shooter.setArmed(armed);
    }

    public boolean isArmed() {
        return shooter.isArmed();
    }

    public boolean isReady() {
        return shooter.isReady();
    }

    public boolean isFeeding() {
        return feeder.isFeeding();
    }

    /** Changes alliance, resets the HIVE to its match-start state (pose is not changed). */
    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
        shooter.setAlliance(alliance);
    }

    public Alliance alliance() {
        return alliance;
    }

    public void reportHiveTip() {
        shooter.reportHiveTip();
    }

    public void undoHiveTip() {
        shooter.undoHiveTip();
    }

    public HiveCell upCell() {
        return shooter.upCell();
    }

    public void setUpCell(HiveCell cell) {
        shooter.setUpCell(cell);
    }

    public BallType nextBall() {
        return shooter.nextBall();
    }

    public void setNextBall(BallType ball) {
        shooter.setNextBall(ball);
    }

    /** Aim trim (deg), added to every shot; positive turns the shot counter-clockwise. */
    public void addAimTrimDeg(double deg) {
        solver().aimTrimRad += Math.toRadians(deg);
    }

    /** Distance trim (in), added before the flywheel/hood lookup; positive shoots farther. */
    public void addDistanceTrimIn(double inches) {
        solver().distanceTrimIn += inches;
    }

    public void clearTrims() {
        solver().aimTrimRad = 0.0;
        solver().distanceTrimIn = 0.0;
    }

    /** Calibration only: fixed flywheel rpm and hood position instead of the shot table. */
    public void setCalibrationOverride(double rpm, double hoodPosition) {
        shooter.setCalibrationOverride(rpm, hoodPosition);
    }

    public void clearCalibrationOverride() {
        shooter.clearCalibrationOverride();
    }

    /** True when the shot is blocked by something the driver can fix (side, angle, speed, dead zone). */
    public boolean isBlockedByDriving() {
        return shooter.isBlockedByDriving();
    }

    public ShotSolution solution() {
        return shooter.solution();
    }

    public ShotSolver solver() {
        return shooter.solver();
    }

    // ------------------------------------------------------------------ subsystems
    public Follower follower() {
        return follower;
    }

    public ShootOnTheMove shooter() {
        return shooter;
    }

    public Feeder feeder() {
        return feeder;
    }

    public Gate gate() {
        return gate;
    }

    public Intake intake() {
        return intake;
    }

    public IntakeServo intakeServo() {
        return intakeServo;
    }

    // ------------------------------------------------------------------ intake controls
    /** Switch the intake on/off by hand. It always runs during launches regardless. */
    public void toggleIntake() {
        intake.toggleManual();
    }

    /** Intake servo position: INIT (stowed), COLLECT_ALL or COLLECT_POLLEN. */
    public void setIntakeMode(IntakeServo.Mode mode) {
        intakeServo.setMode(mode);
    }

    public IntakeServo.Mode intakeMode() {
        return intakeServo.mode();
    }

    /** Hold true to run the intake backwards (clears a jam). */
    public void setIntakeReverse(boolean reverse) {
        intake.setReverse(reverse);
    }

    // ------------------------------------------------------------------ telemetry
    public void addTelemetry(Telemetry telemetry) {
        Pose pose = follower.pose();
        telemetry.addData("Pose", "x %.1f  y %.1f  h %.1f deg", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Trims", "aim %.1f deg, distance %.0f in",
                Math.toDegrees(solver().aimTrimRad), solver().distanceTrimIn);
        telemetry.addData("Intake", "%s  (manual %s), servo %s",
                intake.isRunning() ? "RUNNING" : "off", intake.isManualOn() ? "on" : "off", intakeServo.mode());
        telemetry.addData("Gate / feeding", "%s / %b%s  (feeds %d)",
                gate.wantsOpen() ? "OPEN" : "closed",
                feeder.isFeeding(), feeder.isFeeding() && forced ? " (FORCED)" : "", feeder.feeds());
        shooter.addTelemetry(telemetry);
    }
}
