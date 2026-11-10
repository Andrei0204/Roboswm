package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.api.Paths;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.aiming.BallType;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.BioBuzzField;
import org.firstinspires.ftc.teamcode.field.HiveCell;
import org.firstinspires.ftc.teamcode.robot.RobotSys;
import org.firstinspires.ftc.teamcode.subsystems.IntakeServo;

/**
 * The "dictionary" of autonomous building blocks (see docs/auto.md). Every method returns an Ivy
 * {@link Command} (or a Pedro {@link Path} / {@link Pose}); nothing here moves the robot until the
 * command runs inside a sequence that was scheduled.
 *
 * <p>Build one per AUTO from the {@link RobotSys}, build the routine with
 * {@link Groups#sequential}, then every loop call {@link #run()} once.
 *
 * <p>Poses are written for the RED alliance; {@link #pose} rotates them 180 deg about the field
 * center for BLUE (the field is rotationally symmetric, not mirrored).
 */
public final class AutoBlocks {
    private final RobotSys robot;
    private final Alliance alliance;
    /** Shot wanted this loop: set by {@link #shoot}, cleared when that command ends. */
    private boolean fire;
    private boolean force;

    public AutoBlocks(RobotSys robot) {
        this.robot = robot;
        this.alliance = robot.alliance();
        Scheduler.reset();
    }

    // ------------------------------------------------------------------ the loop
    /**
     * Call once per loop, after the routine was scheduled: runs the commands, then the robot cycle
     * ({@code update}, which also saves the pose for TeleOp) and the shooting.
     */
    public void run() {
        Scheduler.execute();
        robot.update();
        robot.shoot(fire, force);
    }

    /** Schedule the whole routine (call once, at START). */
    public void start(Command routine) {
        Scheduler.schedule(routine);
    }

    public boolean isFinished(Command routine) {
        return !Scheduler.isScheduled(routine);
    }

    // ------------------------------------------------------------------ poses and paths
    /** A field pose written for RED (inches, degrees); rotated 180 deg for BLUE. */
    public Pose pose(double x, double y, double headingDeg) {
        return pose(alliance, x, y, headingDeg);
    }

    /** Same, for an alliance given explicitly (INIT, before the AutoBlocks exists). */
    public static Pose pose(Alliance alliance, double x, double y, double headingDeg) {
        double h = Math.toRadians(headingDeg);
        if (alliance == Alliance.RED) return new Pose(x, y, h);
        double size = 2.0 * BioBuzzField.FIELD_CENTER_IN;
        return new Pose(size - x, size - y, h + Math.PI);
    }

    /** Straight line, heading turning smoothly from the first pose's to the second's. */
    public Path line(Pose from, Pose to) {
        return Paths.line(from, to).linear(from.heading(), to.heading());
    }

    /** Straight line that keeps one heading the whole way. */
    public Path lineConstantHeading(Pose from, Pose to) {
        return Paths.line(from, to).constant(to.heading());
    }

    /** Bezier curve through control points (first = start, last = end), heading start to end. */
    public Path curve(Pose... controlPoints) {
        Pose first = controlPoints[0];
        Pose last = controlPoints[controlPoints.length - 1];
        return Paths.curve(controlPoints).linear(first.heading(), last.heading());
    }

    /** Bezier curve whose heading follows the direction of travel. */
    public Path curveTangent(Pose... controlPoints) {
        return Paths.curve(controlPoints).tangent();
    }

    // ------------------------------------------------------------------ driving
    /** Follow a path; done when the follower reaches its end. */
    public Command follow(Path path) {
        return PedroCommands.follow(robot.follower(), path).requiring(robot.follower());
    }

    /** Follow a path but give up after {@code timeoutMs} (the robot then just stops following). */
    public Command follow(Path path, double timeoutMs) {
        return Groups.race(follow(path), Commands.waitMs(timeoutMs));
    }

    /** Drive in a straight line from wherever the robot is when this runs to {@code target}. */
    public Command driveTo(Pose target) {
        return Commands.lazy(() -> follow(line(robot.pose(), target)));
    }

    /** Same as {@link #driveTo} with a timeout. */
    public Command driveTo(Pose target, double timeoutMs) {
        return Groups.race(driveTo(target), Commands.waitMs(timeoutMs));
    }

    /** Hold the position the robot is at when this runs. */
    public Command hold() {
        return PedroCommands.hold(robot.follower());
    }

    /** Hold a given pose. */
    public Command hold(Pose pose) {
        return PedroCommands.hold(robot.follower(), pose);
    }

    /** Overwrite the pose estimate (e.g. a wall bump re-localization). */
    public Command setPose(Pose pose) {
        return Commands.instant(() -> robot.setPose(pose.x(), pose.y(), pose.heading()));
    }

    // ------------------------------------------------------------------ shooter
    /** Spin the flywheel up (turret, hood and flywheel then track the shot solution). */
    public Command arm() {
        return Commands.instant(() -> robot.setArmed(true));
    }

    public Command disarm() {
        return Commands.instant(() -> robot.setArmed(false));
    }

    /** Done when every shot check passes (or after {@code timeoutMs}). Does not fire. */
    public Command waitReady(double timeoutMs) {
        return Groups.race(Commands.waitUntil(robot::isReady), Commands.waitMs(timeoutMs));
    }

    /** Hold fire for {@code ms}; feeds only while every check passes. */
    public Command shoot(double ms) {
        return fireFor(false, ms);
    }

    /** Hold FORCE fire for {@code ms}: feeds regardless of every check. Use with care. */
    public Command forceShoot(double ms) {
        return fireFor(true, ms);
    }

    /**
     * Fire until {@code balls} feed pulses finished, or {@code timeoutMs}. Only counts pulses when
     * FEEDER_PULSE_SEC is &gt; 0; with 0 the feeder never pulses, so use {@link #shoot}.
     */
    public Command shootBalls(int balls, double timeoutMs) {
        int[] startFeeds = new int[1];
        Command count = Command.build()
                .setStart(() -> startFeeds[0] = robot.feeder().feeds())
                .setExecute(() -> fire = true)
                .setDone(() -> robot.feeder().feeds() - startFeeds[0] >= balls && !robot.isFeeding())
                .setEnd(e -> fire = false);
        return Groups.race(count, Commands.waitMs(timeoutMs));
    }

    private Command fireFor(boolean forced, double ms) {
        long[] begin = new long[1];
        return Command.build()
                .setStart(() -> begin[0] = System.nanoTime())
                .setExecute(() -> {
                    if (forced) force = true;
                    else fire = true;
                })
                .setDone(() -> (System.nanoTime() - begin[0]) * 1e-6 >= ms)
                .setEnd(e -> {
                    fire = false;
                    force = false;
                });
    }

    /** Which ball type the next shot uses (only matters if ShooterConstants.SHOOTS_NECTAR). */
    public Command nextBall(BallType ball) {
        return Commands.instant(() -> robot.setNextBall(ball));
    }

    /** Tell the robot which HIVE CELL is up (e.g. after a known TIP). */
    public Command upCell(HiveCell cell) {
        return Commands.instant(() -> robot.setUpCell(cell));
    }

    /** Report that our HIVE tipped (switches the CELL and pauses shooting briefly). */
    public Command hiveTipped() {
        return Commands.instant(robot::reportHiveTip);
    }

    /** Aim trim in degrees added to every shot (positive = counter-clockwise). */
    public Command aimTrimDeg(double deg) {
        return Commands.instant(() -> robot.addAimTrimDeg(deg));
    }

    /** Distance trim in inches added before the shot lookup (positive = farther). */
    public Command distanceTrimIn(double inches) {
        return Commands.instant(() -> robot.addDistanceTrimIn(inches));
    }

    public Command clearTrims() {
        return Commands.instant(robot::clearTrims);
    }

    // ------------------------------------------------------------------ intake
    /** Intake motor on (stays on until {@link #intakeOff}). */
    public Command intakeOn() {
        return Commands.instant(() -> robot.intake().setManual(true));
    }

    public Command intakeOff() {
        return Commands.instant(() -> robot.intake().setManual(false));
    }

    /** Run the intake backwards while this command runs (clears a jam); then forward-off. */
    public Command intakeReverse(double ms) {
        return Groups.sequential(
                Commands.instant(() -> robot.setIntakeReverse(true)),
                Commands.waitMs(ms),
                Commands.instant(() -> robot.setIntakeReverse(false)));
    }

    /** Intake servo position: INIT, COLLECT_ALL or COLLECT_POLLEN. */
    public Command intakeMode(IntakeServo.Mode mode) {
        return Commands.instant(() -> robot.setIntakeMode(mode));
    }

    /** Servo to {@code mode} and motor on, in one step. */
    public Command collect(IntakeServo.Mode mode) {
        return Groups.parallel(intakeMode(mode), intakeOn());
    }

    /**
     * Take balls in ONLY: intake on, feeder not driven, gate forced closed so nothing goes past
     * it. Stays on until {@link #intakeOff} (or {@link #stopCollecting}).
     */
    public Command takeBallsGateClosed() {
        return Commands.instant(() -> {
            robot.setManualFeed(false);
            robot.gate().close();
            robot.intake().setManual(true);
        });
    }

    /**
     * LOAD balls without shooting: intake and feeder run together while the gate stays CLOSED,
     * so the balls are taken in and moved up but never launched. Stays on until {@link #intakeFeedOff}.
     */
    public Command intakeFeedOn() {
        return Commands.instant(() -> {
            robot.gate().close();
            robot.setManualFeed(true);
        });
    }

    /** Stop loading (intake and feeder off). */
    public Command intakeFeedOff() {
        return Commands.instant(() -> robot.setManualFeed(false));
    }

    /** Load balls (gate closed, no shooting) for {@code ms}, then stop. */
    public Command intakeFeed(double ms) {
        return Groups.sequential(intakeFeedOn(), Commands.waitMs(ms), intakeFeedOff());
    }

    /** Motor off and servo back to INIT. */
    public Command stopCollecting() {
        return Groups.parallel(intakeOff(), intakeMode(IntakeServo.Mode.INIT));
    }

    // ------------------------------------------------------------------ timing and logic
    public Command waitMs(double ms) {
        return Commands.waitMs(ms);
    }

    public Command waitUntil(java.util.function.BooleanSupplier condition) {
        return Commands.waitUntil(condition);
    }

    /** Run any code once (telemetry flag, custom mechanism, ...). */
    public Command run(Runnable action) {
        return Commands.instant(action);
    }

    /** Run {@code ifTrue} or {@code ifFalse}, chosen when this block is reached. */
    public Command either(java.util.function.BooleanSupplier condition, Command ifTrue, Command ifFalse) {
        return Commands.conditional(condition, ifTrue, ifFalse);
    }

    /** Stop everything at the end of AUTO: flywheel, feeder, gate, intake. Not the follower. */
    public Command stopAll() {
        return Commands.instant(robot::stop);
    }

    // ------------------------------------------------------------------ groups (shortcuts)
    /** One after the other. */
    public static Command sequence(Command... commands) {
        return Groups.sequential(commands);
    }

    /** All at once; done when all are done. */
    public static Command together(Command... commands) {
        return Groups.parallel(commands);
    }

    /** All at once; done when the FIRST one is done (the others are cancelled). */
    public static Command firstOf(Command... commands) {
        return Groups.race(commands);
    }

    /** All at once; done when {@code leader} is done (the others are cancelled). */
    public static Command whileDoing(Command leader, Command... alongside) {
        return Groups.deadline(leader, alongside);
    }

    /** Repeat a command {@code times} times. */
    public static Command repeat(Command command, int times) {
        return Groups.repeat(command, times);
    }
}
