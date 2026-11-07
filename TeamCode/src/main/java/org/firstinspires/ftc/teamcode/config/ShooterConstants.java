package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config .Config;

import org.firstinspires.ftc.teamcode.aiming.BallType;

/**
 * Every tunable number used by the shoot-on-the-move system, in one place.
 *
 * Units: inches, seconds and radians, unless the name ends in _DEG (degrees) or _RPM.
 * Frames: the Pedro field frame described in {@code BioBuzzField}, and Pedro's robot frame
 * (+x forward, +y left, angles counter-clockwise positive, 0 = robot forward).
 *
 * Values tagged MEASURE are placeholders. The code runs with them, but the robot will not
 * hit the HIVE until they have been measured on your robot and field.
 *
 * The fields are public static and non-final so a live-tuning dashboard can edit them.
 * Edited shot tables are picked up automatically by ShotSolver on the next solve.
 */
@Config
public final class  ShooterConstants {
    private ShooterConstants() {}

    // ------------------------------------------------------------------ hardware names
    /** Continuous-rotation servos, configured as plain Servos: position 0.5 = stop, 0.5 + 0.5 * power. */
    public static String[] TURRET_SERVO_NAMES = {"turp"};
    /**
     * One entry per turret servo: true reverses that servo. Set so POSITIVE power turns the
     * turret counter-clockwise (the direction in which the encoder angle increases).
     */
    public static boolean[] TURRET_SERVO_REVERSED = {false};
    /**
     * Motor port (configured as a motor in the Robot Configuration) whose encoder slot has the
     * external turret encoder. Homing resets its count to 0 at the turret's zero, and the hub keeps
     * the count from AUTO to TeleOp, so nothing else may reset this port. Never use a Pedro
     * dead-wheel port (Pedro resets those).
     */
    public static String TURRET_ENCODER_NAME = "FR";
    /** Flywheel motors. The FIRST one must have the flywheel encoder plugged in. */
    public static String[] FLYWHEEL_MOTOR_NAMES = {"launcherp"};
    /** One entry per flywheel motor: true reverses that motor so +power spins the shot forward. */
    public static boolean[] FLYWHEEL_MOTOR_REVERSED = {true};
    public static String HOOD_SERVO_NAME = "hoodp";
    /** Feeder/transfer that pushes a ball into the flywheel: a motor or a CR servo. */
    public static String FEEDER_NAME = "transferp";
    /** Gate servo between the feeder and the launcher. */
    public static String GATE_SERVO_NAME = "gatep";
    /** Intake motor that collects balls into the feeder. */
    public static String INTAKE_MOTOR_NAME = "intake";
    /** Intake servo with three positions (see IntakeServo.Mode, which holds the positions). */
    public static String INTAKE_SERVO_NAME = "intakes";

    // ------------------------------------------------------------------ robot geometry
    /**
     * Turret pivot position relative to the point Pedro's pose describes (the robot center
     * used for your localizer pod offsets). MEASURE. +x forward, +y left, inches.
     */
    public static double TURRET_OFFSET_X_IN = -2;
    public static double TURRET_OFFSET_Y_IN = 0.0;

    // ------------------------------------------------------------------ turret (CR servo + encoder)
    /*
     * Turret angles are robot-relative, counter-clockwise positive seen from above, and 0 means
     * shooting straight forward. The continuous-rotation servo only takes a power; the external
     * encoder is the turret's only position feedback, so the controller closes the loop on it:
     *   power = KP*error + KD*(targetRate - rate), plus an opposing servo bias to remove backlash
     * Find the encoder numbers with the "Turret Servo Setup" OpMode.
     */
    /**
     * Where the shooter points, relative to the robot's front, when the turret reads 0 (encoder
     * 0 ticks). 0 = shooter faces forward; 180 = shooter faces the back of the robot. Home the
     * turret (INIT, gamepad1 Y) with the shooter pointing that way. TURRET_MIN/MAX_DEG are in the
     * turret's own frame (0 = this direction), so the cable dead zone is opposite to it.
     */
    public static double TURRET_ZERO_OFFSET_DEG = 0;
    /** Power per rad of angle error. TUNE: raise until it settles fast without oscillating. */
    public static double TURRET_KP = 0.3;
    /** Power per rad/s of rate error (damping). Raise if the turret overshoots. */
    public static double TURRET_KD = 0.025;
    /**
     * Anti-backlash bias (needs 2+ turret servos in TURRET_SERVO_NAMES, driving the turret gear
     * from opposite sides of the teeth): servo 0 gets +bias and servo 1 gets -bias on top of the
     * command, so they always fight each other slightly and every tooth stays loaded on both
     * flanks, giving zero play. 0 = off. TUNE: raise until the play is gone, lower if the
     * servos get hot or it hums.
     */
    public static double TURRET_BIAS_POWER = 4;
    /**
     * The bias fades out linearly as the command grows to this power, so a big move gets full
     * torque from both servos and only holding/small moves are preloaded.
     */
    public static double TURRET_BIAS_FADE_POWER = 0.4;
    /**
     * Servo deadzone: the continuous servo does not move between neutral (position 0.5) and this
     * much power. Every non-zero command is mapped onto [this, 1] so it starts moving right away
     * and full power is still 1. MEASURE: raise until the turret just starts to move for a tiny
     * command, then back off a little. 0 = off.
     */
    public static double TURRET_SERVO_DEADBAND_POWER = 0.03;
    /** Commands smaller than this are treated as 0 (no added power), so noise does not make it twitch. */
    public static double TURRET_SERVO_ZERO_BELOW = 0.0025;
    /** Limit on the turret power (0..1). */
    public static double TURRET_MAX_POWER = 1.0;
    /**
     * Soft limits (cables, frame), robot-relative. A continuous servo has no end of travel, so
     * these alone keep the turret off its cables: power pushing past them is cut. A turret
     * limited to under 360 deg has a dead zone behind it where targets are never fired at.
     * MEASURE.
     */
    public static double TURRET_MIN_DEG = -179.0;
    public static double TURRET_MAX_DEG = 179.0;

    /** External encoder counts per encoder-shaft revolution (REV Through Bore = 8192). MEASURE. */
    public static double TURRET_ENCODER_TICKS_PER_REV = 8192.0;
    /**
     * Encoder-shaft revolutions per turret revolution, depending on where the encoder is mounted:
     * on the 76T turret shaft = 1.0; on the 24T/26T compound shaft = 76/26 = 2.923; on the servo
     * shaft = (24/60)*(76/26) = 1.169. "Turret Servo Setup" checks this. MEASURE.
     */
    public static double TURRET_ENCODER_REVS_PER_TURRET_REV = 2.92;
    /** +1 or -1 so the encoder angle INCREASES when the turret turns counter-clockwise. MEASURE. */
    public static int TURRET_ENCODER_SIGN = 1;
    /**
     * Homing (gamepad1 Y in INIT): with the turret turned by hand to point straight forward, waits
     * until the encoder shows it is still, and the encoder count is reset to 0 there.
     */
    public static double TURRET_HOME_STILL_DEG_PER_SEC = 3.0;
    public static double TURRET_HOME_STILL_SEC = 0.15;
    public static double TURRET_HOME_MIN_SEC = 0.25;
    public static double TURRET_HOME_TIMEOUT_SEC = 1.5;
    /** A shot needs the encoder rate within this of the commanded rate (no swinging through). */
    public static double TURRET_RATE_TOLERANCE_DEG_PER_SEC = 40.0;
    /** Clamp on the turret rate feedforward (rad/s); it grows without bound right next to the HIVE. */
    public static double TURRET_MAX_FEEDFORWARD_RATE = 10.0;

    // ------------------------------------------------------------------ flywheel
    /** Encoder counts per revolution of the shaft the flywheel encoder is on (goBILDA 5203 1:1 = 28). */
    public static double FLYWHEEL_TICKS_PER_REV = 28.0;
    /** Feedforward: power = KS + KV * rpm (at NOMINAL_VOLTAGE). MEASURE with the calibration OpMode. */
    public static double FLYWHEEL_KS = 0.0087;
    public static double FLYWHEEL_KV = 0.00025076;
    /** Proportional power per rpm of error. */
    public static double FLYWHEEL_KP = 0.001;
    /**
     * Integral gain, power per (rpm * s) of error. Removes the steady rpm error the feedforward
     * leaves (battery, wear, temperature), so the wheel holds the target precisely. 0 = off.
     */
    public static double FLYWHEEL_KI = 0.0003;
    /** The integral only works when the error is inside this many rpm (no windup during spin-up/recovery). */
    public static double FLYWHEEL_I_ZONE_RPM = 150.0;
    /** Max power the integral term may add or remove. */
    public static double FLYWHEEL_I_CLAMP = 0.15;
    /**
     * Smoothing of the measured rpm (0..1): filtered += alpha * (raw - filtered). 1 = no filter.
     * The 28 ticks/rev encoder is coarse, so a lower value (0.3-0.6) removes the jitter that
     * would otherwise make the power hunt. Too low adds lag.
     */
    public static double FLYWHEEL_RPM_FILTER = 0.5;
    /** When more than this far BELOW target (e.g. right after a shot), apply full power. */
    public static double FLYWHEEL_BANG_BANG_RPM = 100;
    /** The flywheel counts as "at speed" inside [target - BELOW, target + ABOVE]. */
    public static double FLYWHEEL_TOLERANCE_BELOW_RPM = 300; //am modificat de la 100
    public static double FLYWHEEL_TOLERANCE_ABOVE_RPM = 300; //si aici
    /** Speed held while the shooter is not armed (0 = coast). */
    public static double FLYWHEEL_IDLE_RPM = 0.0;
    /**
     * Recoil compensation: each ball slows the flywheel, so while a ball is being fed the
     * controller aims this many rpm ABOVE the shot target and the wheel dips less / recovers
     * faster. Only the power is boosted: the "at speed" window still uses the real target. TUNE:
     * about the rpm you see the wheel lose per shot. 0 = off.
     */
    public static double FLYWHEEL_RECOIL_BOOST_RPM = 0;
    /** The boost stays on this long after a feed ends, until the wheel has recovered. */
    public static double FLYWHEEL_RECOIL_HOLD_SEC = 0.05;
    /**
     * Flywheel power changes smaller than this are not sent to the hub (saves a write most loops).
     * 0.005 is about 30 rpm at KV = 1/6000; keep THRESHOLD / KV well inside the at-speed window.
     */
    public static double FLYWHEEL_WRITE_THRESHOLD = 0;

    // ------------------------------------------------------------------ hood
    /** Hood servo position limits; keep them inside the mechanism's hard stops. MEASURE. */
    public static double HOOD_MIN_POSITION = 0.24
            ;
    public static double HOOD_MAX_POSITION = 0.36;
    /** Hood position for the start of TeleOp (the "servos to start" button in INIT). MEASURE. */
    public static double HOOD_START_POSITION = 0.34;
    /** Seconds the servo needs to sweep the full 0..1 position range under load. MEASURE. */
    public static double HOOD_SECONDS_PER_FULL_RANGE = 0.375;
    /** Extra wait after the modeled arrival before the hood counts as settled. */
    public static double HOOD_SETTLE_SEC = 0.04;
    /**
     * Recoil compensation: while a ball is being fed (and HOOD_RECOIL_HOLD_SEC after) the hood is
     * sent to the shot position PLUS this (servo units, signed; negative = the other way), to
     * cancel the way the shot pushes the hood. Clamped to the hood limits. It does not change
     * the "settled" check, so it never blocks the next ball. TUNE by watching where shots land.
     * 0 = off.
     */
    public static double HOOD_RECOIL_OFFSET = 0.00;
    public static double HOOD_RECOIL_HOLD_SEC = 0.0;
    /** Set tr ue to widen the servo pulse range to HOOD_PWM_MIN_US..HOOD_PWM_MAX_US (Axon / goBILDA). */
    public static boolean HOOD_SET_PWM_RANGE = false;
    public static double HOOD_PWM_MIN_US = 500.0;
    public static double HOOD_PWM_MAX_US = 2500.0;

    // ------------------------------------------------------------------ intake
    /** Intake power when collecting; make it negative if the intake spins the wrong way. MEASURE. */
    public static double INTAKE_POWER = -1.0; //REVERSEINTAKE
    /** Intake encoder ticks per intake-motor revolution (537.7 = goBILDA 312 rpm, 384.5 = 435 rpm, 28 = bare motor). MEASURE. */
    public static double INTAKE_TICKS_PER_REV = 537.7;

    // ------------------------------------------------------------------ gate
    /** Gate servo positions. MEASURE with "Hood & Feeder Test". */
    public static double GATE_CLOSED_POSITION = 0.4;
    public static double GATE_OPEN_POSITION = 0.27;
    /** Wait after commanding the gate open before the feeder may push a ball. MEASURE. */
    public static double GATE_OPEN_DELAY_SEC = 0.15;

    // ------------------------------------------------------------------ feeder
    public static double FEEDER_FIRE_POWER = 1; //REVERSEFEEDER
    /** Power when not firing (0, or slightly negative to hold balls back). */
    public static double FEEDER_IDLE_POWER = 0.0;
    /**
     * Length of one feed pulse (one ball). After a pulse the shooter must be ready again before
     * the next one. Set to 0 or less for continuous feeding while ready.
     */
    public static double FEEDER_PULSE_SEC =0.08;
    /**
     * Feed pulse length measured in INTAKE rotations (one ball per pulse): a pulse ends once the
     * intake motor has turned this many revolutions since the pulse started. Needs the intake
     * encoder (the intake motor's own port). 0 or less = use the time FEEDER_PULSE_SEC instead.
     * TUNE: raise if the ball does not fully leave, lower if a second ball sneaks in.
     */
    public static double FEEDER_PULSE_INTAKE_REVS = 0;
    /** Failsafe for the rotation pulse: it ends after this long even if the intake did not turn (jam). */
    public static double FEEDER_PULSE_MAX_SEC = 1.0;
    /**
     * Rest between two pulses (balls), counted from the end of a pulse. It applies to normal and
     * force fire alike, so balls leave one at a time even when a button is held. Ignored when
     * FEEDER_PULSE_SEC <= 0.
     */
    public static double FEEDER_GAP_SEC = 0.05;
    /**
     * Closed-loop feeder speed while shooting, so every ball is pushed at the same speed whatever
     * the battery or the load. Needs the feeder to be a motor with its encoder plugged into the
     * same port. When false (or the feeder is a CR servo) FEEDER_FIRE_POWER is used open loop.
     */
    public static boolean FEEDER_PID_ENABLED = false;
    /**
     * Name of the motor port the feeder's encoder is plugged into, when it is NOT the feeder's own
     * port (the config name of that port's motor). Empty = read the feeder motor's own encoder.
     */
    public static String FEEDER_ENCODER_NAME = "BR";
    /** Flip if the encoder counts negative when the feeder pushes balls toward the flywheel. */
    public static boolean FEEDER_ENCODER_REVERSED = false;
    /** Target feeder speed in encoder ticks/s (magnitude; direction follows FEEDER_FIRE_POWER). TUNE. */
    public static double FEEDER_TARGET_TPS = 1800;
    /** Feedforward: power per tick/s. Start at ~1 / (max free speed in ticks/s). TUNE FIRST. */
    public static double FEEDER_KV = 1.0/2400;
    /** Proportional gain: power per tick/s of error. */
    public static double FEEDER_KP = 0.0005;
    /** Integral gain: power per (tick/s * s) of error. Removes steady-state error. */
    public static double FEEDER_KI = 0.002;
    /** Max power the integral term may add or remove. */
    public static double FEEDER_I_CLAMP = 0.3;

    // ------------------------------------------------------------------ shot solver
    /**
     * How far ahead the pose is predicted before aiming: sensor latency + one loop period +
     * mechanism response lag. NOT the feed delay: the mechanisms keep tracking until the ball
     * leaves. Tune 0.02 - 0.10 s.
     */
    public static double LOOKAHEAD_SEC = 0 ;
    /** Use filtered acceleration in the pose prediction (p + v*t + a*t^2/2). */
    public static boolean USE_ACCELERATION = true;
    /** First-order low-pass time constants for Pedro's velocity and for the derived acceleration (0 = off). */
    public static double VELOCITY_FILTER_SEC = 0.0;
    public static double ACCEL_FILTER_SEC = 0.08;
    /** Clamp on the acceleration estimate, in/s^2, to reject odometry glitches. */
    public static double MAX_ACCEL_ESTIMATE = 250.0;
    /**
     * Drag correction of the lead: the robot velocity is multiplied by (1 - e^(-c*t)) / c
     * instead of t. 0 = no drag (pure virtual goal). Tune by shooting while strafing past the
     * HIVE at constant speed: if shots land BEHIND (offset opposite to the direction the robot
     * is driving) the lead is too big, so raise it; if they land AHEAD, lower it (it may go
     * slightly negative). FRC 6328 used 0.375 for their 2026 ball.
     */
    public static double LEAD_DRAG_COEFF = 0;
    public static int SOLVER_MAX_ITERATIONS = 10;
    public static double SOLVER_TOLERANCE_SEC = 0.001;

    // ------------------------------------------------------------------ shot gating
    /**
     * Safety margin (in) kept on each side of the ball inside the CELL opening. The allowed
     * sideways miss from turret error is (20 in * cos(approach) - ball diameter) / 2 - this, so
     * it shrinks for oblique shots and for NECTAR. The margin absorbs pose, table and ball-to-ball
     * errors. The opponent's CELL is only about 5.5 in away (G417), so do not shrink it lightly.
     */
    public static double AIM_MARGIN_IN = 2.0;
    /**
     * Largest allowed angle between the ball's horizontal path and the CELL opening's inward
     * direction. The opening's usable width shrinks with cos(angle).
     */
    public static double MAX_APPROACH_ANGLE_DEG = 45.0;
    /** Every check must pass for this many consecutive loops before a ball is fed. */
    public static int READY_DEBOUNCE_LOOPS = 1;
    /**
     * Do not fire above these limits: latency, velocity noise and the constant-acceleration
     * prediction cause errors that grow with speed, turn rate and acceleration. Raise them as
     * testing proves accuracy (FRC 1706 needed acceleration compensation above about 3 m/s).
     */
    public static double MAX_SHOT_SPEED_IN_PER_SEC = 60.0;
    public static double MAX_SHOT_OMEGA_RAD_PER_SEC = 3.0;
    public static double MAX_SHOT_ACCEL_IN_PER_SEC2 = 120.0;
    /** Pause firing for this long after the operator reports a HIVE TIP (it is still moving). */
    public static double HIVE_TIP_PAUSE_SEC = 1.0;

    // ------------------------------------------------------------------ battery
    public static double NOMINAL_VOLTAGE = 12.0;
    public static double VOLTAGE_READ_PERIOD_SEC = 0.25;

    // ------------------------------------------------------------------ shot tables
    /*
     * Stationary shot tables, one per ball type, one row per calibration distance:
     *   { horizontal distance from the turret pivot to the CELL aim point (in),
     *     flywheel speed at the encoder shaft (rpm),
     *     hood servo position,
     *     time of flight from leaving the hood to crossing the CELL opening (s) }
     * Distances must be strictly increasing. Values between rows are linearly interpolated;
     * outside a table the shot is flagged out of range.
     *
     * MEASURE every row with the "Shooter Calibration" OpMode: 6-10 rows across the distances
     * you can actually shoot from. BIOBUZZ shots are short, about 15-80 in, because you must
     * stand in front of the up CELL. Choose the hood so balls arrive descending about 20-40 deg,
     * since the opening faces 30 deg up. The rpm and hood numbers below are placeholders. The
     * time-of-flight column is a drag-free estimate (exit 18.4 in, aim 58.4 in, 30 deg descent),
     * so the lead is roughly right until you measure it with slow-motion video.
     */
    public static double[][] SHOT_TABLE_POLLEN = {

//            {27.0,1800,0.320,0.6},
//            {30.1,1690,0.310,0.6},
            {28.4,1750,0.310,0.6},
            {33.9,1730 ,0.288,0.6},
            {38,1767,0.282,0.6},
            {42.2,1791,0.273,0.6},
            {45.5,1830,0.270,0.6},
            {48.6,1810,0.270,0.6},
            {52,1830,0.270,0.6}
//            {44.0,1790,0.280,0.6},
//            {45.1,1800,0.290,0.6},
//            {50.7,1700,0.280,0.6} //nu e buna dar orientativ






    };
    public static double[][] SHOT_TABLE_NECTAR = {
            {16.0, 2500.0, 0.80, 0.505},
            {24.0, 2600.0, 0.75, 0.528},
            {32.0, 2700.0, 0.70, 0.550},
            {40.0, 2800.0, 0.65, 0.572},
            {48.0, 2900.0, 0.60, 0.592},
            {56.0, 3000.0, 0.55, 0.612},
            {64.0, 3100.0, 0.50, 0.631},
            {72.0, 3200.0, 0.45, 0.650},
            {80.0, 3300.0, 0.40, 0.668},
            {88.0, 3400.0, 0.35, 0.686},
    };

    /**
     * This shooter launches POLLEN only. While false, TeleOp ignores the NECTAR toggle, so an
     * accidental press can never switch to the NECTAR table mid-match.
     */
    public static boolean SHOOTS_NECTAR = false;

    public static double[][] shotTable(BallType ball) {
        return ball == BallType.NECTAR ? SHOT_TABLE_NECTAR : SHOT_TABLE_POLLEN;
    }
}
