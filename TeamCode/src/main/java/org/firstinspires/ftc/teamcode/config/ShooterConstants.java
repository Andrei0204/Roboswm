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
    public static double TURRET_OFFSET_X_IN = -2.94;
    public static double TURRET_OFFSET_Y_IN = 0.0;

    // ------------------------------------------------------------------ turret (CR servo + encoder)
    /*
     * Turret angles are robot-relative, counter-clockwise positive seen from above, and 0 means
     * shooting straight forward. The continuous-rotation servo only takes a power; the external
     * encoder is the turret's only position feedback, so the controller closes the loop on it:
     *   power = KP * error + KD * rateError + KV * targetRate + KS * sign(error)
     * Find the encoder numbers with the "Turret Servo Setup" OpMode.
     */
    /** Power per rad of angle error. TUNE: raise until it settles fast without oscillating. */
    public static double TURRET_KP = 0.2;
    /** Power per rad/s of rate error (damping). Raise if the turret overshoots. */
    public static double TURRET_KD = 0.018;
    /** Power per rad/s of target rate: the feedforward that tracks a moving target. TUNE with the spin test. */
    public static double TURRET_KV = 0.05;
    /** Power that just breaks friction; added in the direction of the error outside the deadband. */
    public static double TURRET_KS = 0.04;
    /** Limit on the turret power (0..1). */
    public static double TURRET_MAX_POWER = 1.0;
    /** Inside this error the turret is left unpowered, so it does not hunt on gear backlash. */
    public static double TURRET_DEADBAND_DEG = 5;
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
    public static double FLYWHEEL_KS = 0.09;
    public static double FLYWHEEL_KV = 0.00022076;
    /** Proportional power per rpm of error. */
    public static double FLYWHEEL_KP = 0.005;
    /** When more than this far BELOW target (e.g. right after a shot), apply full power. */
    public static double FLYWHEEL_BANG_BANG_RPM = 50;
    /** The flywheel counts as "at speed" inside [target - BELOW, target + ABOVE]. */
    public static double FLYWHEEL_TOLERANCE_BELOW_RPM = 300; //am modificat de la 100
    public static double FLYWHEEL_TOLERANCE_ABOVE_RPM = 300; //si aici
    /** Speed held while the shooter is not armed (0 = coast). */
    public static double FLYWHEEL_IDLE_RPM = 0.0;
    /**
     * Flywheel power changes smaller than this are not sent to the hub (saves a write most loops).
     * 0.005 is about 30 rpm at KV = 1/6000; keep THRESHOLD / KV well inside the at-speed window.
     */
    public static double FLYWHEEL_WRITE_THRESHOLD = 0.005;

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
    /** Set tr ue to widen the servo pulse range to HOOD_PWM_MIN_US..HOOD_PWM_MAX_US (Axon / goBILDA). */
    public static boolean HOOD_SET_PWM_RANGE = false;
    public static double HOOD_PWM_MIN_US = 500.0;
    public static double HOOD_PWM_MAX_US = 2500.0;

    // ------------------------------------------------------------------ intake
    /** Intake power when collecting; make it negative if the intake spins the wrong way. MEASURE. */
    public static double INTAKE_POWER = -1.0; //REVERSEINTAKE

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
     * Rest between two pulses (balls), counted from the end of a pulse. It applies to normal and
     * force fire alike, so balls leave one at a time even when a button is held. Ignored when
     * FEEDER_PULSE_SEC <= 0.
     */
    public static double FEEDER_GAP_SEC = 0.02;

    // ------------------------------------------------------------------ shot solver
    /**
     * How far ahead the pose is predicted before aiming: sensor latency + one loop period +
     * mechanism response lag. NOT the feed delay: the mechanisms keep tracking until the ball
     * leaves. Tune 0.02 - 0.10 s.
     */
    public static double LOOKAHEAD_SEC = 0.03;
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
    public static double LEAD_DRAG_COEFF = 0.4;
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



            {27.0,1800,0.320,0.6},
            {30.1,1690,0.310,0.6},
            {35.4,1750,0.310,0.6},
            {44.0,1790,0.280,0.6},
            {45.1,1800,0.290,0.6},
            {50.7,1700,0.280,0.6} //nu e buna dar orientativ






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
