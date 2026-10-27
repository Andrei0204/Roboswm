package org.firstinspires.ftc.teamcode.shooter;

/**
 * Every tunable number used by the shoot-on-the-move system, in one place.
 *
 * Units: inches, seconds and radians, unless the name ends in _DEG (degrees) or _RPM.
 * Frames: the Pedro field frame described in {@link BioBuzzField}, and Pedro's robot frame
 * (+x forward, +y left, angles counter-clockwise positive, 0 = robot forward).
 *
 * Values tagged MEASURE are placeholders. The code runs with them, but the robot will not
 * hit the HIVE until they have been measured on your robot and field.
 *
 * The fields are public static and non-final so a live-tuning dashboard can edit them.
 * After editing a shot table at runtime, call {@link ShotSolver#reloadTables()}.
 */
public final class ShooterConstants {
    private ShooterConstants() {}

    // ------------------------------------------------------------------ hardware names
    /** Positional servos (servo mode) that turn the turret, all sent the same position. */
    public static String[] TURRET_SERVO_NAMES = {"turret"};
    /** One entry per turret servo: true reverses that servo (e.g. a mirrored second servo). */
    public static boolean[] TURRET_SERVO_REVERSED = {false};
    /**
     * Motor port (configured as a motor in the Robot Configuration) whose encoder slot has the
     * external turret encoder. Homing resets its count to 0 at the turret's zero, and the hub keeps
     * the count from AUTO to TeleOp, so nothing else may reset this port. Never use a Pedro
     * dead-wheel port (Pedro resets those).
     */
    public static String TURRET_ENCODER_NAME = "turretEncoder";
    /** Flywheel motors. The FIRST one must have the flywheel encoder plugged in. */
    public static String[] FLYWHEEL_MOTOR_NAMES = {"flywheel"};
    /** One entry per flywheel motor: true reverses that motor so +power spins the shot forward. */
    public static boolean[] FLYWHEEL_MOTOR_REVERSED = {false};
    public static String HOOD_SERVO_NAME = "hood";
    /** Feeder/transfer that pushes a ball into the flywheel: a motor or a CR servo. */
    public static String FEEDER_NAME = "feeder";
    /** Gate servo between the feeder and the launcher. */
    public static String GATE_SERVO_NAME = "gate";
    /** Intake motor that collects balls into the feeder. */
    public static String INTAKE_MOTOR_NAME = "intake";

    // ------------------------------------------------------------------ robot geometry
    /**
     * Turret pivot position relative to the point Pedro's pose describes (the robot center
     * used for your localizer pod offsets). MEASURE. +x forward, +y left, inches.
     */
    public static double TURRET_OFFSET_X_IN = 0.0;
    public static double TURRET_OFFSET_Y_IN = 0.0;

    // ------------------------------------------------------------------ turret (servo + encoder)
    /*
     * Turret angles are robot-relative, counter-clockwise positive seen from above, and 0 means
     * shooting straight forward. The Axon (servo mode) moves the turret through this map:
     *   position = TURRET_SERVO_POSITION_AT_ZERO + angleDeg / TURRET_DEG_PER_SERVO_UNIT
     * and the external encoder measures where the turret really is. Find the numbers with the
     * "Turret Servo Setup" OpMode.
     */
    /** Servo position at which the turret shoots straight forward. MEASURE. */
    public static double TURRET_SERVO_POSITION_AT_ZERO = 0.5;
    /**
     * Turret gearing: Axon MAX 60T drives 24T, compounded with 26T driving the 76T turret gear.
     * Two meshes, so the turret turns the same way as the servo, at 0.855 deg per servo deg.
     */
    public static final double TURRET_GEAR_RATIO = (60.0 / 24.0) * (26.0 / 76.0);
    /**
     * Turret degrees per 1.0 of servo position, counter-clockwise positive. Negative if a higher
     * position turns the turret clockwise. Default: about 350 deg of Axon MAX travel over the
     * 500-2500 us range (ServoCity listing) times the gearing, about 299 deg. MEASURE with
     * "Turret Servo Setup" and paste the measured value here.
     */
    public static double TURRET_DEG_PER_SERVO_UNIT = 350.0 * TURRET_GEAR_RATIO;
    /**
     * Soft limits (cables, frame, hard stops), robot-relative. The turret is also kept inside
     * the angles the servo can reach (positions 0 to 1, about +-150 deg here). A servo turret
     * covers under 360 deg, so targets in the gap behind it are unreachable and never fired at.
     * MEASURE.
     */
    public static double TURRET_MIN_DEG = -145.0;
    public static double TURRET_MAX_DEG = 145.0;
    /**
     * The servo's internal position loop trails a moving command. Commanding
     * angle + rate * this cancels that lag (velocity feedforward for a servo). Tune with the spin
     * test: 0.02-0.08 s is typical.
     */
    public static double TURRET_SERVO_LEAD_SEC = 0.04;
    /** Widen the turret servo pulse range to TURRET_PWM_MIN_US..MAX_US, the Axon's full travel. */
    public static boolean TURRET_SET_PWM_RANGE = true;
    public static double TURRET_PWM_MIN_US = 500.0;
    public static double TURRET_PWM_MAX_US = 2500.0;

    /** External encoder counts per encoder-shaft revolution (REV Through Bore = 8192). MEASURE. */
    public static double TURRET_ENCODER_TICKS_PER_REV = 8192.0;
    /**
     * Encoder-shaft revolutions per turret revolution, depending on where the encoder is mounted:
     * on the 76T turret shaft = 1.0; on the 24T/26T compound shaft = 76/26 = 2.923; on the servo
     * shaft = (24/60)*(76/26) = 1.169. "Turret Servo Setup" checks this. MEASURE.
     */
    public static double TURRET_ENCODER_REVS_PER_TURRET_REV = 1.0;
    /** +1 or -1 so the encoder angle INCREASES when the turret turns counter-clockwise. MEASURE. */
    public static int TURRET_ENCODER_SIGN = 1;
    /**
     * Homing (gamepad1 Y in INIT): the turret moves to the servo's zero position, waits until the
     * encoder shows it is still, and the encoder count is reset to 0 there.
     */
    public static double TURRET_HOME_STILL_DEG_PER_SEC = 3.0;
    public static double TURRET_HOME_STILL_SEC = 0.15;
    public static double TURRET_HOME_MIN_SEC = 0.25;
    public static double TURRET_HOME_TIMEOUT_SEC = 1.5;
    /**
     * Encoder correction of the servo: while aiming at a slowly moving target, the error the
     * encoder sees (backlash, load, servo map error) is integrated into a trim on the servo
     * command, at this rate (1/s). 0 disables it. The trim is limited to MAX_CORRECTION and only
     * learns inside the error window and below the rate limit, so it never fights a fast swing.
     */
    public static double TURRET_CORRECTION_GAIN = 4.0;
    public static double TURRET_MAX_CORRECTION_DEG = 8.0;
    public static double TURRET_CORRECTION_WINDOW_DEG = 6.0;
    public static double TURRET_CORRECTION_MAX_RATE_DEG_PER_SEC = 30.0;
    /**
     * Proportional encoder correction inside the window, also while tracking: the servo is
     * commanded an extra KP * error. It cuts tracking error from servo-scale error by about
     * 1 / (1 + KP). Keep it below 1: the outer loop is then stable for any servo lag. Errors
     * under the deadband (gear backlash) are ignored so the turret does not hunt.
     */
    public static double TURRET_ENCODER_KP = 0.5;
    public static double TURRET_P_DEADBAND_DEG = 0.3;
    /** A shot needs the encoder rate within this of the commanded rate (no swinging through). */
    public static double TURRET_RATE_TOLERANCE_DEG_PER_SEC = 40.0;
    /** Clamp on the turret rate feedforward (rad/s); it grows without bound right next to the HIVE. */
    public static double TURRET_MAX_FEEDFORWARD_RATE = 10.0;

    // ------------------------------------------------------------------ flywheel
    /** Encoder counts per revolution of the shaft the flywheel encoder is on (goBILDA 5203 1:1 = 28). */
    public static double FLYWHEEL_TICKS_PER_REV = 28.0;
    /** Feedforward: power = KS + KV * rpm (at NOMINAL_VOLTAGE). MEASURE with the calibration OpMode. */
    public static double FLYWHEEL_KS = 0.03;
    public static double FLYWHEEL_KV = 1.0 / 6000.0;
    /** Proportional power per rpm of error. */
    public static double FLYWHEEL_KP = 0.0006;
    /** When more than this far BELOW target (e.g. right after a shot), apply full power. */
    public static double FLYWHEEL_BANG_BANG_RPM = 300.0;
    /** The flywheel counts as "at speed" inside [target - BELOW, target + ABOVE]. */
    public static double FLYWHEEL_TOLERANCE_BELOW_RPM = 60.0;
    public static double FLYWHEEL_TOLERANCE_ABOVE_RPM = 120.0;
    /** Speed held while the shooter is not armed (0 = coast). */
    public static double FLYWHEEL_IDLE_RPM = 0.0;
    /**
     * Flywheel power changes smaller than this are not sent to the hub (saves a write most loops).
     * 0.005 is about 30 rpm at KV = 1/6000; keep THRESHOLD / KV well inside the at-speed window.
     */
    public static double FLYWHEEL_WRITE_THRESHOLD = 0.005;

    // ------------------------------------------------------------------ hood
    /** Hood servo position limits; keep them inside the mechanism's hard stops. MEASURE. */
    public static double HOOD_MIN_POSITION = 0.10;
    public static double HOOD_MAX_POSITION = 0.90;
    /** Seconds the servo needs to sweep the full 0..1 position range under load. MEASURE. */
    public static double HOOD_SECONDS_PER_FULL_RANGE = 0.60;
    /** Extra wait after the modeled arrival before the hood counts as settled. */
    public static double HOOD_SETTLE_SEC = 0.04;
    /** Set true to widen the servo pulse range to HOOD_PWM_MIN_US..HOOD_PWM_MAX_US (Axon / goBILDA). */
    public static boolean HOOD_SET_PWM_RANGE = false;
    public static double HOOD_PWM_MIN_US = 500.0;
    public static double HOOD_PWM_MAX_US = 2500.0;

    // ------------------------------------------------------------------ intake
    /** Intake power when collecting; make it negative if the intake spins the wrong way. MEASURE. */
    public static double INTAKE_POWER = 1.0;

    // ------------------------------------------------------------------ gate
    /** Gate servo positions. MEASURE with "Hood & Feeder Test". */
    public static double GATE_CLOSED_POSITION = 0.20;
    public static double GATE_OPEN_POSITION = 0.60;
    /** Wait after commanding the gate open before the feeder may push a ball. MEASURE. */
    public static double GATE_OPEN_DELAY_SEC = 0.15;

    // ------------------------------------------------------------------ feeder
    public static double FEEDER_FIRE_POWER = 1.0;
    /** Power when not firing (0, or slightly negative to hold balls back). */
    public static double FEEDER_IDLE_POWER = 0.0;
    /**
     * Length of one feed pulse (one ball). After a pulse the shooter must be ready again before
     * the next one. Set to 0 or less for continuous feeding while ready.
     */
    public static double FEEDER_PULSE_SEC = 0.15;

    // ------------------------------------------------------------------ shot solver
    /**
     * How far ahead the pose is predicted before aiming: sensor latency + one loop period +
     * mechanism response lag. NOT the feed delay: the mechanisms keep tracking until the ball
     * leaves. Tune 0.02 - 0.10 s.
     */
    public static double LOOKAHEAD_SEC = 0.05;
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
    public static double LEAD_DRAG_COEFF = 0.0;
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
    public static int READY_DEBOUNCE_LOOPS = 2;
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
            {16.0, 2400.0, 0.80, 0.505},
            {24.0, 2500.0, 0.75, 0.528},
            {32.0, 2600.0, 0.70, 0.550},
            {40.0, 2700.0, 0.65, 0.572},
            {48.0, 2800.0, 0.60, 0.592},
            {56.0, 2900.0, 0.55, 0.612},
            {64.0, 3000.0, 0.50, 0.631},
            {72.0, 3100.0, 0.45, 0.650},
            {80.0, 3200.0, 0.40, 0.668},
            {88.0, 3300.0, 0.35, 0.686},
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
