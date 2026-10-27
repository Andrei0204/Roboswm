# Pedro Pathing v3 API (core 3.0.2 / revhub 3.0.2 / tuning 1.0.2): pose, velocity, acceleration, relocalization and teleop for shoot-on-the-move

Method notes. Everything marked "javap" was checked with `javap -p -c -l` (JDK 25 from `/snap/android-studio/247/jbr/bin/javap`) against the compiled jars in the local Gradle cache. Everything marked "source" was read from the `-sources.jar` files that ship next to the revhub and tuning AARs. Extracted copies are in `/tmp/claude-1000/-home-andrei0204-Coding-Robo-FtcRobotController-12-0/a9ad61c7-931c-4680-864f-5ff7b6c92b3d/scratchpad/pedro/`.

Where the artifacts are:
- `com.pedropathing:core:3.0.2` is a plain JAR, pulled in transitively by revhub: [core-3.0.2.jar](file:///home/andrei0204/.gradle/caches/modules-2/files-2.1/com.pedropathing/core/3.0.2/971d7ef36fa68b8499e867efff8e915c2a7bc3e6/core-3.0.2.jar). No sources jar is cached, so the core API was verified with javap only. The classes do contain LocalVariableTable debug info, which is how the parameter names below were recovered.
- `com.pedropathing:revhub:3.0.2`: [revhub-3.0.2.aar](file:///home/andrei0204/.gradle/caches/modules-2/files-2.1/com.pedropathing/revhub/3.0.2/5740970f170130fbd79473ecc85f20c7b047bf79/revhub-3.0.2.aar) and [revhub-3.0.2-sources.jar](file:///home/andrei0204/.gradle/caches/modules-2/files-2.1/com.pedropathing/revhub/3.0.2/764a1913a7175783a47cf7832b0c1aaab72ccfaf/revhub-3.0.2-sources.jar)
- `com.pedropathing:tuning:1.0.2`: [tuning-1.0.2.aar](file:///home/andrei0204/.gradle/caches/modules-2/files-2.1/com.pedropathing/tuning/1.0.2/af5068b23313098df4df26d1b8502d64bed71f70/tuning-1.0.2.aar) and [tuning-1.0.2-sources.jar](file:///home/andrei0204/.gradle/caches/modules-2/files-2.1/com.pedropathing/tuning/1.0.2/da526f40596336b4422ceabededdb2ffcea24e6a/tuning-1.0.2-sources.jar)

## Q1. Packages, classes and how to construct a Follower in `Constants.create(HardwareMap)`

### Takeaway
The verified v3 constructor is `new Follower(Localizer localizer, Drivetrain drivetrain, Algorithm algorithm)`, with the **localizer first**. The comment in the local and upstream `Constants.java` stub (`new Follower(Drivetrain, Localizer, Foresight)`) has the wrong argument order. The usual algorithm is `new Foresight(ForesightConfig)`. Every config class is built with a lambda: `new XConfig(c -> { c.field.set(value); })`.

### Cited Findings
- The library is pure Java. No class in core, revhub or tuning carries `kotlin/Metadata`, so methods are ordinary Java methods with no Kotlin property getters. The revhub POM still declares a compile dependency on `org.jetbrains.kotlin:kotlin-stdlib:2.3.21`. — [core-3.0.2.jar, javap/grep](file:///home/andrei0204/.gradle/caches/modules-2/files-2.1/com.pedropathing/core/3.0.2/971d7ef36fa68b8499e867efff8e915c2a7bc3e6/core-3.0.2.jar); [revhub-3.0.2.pom](file:///home/andrei0204/.gradle/caches/modules-2/files-2.1/com.pedropathing/revhub/3.0.2/d1808891fc5e075d2a579a526046aed6a8bea881/revhub-3.0.2.pom)
- Core packages and classes (javap). These replace the guessed names in the assignment. There is **no** `com.pedropathing.math.Angle`, and `Angle`/`Utils` live in `com.pedropathing.utils`.
  - `com.pedropathing.follower`: `Follower`, `Follower.Mode` (enum `FOLLOW, HOLD, MANUAL, IDLE`), `ManualDrive`, `FollowerLog`
  - `com.pedropathing.localization`: `Localizer` (interface), `MotionState`, `FusionLocalizer`
  - `com.pedropathing.math`: `Pose`, `Vector2D`, `Vector` (n-dimensional), `Velocity`, `Twist`, `Matrix`
  - `com.pedropathing.drivetrain`: `Drivetrain` (interface), `DrivePowers`
  - `com.pedropathing.algorithm`: `Algorithm` (interface), `Foresight`, `ForesightConfig`, `ForesightPowerAllocator`
  - `com.pedropathing.utils`: `Angle`, `Angle.Unit` (`DEGREES`, `RADIANS`), `Utils`, `Timer`, `Pair`
  - `com.pedropathing.api`: `Paths` (`line`, `curve`, `through`, `path`), `PoseFactory`
  - `com.pedropathing.controllers`: `Controller`, `PIDController`, `PiecewiseController`
  - `com.pedropathing.config`: `ConfigVar<T>`, `Configuration<T>`

  — [core-3.0.2.jar (javap)](file:///home/andrei0204/.gradle/caches/modules-2/files-2.1/com.pedropathing/core/3.0.2/971d7ef36fa68b8499e867efff8e915c2a7bc3e6/core-3.0.2.jar)
- `public Follower(Localizer localizer, Drivetrain drivetrain, Algorithm algorithm)` is the **only** constructor. `Follower` also has the public final fields `localizer`, `drivetrain` and `holdEnd` (`ConfigVar<Boolean>`), and `public Follower withLogger(Consumer<FollowerLog> logger)`. — [core-3.0.2.jar (javap -l)](file:///home/andrei0204/.gradle/caches/modules-2/files-2.1/com.pedropathing/core/3.0.2/971d7ef36fa68b8499e867efff8e915c2a7bc3e6/core-3.0.2.jar)
- `public class Foresight implements Algorithm` has the constructor `Foresight(ForesightConfig config)`. `ForesightConfig(Configuration<ForesightConfig>)` is `final`, and its fields are `ConfigVar`s.
  - **Required** fields: `headingFeedback`, `forwardTranslational`, `strafeTranslational`, `brake`, `coast` (all `Controller`); `linearBrakeCoefficients`, `quadraticBrakeCoefficients` (`Matrix`); `headingBrakeCoefficients` (`Vector2D`); `maxAchievableForwardVelocity`, `maxAchievableStrafeVelocity`, `naturalForwardDeceleration`, `naturalStrafeDeceleration` (`Double`).
  - Fields with defaults (for example): `holdPointTranslationalScaling` = 0.45, `holdPointHeadingScaling` = 0.35, `maxBrakingPower` = 0.2, `translationalDeviationTolerance` = 2.5, `headingDeviationTolerance` = 11.25, `brakeAtEnd` = true, `timeoutConstraint` = 100.0.

  — [core-3.0.2.jar (javap -c ForesightConfig.<init>)](file:///home/andrei0204/.gradle/caches/modules-2/files-2.1/com.pedropathing/core/3.0.2/971d7ef36fa68b8499e867efff8e915c2a7bc3e6/core-3.0.2.jar)
- Config pattern. `Configuration<T>` is a single-method interface `void configure(T)`. `ConfigVar<T> implements Supplier<T>` with `T get()`, `void set(T)` and `Modifier at(T)`. Factories are `required(...)`, `of(default, ...)` and the `Nullable` variants. Each config constructor just calls `config.configure(this)`. — [core-3.0.2.jar (javap)](file:///home/andrei0204/.gradle/caches/modules-2/files-2.1/com.pedropathing/core/3.0.2/971d7ef36fa68b8499e867efff8e915c2a7bc3e6/core-3.0.2.jar); [revhub sources PinpointConfig.java](file:///home/andrei0204/.gradle/caches/modules-2/files-2.1/com.pedropathing/revhub/3.0.2/764a1913a7175783a47cf7832b0c1aaab72ccfaf/revhub-3.0.2-sources.jar)
- revhub drivetrains (package `com.pedropathing.revhub.drivetrains`, source). `public class Mecanum implements Drivetrain` has the constructor `Mecanum(HardwareMap map, MecanumConfig config)`.
  - Required `MecanumConfig` fields: `frontLeftName`, `backLeftName`, `frontRightName`, `backRightName` (`String`), and the four `*Direction` fields (`DcMotorSimple.Direction`).
  - Optional fields: `manualBrakeMode` (default `true`) and `powerThreshold` (default 0.01).
  - Other classes in the package: `Swerve`/`SwerveConfig`, `CoaxialPod`, `CachedMotor`.

  — [revhub sources Mecanum.java / MecanumConfig.java](file:///home/andrei0204/.gradle/caches/modules-2/files-2.1/com.pedropathing/revhub/3.0.2/764a1913a7175783a47cf7832b0c1aaab72ccfaf/revhub-3.0.2-sources.jar)
- revhub localizers (package `com.pedropathing.revhub.localizers`, source). All take `(HardwareMap, XConfig)`:
  - `PinpointLocalizer(HardwareMap, PinpointConfig)`
    - Required: `name`, `xPodDirection`, `yPodDirection` (`GoBildaPinpointDriver.EncoderDirection`), `xPodOffset`, `yPodOffset`.
    - Defaults: `offsetUnits` = INCH, `globalDistanceUnit` = INCH, `encoderResolutionUnit` = INCH, `podType` = `goBILDA_4_BAR_POD`, `ticksPerUnit` = `OptionalDouble.empty()`, `resetMode` = `PinpointLocalizer.ResetMode.RECALIBRATE_IMU`.
  - `ThreeWheelLocalizer(HardwareMap, ThreeWheelConfig)`
    - Required: encoder names, `leftPodY`, `rightPodY`, `strafePodX`, `forwardTicksToInches`, `strafeTicksToInches`, `turnTicksToRadians`, and three `*EncoderDirection` values as `Double` (`Encoder.FORWARD` = 1, `Encoder.REVERSE` = -1).
  - `ThreeWheelIMULocalizer(HardwareMap, ThreeWheelIMUConfig)`: the same fields plus `imuName` and `imu` (`CustomIMU`, for example `new RevHubIMU(RevHubOrientationOnRobot)`).
  - `TwoWheelLocalizer(HardwareMap, TwoWheelConfig)`: `xPodName`, `yPodName`, `imuName`, offsets, ticks-to-inches values, directions and `imu`.
  - `OTOSLocalizer(HardwareMap, OTOSConfig)`: `name`, `linearUnit` = INCH, `linearScalar` = 1.0, `angularScalar` = 1.0, and the required `offset` (`Pose`).
  - `OctoQuadLocalizer(HardwareMap, OctoQuadConfig)`: `name`, pod directions, ports (defaults 0 and 1), offsets, the required `ticksPerUnit`, `headingScalar` = 1.0, and `localizerVelocityIntervalMS` = 25.

  — [revhub sources localizers/*.java](file:///home/andrei0204/.gradle/caches/modules-2/files-2.1/com.pedropathing/revhub/3.0.2/764a1913a7175783a47cf7832b0c1aaab72ccfaf/revhub-3.0.2-sources.jar)
- Side effects when the localizer is constructed:
  - `PinpointLocalizer` calls `reset()`, which with the default `RECALIBRATE_IMU` runs `pinpoint.recalibrateIMU()` followed by `Thread.sleep(500)`, then calls `update()`. The 500 ms sleep was added in v3.0.1.
  - `OTOSLocalizer` calls `calibrateImu()` and `resetTracking()`.
  - `OctoQuadLocalizer` busy-waits until `LocalizerStatus.RUNNING`.

  — [revhub sources](file:///home/andrei0204/.gradle/caches/modules-2/files-2.1/com.pedropathing/revhub/3.0.2/764a1913a7175783a47cf7832b0c1aaab72ccfaf/revhub-3.0.2-sources.jar); [v3.0.1 release notes](https://github.com/Pedro-Pathing/PedroPathing/releases/tag/v3.0.1)
- The local `TeamCode/.../pedro/Constants.java` is a stub that returns `null`. Its only hint is the comment `// return new Follower(Drivetrain, Localizer, Foresight);`. The upstream Quickstart `master` has the identical stub but depends on `revhub:3.0.1` and `tuning:1.0.1`. The local project uses `revhub:3.0.2` and `tuning:1.0.2`. — [local Constants.java](file:///home/andrei0204/Coding/Robo/FtcRobotController-12.0/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/Constants.java); [local build.dependencies.gradle](file:///home/andrei0204/Coding/Robo/FtcRobotController-12.0/build.dependencies.gradle); [Quickstart Constants.java](https://github.com/Pedro-Pathing/Quickstart/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/Constants.java); [Quickstart build.dependencies.gradle](https://github.com/Pedro-Pathing/Quickstart/blob/master/build.dependencies.gradle)
- The docs say "The `create()` method in your `Constants` class creates a follower using the drivetrain, localizer, and algorithm you configured," and use `follower = Constants.create(hardwareMap);`. — [pedropathing.com: Setting Up Your Autonomous](https://pedropathing.com/docs/pathing/guide/setting-up-auto); [TeleOp Usage](https://pedropathing.com/docs/pathing/guide/teleop-usage)
- The local tuner `Tests.java` builds the follower exactly as `new Follower(localizerFunction.apply(hardwareMap), drivetrainFunction.apply(hardwareMap), algorithmSupplier.get())`, which is localizer, then drivetrain, then algorithm. The AutoTune tuners emit config code in this shape:
  - `MecanumTuner`: `public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> { c.frontLeftName.set(...); ... c.frontLeftDirection.set(DcMotorSimple.Direction.X); ... });`
  - `PinpointTuner`: `public static PinpointConfig localizerConfig = new PinpointConfig(c -> { c.name.set(...); c.podType.set(...); c.xPodOffset.set(..); c.yPodOffset.set(..); c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.X); ... c.globalDistanceUnit.set(DistanceUnit.INCH); c.offsetUnits.set(DistanceUnit.INCH); });`
  - `ForesightTuner`: `public static ForesightConfig foresightConfig = new ForesightConfig(c -> { c.forwardTranslational.set(Controller.piecewise(...).put(2.5, ...)); c.coast.set(Controller.proportionalFeedforward(..)); c.brake.set(...); c.headingFeedback.set(Controller.proportional(..)); c.headingBrakeCoefficients.set(Vector2D.cartesian(..,..)); c.linearBrakeCoefficients.set(Matrix.diag(..,..)); c.quadraticBrakeCoefficients.set(Matrix.diag(..,..)); c.maxAchievableForwardVelocity.set(..); ... });`

  — [local procedures/Tests.java](file:///home/andrei0204/Coding/Robo/FtcRobotController-12.0/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/procedures/Tests.java); [MecanumTuner.java](file:///home/andrei0204/Coding/Robo/FtcRobotController-12.0/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/procedures/MecanumTuner.java); [PinpointTuner.java](file:///home/andrei0204/Coding/Robo/FtcRobotController-12.0/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/procedures/PinpointTuner.java); [ForesightTuner.java](file:///home/andrei0204/Coding/Robo/FtcRobotController-12.0/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/procedures/ForesightTuner.java)
- The `tuning` artifact (`com.pedropathing.tuning.autotune`) registers procedures through static, zero-argument methods annotated `@Tuner(name = "...")` that return `Procedure`; `TunerScanner` rejects any other shape. `Procedure(String name, String description)` and `TuningOpMode<T>(String name, String description, boolean canStop)` are the base classes the local procedures extend. The local `Tuning.java` is an empty placeholder (`// Tuners go here`). — [tuning sources Tuner.java / TunerScanner.java](file:///home/andrei0204/.gradle/caches/modules-2/files-2.1/com.pedropathing/tuning/1.0.2/da526f40596336b4422ceabededdb2ffcea24e6a/tuning-1.0.2-sources.jar); [local Tuning.java](file:///home/andrei0204/Coding/Robo/FtcRobotController-12.0/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/Tuning.java)

### Inferences
- Below is a working `Constants.create` assembled only from verified signatures. Numeric values are placeholders until the AutoTune procedures produce real ones, and imports are omitted.
  ```java
  public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> { /* names + directions */ });
  public static PinpointConfig localizerConfig = new PinpointConfig(c -> { /* name, pod type, offsets, directions */ });
  public static ForesightConfig foresightConfig = new ForesightConfig(c -> { /* all REQUIRED fields above */ });

  public static Follower create(HardwareMap h) {
      Localizer localizer = new PinpointLocalizer(h, localizerConfig);   // ~0.5 s IMU recalibration sleep
      Drivetrain drivetrain = new Mecanum(h, drivetrainConfig);
      return new Follower(localizer, drivetrain, new Foresight(foresightConfig));
  }
  ```
- Missing `required()` fields are probably only detected lazily, when `ConfigVar.get()` runs (`get()` calls a private `require()`). Constructing a config therefore does not validate it; a missing field throws later, for example inside `new Mecanum(...)` or during the first path calculation. This was observed in bytecode but not run.
- The config objects are `public static` in the tuner output, so `Constants` can keep them as static fields as the generated code does.

### Gaps
- I did not run the code on a robot, and I did not find the exact exception type thrown by `ConfigVar.require()` for a missing required field.
- `Swerve` and `CoaxialPod` constructors were not examined.

## Q2. Reading pose, field velocity, angular velocity and acceleration each loop: methods, frames, units, filtering, `update()`

### Takeaway
- **Pose:** `follower.pose()`.
- **Field-frame velocity:** `follower.velocity()`, a `Velocity` with public final fields `vx`, `vy` (in/s) and `omega` (rad/s).
- **Robot-frame velocity:** `follower.twist()`, a `Twist` with the same field names, where x is forward and y is left.
- **Acceleration:** v3 has **no** acceleration API (v2's `getAcceleration()` was removed), so you must difference and filter velocity yourself.
- **Filtering:** Pedro filters nothing. Velocity quality depends entirely on the localizer.
- **`update()`:** `follower.update()` must be called exactly once per loop. It is the only thing that refreshes the localizer.

### Cited Findings
- `Follower` getters (javap): `Pose pose()`, `Velocity velocity()`, `Twist twist()`, `double tangentialVelocity()`, `Mode mode()`, `boolean following()`, `isBusy()`, `holding()`, `manual()`, `idle()`. They delegate directly to `localizer.pose()`, `localizer.velocity()` and `localizer.twist()`. **No** method named `getPose`, `getVelocity`, `getAcceleration` or `getAngularVelocity` exists in v3. — [core-3.0.2.jar (javap -c Follower)](file:///home/andrei0204/.gradle/caches/modules-2/files-2.1/com.pedropathing/core/3.0.2/971d7ef36fa68b8499e867efff8e915c2a7bc3e6/core-3.0.2.jar)
- `Localizer` interface:
  - Abstract methods: `setPose(Pose)`, `MotionState state()`, `update()`, `reset()`.
  - Default methods: `setX`, `setY`, `setHeading` (each implemented as `setPose(pose().withX(..))` and so on), `pose()`, `velocity()` and `twist()` (each is `state().pose()` / `state().velocity()` / `state().twist()`), and `Map<String,Object> debug()`.
  - `MotionState` is an immutable snapshot `{Pose pose, Velocity velocity, Twist twist}`, created with `ofVelocity(Pose, Velocity)` (the twist is computed as `velocity.toTwist(pose.heading())`), with `ofTwist(Pose, Twist)`, or with `withPose(Pose)`.

  — [core-3.0.2.jar (javap -c Localizer, MotionState)](file:///home/andrei0204/.gradle/caches/modules-2/files-2.1/com.pedropathing/core/3.0.2/971d7ef36fa68b8499e867efff8e915c2a7bc3e6/core-3.0.2.jar)
- Frames, verified in bytecode:
  - `Velocity.toTwist(double heading)` returns `Twist(vx·cosh + vy·sinh, −vx·sinh + vy·cosh, omega)`, i.e. a rotation by −heading from field frame to body frame.
  - `Twist.toVelocity(double heading)` rotates by +heading.
  - `Vector2D.toBodyFrame(h)` is `rotate(−h)` and `toWorldFrame(h)` is `rotate(h)`. `rotate` is the standard counter-clockwise rotation.
  - `omega` is copied unchanged, so it is identical in both frames.

  — [core-3.0.2.jar (javap -c Velocity, Twist, Vector2D)](file:///home/andrei0204/.gradle/caches/modules-2/files-2.1/com.pedropathing/core/3.0.2/971d7ef36fa68b8499e867efff8e915c2a7bc3e6/core-3.0.2.jar)
- The official docs agree. `follower.velocity()` gives motion "along each axis of the absolute field frame" via `.vx`, `.vy`, `.omega`. `follower.twist()` is robot-local, with positive x forward and positive y left. `twist().omega` equals `velocity().omega`. The page gives no units and says nothing about acceleration. — [pedropathing.com: Types of Velocities](https://pedropathing.com/docs/pathing/reference/velocity)
- The local tuners use these frames too: `localizer.twist().toVector2D().x()` for forward speed, `.y()` for strafe speed, and `localizer.velocity().omega` for turn rate. — [local ForesightTuner.java (lines ~167, 225, 283, 377, 502, 794)](file:///home/andrei0204/Coding/Robo/FtcRobotController-12.0/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/procedures/ForesightTuner.java)
- `Velocity` API: public final `vx`, `vy`, `omega`; `Velocity(double vx, double vy, double omega)`, `toTwist(double heading)`, `toVector2D()`, `linSpeed()` (= hypot(vx, vy)), `plus`, `minus`, `scale(double time)`, `Velocity.zero()`, `fromVector(Vector2D)`, `fromPose(Pose)`. `Twist` mirrors this with `times(double)`, `toVelocity(double heading)` and `riemannianLog(Pose a, Pose b)`. — [core-3.0.2.jar (javap -l)](file:///home/andrei0204/.gradle/caches/modules-2/files-2.1/com.pedropathing/core/3.0.2/971d7ef36fa68b8499e867efff8e915c2a7bc3e6/core-3.0.2.jar)
- Pose prediction helpers that are useful for lead or latency compensation:
  - `Pose.exp(Velocity v, double time)` returns `Pose(x + vx·t, y + vy·t, heading + omega·t)`, a straight-line field-frame extrapolation.
  - `Pose.exp(Twist tw, double time)` integrates a constant body twist along an arc (SE(2) exponential), and falls back to the Velocity version when |omega| is tiny.
  - Also available: `Pose.compose`, `invert`, `log()` (returns `Twist`) and `Pose.interpolate(lower, upper, ratio)`.

  — [core-3.0.2.jar (javap -c Pose)](file:///home/andrei0204/.gradle/caches/modules-2/files-2.1/com.pedropathing/core/3.0.2/971d7ef36fa68b8499e867efff8e915c2a7bc3e6/core-3.0.2.jar)
- Units and filtering for each localizer (source):
  - **Pinpoint:** pose is `getPosX/getPosY(globalDistanceUnit)` (default INCH) and `getHeading(AngleUnit.RADIANS)`. Velocity is `new Velocity(getVelX(unit), getVelY(unit), getHeadingVelocity(UnnormalizedAngleUnit.RADIANS))`, passed directly into `MotionState.ofVelocity`. Pedro therefore treats Pinpoint's velocity as field-frame, with no Pedro-side filtering.
  - **OTOS:** uses `otos.getVelocity()` directly, in `linearUnit` (default INCH) and radians/s.
  - **OctoQuad:** uses device `velX_mmS`/`velY_mmS` converted to `globalDistanceUnit`, plus `velHeading_radS`. The device's velocity window is `localizerVelocityIntervalMS` (default 25 ms; "Longer periods give higher resolution with more latency").
  - **ThreeWheel / TwoWheel / ThreeWheelIMU:** velocity is a raw one-loop finite difference, `globalDeltas / dtSeconds`, with `dt` taken from a `Timer` in nanoseconds/1e9. The result is in/s and rad/s with no smoothing.

  — [revhub sources PinpointLocalizer/OTOSLocalizer/OctoQuadLocalizer/ThreeWheelLocalizer/TwoWheelLocalizer/OctoQuadConfig](file:///home/andrei0204/.gradle/caches/modules-2/files-2.1/com.pedropathing/revhub/3.0.2/764a1913a7175783a47cf7832b0c1aaab72ccfaf/revhub-3.0.2-sources.jar)
- Pinpoint's velocity frame is ambiguous in the SDK Javadoc, which says `getVelX` returns "the estimated X (forward) velocity of the robot in specified unit/sec". Road Runner's official quickstart treats the same values as world-frame (`Vector2d worldVelocity = new Vector2d(driver.getVelX(...), driver.getVelY(...)); robotVelocity = Rotation2d.fromDouble(-heading).times(worldVelocity)`). Pedro also treats them as field-frame. — [FTC SDK 12.0 Hardware-sources GoBildaPinpointDriver.java](file:///home/andrei0204/.gradle/caches/modules-2/files-2.1/org.firstinspires.ftc/Hardware/12.0.0/8a24b109760c40594b6eb7341e05ec16834c7174/Hardware-12.0.0-sources.jar); [Road Runner quickstart PinpointLocalizer.java](https://github.com/acmerobotics/road-runner-quickstart/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/PinpointLocalizer.java)
- What `Follower.update()` does (javap):
  - It reads `System.nanoTime()` and computes `deltaTime` in seconds. **The first call passes 0.0.**
  - It calls `update(double deltaTime)`, which clears the debug log and calls `localizer.update()`.
  - It then switches on mode:
    - `FOLLOW` runs `algorithm.calculatePath(drivetrain, pathTracker, localizer.state(), dt)`, then `drivetrain.drive(powers, false)`. When the path is done it auto-switches to `hold(endPose)` if `holdEnd` is set, otherwise `stop()`.
    - `HOLD` runs `algorithm.calculateHold(...)`, then `drive(powers, false)`.
    - `MANUAL` runs `drivetrain.drive(manualPowers, true)`.
    - `IDLE` runs `drivetrain.stop()`.
  - Finally it calls any loggers registered with `withLogger`.

  `update(double)` is public, so a caller can supply its own dt. — [core-3.0.2.jar (javap -c Follower.update)](file:///home/andrei0204/.gradle/caches/modules-2/files-2.1/com.pedropathing/core/3.0.2/971d7ef36fa68b8499e867efff8e915c2a7bc3e6/core-3.0.2.jar)
- Docs: "The follower must be updated every loop while using manual drive." `follower.pose()` returns x/y in inches and heading in radians. — [pedropathing.com: TeleOp Usage](https://pedropathing.com/docs/pathing/guide/teleop-usage)
- Acceleration:
  - Measured acceleration: `grep -i accel` across all core, revhub and tuning classes finds it only in `ForesightConfig` (`maxAccelerationConstraint`, `maxDecelerationConstraint`), `Foresight` and `Drivetrain.interpolateAcceleration(double, double, double)`, which is a default drivetrain-limit helper. None of these is a measured acceleration getter.
  - For comparison, v2's `PoseTracker.getAcceleration()` was an unfiltered difference: `getVelocity().minus(previousVelocity)` scaled by 1/dt.

  — [core-3.0.2.jar (grep/javap)](file:///home/andrei0204/.gradle/caches/modules-2/files-2.1/com.pedropathing/core/3.0.2/971d7ef36fa68b8499e867efff8e915c2a7bc3e6/core-3.0.2.jar); [v2.1.2 PoseTracker.java](https://github.com/Pedro-Pathing/PedroPathing/blob/v2.1.2/core/src/main/java/com/pedropathing/localization/PoseTracker.java)
- `follower.tangentialVelocity()` is a scalar speed along the current path curve, which the Follower uses to decide when a path is finished. It only means something while following a path. — [pedropathing.com: Types of Velocities](https://pedropathing.com/docs/pathing/reference/velocity); [core-3.0.2.jar (javap)](file:///home/andrei0204/.gradle/caches/modules-2/files-2.1/com.pedropathing/core/3.0.2/971d7ef36fa68b8499e867efff8e915c2a7bc3e6/core-3.0.2.jar)

### Inferences
- Recommended per-loop read for shoot-on-the-move, after exactly one `follower.update()`:
  ```java
  follower.update();
  MotionState s = follower.localizer.state();   // one consistent snapshot
  Pose p = s.pose();          // p.x(), p.y() [in], p.heading() [rad, 0..2π)
  Velocity v = s.velocity();  // v.vx, v.vy [in/s, field frame], v.omega [rad/s, CCW+]
  Twist t = s.twist();        // t.vx fwd, t.vy left [in/s, robot frame]
  ```
  To get acceleration, keep the previous `Velocity` and a timestamp, compute `(v − vPrev)/dt`, and low-pass filter it (EMA or similar). This matters most with two- or three-wheel odometry, where velocity is already a raw one-loop difference, so differencing it again amplifies noise heavily.
- Do **not** also call `localizer.update()` yourself, because `Follower.update()` already does. A second call per loop would cost an extra Pinpoint I2C read. With two- or three-wheel odometry it would also produce a near-zero `dt` and a noisy velocity spike.
- The first `follower.update()` passes dt = 0 to the algorithm, but the localizer's own velocity does not depend on Follower's dt.
- `Pose.exp(follower.velocity(), latencySeconds)` is a ready-made linear predictor of where the robot will be when the ball leaves the shooter. `pose.exp(follower.twist(), t)` is the arc-based alternative.

### Gaps
- I found no documentation of how Pinpoint, OTOS or OctoQuad firmware filter their velocity estimates beyond the OctoQuad interval comment. The noise level of each source would have to be measured.
- I could not confirm from the vendor that Pinpoint `getVelX/getVelY` are field-frame. This rests on Pedro's and Road Runner's usage, while the SDK Javadoc says "X (forward)".

## Q3. Coordinate system, heading convention, and Pedro-to-FTC conversion

### Takeaway
- **Pedro frame:** origin at the bottom-left field corner, +x to the right, +y up, values in inches, a nominal 144-inch field.
- **Heading:** radians, counter-clockwise positive, 0 along +x. Every `Pose` heading is normalized to **[0, 2π)**.
- **Conversion:** v3 ships **no** Pedro-to-FTC helper (v2's `FTCCoordinates` and `PoseConverter` are gone). The docs give a manual formula, and `PoseFactory` can apply mappings.

### Cited Findings
- Docs (Coordinates):
  - Pedro uses a right-hand system where x increases to the right and y increases up the field, with the origin at the bottom-left corner.
  - Heading is in radians: 0 faces right, π/2 faces up, and counter-clockwise is positive.
  - Pedro "does not include a built-in coordinate conversion method". Its example is `new Pose(y + 72, 72 - x, heading - Math.PI / 2)` built from an FTC `Pose2D` (inches, radians).

  — [pedropathing.com: Coordinates](https://pedropathing.com/docs/pathing/reference/coordinates)
- `new Pose(x, y, heading)` stores `Angle.normalize(heading)`, which is `heading mod 2π`, shifted into [0, 2π). This also applies to `withHeading`, `plus`, `minus` and `compose`. `toString()` prints `(x, y, heading)` in radians. — [core-3.0.2.jar (javap -c Pose, Angle)](file:///home/andrei0204/.gradle/caches/modules-2/files-2.1/com.pedropathing/core/3.0.2/971d7ef36fa68b8499e867efff8e915c2a7bc3e6/core-3.0.2.jar)
- `com.pedropathing.utils.Angle`:
  - `normalize(double angleRadians)` maps to [0, 2π).
  - `normalizeSigned(double)` maps to [−π, π).
  - `error(double current, double target)` = `normalizeSigned(target − current)`.
  - `smallestDifference(double, double)` and `turnDirection(double start, double end)` (returns ±1) are also available.

  Angles are radians throughout the core library. — [core-3.0.2.jar (javap -c -l Angle)](file:///home/andrei0204/.gradle/caches/modules-2/files-2.1/com.pedropathing/core/3.0.2/971d7ef36fa68b8499e867efff8e915c2a7bc3e6/core-3.0.2.jar)
- Robot body frame: `DrivePowers(forward, strafe, turn)`. Mecanum mixes them as `fl = f − s − t`, `fr = f + s + t`, `bl = f + s − t`, `br = f − s + t`, so +strafe is left and +turn is counter-clockwise. This matches the `Twist` convention (x forward, y left) and the CCW-positive heading. — [revhub sources Mecanum.java](file:///home/andrei0204/.gradle/caches/modules-2/files-2.1/com.pedropathing/revhub/3.0.2/764a1913a7175783a47cf7832b0c1aaab72ccfaf/revhub-3.0.2-sources.jar); [pedropathing.com: Types of Velocities](https://pedropathing.com/docs/pathing/reference/velocity)
- The hardware localizers pass device coordinates straight into `Pose`.
  - Pinpoint: `getPosX`, `getPosY` and heading in radians.
  - OTOS: heading is shifted by π/2 (`pose2D.h − π/2` on read, `+ π/2` on write), and the sensor offset heading is set to `offset.heading() + π/2`.

  Pedro's frame is therefore whatever frame you seed with `setPose`. The field origin and orientation are a convention set by your starting pose. — [revhub sources PinpointLocalizer.java / OTOSLocalizer.java](file:///home/andrei0204/.gradle/caches/modules-2/files-2.1/com.pedropathing/revhub/3.0.2/764a1913a7175783a47cf7832b0c1aaab72ccfaf/revhub-3.0.2-sources.jar)
- `com.pedropathing.api.PoseFactory` is the v3 mapping tool. Construct it with `PoseFactory.degrees()`, `PoseFactory.radians()` or `new PoseFactory(Operation, Angle.Unit)`, then call `of(x, y, heading)`. Its operations are `map(Operation)`, `mirrorX(axis)`, `mirrorY(axis)`, `mirrorAroundPoint(cx, cy)`, `rotateAround(cx, cy, angle)`, `mapX/mapY/mapHeading(DoubleUnaryOperator)`. Per the v3.0.1 notes, PoseFactory "use[s] explicit angle units". — [core-3.0.2.jar (javap -l PoseFactory)](file:///home/andrei0204/.gradle/caches/modules-2/files-2.1/com.pedropathing/core/3.0.2/971d7ef36fa68b8499e867efff8e915c2a7bc3e6/core-3.0.2.jar); [v3.0.1 release](https://github.com/Pedro-Pathing/PedroPathing/releases/tag/v3.0.1)
- v2 by contrast:
  - It shipped `com.pedropathing.ftc.FTCCoordinates`, `InvertedFTCCoordinates`, `PoseConverter.pose2DToPose(Pose2D, CoordinateSystem)` and `poseToPose2D(Pose, CoordinateSystem)`.
  - `FTCCoordinates.convertToPedro` = `rotate(−π/2)` then `+ (72, 72)`, the same as the v3 docs formula. `convertFromPedro` = `minus(72, 72)` then `rotate(−π/2)`.
  - v2.0.6 changed `pose.mirror()` to use a 141.5-inch field length instead of 144.

  — [v2.1.2 FTCCoordinates.java](https://github.com/Pedro-Pathing/PedroPathing/blob/v2.1.2/ftc/src/main/java/com/pedropathing/ftc/FTCCoordinates.java); [v2.1.2 PoseConverter.java](https://github.com/Pedro-Pathing/PedroPathing/blob/v2.1.2/ftc/src/main/java/com/pedropathing/ftc/PoseConverter.java); [v2.0.6 release](https://github.com/Pedro-Pathing/PedroPathing/releases/tag/v2.0.6)

### Inferences
- The exact inverse of the docs' FTC-to-Pedro formula, derived algebraically, is:
  - x_ftc = 72 − y_pedro
  - y_ftc = x_pedro − 72
  - heading_ftc = heading_pedro + π/2

  Do not copy v2's `convertFromPedro`. Applying `rotate(−π/2)` in both directions composes to a 180° rotation, so v2.1.2's from-Pedro does not appear to be the true inverse of its to-Pedro.
- Pose headings come back in [0, 2π). Before using any heading difference in a turret controller, wrap it with `Angle.normalizeSigned(...)` or `Angle.error(current, target)`. `pose.minus(other).heading()` is not signed.
- AprilTag poses from the FTC SDK (FTC center-origin frame) need the conversion above before `setPose`. Which axis the FTC frame's +X points along this season is outside this note's scope.

### Gaps
- The v3 docs never state the field length (144 vs 141.5 in). I found no v3 constant for field size in the jar.
- I did not verify the FTC SDK's own field-frame axis orientation for the 2026-27 season.

## Q4. Resetting or setting the pose, for example from AprilTag relocalization

### Takeaway
- **Hard reset:** `follower.setPose(Pose)`, plus the `setX`, `setY` and `setHeading` variants, overwrites the localizer pose immediately and keeps the current velocity.
- **Fused correction:** for latency-compensated soft corrections, wrap the hardware localizer in `com.pedropathing.localization.FusionLocalizer` (a Kalman filter with a history buffer) and feed vision with `addMeasurement(Pose, long timestampNanos)`.

### Cited Findings
- The `Follower` setters `setPose(Pose pose)`, `setX(double)`, `setY(double)` and `setHeading(double)` each forward to the localizer. The `Localizer` defaults for `setX/Y/Heading` call `setPose(pose().withX/withY/withHeading(...))`. — [core-3.0.2.jar (javap -c)](file:///home/andrei0204/.gradle/caches/modules-2/files-2.1/com.pedropathing/core/3.0.2/971d7ef36fa68b8499e867efff8e915c2a7bc3e6/core-3.0.2.jar)
- `setPose` for each localizer (source):
  - **Pinpoint:** `pinpoint.setPosition(new Pose2D(globalDistanceUnit, x, y, AngleUnit.RADIANS, heading))`, then `motionState = motionState.withPose(pose)`, which keeps the last velocity.
  - **OTOS:** `otos.setPosition(x, y, heading + π/2)`.
  - **OctoQuad:** `setLocalizerPose((int) mm, (int) mm, (float) heading)`. The integer millimetres truncate the position to 1 mm.
  - **Two/ThreeWheel:** store the pose and call `Encoder.reset()` on every pod. `Encoder.reset()` sets `STOP_AND_RESET_ENCODER` and then `RUN_WITHOUT_ENCODER` on the `DcMotorEx` the pod is plugged into.

  — [revhub sources](file:///home/andrei0204/.gradle/caches/modules-2/files-2.1/com.pedropathing/revhub/3.0.2/764a1913a7175783a47cf7832b0c1aaab72ccfaf/revhub-3.0.2-sources.jar)
- `Localizer.reset()` behaves differently per localizer (source):
  - **Pinpoint:** with the default mode it recalibrates the IMU and sleeps 500 ms.
  - **Two/ThreeWheel:** resets the encoders and sets the pose to zero.
  - **OTOS:** calls `resetTracking()`.
  - **OctoQuad:** calls `resetLocalizerAndCalibrateIMU()`.

  — [revhub sources](file:///home/andrei0204/.gradle/caches/modules-2/files-2.1/com.pedropathing/revhub/3.0.2/764a1913a7175783a47cf7832b0c1aaab72ccfaf/revhub-3.0.2-sources.jar)
- Docs relocalization example: `Pose cornerPose = new Pose(10.5, 10.5, Math.toRadians(90)); follower.setPose(cornerPose);`. Carrying the autonomous end pose into teleop: `follower.setPose(OpModeStorage.autonomousEndPose); follower.update();`, where `OpModeStorage` is user code. — [pedropathing.com: TeleOp Usage](https://pedropathing.com/docs/pathing/guide/teleop-usage)
- `FusionLocalizer implements Localizer` (javap with parameter names). The constructor is `FusionLocalizer(Localizer deadReckoning, Pose initialCovariance, Pose processVariance, Pose measurementVariance, int bufferSize)`. The three `Pose` arguments are used as diagonal variances (x, y, heading), each clamped below by `public static double EPSILON`. Its methods:
  - `update()` calls `deadReckoning.update()` and stores a history entry keyed by `System.nanoTime()`.
  - `addMeasurement(Pose measuredPose, long timestamp)` and `addMeasurement(Pose measuredPose, long timestamp, Pose measurementVariance)`: the measurement is **silently ignored** if `timestamp` falls before the oldest or after the newest buffered entry. Otherwise the filter interpolates the state at that time and applies a Kalman correction; the heading innovation is wrapped with `normalizeSigned`.
  - `setPose(Pose)` hard-resets: it calls `deadReckoning.setPose` and clears the history.
  - `setStartPose(Pose)` and `correctPose(Pose correctedPose)`, which shifts the whole history by a rigid transform.
  - `reset()`.

  — [core-3.0.2.jar (javap -c -l FusionLocalizer)](file:///home/andrei0204/.gradle/caches/modules-2/files-2.1/com.pedropathing/core/3.0.2/971d7ef36fa68b8499e867efff8e915c2a7bc3e6/core-3.0.2.jar)
- `FusionLocalizer.update()` publishes `MotionState.ofVelocity(fusedPose, deadReckoning.velocity())`. The velocity is the raw localizer's field-frame `Velocity`, unchanged; it is **not** rotated into the corrected frame. — [core-3.0.2.jar (javap -c FusionLocalizer.update)](file:///home/andrei0204/.gradle/caches/modules-2/files-2.1/com.pedropathing/core/3.0.2/971d7ef36fa68b8499e867efff8e915c2a7bc3e6/core-3.0.2.jar)
- The v3.0.2 release notes (2026-10-16) include "Fixed FusionLocalizer", "Fixed octoquad offsets" and "Add vector and matrix and twist and velocity api". — [PedroPathing v3.0.2 release](https://github.com/Pedro-Pathing/PedroPathing/releases/tag/v3.0.2)

### Inferences
- **Simplest relocalization:**
  1. Convert the AprilTag robot pose to Pedro coordinates.
  2. Call `follower.setPose(p)`, ideally when the robot is nearly stationary or the tag pose is fresh. Camera latency is not compensated.
  3. Velocity is preserved, so turret aiming is not disrupted.
- **With Two/ThreeWheel odometry,** `setPose` stops and resets the encoders on the motor ports the pods are plugged into. If those ports are also drive motors, this may briefly interrupt their power and costs extra hardware transactions. Avoid frequent mid-drive `setPose` calls with these localizers.
- **For continuous vision fusion while moving,** construct `new FusionLocalizer(new PinpointLocalizer(h, cfg), P0, Q, R, bufferSize)` and pass it to `new Follower(...)`. Then call `((FusionLocalizer) follower.localizer).addMeasurement(tagPosePedro, frameCaptureNanoTime)`. The timestamp must be on the `System.nanoTime()` clock, and `bufferSize` loops must cover the camera latency, or the measurement is dropped. Variances are in in² and rad² for the diagonal.
- **Heading offset caveat:** after a fusion heading correction, the published field-frame `velocity()` is still in the raw odometry frame. The robot-frame `twist()` is computed using the fused heading. A large heading correction therefore introduces a small rotation error in both. Re-rotating the raw velocity yourself avoids this.

### Gaps
- I found no official docs page for `FusionLocalizer`: the docs sidebar lists no localization or fusion page. Recommended covariance values and buffer size are unknown.
- There is no documentation on whether `addMeasurement` expects the timestamp in `System.nanoTime()` units, although the history keys are `System.nanoTime()` in the bytecode.

## Q5. TeleOp driving in v3 while the turret auto-aims

### Takeaway
`startTeleopDrive()` and `setTeleOpDrive(...)` are gone. In v3 you call `follower.manual(forward, lateral, turn)` or `follower.manual(DrivePowers)` every loop, then `follower.update()`. `ManualDrive` provides static helpers for field-centric driving, heading lock, and an auto-hold when the sticks are idle.

### Cited Findings
- `Follower`:
  - `manual(DrivePowers powers)` and `manual(double forward, double lateral, double heading)` both set `Mode.MANUAL`. The third parameter is named `heading` in the bytecode but is used as the **turn power**, via `new DrivePowers(forward, lateral, heading)`.
  - In `MANUAL` mode, `update()` calls `drivetrain.drive(manualPowers, true)`. For `Mecanum` this selects BRAKE zero-power behaviour when `manualBrakeMode` is true; path following uses FLOAT.
  - `stop()` sets `Mode.IDLE`, and every later `update()` then calls `drivetrain.stop()`.
  - `hold(Pose)` and `hold(Pose, boolean useScaling)` set `Mode.HOLD`.
  - `follow(Path)` sets `FOLLOW`.

  — [core-3.0.2.jar (javap -c -l Follower)](file:///home/andrei0204/.gradle/caches/modules-2/files-2.1/com.pedropathing/core/3.0.2/971d7ef36fa68b8499e867efff8e915c2a7bc3e6/core-3.0.2.jar); [revhub sources Mecanum.java](file:///home/andrei0204/.gradle/caches/modules-2/files-2.1/com.pedropathing/revhub/3.0.2/764a1913a7175783a47cf7832b0c1aaab72ccfaf/revhub-3.0.2-sources.jar)
- `ManualDrive` (static, javap -l):
  - `fieldCentric(DrivePowers powers, double currentHeading[, double offsetHeading])` and `fieldCentric(double forward, double lateral, double turn, double currentHeading[, double offsetHeading])`. These rotate (forward, lateral) by −(currentHeading + offsetHeading) and leave turn unchanged.
  - `headingLock(Follower, Controller|PIDController, DrivePowers, double targetHeading)`. This replaces turn with `controller.calculate(target, normalizeSigned(target − heading), twist.omega)`; the controller uses the second argument as the error.
  - `headingLock(Follower, Controller headingFeedback, DrivePowers, double targetHeading, double headingLinear, double headingQuadratic[, double strength = 0.5])`. This variant uses predicted braking: lead = `headingLinear·ω + headingQuadratic·ω²·sign(ω)`, with the output clamped to [−0.3, 1] and multiplied by strength.
  - `driveOrHold(Follower, DrivePowers[, double inputThreshold = 0.1, double velocityThreshold = 0.2])` and `driveOrHold(Follower, forward, lateral, turn[, thresholds])`. If the follower is in manual or idle, the sticks are idle, and `|velocity| < velocityThreshold`, it calls `hold(pose())`. Otherwise, when there is input or the mode is already manual, it calls `manual(powers)`.

  — [core-3.0.2.jar (javap -c -l ManualDrive, PIDController)](file:///home/andrei0204/.gradle/caches/modules-2/files-2.1/com.pedropathing/core/3.0.2/971d7ef36fa68b8499e867efff8e915c2a7bc3e6/core-3.0.2.jar)
- Official teleop snippets:
  - Robot-centric: `follower.manual(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x); follower.update();`
  - Field-centric: `DrivePowers powers = ManualDrive.fieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, follower.pose().heading()); follower.manual(powers); follower.update();`
  - Auto-hold: `ManualDrive.driveOrHold(follower, -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x); follower.update();`

  — [pedropathing.com: TeleOp Usage](https://pedropathing.com/docs/pathing/guide/teleop-usage)
- `driveOrHold()` was added in v3.0.1 ("Add driveOrHold() for auto-holding when joysticks are idle"). — [v3.0.1 release](https://github.com/Pedro-Pathing/PedroPathing/releases/tag/v3.0.1)

### Inferences
- **Sign conventions:** the docs pass `gamepad1.left_stick_x` as lateral and `gamepad1.right_stick_x` as turn, while the Mecanum mixing makes +strafe left and +turn CCW. So with those signs, pushing the stick right would strafe left and turn left (CCW). Teams may need to negate lateral and turn. This should be checked on the robot, because the doc snippet and the verified mixing disagree on intuition.
- **Ordering:** a turret-aim loop can sit alongside driving: (1) set `follower.manual(...)` or `driveOrHold`, (2) `follower.update()`, (3) read `pose()`, `velocity()` and `twist()`, (4) compute the turret command. The pose and velocity read after `update()` are the freshest.
- **Heading lock:** `ManualDrive.headingLock(...)` could lock the chassis toward the goal if the turret is out of range, using the same heading-error machinery.

### Gaps
- The docs do not say whether `follower.manual()` must be called every loop or only once. The bytecode shows it latches `manualPowers` until changed, so calling it every loop with fresh stick values is necessary.

## Q6. How v3 differs from v1/v2 (online docs vs the jar)

### Takeaway
v3 (released 2026-09-10, latest 3.0.2 on 2026-10-16) is described as "a major update and full rewrite". Accessors lost their `get` prefix, velocity became the `Velocity`/`Twist` types, acceleration and coordinate converters were removed, the FollowerBuilder/Constants-builder pattern was replaced by direct construction plus `XConfig` lambdas, and the Maven artifacts became `core`, `revhub` and `tuning`. The online docs match the jar wherever they cover the same API.

### Cited Findings
- **Release history:** v3.0.0 (2026-09-10) introduced Foresight, PoseFactory, a new Path API, AutoTune and logging. v3.0.1 (2026-09-18) and v3.0.2 (2026-10-16) are listed in Q4/Q5. The last v2 was v2.1.2 (2026-05-07). — [GitHub releases](https://github.com/Pedro-Pathing/PedroPathing/releases); [What's new with Pedro 3](https://pedropathing.com/docs/pathing/pedro3)
- **v2.1.2 `Follower` methods (source):**
  - `Pose getPose()`
  - `Vector getVelocity()`
  - `Vector getAcceleration()`
  - `double getAngularVelocity()`
  - `double getHeading()`, `double getTotalHeading()`
  - `void setStartingPose(Pose)`, `void setPose(Pose)`
  - `startTeleopDrive()` / `startTeleopDrive(boolean useBrakeMode)` / `startTeleOpDrive(...)`
  - `setTeleOpDrive(double forward, double strafe, double turn[, boolean isRobotCentric][, double offsetHeading])`
  - `update()`

  The v2 FTC module had `com.pedropathing.ftc.FollowerBuilder`. — [v2.1.2 Follower.java](https://github.com/Pedro-Pathing/PedroPathing/blob/v2.1.2/core/src/main/java/com/pedropathing/follower/Follower.java); [v2.1.2 tree](https://github.com/Pedro-Pathing/PedroPathing/tree/v2.1.2)
- **v3.0.2 jar equivalents (javap):**
  - `getPose()` became `pose()`.
  - `getVelocity()` became `velocity()` (field-frame `Velocity`) and `twist()` (robot-frame `Twist`).
  - `getAngularVelocity()` became `velocity().omega` or `twist().omega`.
  - `getHeading()` became `pose().heading()`.
  - `getAcceleration()`, `getTotalHeading()` and `setStartingPose()` have no Follower equivalent. `ThreeWheelLocalizer` and `ThreeWheelIMULocalizer` still expose `getTotalHeading()` on the localizer class itself.
  - `startTeleopDrive` and `setTeleOpDrive` became `manual(...)` plus `ManualDrive.*`.
  - `followPath(...)` became `follow(Path)`.
  - The constructor became `new Follower(Localizer, Drivetrain, Algorithm)`.
  - Packages `com.pedropathing.geometry.*` and `com.pedropathing.ftc.*` (v2) became `com.pedropathing.math.*` and `com.pedropathing.revhub.*` (v3).

  — [core-3.0.2.jar (javap)](file:///home/andrei0204/.gradle/caches/modules-2/files-2.1/com.pedropathing/core/3.0.2/971d7ef36fa68b8499e867efff8e915c2a7bc3e6/core-3.0.2.jar); [revhub sources](file:///home/andrei0204/.gradle/caches/modules-2/files-2.1/com.pedropathing/revhub/3.0.2/764a1913a7175783a47cf7832b0c1aaab72ccfaf/revhub-3.0.2-sources.jar)
- **Docs vs jar:** the v3 docs pages for TeleOp Usage, Types of Velocities and Coordinates use `follower.pose()`, `follower.velocity()`, `follower.twist()`, `follower.manual(...)`, `ManualDrive.fieldCentric/driveOrHold` and `follower.setPose(...)`, all of which exist in the 3.0.2 jar with matching types. None of the fetched docs pages names a version number, and the "What's new with Pedro 3" page has no API-rename table. — [TeleOp Usage](https://pedropathing.com/docs/pathing/guide/teleop-usage); [Types of Velocities](https://pedropathing.com/docs/pathing/reference/velocity); [Coordinates](https://pedropathing.com/docs/pathing/reference/coordinates); [What's new with Pedro 3](https://pedropathing.com/docs/pathing/pedro3)
- **Third-party summaries:** pages such as skills.sh and context7 still show v2-style usage (e.g. `followPath`). They are not authoritative for v3. — [skills.sh pedro-pathing](https://skills.sh/ncssm-robotics/ftc-claude/pedro-pathing); [context7 summary](https://context7.com/pedro-pathing/pedropathing)

### Inferences
- Code or AI-generated examples using `getPose()`, `getVelocity()`, `getAcceleration()`, `setStartingPose()`, `startTeleopDrive()`, `PoseConverter` or `FTCCoordinates` are v2 and will not compile against 3.0.2.
- The coordinate convention (0–144 in corner origin, CCW radians) carried over from v2 unchanged, based on the docs formula matching v2's `convertToPedro`.

### Gaps
- I did not check Discord changelogs, because they are not reachable without an account. GitHub release notes were used instead.
- I found no official v2-to-v3 migration guide.
