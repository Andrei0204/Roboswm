# Autonomous: block dictionary

Files (all under `src/main/java/org/firstinspires/ftc/teamcode/`):

| File | What it is |
|---|---|
| `auto/AutoBlocks.java` | The dictionary: every block you can use, one method each |
| `auto/TemplateAuto.java` | Working example OpMode. Copy it, rename it, edit `buildRoutine` |
| `docs/auto.md` | This file |

AUTO is built with **Ivy** (the command library that ships with Pedro, already in
`build.dependencies.gradle`). A *block* is a `Command`. You glue blocks into a *sequence*, schedule
it once, and call `auto.run()` every loop. Blocks do nothing until the sequence reaches them.

## How an AUTO is laid out

```java
@Autonomous(name = "My Auto", group = "BioBuzz", preselectTeleOp = "Shoot On The Move")
public class MyAuto extends LinearOpMode {
    @Override public void runOpMode() {
        RobotSys robot = new RobotSys(hardwareMap, Alliance.RED);

        // 1. INIT loop: pick alliance, set the start pose, home the turret (see TemplateAuto)

        // 2. After INIT:
        AutoBlocks auto = new AutoBlocks(robot);
        Command routine = buildRoutine(auto);   // your sequence
        auto.start(routine);                     // schedule it ONCE

        // 3. Loop
        while (opModeIsActive()) {
            auto.run();                          // commands -> robot.update() -> robot.shoot()
        }
        robot.stop();
    }
}
```

Rules:

- `auto.run()` once per loop. It replaces `robot.update()` and `robot.shoot(...)`; do not call those yourself.
- Create the `AutoBlocks` **after** INIT (it resets the scheduler and remembers the alliance).
- `robot.update()` saves the pose, alliance and up CELL, so TeleOp starts where AUTO ended
  (`ShootOnTheMoveTeleOp` restores it). Set `preselectTeleOp` to its name.
- The turret zeroes its encoder when an OpMode ends: start with the turret straight forward, or press gamepad1 Y in INIT.
- Nothing moves in INIT except Y (home turret) and A (servos to start).

## Poses and coordinates

Pedro frame, inches, heading in degrees at the API (`pose(x, y, headingDeg)`): origin at the red wall / audience corner,
+x toward the blue wall, +y toward the rear wall, heading 0 = +x.

**Write every pose for RED.** `a.pose(...)` rotates it 180 degrees about the field center for BLUE
(the field is rotationally symmetric, not mirrored). So one routine serves both alliances.

| Block | Returns | Meaning |
|---|---|---|
| `a.pose(x, y, deg)` | `Pose` | A field pose written for RED, auto-rotated for BLUE |
| `AutoBlocks.pose(alliance, x, y, deg)` | `Pose` | Same but static, for INIT before `AutoBlocks` exists |

The numbers in `TemplateAuto` are **placeholders (MEASURE)**.

## Paths (build them once, then `follow` them)

A `Path` is geometry only; it does not move the robot. Paths start where you say, not where the robot is,
so chain them from the pose the previous one ended at.

| Block | Meaning |
|---|---|
| `a.line(from, to)` | Straight line, heading turns smoothly from `from`'s heading to `to`'s |
| `a.lineConstantHeading(from, to)` | Straight line, heading fixed at `to`'s heading (good for sweeping across balls) |
| `a.curve(p0, p1, ..., pn)` | Bezier curve. First = start, last = end, middle = control points (they pull the curve, the robot does not pass through them). Heading start to end |
| `a.curveTangent(p0, ..., pn)` | Bezier curve, robot faces the direction it travels |

## Driving blocks

| Block | Done when | Meaning |
|---|---|---|
| `a.follow(path)` | follower reaches the path end | Follow a path |
| `a.follow(path, timeoutMs)` | end reached **or** timeout | Same, never gets stuck |
| `a.driveTo(pose)` | end reached | Straight line from wherever the robot is **when the block runs** (needs no start pose) |
| `a.driveTo(pose, timeoutMs)` | end reached or timeout | Same with timeout |
| `a.hold()` | instantly | Hold the current position (brakes/holds against pushes until the next follow) |
| `a.hold(pose)` | instantly | Hold a given pose |
| `a.setPose(pose)` | instantly | Overwrite the position estimate, e.g. after a wall bump |

Always give `follow` / `driveTo` a timeout in a real match: a blocked robot otherwise stalls the whole routine.

## Shooter blocks

The shooter aims itself (turret, flywheel rpm, hood come from the shot solver for the up CELL). You only
arm it and say when to feed.

| Block | Done when | Meaning |
|---|---|---|
| `a.arm()` | instantly | Spin the flywheel to the shot rpm. Do it early (flywheel needs seconds) |
| `a.disarm()` | instantly | Flywheel back to idle rpm |
| `a.waitReady(timeoutMs)` | all checks pass **or** timeout | Waits for flywheel, hood, turret aim and shot checks. Does not fire |
| `a.shoot(ms)` | after `ms` | Hold fire for `ms`: feeds **only while every check passes** (normal, safe) |
| `a.forceShoot(ms)` | after `ms` | Hold FORCE fire: feeds regardless of every check. Last resort |
| `a.shootBalls(n, timeoutMs)` | `n` feed pulses done **or** timeout | Fire a counted number of balls. Only counts when `ShooterConstants.FEEDER_PULSE_SEC > 0`; with 0 (currently) use `shoot(ms)` |
| `a.nextBall(BallType)` | instantly | Ball type for the next shot (only matters if `SHOOTS_NECTAR`) |
| `a.upCell(HiveCell)` | instantly | Declare which HIVE CELL is up |
| `a.hiveTipped()` | instantly | Report that our HIVE tipped (switches CELL, pauses shooting briefly) |
| `a.aimTrimDeg(deg)` | instantly | Add to the aim trim (positive = counter-clockwise) |
| `a.distanceTrimIn(in)` | instantly | Add to the distance trim (positive = farther) |
| `a.clearTrims()` | instantly | Reset both trims |

Typical shot: `a.arm()` (early) ... `a.waitReady(1500)` then `a.shoot(3000)`. Expect the first HIVE TIP after
about 3 POLLEN (see README).

## Intake blocks

| Block | Done when | Meaning |
|---|---|---|
| `a.intakeOn()` | instantly | Intake motor on until `intakeOff` (it also runs by itself during feed pulses) |
| `a.intakeOff()` | instantly | Intake motor off |
| `a.intakeReverse(ms)` | after `ms` | Run backwards for `ms` to clear a jam, then stop reversing |
| `a.intakeMode(mode)` | instantly | Intake servo: `IntakeServo.Mode.INIT`, `COLLECT_ALL` or `COLLECT_POLLEN` |
| `a.collect(mode)` | instantly | Servo to `mode` **and** motor on |
| `a.stopCollecting()` | instantly | Motor off and servo back to `INIT` |

## Timing and logic blocks

| Block | Meaning |
|---|---|
| `a.waitMs(ms)` | Pause |
| `a.waitUntil(() -> condition)` | Pause until the condition is true (no timeout: combine with `firstOf` and `waitMs`) |
| `a.run(() -> { ... })` | Run your own code once (set a flag, add telemetry, move something custom) |
| `a.either(cond, ifTrue, ifFalse)` | Choose a block when the sequence reaches it (condition is read **then**, not when built) |
| `a.stopAll()` | Flywheel, feeder, gate and intake off. Last block of the routine |

## Groups (the "sequences")

These combine blocks. They nest freely. In the example they are imported with
`import static ...auto.AutoBlocks.sequence;` and so on.

| Group | Done when | Meaning |
|---|---|---|
| `sequence(a, b, c)` | c is done | One after the other. The top of every routine |
| `together(a, b, c)` | all are done | Run at the same time |
| `firstOf(a, b)` | the first one is done | Run together; cancel the rest when one finishes (use for timeouts) |
| `whileDoing(leader, x, y)` | `leader` is done | Run `x`, `y` alongside `leader`; cancel them when it ends |
| `repeat(block, n)` | `n` times | Repeat a block |

You can also chain on any block: `blockA.then(blockB)` (sequence), `blockA.with(blockB)` (together),
`blockA.until(() -> cond)`, `blockA.unless(() -> cond)`.

### Patterns

```java
// Arm while driving
together(a.arm(), a.follow(toShoot, 4000))

// Drive over the balls with the intake running; intake stops with the drive
whileDoing(a.follow(sweep, 3000), a.collect(IntakeServo.Mode.COLLECT_ALL))

// Never wait forever for ready
a.waitReady(1500), a.shoot(3000)

// A whole cycle repeated twice
repeat(sequence(a.follow(toPickup, 4000), a.follow(sweepBack, 3000), a.shoot(2500)), 2)

// Choose at run time
a.either(() -> robot.isReady(), a.shoot(2000), a.forceShoot(2000))
```

## Example routine (from `TemplateAuto`)

```java
return sequence(
    together(a.arm(), a.follow(toShoot, 4000)),        // 1 spin up + drive
    a.waitReady(1500), a.shoot(3000),                    // 2 shoot preload
    a.follow(toPickup, 4000),                            // 3 go to balls
    whileDoing(a.follow(sweep, 3000), a.collect(IntakeServo.Mode.COLLECT_ALL)),
    a.stopCollecting(),
    a.follow(backToShoot, 4000),                         // 4 back, shoot again
    a.waitReady(1500), a.shoot(3000),
    a.disarm(),                                          // 5 park
    a.driveTo(park, 4000), a.hold(), a.stopAll());
```

## Adding your own block

Anything that is not in the list: wrap it with `a.run(() -> ...)` for one-shot actions, or add a method to
`AutoBlocks`:

```java
public Command myThing() {
    return Commands.instant(() -> robot.gate().update(true));     // one-shot
}
// For something that takes time, use Command.build():
Command.build().setStart(...).setExecute(...).setDone(() -> condition).setEnd(e -> ...);
```

`setExecute` runs every loop, `setDone` ends the block, `setEnd` runs when it ends or is cancelled (use it to
turn things off, as `shoot` does for the fire flag).

## Checklist before running AUTO

1. Pedro tuned, field frame checked (see `TUNING.md`); start pose measured and the robot placed there.
2. Turret straight forward (or gamepad1 Y in INIT), servos to start (gamepad1 A).
3. Every `follow` / `driveTo` has a timeout; the routine ends with `stopAll()`.
4. Total time under 30 s: add up the timeouts.
5. Run it on both alliances: BLUE is the 180-degree rotation, so check it on the field once.

## Quick lookup

`pose` `line` `lineConstantHeading` `curve` `curveTangent` `follow` `driveTo` `hold` `setPose` `arm` `disarm`
`waitReady` `shoot` `forceShoot` `shootBalls` `nextBall` `upCell` `hiveTipped` `aimTrimDeg` `distanceTrimIn`
`clearTrims` `intakeOn` `intakeOff` `intakeReverse` `intakeMode` `collect` `stopCollecting` `waitMs` `waitUntil`
`run` `either` `stopAll` | groups: `sequence` `together` `firstOf` `whileDoing` `repeat`
