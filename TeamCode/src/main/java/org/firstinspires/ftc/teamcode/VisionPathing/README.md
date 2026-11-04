# VisionPathing

Finds **Pollen** (yellow) and **Nectar** (red / blue) with one webcam, tells you where every ball
is in inches, and can drive to Nectar with Pedro Pathing while steering around Pollen.

| File | What it is |
|---|---|
| `VisionConstants.java` | Every setting, as `public static` fields. `@Configurable`, so you can change them live in Panels. |
| `VisionSubsystem.java` | FTCLib subsystem. Opens the camera, detects the balls, tracks them, and exposes the results through simple getters. **This is the class you use from other code.** |
| `NectarChaser.java` | TeleOp "Nectar Chaser (avoids Pollen)". Plans a Pedro path to the nearest reachable alliance Nectar that keeps the intake away from every Pollen. |

The older single-file OpModes live in `teamcode/Vision/`:

| File | OpMode | What it does |
|---|---|---|
| `blobTrack.java` | Pollen Cluster Tracker | Tracks Pollen and picks the target by group size (`score = ballCount - DIST_PENALTY * distance`). Camera preview and telemetry only; it doesn't drive. |
| `moveToBall.java` | Move To Ball | Turns toward a confirmed Pollen and drives to it with raw motor powers (no Pedro). Holding A stops it. |

Both are standalone `LinearOpMode`s. Neither uses `VisionSubsystem`, and `VisionSubsystem` doesn't use them.

---

## Before you start

1. **Camera.** `VisionSubsystem` opens the USB webcam named `VisionConstants.WEBCAM_NAME`
   (default `"Webcam 1"`, the same as `blobTrack` / `moveToBall`). The active Robot Controller
   configuration needs a webcam with that name. The team's `biobuzz.xml` only has the
   Limelight, so add the webcam to whichever config you run.
2. **Measure the robot.** Set these fields in `VisionConstants`, in inches from the **robot
   center** (the point Pedro reports as the pose):
   - Marked `<- MEASURE`: `LENS_HEIGHT`, `CAM_FORWARD_OFFSET`, `CAM_RIGHT_OFFSET`,
     `ROBOT_WIDTH`, `ROBOT_LENGTH`, `FRONT_EXTENT`.
   - Also check: `INTAKE_WIDTH` (set to ~16 in) and `END_CENTER_TO_BALL`, which is where the
     robot center stops relative to the ball when the intake has it.
3. **Tune the colors.** In Panels set `MASK_VIEW` to 1 (Pollen), 2 (red) or 3 (blue). The preview
   then blacks out every pixel that doesn't match that color. Adjust the `P_*` / `R_*` / `B_*`
   YCrCb ranges until each ball shows as a solid, clean disk and nothing else survives. The red
   and blue ranges are starting guesses and **need** tuning on the real balls.
4. **Calibrate the camera** (tilt and focal length), once per camera mount:
   1. Put **one** ball straight ahead, `CAL_FLOOR_DIST` (default 30 in) along the floor from the
      point under the lens.
   2. Call `vision.startCalibration(BallType.POLLEN)`, or press X (Pollen) / B (Nectar) during
      `NectarChaser` init.
   3. When it finishes, it writes `TILT_DEG` and `FOCAL_PX`. Copy them into `VisionConstants`
      so they survive a restart.

---

## Using `VisionSubsystem` in your code

### 1. Create it and give it the robot pose

```java
DriveSubsystem drive = new DriveSubsystem(hardwareMap, PanelsTelemetry.INSTANCE.getTelemetry());
VisionSubsystem vision = new VisionSubsystem(hardwareMap);   // registers itself with FTCLib
vision.setRobotPoseSupplier(drive::getPose);                // Pedro pose, inches + radians
```

- **Create it after `DriveSubsystem`.** FTCLib runs `periodic()` in registration order, so
  `DriveSubsystem` updates the pose first and vision then uses that fresh pose. If you use the
  `Incredibot` container, create vision after it and pass `incredibot.driveSubsystem::getPose`.
- **Always give it a pose supplier when the robot moves.** With one, balls are tracked in field
  coordinates. A ball stays put while the robot drives and turns, and a ball that slips out of
  view (the last ~11 in in front of the intake) is remembered for `MEMORY_S` seconds while its
  robot-relative position keeps updating from odometry. Without one, everything is in the robot
  frame, which is only correct while the robot sits still.
- Don't create a second Pedro `Follower` just for vision. `DriveSubsystem` already owns the
  drive motors and the Pinpoint.

### 2. Let it update every loop

`CommandScheduler.getInstance().run()` calls `vision.periodic()` for you. If you aren't using
the scheduler, call `vision.periodic()` yourself once per loop, after the pose has updated.
`periodic()` is cheap when no new camera frame has arrived.

### 3. Read the balls

```java
List<VisionSubsystem.Ball> pollen = vision.getBalls(BallType.POLLEN);  // confirmed, nearest first
VisionSubsystem.Ball red          = vision.getNearest(BallType.RED_NECTAR); // null if none
List<VisionSubsystem.Ball> mine   = vision.getNectar(0);                // 0 red, 1 blue, 2 either

if (red != null) {
    double inches  = red.distance();     // floor distance from robot center
    double degrees = red.bearingDeg();   // off the robot's nose, + = to the RIGHT
    double right   = red.x;              // robot frame: inches right of center
    double forward = red.y;              // robot frame: inches forward of center
    double fx = red.fieldX, fy = red.fieldY;   // field frame (Pedro), inches
    Pose target = red.fieldPose();       // same point as a Pedro Pose (heading 0)
    boolean seen = red.isVisibleNow();   // false = remembered from odometry, not in view right now
}
```

**All the getters**

| Method | Returns |
|---|---|
| `getBalls()` | Confirmed balls of every type, nearest first |
| `getBalls(type)` | Confirmed balls of one type, nearest first |
| `getBalls(type, true)` | Same, plus balls seen only once or twice (use these as obstacles, not targets) |
| `getNearest(type)` | Nearest confirmed ball of that type, or `null` |
| `getNectar(alliance)` | Confirmed Nectar for `0` red / `1` blue / `2` either, nearest first |
| `getBall(id)` | The ball with that id (confirmed or not), or `null` once it is no longer tracked |
| `getDetections()` | Every raw blob in the latest frame, with why it was kept or dropped (`status`) |
| `VisionSubsystem.isAllianceNectar(type, alliance)` | Whether a ball type is the Nectar color an alliance wants |

**What's on a `Ball`** (an immutable snapshot, so it's safe to keep around)

| Field / method | Meaning |
|---|---|
| `id` | Stays the same as long as the ball stays tracked. Use it to follow one ball across loops. |
| `type` | `POLLEN`, `RED_NECTAR` or `BLUE_NECTAR`. `type.radius()` is the real radius in inches, and `type.isNectar()` tells you whether it's Nectar. |
| `x`, `y` | Robot frame, inches: `x` = right of robot center, `y` = forward of robot center |
| `fieldX`, `fieldY` | Field frame, inches: the same frame as your Pedro pose |
| `distance()` | `hypot(x, y)`, the floor distance from robot center |
| `bearingDeg()` | Angle off the nose, **+ = right**. Pedro turn power is + = counter-clockwise (left), so negate it when you steer. |
| `sigma` | 1-sigma position uncertainty, inches. About 0.75 in up close, growing with distance. |
| `confirmed`, `hits` | Seen `CONFIRM_HITS` times or more / number of times seen |
| `ageS` | Seconds since the camera actually saw it |
| `isVisibleNow()` | `ageS <= FRESH_S` |

Distances are along the floor to the ball center, using the official ball sizes from the
BIOBUZZ manual (TU07 §9.8): Pollen ≈ 2.8 in, Nectar ≈ 3.6 in (`POLLEN_DIAMETER`,
`NECTAR_DIAMETER`). Each detection also has to match the size a real ball would appear at that
spot. That is how it ignores yellow field parts, two touching balls merged into one blob, and
anything that isn't sitting on the floor.

### 4. Other controls

```java
vision.forget(ball.id);        // e.g. after intaking it. If it's really still there, it comes back.
vision.forgetAll();
vision.highlight(ball.id);     // green ring on the camera preview; -1 = none
vision.isFrameFresh();         // a frame was processed in the last 0.3 s
vision.getFps();
vision.addTelemetry(telemetry);   // compact status block (SDK Telemetry)
vision.close();                // stop the camera when the OpMode ends (put it in a finally block)
```

Calibration: `startCalibration(type)`, `startCalibration(type, floorDist)`,
`cancelCalibration()`, `isCalibrating()`, `getCalibrationStatus()`. While calibrating, no balls
are tracked.

### Example: a read-only OpMode

```java
@TeleOp(name = "Vision Readout", group = "VisionPathing")
public class VisionReadout extends LinearOpMode {
    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();
        DriveSubsystem drive = new DriveSubsystem(hardwareMap, PanelsTelemetry.INSTANCE.getTelemetry());
        VisionSubsystem vision = new VisionSubsystem(hardwareMap);   // after drive
        vision.setRobotPoseSupplier(drive::getPose);

        waitForStart();
        try {
            while (opModeIsActive()) {
                CommandScheduler.getInstance().run();   // drive.periodic(), then vision.periodic()

                VisionSubsystem.Ball nectar = vision.getNearest(BallType.RED_NECTAR);
                if (nectar != null) {
                    telemetry.addData("red nectar", "%.1f in, %+.0f deg, field (%.1f, %.1f)",
                            nectar.distance(), nectar.bearingDeg(), nectar.fieldX, nectar.fieldY);
                }
                telemetry.addData("pollen in view", vision.getBalls(BallType.POLLEN).size());
                vision.addTelemetry(telemetry);
                telemetry.update();
            }
        } finally {
            vision.close();
            CommandScheduler.getInstance().reset();
        }
    }
}
```

### Example: a command that turns to face the nearest Nectar

```java
public class TurnToNectarCommand extends CommandBase {
    public static double KP = 0.015;   // turn power per degree
    private final DriveSubsystem drive;
    private final VisionSubsystem vision;
    private final BallType color;

    public TurnToNectarCommand(DriveSubsystem drive, VisionSubsystem vision, BallType color) {
        this.drive = drive;
        this.vision = vision;
        this.color = color;
        addRequirements(drive);        // NOT vision: it's only being read
    }

    @Override
    public void execute() {
        VisionSubsystem.Ball b = vision.getNearest(color);
        if (b == null) { drive.drive(0, 0, 0); return; }
        // bearingDeg is + to the right; Pedro turn is + counter-clockwise, so negate.
        drive.drive(0, 0, Range.clip(-KP * b.bearingDeg(), -0.4, 0.4));
    }

    @Override
    public boolean isFinished() {
        VisionSubsystem.Ball b = vision.getNearest(color);
        return b != null && Math.abs(b.bearingDeg()) < 3;
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0);
    }
}
```

---

## `NectarChaser` (driving to Nectar around Pollen)

**Why it avoids Pollen.** G407 says a robot may not control more than 4 scoring elements, and
the manual counts pushing a ball (herding) as control. So the robot must neither intake Pollen
nor plow it out of the way.

**How it works:**

1. **Search.** Waits until a confirmed Nectar of the `TARGET_ALLIANCE` color is tracked.
2. **Face.** Turns toward it, so the camera sees the ball and the Pollen around it.
3. **Plan.** Tries approach directions: straight-in first, then ±15°, ±30°, and so on.
   - Every approach ends with a straight drive-in (`FINAL_APPROACH_LEN`) at reduced speed
     (`FINAL_LEG_SPEED`), so the intake meets the ball head-on.
   - The trip to the start of that drive-in is a smooth curve. If Pollen is in the way, it adds
     up to `MAX_DETOURS` waypoints beside the blocking ball.
   - Every candidate path is checked against a corridor as wide as the intake/chassis plus
     `POLLEN_CLEARANCE`, including the area in front of the intake where it stops. The cheapest
     clear path wins. If none is clear, the ball is skipped.
4. **Drive.** Follows the path and turns the intake on for the last `INTAKE_ON_WITHIN` inches.
   It re-plans if new Pollen appears in the way or the ball's estimate moves. The final inches,
   where the camera can no longer see the ball, are driven on odometry.
5. **Collect.** Dwells with the intake running for `COLLECT_DWELL_S`, forgets the ball, and goes
   back to step 1.

The other alliance's Nectar is also avoided when `AVOID_OTHER_NECTAR` is on.

**Controls:**
- During init: X calibrates on one Pollen, and B calibrates on one alliance-color Nectar.
- While running: holding A stops the robot and lets the sticks drive it.

**Caveats:**
- It builds its **own** Pedro follower from `pedro/Constants.create()`, so it's a standalone
  test OpMode. Don't run it together with `DriveSubsystem`.
- It only runs `intakeMotor`. The team's `IntakeSubsystem` also uses `intakeServo1/2` and
  `transferMotor`, so drive those too if the intake needs them to hold a ball.
- There's no ball check after collecting. If the ball is still there, vision sees it again and
  the robot goes back for it.

The path planner (`Planner.plan(pose, ballX, ballY, obstacles)` → `Planner.buildPath(plan)`)
is pure geometry with no hardware in it. Its classes are package-private, so code in this
package can reuse them (e.g. a VisionPathing autonomous). Make `Planner`, `Plan` and
`Obstacle` public if another package needs them.

---

## `VisionConstants` at a glance

| Group | Main fields |
|---|---|
| Ball sizes | `POLLEN_DIAMETER` 2.8, `NECTAR_DIAMETER` 3.6 |
| Camera | `WEBCAM_NAME`, `LENS_HEIGHT`, `CAM_FORWARD_OFFSET`, `CAM_RIGHT_OFFSET`, `CAM_YAW_DEG`, `TILT_DEG`, `FOCAL_PX`, `EXPOSURE_MS`, `GAIN` |
| Calibration | `CAL_FLOOR_DIST`, `CAL_SAMPLES`, `AUTO_CALIBRATE` (off: calibration only runs when asked) |
| Colors | `DETECT_POLLEN/RED/BLUE`, YCrCb ranges `P_*`, `R_*`, `B_*`, `RED_EXCLUDES_YELLOW`, `MASK_VIEW` |
| Detection | `MIN_RADIUS_PX`, `MAX_BALLS_PER_COLOR`, `SIZE_MIN/MAX`, `MAX_DETECT_DIST` |
| Tracking | `CONFIRM_HITS`, `MAX_VISIBLE_MISSES`, `MEMORY_S`, `FRESH_S`, `GATE_*` |
| Robot | `INTAKE_WIDTH` (16), `ROBOT_WIDTH`, `ROBOT_LENGTH`, `FRONT_EXTENT`, `END_CENTER_TO_BALL` |
| Planning | `POLLEN_CLEARANCE`, `AVOID_OTHER_NECTAR`, `FINAL_APPROACH_LEN`, `FINAL_LEG_SPEED`, `MAX_DETOURS`, `USE_FIELD_BOUNDS` |
| Chase | `TARGET_ALLIANCE` (0 red, 1 blue, 2 either), `MAX_TARGET_DIST`, `CHASE_TIMEOUT_S`, `MAX_TO_COLLECT` |
| Intake / start | `INTAKE_MOTOR` (`"intakeMotor"`), `INTAKE_POWER`, `INTAKE_ON_WITHIN`, `START_X/Y/HEADING_DEG` |

Frames used everywhere:
- **Robot frame:** x = right, y = forward, inches from robot center.
- **Field frame:** Pedro's frame, inches, heading counter-clockwise in radians.

Leave `USE_FIELD_BOUNDS` off until `START_X/Y/HEADING_DEG` match where the robot really starts.
