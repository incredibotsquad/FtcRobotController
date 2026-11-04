# Vision (Pollen)

Webcam vision that finds **Pollen** (yellow, 2.8 in balls), works out where each one is in inches
from the robot, groups balls that are touching, and picks the best group to go after.

| File | Kind | What it does |
|---|---|---|
| `blobTrack.java` | TeleOp **Pollen Cluster Tracker** | The original pipeline, plus a TeleOp that shows what it sees: camera preview, every tracked ball, every group, and the chosen target. It doesn't drive. |
| `moveToBall.java` | TeleOp **Move To Ball** | Uses `blobTrack`'s pipeline to chase the best group with two **P controllers** (turn and drive), on plain motor powers. No odometry. |
| `PollenTrackerSubsystem.java` | FTCLib subsystem | The same pipeline as a subsystem. It gives other code robot-relative **and field** positions, and it also finds red/blue **Nectar**. **Use this from new code.** |
| `PollenPedroExample.java` | TeleOp **Pollen Pedro Example** | Reads `PollenTrackerSubsystem` and drives to Pollen with **Pedro Pathing**. |

`blobTrack` / `moveToBall` and `PollenTrackerSubsystem` each have their own copy of the vision
settings. The subsystem's defaults are `blobTrack`'s current values, so if you retune one, copy
the numbers to the other.

---

## Setup

1. **Camera.** Everything opens the webcam named **`"Webcam 1"`**. It must be in the active Robot
   Controller configuration.
2. **Drive motors.**
   - `moveToBall` uses `frontLeftMotor`, `backLeftMotor`, `frontRightMotor` and
     `backRightMotor`, with the left side reversed.
   - `PollenPedroExample` drives through the team's `DriveSubsystem` and `pedro/Constants`.
3. **Measure the camera.** Set these in inches from the robot center:
   - `LENS_HEIGHT`: floor to lens center
   - `CAM_FORWARD_OFFSET`: + = lens is in front of center
   - `CAM_RIGHT_OFFSET`: + = lens is right of center
4. **Calibrate.** Calibration finds the camera tilt and focal length.
   1. Put **one** ball straight ahead, `CAL_FLOOR_DIST` (30 in) along the floor from the point
      under the lens.
   2. Start calibration:
      - **Pollen Cluster Tracker** calibrates during init (`AUTO_CALIBRATE = true`), and **A**
        recalibrates.
      - **Pollen Pedro Example**: press **X** during init.
      - Your own code: call `startCalibration()` on the subsystem.
   3. Copy the printed tilt / focal into `TILT_DEG` / `FOCAL_PX` so they survive a restart.
5. **Tune the color.** In Panels, set `SHOW_MASK = true` so the preview shows only pixels that
   pass the color filter. Adjust `Y_*`, `CR_*` and `CB_*` (YCrCb) until each ball is a solid disk
   and nothing else survives. `CB_MAX` is the main knob. In the subsystem, `MASK_COLOR` picks
   which color is shown: `"P"`, `"R"` or `"B"`.

The color ranges were tuned with manual exposure (`EXPOSURE_MS = 10`, `GAIN = 100`). Everything
here sets that exposure itself.

---

## How the pipeline works

```
camera frame
  -> color mask -> cleanup -> distance transform -> one circle per ball (pixels)   camera thread
  -> project onto the floor (inches) + size check                                  your loop
  -> tracker: match balls frame to frame, smooth, confirm after CONFIRM_HITS
  -> groups: chain balls within CLUSTER_RADIUS, score = ballCount - DIST_PENALTY * distance
```

- **Size check.** A real ball at that floor position should appear a known size. If the
  measured / expected radius ratio is outside `SIZE_MIN`..`SIZE_MAX`, the blob is thrown out.
  That gets rid of yellow non-balls and two balls merged into one blob.
- **Group score.** With the default `DIST_PENALTY` of 0.03, one extra ball is worth about 33 in of
  extra driving. The aim point is the group's centroid when `AIM_AT_CENTROID` is on, or its
  nearest ball when it's off.
- **Robot frame:** inches from the robot center, `x` + right, `y` + forward. Heading to anything
  is `Math.toDegrees(Math.atan2(x, y))`: 0 is dead ahead, + is right.
- **Field frame** (subsystem only): Pedro's coordinates, in inches.

---

## `PollenTrackerSubsystem`

### Create it and give it the pose

```java
DriveSubsystem drive = new DriveSubsystem(hardwareMap, telemetryM);                  // first
PollenTrackerSubsystem vision = new PollenTrackerSubsystem(hardwareMap, telemetryM); // second
vision.setPoseSupplier(drive::getPose);
```

- **Create it after `DriveSubsystem`.** FTCLib runs every subsystem's `periodic()` in creation
  order, before any command. `DriveSubsystem.periodic()` updates the pose, so vision and your
  commands then all see this loop's pose.
- **With a pose supplier:**
  - Balls are tracked in **field coordinates**, using the pose from when each camera frame was
    taken.
  - A ball stays put while the robot drives and turns. In testing, a robot swinging ±34° while
    driving kept **1** id for a ball, where robot-relative tracking went through 37.
  - Every ball also gets field coordinates, which is what Pedro needs.
- **Without a pose supplier** it works robot-relative, like the old OpModes.
- `telemetryM` is the Panels `TelemetryManager`. It can be `null`.
- Call `vision.close()` when the OpMode ends.

### Reading Pollen

| Method | Returns |
|---|---|
| `getBalls()` | Confirmed Pollen (`TrackedBall`), nearest first |
| `getNearestBall()` | Nearest one (the old PollenTracker's target), or `null` |
| `getGroups()` | Pollen groups (`TrackedGroup`), best score first |
| `getTargetGroup()` | Best group (blobTrack's target), or `null` |
| `getBall(id)` / `getGroupContaining(id)` | Follow one ball / group across loops; `null` once it's gone |
| `hasTarget()` | At least one confirmed Pollen |

**`TrackedBall`** is a read-only snapshot, so it's safe to keep.

| Field / method | Meaning |
|---|---|
| `id` | Stays the same while the ball keeps being tracked |
| `type` | `POLLEN`, `RED_NECTAR` or `BLUE_NECTAR` (`type.radius` in inches, `type.isNectar()`) |
| `x`, `y` | Robot frame **right now**, inches |
| `fieldX`, `fieldY`, `fieldPose()` | Field frame (Pedro), inches. Equal to `x`, `y` without a pose supplier. |
| `distance()`, `headingDeg()` | From robot center; heading + = right |
| `seenThisFrame` | `true` = the camera saw it in the latest frame; `false` = last known position |
| `ageS`, `hits` | Seconds since last seen / frames matched |

**`TrackedGroup`**

| Field / method | Meaning |
|---|---|
| `x`, `y` | Aim point, robot frame |
| `fieldX`, `fieldY` | Aim point, field frame |
| `balls`, `count` | Members (nearest first) and how many |
| `score`, `leadId` | Score, and the id of its nearest ball |
| `distance()`, `headingDeg()` | From robot center; heading + = right |
| `contains(id)`, `seenThisFrame` | Whether a ball is in this group / whether any member was seen this frame |

```java
PollenTrackerSubsystem.TrackedGroup g = vision.getTargetGroup();
if (g != null) {
    double inches = g.distance();        // from robot center
    double degrees = g.headingDeg();     // + = right
    Pose goal = g.fieldPose();           // Pedro field point to drive to
}
```

### Nectar

These exist for later; nothing uses them yet. Nectar (3.6 in) is found by the same pipeline with
its own color ranges (`R_*`, `B_*`, from NectarTracker), and tracked the same way. A track never
matches a ball of a different color.

| Method | Returns |
|---|---|
| `getNectar()` | Confirmed Nectar of both colors, nearest first |
| `getNectar(BallType.RED_NECTAR)` / `getNectar(BallType.BLUE_NECTAR)` | One color, nearest first |
| `getNearestNectar()` / `getNearestNectar(color)` | Nearest, or `null` |
| `getNectarBall(id)` | One Nectar by id, or `null` |

**Settings:**
- `DETECT_RED_NECTAR` / `DETECT_BLUE_NECTAR` turn each color off; each costs camera-thread time.
- `RED_EXCLUDES_YELLOW` stops orange-ish Pollen pixels from counting as red.
- The red and blue ranges still need tuning on real balls (`SHOW_MASK` with `MASK_COLOR` `"R"` /
  `"B"`).

### Path checks

These come from the `path-planning` version of `blobTrack` and are also unused for now.

| Method | What it does |
|---|---|
| `isPathBlockedByNectar(x, y)` / `isPathBlockedByNectar(group)` | Checks whether driving straight at that robot-frame point would hit tracked Nectar. A ball counts once its **edge** is inside the robot's width (`ROBOT_HALF_WIDTH_LEFT` / `RIGHT`). |
| `isPathBlocked(x, y, obstacles)` | `blobTrack`'s version, unchanged (gives identical results). It checks only obstacle **centers**. |
| `isPathBlocked(x, y, obstacles, radius)` | Same, with obstacles treated as balls of that radius |
| `requiresCrossField(...)` | `blobTrack`'s version, unchanged: is a robot-relative target across a field line? |

All of these return a `PathBlockResult` with `blocked`, `suggestedTurnDegrees` (+ = turn right),
`obstaclesLeft` and `obstaclesRight`.

### Everything else

- **Calibration:** `startCalibration()` (one Pollen) or `startCalibration(BallType)`,
  `cancelCalibration()`, `isCalibrating()`, `getCalibrationStatus()`, `getTiltDeg()`,
  `getFocalPx()`. A result outside `CAL_TILT_*` / `CAL_FOCAL_*` is refused and the old values are
  kept.
- **Status:** `isFrameFresh()` (a frame in the last 0.3 s) and `getExposureStatus()`.
- **Reset:** `reset()` forgets every ball.
- **Telemetry:** each loop it adds three lines to Panels: `Vision: pollen`, `Vision: nectar` and
  `Vision: camera`.

**Differences from `blobTrack`:**
- `BETA` defaults to `0`, because balls sit still in the field frame.
- The top image edge counts as "cut off".
- Balls near the image edges are no longer judged too big (they image bigger off-center).
- It never calibrates by itself.

---

## `PollenPedroExample` (Pedro Pathing)

The Pedro version of `moveToBall`. Each chase drives a Pedro path to the target's **field**
position. Once the balls drop out of the camera's view up close, Pedro keeps driving to that
remembered point on odometry, so there's no timed blind finish.

**Controls (gamepad 1)**
- **During init: X** calibrates on one Pollen. Nothing that can move the robot is set up before
  START.
- **A:** start chasing, one target after another.
- **B:** stop.
- **Sticks:** drive the robot whenever it isn't chasing (the team's `DriveRobotCommand`).

**One chase (`DriveToPollenCommand`)**
1. **Pick** the best group, or the nearest ball if `TARGET_GROUPS = false`, and lock onto it.
2. **Face it.** `follower.hold()` a pose turned toward the target, until within `FACE_TOL_DEG`
   (or `FACE_TIMEOUT_S`). If the target is lost for `LOST_S`, it re-picks.
3. **Settle** for `SETTLE_S`.
4. **Follow** `Paths.line(robot, point STOP_DIST short).facingPoint(target)` at `CHASE_SPEED`
   (a fraction of max speed).
   - It re-plans if the target is still seen, it's farther than `NO_REPLAN_INSIDE`, and its
     estimate moved more than `REPLAN_SHIFT`.
   - Re-plans happen at most every `REPLAN_MIN_S`.
5. **Done** when `!follower.isBusy()`, which is Pedro's "path finished and settled" check, or
   after `CHASE_TIMEOUT_S`.
6. **Hold**, wait `REARM_S`, and repeat.

**Notes**
- It only drives. The `// INTAKE:` comments mark where intake code would go.
- `DriveSubsystem` stays the only owner of the Pedro follower.
- The start pose (`START_X/Y/HEADING_DEG`) only matters if you compare against real field
  positions.

---

## Using `blobTrack` directly (older way)

All of `blobTrack`'s tuning values are `public static`, so any code can read them, e.g.
`blobTrack.LENS_HEIGHT` or `blobTrack.EXPOSURE_MS`. Writing one (`blobTrack.GATE = 8.0;`)
changes it for **every** OpMode until the app restarts.

Its pipeline classes (`PollenProcessor`, `Geometry`, `Tracker`, `Track`, `Group`) are
package-private, so your class has to be in `org.firstinspires.ftc.teamcode.Vision`. Every loop:

1. `geo.syncFromStatics()`
2. Take `processor.latest`, and only process it if `frame.timeNanos` changed.
3. Project and size-check each ball, then call `tracker.update(...)`.
4. Read `tracker.getConfirmed()` (balls, nearest first), `tracker.getGroups()` (best first),
   `tracker.getTargetGroup(groups)` or `tracker.groupContaining(id)`.

`Track` has `id`, `x`, `y`, `distance()`, `vx`, `vy`, `hits`, `misses` (0 = seen this frame) and
`confirmed`. `Group` has `x`, `y`, `distance()`, `balls`, `score`, `leadId` and `contains(id)`.
`moveToBall.java` is a complete worked example. For new code, `PollenTrackerSubsystem` does all of
this for you.

---

## `moveToBall`: chasing with P controllers

**State machine:** `HUNTING` → `CHASE` → (`BLIND`) → `COOLDOWN` → `HUNTING`

- **HUNTING.** The sticks drive normally until `tracker.getTargetGroup()` gives a group (closer
  than `MAX_CHASE_DIST`, if set). Then it locks onto that group's `leadId`.
- **CHASE.** The aim point is a rolling average of the last `AVG_SAMPLES` readings.
  - **Turn in place** with `turn = KP_TURN * headingErr` until within `TURN_TOL_DEG` (or
    `TURN_TOL_IN` sideways). Turn power stays between `MIN_TURN` and `MAX_TURN`.
  - **Then drive** with `drive = KP_DRIVE * (distance - STOP_DIST)`, kept between `MIN_DRIVE`
    and `MAX_DRIVE` (`FAR_DRIVE_CAP` beyond `FAR_DIST`). It keeps steering with
    `KP_TURN * headingErr`, limited to ±`STEER_CLAMP`.
  - Done within `DRIVE_TOL`.
  - It never drives on old data: no reading for `STALE_STOP_S` stops the motors.
  - It re-locks after `RELOCK_S`, gives up after `LOST_GIVEUP_S`, and times out after
    `TIMEOUT_S`.
- **BLIND.** If the group was lost closer than `BLIND_START_DIST` and nearly straight ahead, it
  drives forward at `BLIND_POWER` for `(lastDistance - STOP_DIST) / BLIND_SPEED_IPS` seconds,
  capped at `BLIND_MAX_S`. A ball reappearing ends the blind drive.
- **COOLDOWN.** Pauses `REARM_S`, then goes back to hunting.

**Controls (gamepad 1)**
- **Hold A:** stop.
- **Both bumpers:** speed test. Drives at `BLIND_POWER` for `SPEED_TEST_S`. Enter the measured
  distance with dpad left/right, then press dpad down to set `BLIND_SPEED_IPS`.
- **Sticks:** drive whenever nothing is being chased.

**Notes**
- `moveToBall` doesn't calibrate. It uses `blobTrack.TILT_DEG` / `FOCAL_PX`.
- At start it sets `blobTrack.BETA = 0` and `blobTrack.GATE = 8`, because the robot moves and
  the balls don't. Those values stay until the app restarts.
