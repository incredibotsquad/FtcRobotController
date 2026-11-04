# Vision (Pollen)

Webcam vision that finds **Pollen** (yellow, 2.8 in balls), works out where each one is in inches
from the robot, groups balls that are touching, and picks the best group to go after.

| File | OpMode name | What it does |
|---|---|---|
| `blobTrack.java` | **Pollen Cluster Tracker** | The vision pipeline, plus a TeleOp that shows what it sees: camera preview, every tracked ball, every group, and the chosen target. It doesn't drive. |
| `moveToBall.java` | **Move To Ball** | Uses `blobTrack`'s pipeline to chase the best group with two **P controllers** (turn and drive), on plain motor powers. No encoders, no odometry. |

`blobTrack` owns all the vision settings. `moveToBall` reads them from `blobTrack`, so tune
vision in one place.

---

## Setup

1. **Camera.** Both OpModes open the webcam named **`"Webcam 1"`**, which must be in the active
   Robot Controller configuration.
2. **Drive motors (`moveToBall` only).** It uses `frontLeftMotor`, `backLeftMotor`,
   `frontRightMotor` and `backRightMotor`, with the left side reversed.
3. **Measure the camera.** In `blobTrack`, set these in inches from the robot center:
   - `LENS_HEIGHT`: floor to lens center
   - `CAM_FORWARD_OFFSET`: + = lens is in front of center
   - `CAM_RIGHT_OFFSET`: + = lens is right of center
4. **Calibrate.** Calibration finds the camera tilt and focal length.
   1. Put **one** ball straight ahead, `CAL_FLOOR_DIST` (30 in) along the floor from the point
      under the lens.
   2. Run **Pollen Cluster Tracker**. With `AUTO_CALIBRATE = true` it calibrates during init.
      Press **A** to recalibrate at any time.
   3. Copy the printed `tiltDeg` / `focalPx` into `TILT_DEG` / `FOCAL_PX` so they survive a
      restart.
5. **Tune the color.** In Panels, set `SHOW_MASK = true` so the preview shows only pixels that
   pass the color filter. Adjust `Y_*`, `CR_*` and `CB_*` (YCrCb) until each ball is a solid
   disk and nothing else survives. `CB_MAX` is the main knob.

The color ranges were tuned with manual exposure (`EXPOSURE_MS = 10`, `GAIN = 100`). Anything
that uses this pipeline has to set the same exposure, or the colors won't match.

---

## How the pipeline works

```
camera frame
  -> PollenProcessor   (camera thread)  yellow mask -> cleanup -> distance transform -> one circle per ball (pixels)
  -> Geometry.project  (your loop)      pixel circle -> floor position in inches + size check
  -> Tracker.update    (your loop)      matches balls frame to frame, smooths them, confirms them
  -> Tracker.getGroups (your loop)      chains balls within CLUSTER_RADIUS into groups and scores them
```

- **Size check.** A real ball at that floor position should appear a known size. If the
  measured / expected radius ratio is outside `SIZE_MIN`..`SIZE_MAX`, the blob is thrown out.
  That gets rid of yellow non-balls and two balls merged into one blob.
- **Group score.** `score = ballCount - DIST_PENALTY * distance`. With the default 0.03, one
  extra ball is worth about 33 in of extra driving. The aim point is the group's centroid when
  `AIM_AT_CENTROID` is on, or its nearest ball when it's off.

**Coordinates:** robot frame, in inches from the robot center. `x` is + to the right and `y` is
+ forward. Everything is relative to the robot at the moment the frame was taken; there is no
odometry. So when the robot drives or turns, the numbers change too.

---

## Using `blobTrack` from your own code

### Settings: readable from anywhere

All the tuning values are `public static` on `blobTrack`, so any code can read them:

```java
double lens = blobTrack.LENS_HEIGHT;
long exposureMs = blobTrack.EXPOSURE_MS;
int gain = blobTrack.GAIN;
double minRatio = blobTrack.SIZE_MIN, maxRatio = blobTrack.SIZE_MAX;
```

Writing one (`blobTrack.GATE = 8.0;`) changes it for **every** OpMode until the Robot Controller
app restarts.

### Pipeline objects: same package only

`PollenProcessor`, `Geometry`, `Tracker`, `Track`, `Group` and the rest are package-private
nested classes. Your class has to be in `org.firstinspires.ftc.teamcode.Vision` to use them
(that's how `moveToBall` does it). To use them from another package, make them `public`.

Every loop:

1. Call `geo.syncFromStatics()` to pick up live Panels edits to `TILT_DEG` / `FOCAL_PX`.
2. Take `processor.latest`, and only process it if `frame.timeNanos` changed.
3. Project each ball and size-check it, then call `tracker.update(...)`.
4. Read the tracker.

### Example: an OpMode that reads the values

```java
package org.firstinspires.ftc.teamcode.Vision;   // must be this package (see above)

@TeleOp(name = "Pollen Readout", group = "Vision")
public class PollenReadout extends LinearOpMode {
    @Override
    public void runOpMode() {
        blobTrack.PollenProcessor processor = new blobTrack.PollenProcessor();
        blobTrack.Geometry geo = new blobTrack.Geometry();      // starts from TILT_DEG / FOCAL_PX
        blobTrack.Tracker tracker = new blobTrack.Tracker();

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(processor)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        // Same exposure as blobTrack, or the color ranges won't match
        while (!isStopRequested() && portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            sleep(20);
        }
        if (blobTrack.MANUAL_EXPOSURE && !isStopRequested()) {
            ExposureControl exp = portal.getCameraControl(ExposureControl.class);
            exp.setMode(ExposureControl.Mode.Manual);
            sleep(50);
            exp.setExposure(blobTrack.EXPOSURE_MS, TimeUnit.MILLISECONDS);
            portal.getCameraControl(GainControl.class).setGain(blobTrack.GAIN);
        }

        waitForStart();
        long lastFrameTime = 0;
        while (opModeIsActive()) {
            geo.syncFromStatics();
            blobTrack.Frame frame = processor.latest;
            if (frame.timeNanos != lastFrameTime) {               // once per camera frame
                lastFrameTime = frame.timeNanos;
                List<blobTrack.Measurement> meas = new ArrayList<>();
                for (blobTrack.Ball b : frame.balls) {
                    blobTrack.Projection p = geo.project(b.x, b.y, b.r);
                    if (p == null) continue;                       // above the horizon
                    if (p.sizeRatio < blobTrack.SIZE_MIN || p.sizeRatio > blobTrack.SIZE_MAX) continue;
                    meas.add(new blobTrack.Measurement(p.x, p.y, b.x, b.y, b.r));
                }
                tracker.update(meas, frame.timeNanos);
            }

            List<blobTrack.Group> groups = tracker.getGroups();       // best score first
            blobTrack.Group target = tracker.getTargetGroup(groups);  // null if nothing
            if (target != null) {
                double headingDeg = Math.toDegrees(Math.atan2(target.x, target.y));  // + = right
                telemetry.addData("target", "%d ball(s), %.1f in, %.1f deg",
                        target.balls.size(), target.distance(), headingDeg);
            }
            for (blobTrack.Track t : tracker.getConfirmed()) {         // nearest first
                telemetry.addData("ball " + t.id, "x %.1f  y %.1f  dist %.1f", t.x, t.y, t.distance());
            }
            telemetry.update();
        }
        portal.close();
    }
}
```

Imports used: `android.util.Size`, `LinearOpMode`, `TeleOp`, `WebcamName`, `VisionPortal`,
`ExposureControl`, `GainControl`, `java.util.ArrayList`, `java.util.List`,
`java.util.concurrent.TimeUnit`.

### What you can read

**`Tracker`**

| Method | Returns |
|---|---|
| `update(measurements, frameTimeNanos)` | Feeds one frame of measurements in |
| `getConfirmed()` | Confirmed balls (`Track`), nearest first |
| `getGroups()` | All groups (`Group`), highest score first |
| `getTargetGroup(groups)` | The best group, or `null` |
| `groupContaining(id)` | The group that has the ball with this id right now, or `null`. Use it to stay locked on one group across frames. |
| `reset()` | Forget everything (e.g. after calibrating) |

**`Group`** (a clump of touching balls)

| Field / method | Meaning |
|---|---|
| `x`, `y` | Aim point, inches (centroid, or nearest ball if `AIM_AT_CENTROID` is off) |
| `distance()` | `hypot(x, y)`, inches from robot center |
| `balls` | The `Track`s in the group; `balls.size()` is the ball count |
| `score` | `ballCount - DIST_PENALTY * distance` |
| `leadId` | Id of the group's nearest ball, used to identify the group frame to frame |
| `contains(id)` | Whether that ball is in this group |

**`Track`** (one ball)

| Field / method | Meaning |
|---|---|
| `id` | Stays the same while the ball keeps being tracked |
| `x`, `y` | Smoothed position, inches (x right, y forward) |
| `distance()` | `hypot(x, y)` |
| `vx`, `vy` | Apparent velocity, in/s |
| `px`, `py`, `r` | Where it last was in the image, in pixels |
| `hits`, `misses` | Frames matched / frames missed in a row (`misses == 0` means seen this frame) |
| `confirmed` | Seen at least `CONFIRM_HITS` times |

The **heading** to anything is `Math.toDegrees(Math.atan2(x, y))`: 0 is dead ahead, + is right.

**Lower-level objects**

| Object | What it holds |
|---|---|
| `processor.latest` (`Frame`) | `balls`: raw pixel circles (`Ball`: `x`, `y`, `r`), biggest first. `timeNanos`: frame time. |
| `geo.project(px, py, r)` (`Projection`) | `x`, `y` in inches, `expectedR`, and `sizeRatio` = measured / expected radius. `null` if the point is above the horizon. |
| `geo.tiltDeg`, `geo.focalPx` | The camera numbers in use right now |
| `geo.calibrate(px, py, r, floorDist)` | Solves tilt and focal length from one ball and writes `TILT_DEG` / `FOCAL_PX` |

---

## `moveToBall`: chasing with P controllers

**State machine:**
`HUNTING` → `CHASE` → (`BLIND`) → `COOLDOWN` → `HUNTING`

- **HUNTING.** The sticks drive normally. As soon as `tracker.getTargetGroup()` gives a group
  (closer than `MAX_CHASE_DIST`, if set), it locks onto that group's `leadId`.
- **CHASE.** The aim point is a rolling average of the last `AVG_SAMPLES` readings of the
  locked group.
  - `headingErr = atan2(x, y)` in degrees, and `distErr = distance - STOP_DIST`.
  - **Turn in place** with `turn = KP_TURN * headingErr`, until within `TURN_TOL_DEG` (or
    `TURN_TOL_IN` sideways). Turn power is capped at `MAX_TURN` and never below `MIN_TURN`.
  - **Then drive** with `drive = KP_DRIVE * distErr`, capped at `MAX_DRIVE` (or
    `FAR_DRIVE_CAP` beyond `FAR_DIST`) and never below `MIN_DRIVE`. It keeps steering with
    `KP_TURN * headingErr`, limited to ±`STEER_CLAMP`.
  - Done when `distErr <= DRIVE_TOL`.
  - It never drives on old data. If there's no reading for `STALE_STOP_S`, the motors stop.
  - If the locked group vanishes for `RELOCK_S`, it re-locks onto the best group.
  - If nothing is seen for `LOST_GIVEUP_S`, it gives up.
  - The whole chase times out after `TIMEOUT_S`.
- **BLIND.** The camera can't see a ball that's very close (about 21 in). So if the group was
  lost while it was closer than `BLIND_START_DIST` and nearly straight ahead, the robot drives
  forward at `BLIND_POWER` for `(lastDistance - STOP_DIST) / BLIND_SPEED_IPS` seconds, capped
  at `BLIND_MAX_S`. Any ball reappearing ends the blind drive.
- **COOLDOWN.** Pauses `REARM_S`, then goes back to hunting.

**Controls (gamepad 1)**
- **Hold A:** stop. It stays stopped while held.
- **Both bumpers:** speed test. Drives forward at `BLIND_POWER` for `SPEED_TEST_S`.
  1. Measure how far the robot went.
  2. Enter that distance with dpad left/right (steps of `SPEED_TEST_STEP` in).
  3. Press dpad down to set `BLIND_SPEED_IPS`, then copy that value into the code.
- **Sticks:** drive the robot whenever nothing is being chased.

**Notes**
- `moveToBall` doesn't calibrate. It uses `blobTrack.TILT_DEG` / `FOCAL_PX`, so calibrate with
  Pollen Cluster Tracker first.
- At start, `moveToBall` sets `blobTrack.BETA = 0` (no velocity prediction) and
  `blobTrack.GATE = 8` (a wider match gate), because here the robot moves and the balls don't.
  Those values stay until the app restarts, so if you run Pollen Cluster Tracker afterwards it
  uses them too.
- Telemetry shows each chase's result (`STOPPED AT`, `reason`), the groups table, and the
  camera numbers.
