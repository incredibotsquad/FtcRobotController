# BIOBuzz pickup vision

OpenCV color segmentation for yellow pollen, red nectar, and blue nectar. It reports visible color blobs, approximate floor positions, and ranked pickup neighborhoods. The Java `VisionProcessor` runs on the Control Hub with a compatible USB webcam. The Python reference runs on your Mac with that same camera, or exports as a Limelight SnapScript.

This module does not drive motors, move the swivel, perform hive aiming, or change the existing autonomous/teleop routines. Integrate the returned targets with your intake/path planner when ready.

## Start on the Mac

From the **separate BIOBuzz project folder**:

```sh
./tools/biobuzz/run_mac.command --camera 0
```

The launcher creates a local Python virtual environment and installs OpenCV/NumPy on its first run. Python 3.10+ is recommended. If Python is unavailable, install Python 3 first. You can also double-click `run_mac.command` in Finder. Try `--camera 1` or `--camera 2` if index 0 is the built-in camera. Allow Camera access for the launching app in macOS **System Settings → Privacy & Security → Camera**. Close other apps using the USB camera.

For explicit setup, or to repair an interrupted installation:

```sh
python3 -m venv tools/biobuzz/.venv
tools/biobuzz/.venv/bin/python -m pip install -r tools/biobuzz/requirements.txt
tools/biobuzz/.venv/bin/python tools/biobuzz/run.py --camera 0
```

Start with real game pieces on a neutral floor. The initial HSV thresholds are starting values, not field-tuned measurements. Yellow/red/blue household objects can test segmentation, but do not establish competition accuracy.

| Control | Effect |
| --- | --- |
| `Q` | Quit and release the camera |
| `I` | Toggle calibrated intake pose / hive or moving pose; wait the configured settling time |
| `A` | Toggle red/blue alliance |
| `M` | Toggle pollen-only / pollen plus own nectar |
| `T`, then `1`, `2`, or `3` | HSV sliders and mask for pollen, red, or blue |
| Click image | Print that pixel's HSV value in the terminal |
| `C` | Freeze frame and calibrate four measured floor points |
| `S` | Save thresholds to the selected config and a raw test image to `output/` |

The tuner changes the **first** HSV interval for a color; red has a second interval around hue 179, editable in JSON. Hue is 0–179; saturation/value are 0–255. Keep threshold intervals for the three colors distinct. Set camera exposure/white balance consistently using available camera controls; changing lighting or auto-exposure can change segmentation.

## Position calibration and the swivel

No distance estimates or pickup target are returned until calibration is loaded and intake pose is explicitly enabled. Color boxes/counts work immediately. Units are **inches**, measured from your chosen robot origin: **forward positive, left positive**. Image anchors use normalized coordinates: `(0,0)` upper left, `(1,1)` lower right.

1. Set the USB camera to its repeatable **intake stop**. Fix its resolution, crop, focus/zoom, and orientation. Choose a stop that sees a useful floor area without the intake hiding it.
2. Place four small floor marks in a convex quadrilateral covering the pickup area. Measure each mark's forward/left position relative to the robot origin. For example: near-left `(15,18)`, far-left `(65,18)`, far-right `(65,-18)`, near-right `(15,-18)`. These are examples; use your actual measurements.
3. Press `C`. Enter those four positions into the terminal, then click their image locations in the **same perimeter order**. Enter saves; `R` resets the clicks; Escape cancels.
4. Press `I` to confirm the camera is stationary at that intake stop. Calibration includes camera offset/yaw implicitly. It does not need an assumed mounting height or field of view.
5. Check several additional measured locations, including near/far and left/right edges, with actual pieces. Record error and adjust usable range/ROI. Do this again on the field under its lighting.

The homography maps the bottom contour point to the floor. For round objects this is an approximation: the visible silhouette is not exactly the physical contact point. Lens distortion, shadows, occlusion, and merged blobs introduce error. Validate against your intake tolerance; do not interpret the output as precise 3D localization. For higher accuracy, add camera-intrinsic calibration and ball geometry after measuring the actual pieces and mount.

**Before commanding any swivel motion, disable intake pose.** After the swivel physically reaches its calibrated stop, enable it; `settle_ms` adds an extra delay. The camera cannot determine its swivel position automatically. Use mechanical stops, servo state/timing or feedback in the robot controller. Continuously variable intake angles need separate calibration/pose handling; this version uses one fixed intake pose. Never reuse the intake calibration for hive aiming.

The module rejects out-of-polygon, out-of-range, clipped, and aspect-mismatched metric detections. A resolution change with the same aspect is accepted **only on the assumption that the camera preserves its field of view**. Recalibrate after a camera, mount, crop, zoom, or image-rotation change. A Logitech calibration is not a Limelight calibration.

The shared config is `TeamCode/src/main/assets/biobuzz.json`. Mac calibration/tuning saves there by default, so rebuilding the app packages it automatically. Use `--config path/to/camera.json` for independent camera profiles; copy the desired profile into the asset before building.

## Results and ranking

Every frame always detects all three colors. Configuration controls HSV intervals, normalized ROI, minimum/maximum area fraction, fill/aspect filters, morphology, blob limits, usable range, settling/stale timing, and ranking weights.

For each eligible blob, the detector finds located blobs within `cluster_radius_inches`. Its score is:

```text
pollen_weight × pollen blobs
+ nectar_weight × own nectar blobs (only in MIXED mode)
- opponent_penalty × opponent nectar blobs
- distance_penalty_per_inch × target distance
```

Opposing nectar is always reported and never selected as a pickup target. Its default penalty is zero; increase it if nearby opposing nectar makes an area undesirable. Set `nectar_weight` relative to `pollen_weight` to adjust priorities. Pollen-only mode excludes your nectar from target rewards. Zero-weight classes are not candidate anchors. The highest score wins; ties favor a nearer target. Scores can be negative; the robot planner may impose a minimum score.

The target is an actual desired blob's location, not a point between objects. Returned neighborhoods overlap and are alternative destinations; do not sum their counts. Counts measure **connected visible blobs**, not true inventory. Touching same-color pieces may merge and count as one. This version does not claim reliable instance separation or tracking through occlusion. It caps accepted blobs per color to bound ranking work and sets `truncated` on USB/Python when the cap is reached.

The output is an observation, not a collision-free trajectory. It does not detect robots/obstacles, field legality, scoring value, or whether the intake can reach an area. Reobserve during approach and use your robot planner for those decisions. Color lookalikes (field tape, walls, clothing) can pass the filters: constrain ROI and validate shape/lighting on real scenes.

## Use in robot code

`BioBuzzVisionTest` appears under **Tuning → BIOBuzz: Vision Test**. Configure your USB camera as `Webcam 1`. This OpMode only opens the camera and shows telemetry/stream overlays. `A` confirms/disables intake pose, `B` changes alliance, and `X` changes selection mode. No servos or motors are commanded. The OpMode has a complete example of loading the JSON asset and creating/closing the portal.

Core integration, after loading `config`:

```java
BioBuzzProcessor vision = new BioBuzzProcessor(config);
VisionPortal portal = new VisionPortal.Builder()
    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
    .setCameraResolution(new Size(config.width, config.height))
    .addProcessor(vision)
    .build();
vision.setSelection(BioBuzzResult.Alliance.RED, BioBuzzResult.Mode.MIXED);

// BEFORE any swivel command:
vision.setIntakePose(false);
// ... command swivel, wait until it reaches calibrated intake stop ...
vision.setIntakePose(true); // adds configured settling delay

// In each robot loop:
BioBuzzResult observation = vision.getLatest();
BioBuzzResult.Target best = observation.best();
if (best != null) {
    double forwardInches = best.position.forward;
    double leftInches = best.position.left;
    double distanceInches = best.position.distance();
    double bearingRadians = best.position.bearingRadians();
    // Hand these to your planner; do not drive solely from a saved old snapshot.
}

// On shutdown, in finally:
portal.close();
vision.close();
```

`getLatest()` returns an immutable snapshot, with an empty snapshot for stale frames. `blobs` contains pixel boxes, normalized bottom anchors, color and optional robot-relative position. `targets` is sorted best first. `best()` returns null when none is usable; zero is never used as a sentinel coordinate. Calling selection/pose setters only invalidates on a change, so repeated identical calls are fine. If you retain a snapshot, recheck `isFresh(System.nanoTime(), config.maxAgeMs)` before using it. Do not retain a target across a swivel transition.

For field coordinates, use **the robot pose at `observation.captureTimeNanos`**, ideally interpolated from odometry history:

```java
BioBuzzResult.FieldPoint field = best.position.toField(
    poseAtCapture.getX(), poseAtCapture.getY(), poseAtCapture.getHeading());
```

This assumes inches, heading in radians counterclockwise from field +X, and the same robot origin as calibration. It works with Pedro poses using that convention. The module does not automatically sample odometry or transform coordinate conventions. Using the current pose while moving creates latency error. Timestamps are monotonic nanoseconds, not wall-clock time.

## Limelight option

Use a Limelight that supports FTC integration and Python SnapScripts (for example Limelight 3A). It does not attach to `VisionPortal` as a generic USB webcam. The full-list USB path is the recommended first validation path for your Logitech camera. The Limelight path uses exactly the Python detector source bundled into a standalone script.

1. Tune/calibrate using images from the **Limelight itself** with its intake pose and matching resolution/crop. You can use a downloaded raw image with the Mac tester's `--image` option. Do not use a browser screenshot containing scaled/letterboxed UI.
2. Export with its own config:

   ```sh
   tools/biobuzz/.venv/bin/python tools/biobuzz/export_limelight.py \
     --config path/to/limelight.json
   ```

3. Paste `tools/biobuzz/output/limelight_pipeline.py` into a dedicated Python SnapScript pipeline in the Limelight web UI. Check the Limelight console for errors and the stream for detection.
4. Instantiate `BioBuzzLimelight` with the matching config and pipeline index, then switch/start the camera:

   ```java
   Limelight3A ll = hardwareMap.get(Limelight3A.class, "limelight");
   BioBuzzLimelight vision = new BioBuzzLimelight(ll, 0, config);
   ll.pipelineSwitch(0);
   ll.start();
   // The same setSelection / setIntakePose rules apply.
   BioBuzzLimelight.Summary result = vision.getLatest();
   if (result != null && result.position != null) {
       // result.position.forward / left are the best pickup area's inches.
   }
   // ll.stop() when the owner of the camera shuts down.
   ```

Your existing `LimelightSubsystem` selects AprilTag pipelines. A single owner must coordinate switching between that hive/tag mode and the pickup pipeline, disabling intake pose before switching/moving. Do not have both subsystems continually command competing pipelines.

The adapter rejects wrong-pipeline results, stale results, frozen sequence numbers, and unacknowledged selection/pose changes. A signed request ID is echoed from the pipeline to prevent old settings from being mistaken for current ones. Its eight output numbers are `[signedRequestId, forward, left, score, totalPollenBlobs, totalRedBlobs, totalBlueBlobs, sequence]`. A negative ID means no metric target. Counts describe the **whole image**, not the best neighborhood. The full blob list, ranked list, and truncation flag are available on USB/Python, not in this compact Limelight protocol. Limelight capture time is estimated from reported latency.

The SnapScript has automated host-side tests; deployment/performance on physical Limelight hardware still needs verification. See the [official SnapScript documentation](https://docs.limelightvision.io/docs/docs-limelight/pipeline-python/snapscript-pipelines) and [FTC programming API](https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming).

## Repeatable tests and replay

```sh
# Unit/integration tests: segmentation, geometry, ranking, rejection and protocol.
tools/biobuzz/.venv/bin/python -m unittest discover -s tools/biobuzz/tests -v

# Camera-free end-to-end demo. Calibration here is synthetic, not for your camera!
tools/biobuzz/.venv/bin/python tools/biobuzz/demo.py
tools/biobuzz/.venv/bin/python tools/biobuzz/run.py \
  --video tools/biobuzz/output/demo.avi \
  --config tools/biobuzz/output/demo-config.json --intake

# Headless replay and one JSON observation per frame:
tools/biobuzz/.venv/bin/python tools/biobuzz/run.py \
  --video tools/biobuzz/output/demo.avi \
  --config tools/biobuzz/output/demo-config.json --intake --headless \
  --jsonl tools/biobuzz/output/results.jsonl

# A saved real frame (omit --headless for interactive calibration/tuning):
tools/biobuzz/.venv/bin/python tools/biobuzz/run.py --image path/to/frame.png

# Robot geometry/ranking/freshness tests and Android compile (requires Android SDK/JDK 17):
./gradlew :TeamCode:testDebugUnitTest :TeamCode:assembleDebug
```

On macOS with Homebrew JDK 17, prefix Gradle with `JAVA_HOME=/opt/homebrew/opt/openjdk@17` if Java is not on your default path. The repository's existing `local.properties` supplies the Android SDK location when present.

Before robot use, verify both nectar colors, single pieces versus piles, shadows/glare, floor lookalikes, empty view, occlusion and frame edges. Confirm distances at measured locations, a camera disconnect clearing the best target, and every swivel transition invalidating positions. Measure processing time/FPS on the Control Hub; host timings are not robot benchmarks.
