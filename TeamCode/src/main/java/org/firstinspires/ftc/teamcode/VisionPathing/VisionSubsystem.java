package org.firstinspires.ftc.teamcode.VisionPathing;

import static org.firstinspires.ftc.teamcode.VisionPathing.VisionConstants.*;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.camerastream.PanelsCameraStream;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Iterator;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.function.Supplier;

/**
 * FTCLib subsystem that finds POLLEN (yellow, 2.8 in) and NECTAR (red / blue, 3.6 in) with one
 * webcam and tells you where every ball is, in inches, relative to the robot and on the field.
 *
 * ---------------------------------------------------------------------------------------------
 * HOW TO USE
 *   VisionSubsystem vision = new VisionSubsystem(hardwareMap);   // registers itself with FTCLib
 *   vision.setRobotPoseSupplier(follower::pose);                  // optional, but strongly recommended
 *
 *   each loop:  follower.update();                                // pose first...
 *               CommandScheduler.getInstance().run();             // ...then this calls periodic()
 *               (not using the scheduler? call vision.periodic() yourself)
 *
 *   List<VisionSubsystem.Ball> red = vision.getBalls(BallType.RED_NECTAR);
 *   VisionSubsystem.Ball nearest   = vision.getNearest(BallType.POLLEN);
 *   nearest.distance(), nearest.bearingDeg(), nearest.x, nearest.y, nearest.fieldX, nearest.fieldY
 *
 * ---------------------------------------------------------------------------------------------
 * PIPELINE (camera thread, inside Processor.processFrame)
 *   RGB -> YCrCb -> blur -> per color: inRange mask -> open (specks) -> close (holes)
 *   -> distance transform -> repeatedly take the tallest peak: its location is a ball center,
 *   its height is that ball's radius in px. Touching balls pinch the mask, so each gets its own
 *   peak instead of merging into one big blob. Cut-off balls (any edge) are flagged, not used.
 *
 * GEOMETRY (OpMode thread, Camera class)
 *   Pixel -> ray -> intersect with the plane at ball-center height -> robot-frame inches.
 *   Then a SIZE CHECK: measured radius vs. the radius a real ball would have at that spot.
 *   That rejects yellow boxes, two merged balls, and anything not sitting on the floor.
 *
 * TRACKING (OpMode thread, field frame)
 *   With a pose supplier, balls are tracked in FIELD coordinates. A ball that sits still stays
 *   still in the field even while the robot drives and turns, so:
 *     - matching is easy and stable,
 *     - a ball that leaves the camera's view (e.g. under the intake for the last ~11 in) is
 *       REMEMBERED for MEMORY_S, and its robot-relative position keeps updating from odometry.
 *   A ball is only dropped early if it SHOULD be in view (and isn't hidden behind another ball)
 *   but isn't seen for MAX_VISIBLE_MISSES frames. Each track keeps an uncertainty: far readings
 *   (error grows with distance^2) count less than close ones.
 *
 * Fixes carried over from the PollenIntake review:
 *   - "fresh" means actually seen recently (Ball.isVisibleNow()), never a coasted guess.
 *   - Settings are never rewritten by other code. Only calibration writes TILT_DEG / FOCAL_PX.
 *   - Calibration is explicit, checks the ball is single, centered and steady, and sanity-checks
 *     the answer before saving it.
 *   - Edge check includes the top edge. Each color has its own MAX_BALLS budget.
 *   - Red can't steal yellow pixels (RED_EXCLUDES_YELLOW).
 *   - Expected size accounts for off-center balls looking bigger.
 *   - The frame's pose is the robot pose WHEN THE FRAME WAS TAKEN (interpolated history), not
 *     the pose when we got around to processing it. Matters a lot while turning.
 */
public class VisionSubsystem extends SubsystemBase {

    // =====================================================================
    // ========================= PUBLIC TYPES ==============================
    // =====================================================================

    /** What a ball is. Radius comes from VisionConstants so it can be tuned. */
    public enum BallType {
        POLLEN, RED_NECTAR, BLUE_NECTAR;

        /** Real radius in inches. */
        public double radius() {
            return (this == POLLEN ? POLLEN_DIAMETER : NECTAR_DIAMETER) / 2.0;
        }

        public boolean isNectar() {
            return this != POLLEN;
        }

        /** One letter for telemetry: P, R or B. */
        public String letter() {
            return this == POLLEN ? "P" : this == RED_NECTAR ? "R" : "B";
        }
    }

    /**
     * A tracked ball, frozen at the moment periodic() last ran. Safe to keep and read anywhere.
     * Robot-frame numbers use the robot's CURRENT pose, so they stay right while driving even
     * when the ball itself is out of view.
     */
    public static final class Ball {
        public final int id;
        public final BallType type;
        /** Field position (in), Pedro frame. Without a pose supplier this is the robot frame. */
        public final double fieldX, fieldY;
        /** Robot frame (in): x = right of robot center, y = forward of robot center. */
        public final double x, y;
        /** 1-sigma position uncertainty (in). Small = trustworthy. */
        public final double sigma;
        /** Seen CONFIRM_HITS times. Unconfirmed balls are only useful as obstacles. */
        public final boolean confirmed;
        public final int hits;
        /** Seconds since the camera last actually saw it. */
        public final double ageS;

        Ball(int id, BallType type, double fieldX, double fieldY, double x, double y,
             double sigma, boolean confirmed, int hits, double ageS) {
            this.id = id; this.type = type; this.fieldX = fieldX; this.fieldY = fieldY;
            this.x = x; this.y = y; this.sigma = sigma; this.confirmed = confirmed;
            this.hits = hits; this.ageS = ageS;
        }

        /** Straight-line floor distance from robot center (in). */
        public double distance() {
            return Math.hypot(x, y);
        }

        /** Degrees off the robot's nose. + = to the right. */
        public double bearingDeg() {
            return Math.toDegrees(Math.atan2(x, y));
        }

        /** True if the camera saw it within FRESH_S. False = position is remembered, not seen. */
        public boolean isVisibleNow() {
            return ageS <= FRESH_S;
        }

        public Pose fieldPose() {
            return new Pose(fieldX, fieldY);
        }
    }

    /** One raw blob from the latest frame, and what happened to it. For telemetry / tuning. */
    public static final class Detection {
        public final BallType type;
        public final double px, py, rPx;        // image (px)
        public final double x, y;               // robot frame at capture (in), NaN if none
        public final double sizeRatio;          // measured / expected radius, NaN if none
        public final String status;             // "ok" or why it was thrown away

        Detection(BallType type, double px, double py, double rPx,
                  double x, double y, double sizeRatio, String status) {
            this.type = type; this.px = px; this.py = py; this.rPx = rPx;
            this.x = x; this.y = y; this.sizeRatio = sizeRatio; this.status = status;
        }

        public boolean accepted() {
            return "ok".equals(status);
        }
    }

    // =====================================================================
    // ============================ STATE ==================================
    // =====================================================================

    private final VisionPortal portal;
    private final Processor processor;
    private boolean streamingToPanels = false;

    private Supplier<Pose> poseSupplier = null;
    private final ArrayDeque<PoseStamp> poseHistory = new ArrayDeque<>();
    private static final long POSE_HISTORY_NS = 1_000_000_000L;   // keep 1 s

    private final List<Track> tracks = new ArrayList<>();
    private int nextId = 0;
    private long lastSeq = -1;
    private long lastFrameProcessedNanos = 0;
    private double fps = 0;
    private long lastFrameTime = 0;

    private List<Ball> snapshot = Collections.emptyList();
    private List<Detection> detections = Collections.emptyList();
    private int highlightId = -1;

    // Exposure is applied without blocking: a few steps spread over periodic() calls
    private int exposureStep = 0;
    private long exposureNextNanos = 0;
    private long appliedExposureMs = -1;
    private int appliedGain = -1;
    private String exposureStatus = "not set yet";

    // Calibration
    private boolean calibrating = false;
    private BallType calType = BallType.POLLEN;
    private double calFloorDist = CAL_FLOOR_DIST;
    private final List<double[]> calSamples = new ArrayList<>();
    private String calStatus = "not run (using TILT_DEG / FOCAL_PX from VisionConstants)";

    // =====================================================================
    // ========================= CONSTRUCTION ==============================
    // =====================================================================

    /**
     * Opens the webcam named VisionConstants.WEBCAM_NAME and starts processing.
     * FTCLib registers the subsystem automatically, so periodic() runs whenever the
     * CommandScheduler runs.
     */
    public VisionSubsystem(HardwareMap hardwareMap) {
        processor = new Processor();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .setCameraResolution(new Size(IMG_W, IMG_H))
                .addProcessor(processor)
                .build();

        if (STREAM_TO_PANELS) {
            PanelsCameraStream.INSTANCE.startStream(portal, PANELS_STREAM_FPS);
            streamingToPanels = true;
        }
        if (AUTO_CALIBRATE) startCalibration(BallType.POLLEN);
    }

    /**
     * Where the robot is, every loop. Pass {@code follower::pose}. Without it, everything is
     * tracked in the robot frame, which is only right while the robot is not moving.
     */
    public void setRobotPoseSupplier(Supplier<Pose> supplier) {
        poseSupplier = supplier;
        poseHistory.clear();
        tracks.clear();       // old positions were in a different frame
    }

    public boolean hasPoseSupplier() {
        return poseSupplier != null;
    }

    /** Stops the camera. Call once when the OpMode ends. */
    public void close() {
        if (streamingToPanels) {
            PanelsCameraStream.INSTANCE.stopStream();
            streamingToPanels = false;
        }
        portal.close();
    }

    // =====================================================================
    // ============================ PERIODIC ===============================
    // =====================================================================

    /** Called by the CommandScheduler. Cheap when no new frame has arrived. */
    @Override
    public void periodic() {
        long now = System.nanoTime();
        Pose pose = currentPose();
        recordPose(now, pose);
        updateExposure(now);

        Frame frame = processor.latest;
        if (frame.seq != lastSeq) {                 // only handle each camera frame once
            lastSeq = frame.seq;
            lastFrameProcessedNanos = now;
            updateFps(frame.timeNanos);

            if (calibrating) {
                calibrationStep(frame);
            } else {
                handleFrame(frame, poseAt(frame.timeNanos));
            }
        }

        expireTracks(now);
        rebuildSnapshot(now, pose);
        pushLabels();
    }

    // =====================================================================
    // ============================ QUERIES ================================
    // =====================================================================

    /** Confirmed balls of every type, nearest first. */
    public List<Ball> getBalls() {
        List<Ball> out = new ArrayList<>();
        for (Ball b : snapshot) if (b.confirmed) out.add(b);
        return out;
    }

    /** Confirmed balls of one type, nearest first. */
    public List<Ball> getBalls(BallType type) {
        return getBalls(type, false);
    }

    /** Balls of one type, nearest first. includeUnconfirmed = also balls seen only once or twice. */
    public List<Ball> getBalls(BallType type, boolean includeUnconfirmed) {
        List<Ball> out = new ArrayList<>();
        for (Ball b : snapshot) {
            if (b.type == type && (b.confirmed || includeUnconfirmed)) out.add(b);
        }
        return out;
    }

    /** Confirmed Nectar for an alliance setting: 0 = red, 1 = blue, 2 = either. Nearest first. */
    public List<Ball> getNectar(int alliance) {
        List<Ball> out = new ArrayList<>();
        for (Ball b : snapshot) if (b.confirmed && isAllianceNectar(b.type, alliance)) out.add(b);
        return out;
    }

    /** Nearest confirmed ball of a type, or null. */
    public Ball getNearest(BallType type) {
        for (Ball b : snapshot) if (b.type == type && b.confirmed) return b;
        return null;
    }

    /** A ball by id (confirmed or not), or null if it is no longer tracked. */
    public Ball getBall(int id) {
        if (id < 0) return null;
        for (Ball b : snapshot) if (b.id == id) return b;
        return null;
    }

    /** Every blob from the latest frame with the reason it was kept or dropped. */
    public List<Detection> getDetections() {
        return detections;
    }

    /** True for the Nectar colors an alliance setting wants (0 red, 1 blue, 2 either). */
    public static boolean isAllianceNectar(BallType type, int alliance) {
        if (!type.isNectar()) return false;
        if (alliance == 2) return true;
        return alliance == 1 ? type == BallType.BLUE_NECTAR : type == BallType.RED_NECTAR;
    }

    /** Drop a ball, e.g. once it has been intaken. If it's still really there, it comes back. */
    public void forget(int id) {
        Iterator<Track> it = tracks.iterator();
        while (it.hasNext()) if (it.next().id == id) it.remove();
        rebuildSnapshot(System.nanoTime(), currentPose());
    }

    public void forgetAll() {
        tracks.clear();
        snapshot = Collections.emptyList();
    }

    /** Draw this ball's id ring in green on the camera preview. -1 = none. */
    public void highlight(int id) {
        highlightId = id;
    }

    /** True if a camera frame was processed in the last 0.3 s. */
    public boolean isFrameFresh() {
        return lastFrameProcessedNanos != 0 && System.nanoTime() - lastFrameProcessedNanos < 300_000_000L;
    }

    public double getFps() {
        return fps;
    }

    public boolean isStreaming() {
        return portal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    public String getExposureStatus() {
        return exposureStatus;
    }

    // =====================================================================
    // ========================== CALIBRATION ==============================
    // =====================================================================

    /** Calibrate with one ball of this type straight ahead at CAL_FLOOR_DIST. */
    public void startCalibration(BallType type) {
        startCalibration(type, CAL_FLOOR_DIST);
    }

    /**
     * Calibrate TILT_DEG and FOCAL_PX from ONE ball of this type, straight ahead, whose floor
     * contact point is floorDist inches (along the floor) from the point directly under the lens.
     * The robot must be still. Watch getCalibrationStatus() for progress and the result.
     */
    public void startCalibration(BallType type, double floorDist) {
        calType = type;
        calFloorDist = floorDist;
        calSamples.clear();
        calibrating = true;
        calStatus = "waiting for one " + type + " at " + floorDist + " in, straight ahead";
    }

    public void cancelCalibration() {
        calibrating = false;
        calSamples.clear();
        calStatus = "cancelled";
    }

    public boolean isCalibrating() {
        return calibrating;
    }

    public String getCalibrationStatus() {
        return calStatus;
    }

    private void calibrationStep(Frame frame) {
        RawBall only = null;
        int count = 0;
        for (RawBall b : frame.balls) {
            if (b.type == calType) { count++; only = b; }
        }
        if (count != 1) {
            calStatus = count == 0 ? "no " + calType + " in view"
                    : count + " " + calType + " in view, need exactly 1";
            return;
        }
        if (only.cutOff) {
            calStatus = "ball is cut off by the image edge";
            return;
        }
        if (Math.abs(only.x - frame.width / 2.0) > CAL_MAX_OFF_CENTER * frame.width) {
            calStatus = "ball is not straight ahead: move it to the image center";
            return;
        }

        calSamples.add(new double[] { only.x, only.y, only.r });
        calStatus = String.format("collecting %d / %d", calSamples.size(), CAL_SAMPLES);
        if (calSamples.size() < Math.max(3, CAL_SAMPLES)) return;

        // Mean and spread. A moving ball or robot gives a bad answer, so insist it's steady.
        double[] mean = new double[3], sd = new double[3];
        for (double[] s : calSamples) for (int i = 0; i < 3; i++) mean[i] += s[i];
        for (int i = 0; i < 3; i++) mean[i] /= calSamples.size();
        for (double[] s : calSamples) for (int i = 0; i < 3; i++) sd[i] += sq(s[i] - mean[i]);
        for (int i = 0; i < 3; i++) sd[i] = Math.sqrt(sd[i] / calSamples.size());
        calSamples.clear();

        double jitter = Math.max(sd[0], Math.max(sd[1], sd[2]));
        if (jitter > CAL_MAX_JITTER_PX) {
            calStatus = String.format("ball or robot moving (%.1f px jitter), retrying", jitter);
            return;
        }

        double[] sol = Camera.solveCalibration(mean[0], mean[1], mean[2], calFloorDist,
                calType.radius(), frame.width, frame.height);
        if (sol == null || sol[1] < CAL_FOCAL_MIN || sol[1] > CAL_FOCAL_MAX
                || sol[0] < CAL_TILT_MIN || sol[0] > CAL_TILT_MAX) {
            calStatus = sol == null
                    ? "FAILED: geometry impossible. Check LENS_HEIGHT and the distance"
                    : String.format("FAILED: tilt %.1f / focal %.0f out of range. Check LENS_HEIGHT, distance, color",
                    sol[0], sol[1]);
            calibrating = false;
            return;
        }

        TILT_DEG = sol[0];
        FOCAL_PX = sol[1];
        tracks.clear();                 // every old position used the old numbers
        calibrating = false;
        calStatus = String.format("DONE: TILT_DEG = %.2f  FOCAL_PX = %.1f  (copy into VisionConstants)",
                TILT_DEG, FOCAL_PX);
    }

    // =====================================================================
    // ========================== TELEMETRY ================================
    // =====================================================================

    /** A compact vision block for the driver station / Panels. */
    public void addTelemetry(Telemetry t) {
        int p = 0, r = 0, b = 0;
        for (Ball ball : snapshot) {
            if (!ball.confirmed) continue;
            if (ball.type == BallType.POLLEN) p++;
            else if (ball.type == BallType.RED_NECTAR) r++;
            else b++;
        }
        t.addLine("-- vision --");
        t.addData("camera", "%s  %.0f fps  %s", isStreaming() ? "streaming" : "NOT streaming",
                fps, isFrameFresh() ? "" : "(no new frames!)");
        t.addData("exposure", exposureStatus);
        t.addData("pose", poseSupplier == null ? "NONE (robot frame only)" : "from odometry");
        t.addData("tracked", "pollen %d  red %d  blue %d", p, r, b);
        t.addData("calibration", calStatus);
        t.addData("tilt / focal", "%.2f deg / %.1f px", TILT_DEG, FOCAL_PX);

        t.addLine("type   r px   ratio   x      y    status");
        int shown = 0;
        for (Detection d : detections) {
            if (shown++ >= 8) break;
            t.addLine(String.format("%-4s %5.1f  %5.2f  %6.1f %6.1f  %s",
                    d.type.letter(), d.rPx, d.sizeRatio, d.x, d.y, d.status));
        }
    }

    // =====================================================================
    // ========================= FRAME HANDLING ============================
    // =====================================================================

    /** Turn one frame of raw blobs into accepted measurements and feed the tracker. */
    private void handleFrame(Frame frame, Pose framePose) {
        List<Detection> dets = new ArrayList<>();
        List<Measurement> meas = new ArrayList<>();

        for (RawBall b : frame.balls) {
            if (b.cutOff) {
                dets.add(new Detection(b.type, b.x, b.y, b.r, Double.NaN, Double.NaN, Double.NaN,
                        "cut off by edge"));
                continue;
            }
            double radius = b.type.radius();
            double[] p = Camera.project(b.x, b.y, radius, frame.width, frame.height);
            if (p == null) {
                dets.add(new Detection(b.type, b.x, b.y, b.r, Double.NaN, Double.NaN, Double.NaN,
                        "above horizon (check TILT_DEG)"));
                continue;
            }
            double rx = p[0], ry = p[1];
            double ratio = b.r / p[3];
            double sizeMin = b.type == BallType.POLLEN ? POLLEN_SIZE_MIN : SIZE_MIN;
            double[] world = Camera.robotToField(rx, ry, framePose);

            String why = "ok";
            if (ratio < sizeMin) why = "too small for that spot";
            else if (ratio > SIZE_MAX) why = "too big for that spot";
            else if (Math.hypot(rx, ry) > MAX_DETECT_DIST) why = "too far";
            else if (Camera.insideRobot(rx, ry)) why = "inside robot footprint";
            else if (USE_FIELD_BOUNDS && poseSupplier != null && !Camera.insideField(world[0], world[1]))
                why = "outside the field";

            dets.add(new Detection(b.type, b.x, b.y, b.r, rx, ry, ratio, why));
            if ("ok".equals(why)) {
                meas.add(new Measurement(b, world[0], world[1], Camera.measurementVariance(rx, ry, radius)));
            }
        }

        detections = Collections.unmodifiableList(dets);
        updateTracks(meas, framePose, frame);
    }

    // =====================================================================
    // ============================ TRACKER ================================
    // Static-ball Kalman filter per track, greedy nearest matching per type.
    // =====================================================================

    private void updateTracks(List<Measurement> meas, Pose framePose, Frame frame) {
        long t = frame.timeNanos;

        // 1. Predict: a ball might have been nudged since we last looked, so trust the old
        //    position a little less the longer it's been.
        for (Track tr : tracks) {
            double dt = clamp((t - tr.lastUpdateNanos) / 1e9, 0, 1.0);
            tr.var += BALL_DRIFT_IPS * BALL_DRIFT_IPS * dt;
            tr.lastUpdateNanos = t;
            tr.seenThisFrame = false;
        }

        // 2. Every (track, measurement) pair of the same type that is close enough to be the same ball
        List<double[]> pairs = new ArrayList<>();          // {distance, trackIndex, measIndex}
        for (int ti = 0; ti < tracks.size(); ti++) {
            Track tr = tracks.get(ti);
            for (int mi = 0; mi < meas.size(); mi++) {
                Measurement m = meas.get(mi);
                if (m.raw.type != tr.type) continue;
                double d = Math.hypot(tr.x - m.x, tr.y - m.y);
                double gate = GATE_MIN + GATE_SIGMAS * Math.sqrt(tr.var + m.var);
                if (d < gate) pairs.add(new double[] { d, ti, mi });
            }
        }
        Collections.sort(pairs, new Comparator<double[]>() {
            @Override public int compare(double[] a, double[] b) { return Double.compare(a[0], b[0]); }
        });

        // 3. Closest pairs first, each track and each measurement used once
        boolean[] tUsed = new boolean[tracks.size()];
        boolean[] mUsed = new boolean[meas.size()];
        for (double[] p : pairs) {
            int ti = (int) p[1], mi = (int) p[2];
            if (tUsed[ti] || mUsed[mi]) continue;
            tUsed[ti] = true;
            mUsed[mi] = true;
            tracks.get(ti).correct(meas.get(mi), t);
        }

        // 4. Unmatched tracks: only count a miss if the camera should have seen the ball
        for (int ti = 0; ti < tracks.size(); ti++) {
            if (tUsed[ti]) continue;
            Track tr = tracks.get(ti);
            if (shouldBeVisible(tr, framePose, frame)) tr.visibleMisses++;
        }

        // 5. New tracks for measurements nobody claimed
        for (int mi = 0; mi < meas.size(); mi++) {
            if (!mUsed[mi]) tracks.add(new Track(nextId++, meas.get(mi), t));
        }

        // 6. Two tracks on one ball (centers closer than real balls can be) -> merge
        mergeDuplicates();

        // 7. Drop tracks the camera keeps failing to see
        Iterator<Track> it = tracks.iterator();
        while (it.hasNext()) {
            Track tr = it.next();
            if (tr.visibleMisses > MAX_VISIBLE_MISSES || (!tr.confirmed && tr.visibleMisses >= 2)) {
                it.remove();
            }
        }
    }

    /** Time-based forgetting, runs even when no frames arrive. */
    private void expireTracks(long now) {
        Iterator<Track> it = tracks.iterator();
        while (it.hasNext()) {
            Track tr = it.next();
            double age = (now - tr.lastSeenNanos) / 1e9;
            if (age > MEMORY_S || (!tr.confirmed && age > TENTATIVE_LIFE_S)) it.remove();
        }
    }

    /**
     * Should the camera have seen this ball in this frame? True only if it would be fully
     * inside the image, in range, not inside our own robot, and not hidden behind a ball we
     * did see this frame. Balls under the intake, behind the robot or beside it don't lose
     * points for being invisible.
     */
    private boolean shouldBeVisible(Track tr, Pose framePose, Frame frame) {
        double[] r = Camera.fieldToRobot(tr.x, tr.y, framePose);
        if (Camera.insideRobot(r[0], r[1])) return false;
        if (Math.hypot(r[0], r[1]) > MAX_DETECT_DIST) return false;

        double[] pix = Camera.toPixel(r[0], r[1], tr.type.radius(), frame.width, frame.height);
        if (pix == null) return false;
        double px = pix[0], py = pix[1], pr = pix[2];
        if (pr < MIN_RADIUS_PX + 1) return false;

        double margin = EDGE_MARGIN_PX + 3;                  // a bit of slack: positions aren't exact
        double top = frame.height * ROI_TOP_FRAC;
        if (px - pr < margin || px + pr > frame.width - margin) return false;
        if (py + pr > frame.height - margin || py - pr < top + margin) return false;

        for (Track other : tracks) {
            if (other == tr || !other.seenThisFrame) continue;
            if (Math.hypot(other.px - px, other.py - py) < other.rPx + pr * 0.5) return false;   // hidden
        }
        return true;
    }

    private void mergeDuplicates() {
        for (int i = 0; i < tracks.size(); i++) {
            for (int j = tracks.size() - 1; j > i; j--) {
                Track a = tracks.get(i), b = tracks.get(j);
                if (a.type != b.type) continue;
                if (Math.hypot(a.x - b.x, a.y - b.y) > 1.2 * a.type.radius()) continue;
                Track keep = a.hits >= b.hits ? a : b;
                Track drop = keep == a ? b : a;
                keep.absorb(drop);
                tracks.remove(drop);
                if (drop == a) { i--; break; }
            }
        }
    }

    private void rebuildSnapshot(long now, Pose pose) {
        List<Ball> out = new ArrayList<>(tracks.size());
        for (Track tr : tracks) {
            double[] r = Camera.fieldToRobot(tr.x, tr.y, pose);
            out.add(new Ball(tr.id, tr.type, tr.x, tr.y, r[0], r[1], Math.sqrt(tr.var),
                    tr.confirmed, tr.hits, (now - tr.lastSeenNanos) / 1e9));
        }
        Collections.sort(out, new Comparator<Ball>() {
            @Override public int compare(Ball a, Ball b) { return Double.compare(a.distance(), b.distance()); }
        });
        snapshot = Collections.unmodifiableList(out);
    }

    private void pushLabels() {
        List<Label> labels = new ArrayList<>();
        for (Track tr : tracks) {
            if (!tr.seenThisFrame || !tr.confirmed) continue;
            labels.add(new Label(tr.px, tr.py, tr.rPx, "#" + tr.id, tr.type, tr.id == highlightId));
        }
        processor.labels = labels;
    }

    // =====================================================================
    // ========================== POSE HISTORY =============================
    // =====================================================================

    private Pose currentPose() {
        if (poseSupplier == null) return Pose.zero();
        Pose p = poseSupplier.get();
        return p == null ? Pose.zero() : p;
    }

    private void recordPose(long now, Pose pose) {
        poseHistory.addLast(new PoseStamp(now, pose));
        while (poseHistory.size() > 2 && now - poseHistory.peekFirst().nanos > POSE_HISTORY_NS) {
            poseHistory.removeFirst();
        }
    }

    /** Robot pose at time t (System.nanoTime base), interpolated from history. */
    private Pose poseAt(long t) {
        if (poseHistory.isEmpty()) return currentPose();
        PoseStamp prev = null;
        for (PoseStamp s : poseHistory) {
            if (s.nanos >= t) {
                if (prev == null) return s.pose;                     // older than our history
                double span = s.nanos - prev.nanos;
                double ratio = span <= 0 ? 1 : (t - prev.nanos) / span;
                return Pose.interpolate(prev.pose, s.pose, ratio);
            }
            prev = s;
        }
        return poseHistory.peekLast().pose;                          // newer than our history
    }

    // =====================================================================
    // ============================ CAMERA =================================
    // =====================================================================

    private void updateExposure(long now) {
        if (!MANUAL_EXPOSURE) {
            exposureStatus = "AUTO (colors may drift between venues)";
            return;
        }
        // Live edits from Panels: re-apply
        if (exposureStep >= 3 && (appliedExposureMs != EXPOSURE_MS || appliedGain != GAIN)) exposureStep = 1;
        if (exposureStep >= 3) return;

        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            exposureStatus = "waiting for camera";
            return;
        }
        if (now < exposureNextNanos) return;

        try {
            ExposureControl exp = portal.getCameraControl(ExposureControl.class);
            if (exposureStep == 0) {
                if (exp.getMode() != ExposureControl.Mode.Manual) exp.setMode(ExposureControl.Mode.Manual);
                exposureStep = 1;
                exposureNextNanos = now + 50_000_000L;              // let the mode change settle
            } else if (exposureStep == 1) {
                exp.setExposure(EXPOSURE_MS, TimeUnit.MILLISECONDS);
                exposureStep = 2;
                exposureNextNanos = now + 20_000_000L;
            } else {
                portal.getCameraControl(GainControl.class).setGain(GAIN);
                exposureStep = 3;
                appliedExposureMs = EXPOSURE_MS;
                appliedGain = GAIN;
                exposureStatus = String.format("manual %d ms, gain %d", EXPOSURE_MS, GAIN);
            }
        } catch (RuntimeException e) {
            exposureStep = 3;
            appliedExposureMs = EXPOSURE_MS;
            appliedGain = GAIN;
            exposureStatus = "FAILED: " + e.getMessage();
        }
    }

    private void updateFps(long frameTime) {
        if (lastFrameTime != 0 && frameTime > lastFrameTime) {
            double inst = 1e9 / (frameTime - lastFrameTime);
            fps = fps == 0 ? inst : 0.9 * fps + 0.1 * inst;
        }
        lastFrameTime = frameTime;
    }

    // =====================================================================
    // ============================ GEOMETRY ===============================
    // The ONLY place pixels become inches. Pure math on VisionConstants.
    // =====================================================================

    static final class Camera {
        private Camera() {}

        /**
         * Floor position of a ball whose image center is (px, py).
         * Returns {x right (in), y forward (in), slant distance from lens (in), expected radius (px)},
         * or null if that pixel looks at or above the horizon.
         */
        static double[] project(double px, double py, double radius, int w, int h) {
            double f = FOCAL_PX;
            double u = (px - w / 2.0) / f;                 // right +   (a tangent, not an angle)
            double v = (h / 2.0 - py) / f;                 // up +
            double a = Math.toRadians(TILT_DEG);

            // Ray through the pixel, in camera-mount axes {forward, right, up}
            double dF = Math.cos(a) - v * Math.sin(a);
            double dR = u;
            double dU = Math.sin(a) + v * Math.cos(a);
            if (dU > -1e-3) return null;                   // never reaches the floor

            double drop = LENS_HEIGHT - radius;            // lens down to ball-center height
            if (drop <= 0.1) return null;
            double t = drop / -dU;                         // > 0
            double slant = t * Math.sqrt(dF * dF + dR * dR + dU * dU);

            double[] robot = camToRobot(t * dF, t * dR);
            return new double[] { robot[0], robot[1], slant, expectedRadiusPx(slant, radius, u, v) };
        }

        /**
         * Inverse of project(): where a ball at robot-frame (x, y) would appear.
         * Returns {px, py, radius px}, or null if it's behind the camera.
         */
        static double[] toPixel(double x, double y, double radius, int w, int h) {
            double[] c = robotToCam(x, y);                 // {forward, right} from the lens
            double up = radius - LENS_HEIGHT;
            double a = Math.toRadians(TILT_DEG);
            double z = c[0] * Math.cos(a) + up * Math.sin(a);       // along the optical axis
            if (z < 0.5) return null;
            double yc = -c[0] * Math.sin(a) + up * Math.cos(a);     // image-up component
            double u = c[1] / z, v = yc / z;
            double slant = Math.sqrt(c[0] * c[0] + c[1] * c[1] + up * up);
            return new double[] { w / 2.0 + FOCAL_PX * u, h / 2.0 - FOCAL_PX * v,
                    expectedRadiusPx(slant, radius, u, v) };
        }

        /**
         * Radius (px) the distance transform should report for a real ball: the inscribed circle
         * of its image. Off-center balls image as ellipses whose short axis is stretched by
         * sqrt(1 + u^2 + v^2), so they look up to ~10% bigger at the frame corners.
         */
        static double expectedRadiusPx(double slant, double radius, double u, double v) {
            double s2 = slant * slant - radius * radius;
            return FOCAL_PX * radius / Math.sqrt(Math.max(s2, 1e-6)) * Math.sqrt(1 + u * u + v * v);
        }

        /**
         * One ball at a known floor distance -> {tiltDeg, focalPx}, or null.
         * Focal comes from the ball's SIZE, tilt from its HEIGHT in the image, using
         * sin a + v cos a = sqrt(1 + v^2) sin(a + atan v).
         */
        static double[] solveCalibration(double px, double py, double rPx, double floorDist,
                                         double radius, int w, int h) {
            double drop = LENS_HEIGHT - radius;
            if (drop <= 0.1 || floorDist <= 0 || rPx <= 0) return null;
            double slant = Math.hypot(floorDist, drop);
            double k = Math.sqrt(slant * slant - radius * radius) / radius;

            double f = rPx * k;                            // first guess: ball dead center
            for (int i = 0; i < 5; i++) {                  // undo the off-center stretch (converges fast)
                double u = (px - w / 2.0) / f, v = (h / 2.0 - py) / f;
                f = rPx * k / Math.sqrt(1 + u * u + v * v);
            }
            double u = (px - w / 2.0) / f, v = (h / 2.0 - py) / f;
            double arg = -(drop / slant) * Math.sqrt(1 + u * u + v * v) / Math.sqrt(1 + v * v);
            if (arg < -1 || arg > 1) return null;
            double tilt = Math.toDegrees(Math.asin(arg) - Math.atan(v));
            return new double[] { tilt, f };
        }

        /** Lens-relative {forward, right} on the floor -> robot frame {x right, y forward}. */
        static double[] camToRobot(double fwd, double right) {
            double s = Math.toRadians(CAM_YAW_DEG);
            double f = fwd * Math.cos(s) - right * Math.sin(s);
            double r = fwd * Math.sin(s) + right * Math.cos(s);
            return new double[] { r + CAM_RIGHT_OFFSET, f + CAM_FORWARD_OFFSET };
        }

        /** Robot frame {x right, y forward} -> lens-relative {forward, right}. */
        static double[] robotToCam(double x, double y) {
            double s = Math.toRadians(CAM_YAW_DEG);
            double dF = y - CAM_FORWARD_OFFSET, dR = x - CAM_RIGHT_OFFSET;
            return new double[] { dF * Math.cos(s) + dR * Math.sin(s), -dF * Math.sin(s) + dR * Math.cos(s) };
        }

        /** Robot frame (x right, y forward) -> field {x, y} for a robot at pose (Pedro, CCW heading). */
        static double[] robotToField(double x, double y, Pose pose) {
            double c = Math.cos(pose.heading()), s = Math.sin(pose.heading());
            return new double[] { pose.x() + y * c + x * s, pose.y() + y * s - x * c };
        }

        /** Field {x, y} -> robot frame {x right, y forward} for a robot at pose. */
        static double[] fieldToRobot(double fx, double fy, Pose pose) {
            double c = Math.cos(pose.heading()), s = Math.sin(pose.heading());
            double dx = fx - pose.x(), dy = fy - pose.y();
            return new double[] { dx * s - dy * c, dx * c + dy * s };
        }

        /**
         * How uncertain one reading is (in^2). Range error per pixel is (D^2 + h^2) / (h * f):
         * it grows with distance squared, so a 60 in reading is ~8x noisier than a 20 in one.
         */
        static double measurementVariance(double x, double y, double radius) {
            double[] c = robotToCam(x, y);
            double d = Math.hypot(c[0], c[1]);
            double h = Math.max(0.5, LENS_HEIGHT - radius);
            double sigma = Math.max(MEAS_SIGMA_MIN, PIXEL_NOISE_PX * (d * d + h * h) / (h * FOCAL_PX));
            return sigma * sigma;
        }

        /** A ball centered here would be inside / under our own robot (e.g. already intaken). */
        static boolean insideRobot(double x, double y) {
            return Math.abs(x) <= ROBOT_WIDTH / 2.0 && y <= FRONT_EXTENT && y >= -ROBOT_LENGTH / 2.0;
        }

        static boolean insideField(double fx, double fy) {
            return fx >= 0 && fx <= FIELD_SIZE && fy >= 0 && fy <= FIELD_SIZE;
        }
    }

    // =====================================================================
    // ======================= VISION PROCESSOR ============================
    // Runs on the camera thread. Produces pixels only; no inches here.
    // =====================================================================

    static final class Processor implements VisionProcessor {
        private int width = IMG_W, height = IMG_H;
        private final Mat ycc = new Mat(), mask = new Mat(), yellowRaw = new Mat(),
                dist = new Mat(), inv = new Mat();
        private Mat openKernel, closeKernel;
        private int openKernelSize = -1, closeKernelSize = -1;
        private long seq = 0;

        volatile Frame latest = new Frame(Collections.<RawBall>emptyList(), 0, -1, IMG_W, IMG_H);
        volatile List<Label> labels = Collections.emptyList();

        private final Paint ringPaint = new Paint();
        private final Paint targetPaint = new Paint();
        private final Paint textPaint = new Paint();
        private static final Scalar ZERO = new Scalar(0);

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            this.width = width;
            this.height = height;
            rebuildKernels();
            ringPaint.setStyle(Paint.Style.STROKE);
            targetPaint.setStyle(Paint.Style.STROKE);
            targetPaint.setColor(Color.GREEN);
            textPaint.setStyle(Paint.Style.FILL);
            textPaint.setTextAlign(Paint.Align.CENTER);
            textPaint.setAntiAlias(true);
        }

        /** OPEN_SIZE / CLOSE_SIZE are live-editable, so the kernels follow them. */
        private void rebuildKernels() {
            int o = Math.max(1, OPEN_SIZE);
            if (o != openKernelSize) {
                openKernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new org.opencv.core.Size(o, o));
                openKernelSize = o;
            }
            int c = Math.max(1, CLOSE_SIZE);
            if (c != closeKernelSize) {
                closeKernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new org.opencv.core.Size(c, c));
                closeKernelSize = c;
            }
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            rebuildKernels();

            // Convert and blur once, reuse for every color. The SDK hands us RGB (not BGR).
            Imgproc.cvtColor(frame, ycc, Imgproc.COLOR_RGB2YCrCb);
            if (BLUR > 1) {
                int b = BLUR | 1;
                Imgproc.GaussianBlur(ycc, ycc, new org.opencv.core.Size(b, b), 0);
            }
            int top = (int) clamp(height * ROI_TOP_FRAC, 0, height - 1);

            List<RawBall> found = new ArrayList<>();
            boolean needYellow = DETECT_POLLEN || (DETECT_RED && RED_EXCLUDES_YELLOW);
            if (needYellow) {
                Core.inRange(ycc, new Scalar(P_Y_MIN, P_CR_MIN, P_CB_MIN),
                        new Scalar(P_Y_MAX, P_CR_MAX, P_CB_MAX), yellowRaw);
            }
            if (DETECT_POLLEN) {
                yellowRaw.copyTo(mask);
                findBalls(BallType.POLLEN, 1, top, frame, found);
            }
            if (DETECT_RED) {
                Core.inRange(ycc, new Scalar(R_Y_MIN, R_CR_MIN, R_CB_MIN),
                        new Scalar(R_Y_MAX, R_CR_MAX, R_CB_MAX), mask);
                if (RED_EXCLUDES_YELLOW) Core.subtract(mask, yellowRaw, mask);   // 255-255 = 0
                findBalls(BallType.RED_NECTAR, 2, top, frame, found);
            }
            if (DETECT_BLUE) {
                Core.inRange(ycc, new Scalar(B_Y_MIN, B_CR_MIN, B_CB_MIN),
                        new Scalar(B_Y_MAX, B_CR_MAX, B_CB_MAX), mask);
                findBalls(BallType.BLUE_NECTAR, 3, top, frame, found);
            }

            // Frame time on the System.nanoTime() clock, so it lines up with the pose history.
            long now = System.nanoTime();
            long t = (captureTimeNanos > 0 && Math.abs(now - captureTimeNanos) < 1_000_000_000L)
                    ? captureTimeNanos
                    : now - (long) (FRAME_LATENCY_S * 1e9);

            Frame f = new Frame(Collections.unmodifiableList(found), t, ++seq, width, height);
            latest = f;
            return f;                                     // arrives in onDrawFrame as userContext
        }

        /** Cleans the current mask and pulls one circle per ball out of it. */
        private void findBalls(BallType type, int viewIndex, int top, Mat frame, List<RawBall> out) {
            if (top > 0) mask.submat(0, top, 0, width).setTo(ZERO);
            if (OPEN_SIZE > 1) Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, openKernel);
            if (CLOSE_SIZE > 1) Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, closeKernel);

            if (MASK_VIEW == viewIndex) {                   // preview: only this color's pixels
                Core.bitwise_not(mask, inv);
                frame.setTo(new Scalar(0, 0, 0, 255), inv);
            }

            // Every white pixel becomes its distance to the nearest black pixel
            Imgproc.distanceTransform(mask, dist, Imgproc.DIST_L2, 5);

            List<RawBall> mine = new ArrayList<>();
            for (int i = 0; i < 30 && mine.size() < MAX_BALLS_PER_COLOR; i++) {
                Core.MinMaxLocResult mm = Core.minMaxLoc(dist);
                double r = mm.maxVal;
                if (r < MIN_RADIUS_PX) break;
                Point p = mm.maxLoc;
                int ix = (int) p.x, iy = (int) p.y;
                double cx = p.x + subPixel(ix, iy, true, r);
                double cy = p.y + subPixel(ix, iy, false, r);

                boolean ok = true;
                for (RawBall b : mine) {
                    double d = Math.hypot(cx - b.x, cy - b.y);
                    if (d < OVERLAP_FRAC * (r + b.r)) { ok = false; break; }              // same ball twice
                    if (d < (r + b.r) && r < SLIVER_RATIO * b.r) { ok = false; break; }    // leftover sliver
                }
                boolean cut = cx - r < EDGE_MARGIN_PX || cx + r > width - EDGE_MARGIN_PX
                        || cy + r > height - EDGE_MARGIN_PX || cy - r < top + EDGE_MARGIN_PX;
                if (ok) mine.add(new RawBall(type, cx, cy, r, cut));

                Imgproc.circle(dist, p, (int) Math.ceil(r), ZERO, -1);                   // erase, look again
            }
            out.addAll(mine);
        }

        /** Parabola through the peak and its two neighbors -> sub-pixel offset. */
        private double subPixel(int x, int y, boolean horizontal, double c) {
            double l, rr;
            if (horizontal) {
                if (x <= 0 || x >= width - 1) return 0;
                l = dist.get(y, x - 1)[0];
                rr = dist.get(y, x + 1)[0];
            } else {
                if (y <= 0 || y >= height - 1) return 0;
                l = dist.get(y - 1, x)[0];
                rr = dist.get(y + 1, x)[0];
            }
            double den = l - 2 * c + rr;
            if (den >= 0) return 0;
            return clamp(0.5 * (l - rr) / den, -0.5, 0.5);
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
            float s = scaleBmpPxToCanvasPx;
            ringPaint.setStrokeWidth(2 * scaleCanvasDensity);
            targetPaint.setStrokeWidth(4 * scaleCanvasDensity);
            textPaint.setTextSize(14 * scaleCanvasDensity);

            if (ROI_TOP_FRAC > 0) {
                ringPaint.setColor(Color.WHITE);
                float topY = (float) (height * ROI_TOP_FRAC) * s;
                canvas.drawLine(0, topY, width * s, topY, ringPaint);
            }

            if (userContext instanceof Frame) {
                for (RawBall b : ((Frame) userContext).balls) {
                    ringPaint.setColor(b.cutOff ? Color.GRAY : colorOf(b.type));
                    canvas.drawCircle((float) b.x * s, (float) b.y * s, (float) b.r * s, ringPaint);
                }
            }
            for (Label l : labels) {
                if (l.highlight) canvas.drawCircle(l.px * s, l.py * s, (l.r + 3) * s, targetPaint);
                textPaint.setColor(l.highlight ? Color.GREEN : colorOf(l.type));
                canvas.drawText(l.text, l.px * s, (l.py - l.r) * s - 5 * scaleCanvasDensity, textPaint);
            }
        }

        private static int colorOf(BallType t) {
            return t == BallType.POLLEN ? Color.YELLOW : t == BallType.RED_NECTAR ? Color.RED : Color.rgb(64, 160, 255);
        }
    }

    // =====================================================================
    // ======================= SMALL DATA CLASSES ==========================
    // =====================================================================

    /** One circle out of the distance transform. Pixels only. */
    static final class RawBall {
        final BallType type;
        final double x, y, r;
        final boolean cutOff;
        RawBall(BallType type, double x, double y, double r, boolean cutOff) {
            this.type = type; this.x = x; this.y = y; this.r = r; this.cutOff = cutOff;
        }
    }

    static final class Frame {
        final List<RawBall> balls;
        final long timeNanos;          // capture time, System.nanoTime() clock
        final long seq;                // increments every frame; how we spot a new one
        final int width, height;
        Frame(List<RawBall> balls, long timeNanos, long seq, int width, int height) {
            this.balls = balls; this.timeNanos = timeNanos; this.seq = seq;
            this.width = width; this.height = height;
        }
    }

    private static final class Measurement {
        final RawBall raw;
        final double x, y, var;        // field position (in) and its variance (in^2)
        Measurement(RawBall raw, double x, double y, double var) {
            this.raw = raw; this.x = x; this.y = y; this.var = var;
        }
    }

    private static final class Track {
        final int id;
        final BallType type;
        double x, y, var;              // field position estimate and its variance
        int hits = 1, visibleMisses = 0;
        boolean confirmed;
        long lastSeenNanos, lastUpdateNanos;
        boolean seenThisFrame = true;
        double px, py, rPx;            // last place in the image, for drawing

        Track(int id, Measurement m, long t) {
            this.id = id;
            this.type = m.raw.type;
            this.x = m.x;
            this.y = m.y;
            this.var = m.var;
            this.lastSeenNanos = t;
            this.lastUpdateNanos = t;
            this.confirmed = CONFIRM_HITS <= 1;
            this.px = m.raw.x; this.py = m.raw.y; this.rPx = m.raw.r;
        }

        /** Blend a new reading in, weighted by how much we trust each (Kalman update). */
        void correct(Measurement m, long t) {
            double k = var / (var + m.var);
            x += k * (m.x - x);
            y += k * (m.y - y);
            var *= (1 - k);
            hits++;
            visibleMisses = 0;
            lastSeenNanos = t;
            seenThisFrame = true;
            px = m.raw.x; py = m.raw.y; rPx = m.raw.r;
            if (hits >= CONFIRM_HITS) confirmed = true;
        }

        /** Fold a duplicate track into this one. */
        void absorb(Track o) {
            double k = var / (var + o.var);
            x += k * (o.x - x);
            y += k * (o.y - y);
            var *= (1 - k);
            hits += o.hits;
            confirmed |= o.confirmed;
            if (o.lastSeenNanos > lastSeenNanos) {
                lastSeenNanos = o.lastSeenNanos;
                seenThisFrame |= o.seenThisFrame;
                px = o.px; py = o.py; rPx = o.rPx;
            }
            visibleMisses = Math.min(visibleMisses, o.visibleMisses);
        }
    }

    private static final class Label {
        final float px, py, r;
        final String text;
        final BallType type;
        final boolean highlight;
        Label(double px, double py, double r, String text, BallType type, boolean highlight) {
            this.px = (float) px; this.py = (float) py; this.r = (float) r;
            this.text = text; this.type = type; this.highlight = highlight;
        }
    }

    private static final class PoseStamp {
        final long nanos;
        final Pose pose;
        PoseStamp(long nanos, Pose pose) { this.nanos = nanos; this.pose = pose; }
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double sq(double v) {
        return v * v;
    }
}
