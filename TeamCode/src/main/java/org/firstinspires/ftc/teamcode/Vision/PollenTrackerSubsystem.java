package org.firstinspires.ftc.teamcode.Vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.camerastream.PanelsCameraStream;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
import java.util.Deque;
import java.util.Iterator;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.function.Supplier;

/**
 * The PollenTracker / blobTrack vision pipeline as an FTCLib subsystem.
 *
 * SAME DETECTION AS BEFORE
 *   YCrCb color mask -> open/close cleanup -> distance transform -> one circle per ball
 *   (touching balls each get their own peak) -> project onto the floor -> size check
 *   (measured radius vs. the radius a real ball would have there) -> alpha-beta tracker.
 *   Groups and their score are blobTrack's: score = ballCount - DIST_PENALTY * distance.
 *
 * WHAT'S DIFFERENT FROM THE OPMODE VERSION
 *   - Everything runs in periodic(), once per CommandScheduler.run(). Nothing blocks or sleeps.
 *   - Give it the robot pose (setPoseSupplier(drive::getPose)) and balls are tracked in FIELD
 *     coordinates, using the pose from when each camera frame was TAKEN. A ball then stays put
 *     while the robot drives and turns, and every ball has both robot-relative and field
 *     coordinates, which is what Pedro Pathing needs. Without a pose it works robot-relative,
 *     exactly like the old OpMode.
 *   - Other code gets read-only snapshots (TrackedBall / TrackedGroup), never the live tracks.
 *   - Fixes: the top image edge now counts as "cut off"; balls near the image edges are no
 *     longer judged too big; calibration only runs when asked and its answer is sanity-checked;
 *     every value says whether the ball was actually seen in the latest frame.
 *
 * NECTAR (red / blue, 3.6 in) is found by the same pipeline with its own color ranges, the way
 * NectarTracker did it. Its getters and the path checks (isPathBlocked, requiresCrossField)
 * are here for later; nothing uses them yet.
 *
 * Robot frame: x = right, y = forward, inches from robot center.
 * Field frame: Pedro's (inches, heading counter-clockwise in radians).
 */
@Configurable
public class PollenTrackerSubsystem extends SubsystemBase {

    // =====================================================================
    // ============================ TUNING =================================
    // =====================================================================

    // ---- Camera / robot measurements (tape measure) ----
    public static String WEBCAM_NAME = "Webcam 1";
    public static double LENS_HEIGHT = 9.1;          // floor to lens center (in)  <- MEASURE
    public static double CAM_FORWARD_OFFSET = 0;     // lens forward of robot center (in), negative = behind
    public static double CAM_RIGHT_OFFSET = 2.25;    // lens right of robot center (in)

    // ---- From calibration (copy the printed values here) ----
    public static double TILT_DEG = -25.59;          // negative = tilted down
    public static double FOCAL_PX = 427.2;

    // ---- Calibration (only runs when startCalibration() is called) ----
    public static double CAL_FLOOR_DIST = 30;        // under-lens point to ball's floor contact (in)
    public static int CAL_SAMPLES = 10;
    public static double CAL_TILT_MIN = -70, CAL_TILT_MAX = 20;      // answers outside these are refused
    public static double CAL_FOCAL_MIN = 120, CAL_FOCAL_MAX = 900;

    // ---- Pollen color (YCrCb). 128 = neutral on Cr and Cb ----
    public static int Y_MIN  = 32,  Y_MAX  = 255;    // brightness
    public static int CR_MIN = 128, CR_MAX = 185;    // red vs green
    public static int CB_MIN = 0,   CB_MAX = 105;    // blue vs yellow  <- main knob

    // ---- Nectar colors (from NectarTracker; tune on the real balls) ----
    public static boolean DETECT_RED_NECTAR = true;  // each color costs camera-thread time
    public static boolean DETECT_BLUE_NECTAR = true;
    public static int R_Y_MIN = 32,   R_Y_MAX = 255;
    public static int R_CR_MIN = 150, R_CR_MAX = 255;   // red = high Cr
    public static int R_CB_MIN = 0,   R_CB_MAX = 135;
    public static int B_Y_MIN = 32,   B_Y_MAX = 255;
    public static int B_CR_MIN = 0,   B_CR_MAX = 125;
    public static int B_CB_MIN = 145, B_CB_MAX = 255;   // blue = high Cb
    public static boolean RED_EXCLUDES_YELLOW = true;    // orange-ish Pollen pixels can't count as red

    // ---- Mask cleanup ----
    public static int BLUR = 5;                      // odd number
    public static int OPEN_SIZE = 3;                 // removes specks / thin strips
    public static int CLOSE_SIZE = 5;                // fills the ball's holes
    public static boolean SHOW_MASK = false;         // preview shows only pixels that pass MASK_COLOR
    public static String MASK_COLOR = "P";           // "P" Pollen, "R" red Nectar, "B" blue Nectar

    // ---- Detection ----
    public static double ROI_TOP_FRAC = 0;           // ignore image above this fraction; 0 = whole frame
    public static double MIN_RADIUS = 4;             // px
    public static double OVERLAP_FRAC = 0.5;         // higher = one ball less likely to get 2 circles
    public static double SLIVER_RATIO = 0.4;         // reject small leftovers touching a bigger ball
    public static int EDGE_MARGIN = 2;               // reject balls cut off by any image edge
    public static int MAX_BALLS = 6;                 // Pollen per frame
    public static int MAX_NECTAR_BALLS = 8;          // per Nectar color per frame

    // ---- Size check: measured radius / expected radius ----
    public static double SIZE_MIN = 0.6;
    public static double SIZE_MAX = 1.5;

    // ---- Tracker ----
    public static double GATE = 4.0;                 // in, max jump to count as the same ball
    public static double ALPHA = 0.5;                // position smoothing (lower = smoother, laggier)
    public static double BETA = 0.0;                 // velocity smoothing. 0: balls sit still in the field
    public static int CONFIRM_HITS = 3;              // frames before a ball is trusted
    public static int MAX_MISSES = 5;                // frames a ball can vanish before being dropped

    // ---- Groups (blobTrack's target selection) ----
    public static double CLUSTER_RADIUS = 9.0;       // balls within this of each other are one group (in)
    public static double DIST_PENALTY = 0.03;        // score = count - DIST_PENALTY * distance
    public static boolean AIM_AT_CENTROID = true;    // aim at the middle of the group, else its nearest ball

    // ---- Robot half-widths for isPathBlocked() (center line to each outer edge) ----
    public static double ROBOT_HALF_WIDTH_LEFT = 9.0;    // <- MEASURE
    public static double ROBOT_HALF_WIDTH_RIGHT = 9.0;   // <- MEASURE

    // ---- Camera controls ----
    public static boolean MANUAL_EXPOSURE = true;
    public static long EXPOSURE_MS = 10;
    public static int GAIN = 100;
    public static boolean STREAM_TO_PANELS = true;   // camera preview in Panels (read at construction)

    // =====================================================================

    /** Real ball radii (in), from the BIOBUZZ manual: Pollen 2.8 in, Nectar 3.6 in diameter. */
    public static final double POLLEN_RADIUS = 1.4;
    public static final double NECTAR_RADIUS = 1.8;

    private static final int IMG_W = 320;
    private static final int IMG_H = 240;
    private static final int PANELS_STREAM_FPS = 10;
    private static final long FRESH_FRAME_NS = 300_000_000L;      // isFrameFresh() window
    private static final long POSE_HISTORY_NS = 1_000_000_000L;   // how far back poses are kept

    // =====================================================================
    // ========================= PUBLIC TYPES ==============================
    // =====================================================================

    /** What a ball is. */
    public enum BallType {
        POLLEN(POLLEN_RADIUS, "P"),
        RED_NECTAR(NECTAR_RADIUS, "R"),
        BLUE_NECTAR(NECTAR_RADIUS, "B");

        /** Real radius (in). */
        public final double radius;
        /** One letter for telemetry and the preview. */
        public final String letter;

        BallType(double radius, String letter) {
            this.radius = radius;
            this.letter = letter;
        }

        public boolean isNectar() {
            return this != POLLEN;
        }
    }

    /** One confirmed ball, frozen at the moment it was handed out. Safe to keep. */
    public static final class TrackedBall {
        /** Stays the same while the ball keeps being tracked. */
        public final int id;
        public final BallType type;
        /** Robot frame right now (in): x = right of robot center, y = forward. */
        public final double x, y;
        /** Field frame (in), Pedro coordinates. Equal to x, y when there is no pose supplier. */
        public final double fieldX, fieldY;
        /** Number of frames it has been matched in. */
        public final int hits;
        /** True if the camera saw it in the latest frame; false = last known position. */
        public final boolean seenThisFrame;
        /** Seconds since the camera last saw it. */
        public final double ageS;

        TrackedBall(int id, BallType type, double x, double y, double fieldX, double fieldY,
                    int hits, boolean seenThisFrame, double ageS) {
            this.id = id; this.type = type; this.x = x; this.y = y;
            this.fieldX = fieldX; this.fieldY = fieldY;
            this.hits = hits; this.seenThisFrame = seenThisFrame; this.ageS = ageS;
        }

        /** Floor distance from robot center (in). */
        public double distance() {
            return Math.hypot(x, y);
        }

        /** Degrees off the robot's nose: 0 = dead ahead, + = to the right. */
        public double headingDeg() {
            return Math.toDegrees(Math.atan2(x, y));
        }

        /** The ball's field position as a Pedro Pose (heading 0). */
        public Pose fieldPose() {
            return new Pose(fieldX, fieldY, 0);
        }
    }

    /** A clump of touching Pollen balls and where to drive for it (blobTrack's Group). */
    public static final class TrackedGroup {
        public final List<TrackedBall> balls;    // nearest first
        public final int count;
        /** Aim point, robot frame (in): centroid, or the nearest ball if AIM_AT_CENTROID is off. */
        public final double x, y;
        /** Aim point, field frame (in). */
        public final double fieldX, fieldY;
        /** count - DIST_PENALTY * distance. Higher is better. */
        public final double score;
        /** Id of the group's nearest ball; identifies the group frame to frame. */
        public final int leadId;
        /** True if any ball of the group was seen in the latest frame. */
        public final boolean seenThisFrame;

        TrackedGroup(List<TrackedBall> balls, double x, double y, double fieldX, double fieldY,
                     double score, int leadId, boolean seenThisFrame) {
            this.balls = Collections.unmodifiableList(balls);
            this.count = balls.size();
            this.x = x; this.y = y; this.fieldX = fieldX; this.fieldY = fieldY;
            this.score = score; this.leadId = leadId; this.seenThisFrame = seenThisFrame;
        }

        public double distance() {
            return Math.hypot(x, y);
        }

        /** Degrees off the robot's nose: + = to the right. */
        public double headingDeg() {
            return Math.toDegrees(Math.atan2(x, y));
        }

        public boolean contains(int id) {
            for (TrackedBall b : balls) if (b.id == id) return true;
            return false;
        }

        public Pose fieldPose() {
            return new Pose(fieldX, fieldY, 0);
        }
    }

    /** Returned by isPathBlocked(). Same fields as blobTrack's version. */
    public static class PathBlockResult {
        /** True when at least one obstacle occupies the corridor to the target. */
        public final boolean blocked;
        /**
         * Heading change in degrees that steers the robot clear of the deepest blocker.
         * Positive = turn RIGHT, negative = turn LEFT. Zero when not blocked.
         */
        public final double suggestedTurnDegrees;
        /** Obstacles between robot and target, left / right of the path. */
        public final int obstaclesLeft;
        public final int obstaclesRight;

        PathBlockResult(boolean blocked, double suggestedTurnDeg, int left, int right) {
            this.blocked = blocked;
            this.suggestedTurnDegrees = suggestedTurnDeg;
            this.obstaclesLeft = left;
            this.obstaclesRight = right;
        }
    }

    // =====================================================================
    // ============================== STATE ================================
    // =====================================================================

    private final TelemetryManager telemetry;          // may be null
    private final Processor processor = new Processor();
    private final VisionPortal portal;
    private boolean streaming = false;

    private Supplier<Pose> poseSupplier = null;
    private final Deque<PoseStamp> poseHistory = new ArrayDeque<>();

    private final Tracker tracker = new Tracker();
    private long lastSeq = -1;
    private long lastFrameProcessedNanos = 0;

    // Exposure is set in small steps across loops (a subsystem must never sleep)
    private int exposureStep = 0;
    private long exposureNextNanos = 0;
    private long appliedExposureMs = -1;
    private int appliedGain = -1;
    private String exposureStatus = "starting";

    // Calibration
    private boolean calibrating = false;
    private BallType calType = BallType.POLLEN;
    private final List<double[]> calSamples = new ArrayList<>();
    private String calStatus = "not run";

    // What the getters hand out; rebuilt every periodic()
    private List<TrackedBall> pollen = Collections.emptyList();
    private List<TrackedBall> nectar = Collections.emptyList();
    private List<TrackedGroup> groups = Collections.emptyList();

    // =====================================================================
    // ========================= CONSTRUCTION ==============================
    // =====================================================================

    /**
     * Opens the webcam named WEBCAM_NAME and starts processing. FTCLib registers the subsystem
     * automatically, so periodic() runs on every CommandScheduler.run(). Create it AFTER the
     * drive subsystem so the pose is already updated when vision runs.
     *
     * @param telemetry Panels telemetry for a few status lines; may be null
     */
    public PollenTrackerSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetry = telemetry;
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .setCameraResolution(new Size(IMG_W, IMG_H))
                .addProcessor(processor)
                .build();
        if (STREAM_TO_PANELS) {
            PanelsCameraStream.INSTANCE.startStream(portal, PANELS_STREAM_FPS);
            streaming = true;
        }
    }

    /**
     * Where the robot is. Pass drive::getPose (Pedro Pose). With it, balls are tracked in field
     * coordinates; without it, robot-relative (only right while the robot holds still).
     * Clears all tracks, since old positions were in the other frame.
     */
    public void setPoseSupplier(Supplier<Pose> supplier) {
        poseSupplier = supplier;
        poseHistory.clear();
        tracker.reset();
    }

    public boolean hasPoseSupplier() {
        return poseSupplier != null;
    }

    /** Stops the camera. Call once when the OpMode ends (in a finally block). */
    public void close() {
        if (streaming) {
            PanelsCameraStream.INSTANCE.stopStream();
            streaming = false;
        }
        portal.close();
    }

    // =====================================================================
    // ============================ PERIODIC ===============================
    // =====================================================================

    @Override
    public void periodic() {
        long now = System.nanoTime();
        Pose pose = currentPose();
        if (pose != null) recordPose(now, pose);
        updateExposure(now);

        Frame frame = processor.latest;
        if (frame.seq != lastSeq) {                      // each camera frame is handled once
            lastSeq = frame.seq;
            lastFrameProcessedNanos = now;
            if (calibrating) {
                calibrationStep(frame);
            } else {
                trackFrame(frame, pose == null ? null : poseAt(poseHistory, frame.timeNanos, pose));
            }
        }

        rebuildSnapshot(now, pose);
        pushLabels();
        writeTelemetry();
    }

    // =====================================================================
    // ========================= POLLEN GETTERS ============================
    // =====================================================================

    /** Confirmed Pollen, nearest first. */
    public List<TrackedBall> getBalls() {
        return pollen;
    }

    /** Nearest confirmed Pollen (PollenTracker's target), or null. */
    public TrackedBall getNearestBall() {
        return pollen.isEmpty() ? null : pollen.get(0);
    }

    /** Pollen groups, highest score first. */
    public List<TrackedGroup> getGroups() {
        return groups;
    }

    /** Best-scoring Pollen group (blobTrack's target), or null. */
    public TrackedGroup getTargetGroup() {
        return groups.isEmpty() ? null : groups.get(0);
    }

    /** Confirmed Pollen ball with this id, or null once it is no longer tracked. */
    public TrackedBall getBall(int id) {
        for (TrackedBall b : pollen) if (b.id == id) return b;
        return null;
    }

    /** The Pollen group that contains this ball id right now, or null. */
    public TrackedGroup getGroupContaining(int id) {
        if (id < 0) return null;
        for (TrackedGroup g : groups) if (g.contains(id)) return g;
        return null;
    }

    /** True if at least one confirmed Pollen is tracked. */
    public boolean hasTarget() {
        return !pollen.isEmpty();
    }

    // =====================================================================
    // ========================= NECTAR GETTERS ============================
    // Same pipeline and tracker as Pollen. Not used by anything yet.
    // =====================================================================

    /** Confirmed Nectar of both colors, nearest first. */
    public List<TrackedBall> getNectar() {
        return nectar;
    }

    /** Confirmed Nectar of one color (RED_NECTAR or BLUE_NECTAR), nearest first. */
    public List<TrackedBall> getNectar(BallType color) {
        List<TrackedBall> out = new ArrayList<>();
        for (TrackedBall b : nectar) if (b.type == color) out.add(b);
        return Collections.unmodifiableList(out);
    }

    /** Nearest confirmed Nectar of either color, or null. */
    public TrackedBall getNearestNectar() {
        return nectar.isEmpty() ? null : nectar.get(0);
    }

    /** Nearest confirmed Nectar of one color, or null. */
    public TrackedBall getNearestNectar(BallType color) {
        for (TrackedBall b : nectar) if (b.type == color) return b;
        return null;
    }

    /** Confirmed Nectar with this id, or null once it is no longer tracked. */
    public TrackedBall getNectarBall(int id) {
        for (TrackedBall b : nectar) if (b.id == id) return b;
        return null;
    }

    /**
     * Would driving straight at (targetX, targetY) (robot frame) run into tracked Nectar?
     * Uses every confirmed Nectar, and counts a ball as blocking when its EDGE (not just its
     * center) is inside the robot's width.
     */
    public PathBlockResult isPathBlockedByNectar(double targetX, double targetY) {
        List<double[]> obstacles = new ArrayList<>();
        for (TrackedBall b : nectar) obstacles.add(new double[] { b.x, b.y });
        return isPathBlocked(targetX, targetY, obstacles, NECTAR_RADIUS);
    }

    /** isPathBlockedByNectar() for a Pollen group's aim point. */
    public PathBlockResult isPathBlockedByNectar(TrackedGroup group) {
        return isPathBlockedByNectar(group.x, group.y);
    }

    /**
     * blobTrack's isPathBlocked, unchanged: is any obstacle CENTER inside the straight corridor
     * (ROBOT_HALF_WIDTH_LEFT / RIGHT) from the robot to the target? Everything robot frame (in).
     *
     * @param obstacles each {x, y}, robot frame
     */
    public static PathBlockResult isPathBlocked(double targetX, double targetY, List<double[]> obstacles) {
        return isPathBlocked(targetX, targetY, obstacles, 0);
    }

    /**
     * Same as above, but every obstacle is a ball of obstacleRadius, so the corridor is widened
     * by that much on both sides. obstacleRadius = 0 gives exactly blobTrack's result.
     */
    public static PathBlockResult isPathBlocked(double targetX, double targetY,
                                                List<double[]> obstacles, double obstacleRadius) {
        double pathLen = Math.hypot(targetX, targetY);
        if (pathLen < 1e-3) return new PathBlockResult(false, 0, 0, 0);

        double ux = targetX / pathLen, uy = targetY / pathLen;   // along the path
        double halfLeft = ROBOT_HALF_WIDTH_LEFT + obstacleRadius;
        double halfRight = ROBOT_HALF_WIDTH_RIGHT + obstacleRadius;

        boolean blocked = false;
        int leftCount = 0, rightCount = 0;
        double worstPerp = 0, worstAlong = pathLen * 0.5;
        boolean foundBlocker = false;

        for (double[] obs : obstacles) {
            if (obs == null || obs.length < 2) continue;
            double ox = obs[0], oy = obs[1];

            double along = ox * ux + oy * uy;                    // must be between robot and target
            if (along < 0 || along > pathLen) continue;

            double perp = -ox * uy + oy * ux;                    // + = obstacle is LEFT of the path
            if (perp >= 0) leftCount++;
            else rightCount++;

            boolean inFootprint = perp >= 0 ? perp < halfLeft : -perp < halfRight;
            if (inFootprint) {
                blocked = true;
                if (!foundBlocker || Math.abs(perp) < Math.abs(worstPerp)) {   // deepest one
                    worstPerp = perp;
                    worstAlong = along;
                    foundBlocker = true;
                }
            }
        }

        double suggestedTurnDeg = 0;
        if (blocked) {
            double edgeOffset = worstPerp >= 0 ? halfLeft : halfRight;
            // Shift the path center clear of the blocker, plus a 20% margin
            double clearance = (edgeOffset - Math.abs(worstPerp)) + edgeOffset * 0.2;
            double turnMag = Math.toDegrees(Math.atan2(clearance, worstAlong));
            suggestedTurnDeg = worstPerp >= 0 ? turnMag : -turnMag;   // blocker LEFT -> turn RIGHT
        }
        return new PathBlockResult(blocked, suggestedTurnDeg, leftCount, rightCount);
    }

    /**
     * blobTrack's requiresCrossField, unchanged: is a robot-relative target on the other side of
     * the field line boundaryFieldY? Heading in degrees, counter-clockwise from field +X.
     */
    public static boolean requiresCrossField(double targetX, double targetY,
                                             double robotFieldX, double robotFieldY, double robotHeadingDeg,
                                             double boundaryFieldY, boolean robotBelowBoundary) {
        double heading = Math.toRadians(robotHeadingDeg);
        // Robot forward in the field = (cos, sin); robot right = (sin, -cos). Only Y matters here.
        double fieldTargetY = robotFieldY + targetY * Math.sin(heading) - targetX * Math.cos(heading);
        return robotBelowBoundary ? fieldTargetY > boundaryFieldY : fieldTargetY < boundaryFieldY;
    }

    // =====================================================================
    // ===================== CALIBRATION / STATUS ==========================
    // =====================================================================

    /** Calibrate on ONE Pollen straight ahead, CAL_FLOOR_DIST along the floor from under the lens. */
    public void startCalibration() {
        startCalibration(BallType.POLLEN);
    }

    /** Calibrate on one ball of this type (a Nectar works too: its size is known). */
    public void startCalibration(BallType type) {
        calibrating = true;
        calType = type;
        calSamples.clear();
        calStatus = String.format("put ONE %s straight ahead, %.1f in along the floor",
                type == BallType.POLLEN ? "Pollen" : "Nectar", CAL_FLOOR_DIST);
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

    public double getTiltDeg() {
        return TILT_DEG;
    }

    public double getFocalPx() {
        return FOCAL_PX;
    }

    /** True if a camera frame was processed in the last 0.3 s. */
    public boolean isFrameFresh() {
        return lastFrameProcessedNanos != 0 && System.nanoTime() - lastFrameProcessedNanos < FRESH_FRAME_NS;
    }

    public String getExposureStatus() {
        return exposureStatus;
    }

    /** Forget every tracked ball. */
    public void reset() {
        tracker.reset();
        pollen = Collections.emptyList();
        nectar = Collections.emptyList();
        groups = Collections.emptyList();
    }

    // =====================================================================
    // =========================== INTERNALS ===============================
    // =====================================================================

    /** One frame of pixel circles -> floor positions -> tracker. */
    private void trackFrame(Frame frame, Pose framePose) {
        List<Measurement> meas = new ArrayList<>();
        for (RawBall b : frame.balls) {
            double[] p = Geometry.project(b.x, b.y, b.type.radius, IMG_W, IMG_H);
            if (p == null) continue;                                 // above the horizon
            double ratio = b.r / p[3];
            if (ratio < SIZE_MIN || ratio > SIZE_MAX) continue;      // wrong size for a real ball there
            double tx = p[0], ty = p[1];
            if (framePose != null) {                                 // track in the field frame
                double[] f = Geometry.robotToField(p[0], p[1], framePose);
                tx = f[0];
                ty = f[1];
            }
            meas.add(new Measurement(b.type, tx, ty, b.x, b.y, b.r));
        }
        tracker.update(meas, frame.timeNanos);
    }

    /** Confirmed tracks -> read-only snapshots in robot AND field coordinates. */
    private void rebuildSnapshot(long now, Pose pose) {
        List<TrackedBall> p = new ArrayList<>(), n = new ArrayList<>();
        for (Track t : tracker.tracks) {
            if (!t.confirmed) continue;
            double rx = t.x, ry = t.y;
            if (pose != null) {
                double[] r = Geometry.fieldToRobot(t.x, t.y, pose);
                rx = r[0];
                ry = r[1];
            }
            TrackedBall b = new TrackedBall(t.id, t.type, rx, ry, t.x, t.y, t.hits,
                    t.misses == 0, Math.max(0, (now - t.lastSeenNanos) / 1e9));
            if (t.type == BallType.POLLEN) p.add(b);
            else n.add(b);
        }
        Collections.sort(p, BY_DISTANCE);
        Collections.sort(n, BY_DISTANCE);
        pollen = Collections.unmodifiableList(p);
        nectar = Collections.unmodifiableList(n);
        groups = Collections.unmodifiableList(buildGroups(pollen));
    }

    private static final Comparator<TrackedBall> BY_DISTANCE = new Comparator<TrackedBall>() {
        @Override
        public int compare(TrackedBall a, TrackedBall b) {
            return Double.compare(a.distance(), b.distance());
        }
    };

    /**
     * blobTrack's grouping: any two balls within CLUSTER_RADIUS end up in the same group, so a
     * line of touching balls stays one group. Done in the robot frame so distances and scores
     * match blobTrack exactly; the field aim point is carried along.
     */
    static List<TrackedGroup> buildGroups(List<TrackedBall> balls) {
        List<TrackedGroup> out = new ArrayList<>();
        boolean[] used = new boolean[balls.size()];

        for (int i = 0; i < balls.size(); i++) {
            if (used[i]) continue;
            List<TrackedBall> members = new ArrayList<>();
            members.add(balls.get(i));
            used[i] = true;

            boolean grew = true;
            while (grew) {
                grew = false;
                for (int j = 0; j < balls.size(); j++) {
                    if (used[j]) continue;
                    for (TrackedBall m : members) {
                        if (Math.hypot(m.x - balls.get(j).x, m.y - balls.get(j).y) <= CLUSTER_RADIUS) {
                            members.add(balls.get(j));
                            used[j] = true;
                            grew = true;
                            break;
                        }
                    }
                }
            }

            Collections.sort(members, BY_DISTANCE);
            TrackedBall nearest = members.get(0);
            double sx = 0, sy = 0, sfx = 0, sfy = 0;
            boolean seen = false;
            for (TrackedBall b : members) {
                sx += b.x; sy += b.y; sfx += b.fieldX; sfy += b.fieldY;
                seen |= b.seenThisFrame;
            }
            int c = members.size();
            double ax = AIM_AT_CENTROID ? sx / c : nearest.x;
            double ay = AIM_AT_CENTROID ? sy / c : nearest.y;
            double afx = AIM_AT_CENTROID ? sfx / c : nearest.fieldX;
            double afy = AIM_AT_CENTROID ? sfy / c : nearest.fieldY;
            double score = c - DIST_PENALTY * Math.hypot(ax, ay);
            out.add(new TrackedGroup(members, ax, ay, afx, afy, score, nearest.id, seen));
        }

        Collections.sort(out, new Comparator<TrackedGroup>() {
            @Override
            public int compare(TrackedGroup a, TrackedGroup b) {
                return Double.compare(b.score, a.score);
            }
        });
        return out;
    }

    /** Collects CAL_SAMPLES frames with exactly one ball of calType, then solves tilt + focal. */
    private void calibrationStep(Frame frame) {
        RawBall only = null;
        int count = 0;
        for (RawBall b : frame.balls) {
            if (b.type != calType) continue;
            count++;
            only = b;
        }
        if (count != 1) {
            calStatus = count == 0 ? "no ball in view" : "more than one ball in view";
            return;
        }
        calSamples.add(new double[] { only.x, only.y, only.r });
        calStatus = String.format("sampling %d / %d", calSamples.size(), CAL_SAMPLES);
        if (calSamples.size() < Math.max(1, CAL_SAMPLES)) return;

        double sx = 0, sy = 0, sr = 0;
        for (double[] s : calSamples) { sx += s[0]; sy += s[1]; sr += s[2]; }
        int n = calSamples.size();
        double[] sol = Geometry.solveCalibration(sx / n, sy / n, sr / n, CAL_FLOOR_DIST, calType.radius, IMG_W, IMG_H);
        calibrating = false;
        calSamples.clear();

        if (sol == null || sol[0] < CAL_TILT_MIN || sol[0] > CAL_TILT_MAX
                || sol[1] < CAL_FOCAL_MIN || sol[1] > CAL_FOCAL_MAX) {
            calStatus = sol == null ? "FAILED: no solution (check LENS_HEIGHT / CAL_FLOOR_DIST)"
                    : String.format("FAILED: tilt %.1f focal %.0f is not believable; kept the old values", sol[0], sol[1]);
            return;
        }
        TILT_DEG = sol[0];
        FOCAL_PX = sol[1];
        tracker.reset();                                  // old positions used the old numbers
        calStatus = String.format("done: TILT_DEG %.2f  FOCAL_PX %.1f  (copy into the code)", sol[0], sol[1]);
    }

    private Pose currentPose() {
        if (poseSupplier == null) return null;
        return poseSupplier.get();
    }

    private void recordPose(long now, Pose pose) {
        poseHistory.addLast(new PoseStamp(now, pose));
        while (poseHistory.size() > 2 && now - poseHistory.peekFirst().nanos > POSE_HISTORY_NS) {
            poseHistory.removeFirst();
        }
    }

    /** Robot pose at time t (System.nanoTime base), interpolated from the history. */
    static Pose poseAt(Deque<PoseStamp> history, long t, Pose fallback) {
        if (history.isEmpty()) return fallback;
        PoseStamp prev = null;
        for (PoseStamp s : history) {
            if (s.nanos >= t) {
                if (prev == null) return s.pose;                     // older than the history
                double span = s.nanos - prev.nanos;
                double ratio = span <= 0 ? 1 : (t - prev.nanos) / span;
                return Pose.interpolate(prev.pose, s.pose, ratio);
            }
            prev = s;
        }
        return history.peekLast().pose;                              // newer than the history
    }

    /** Manual exposure + gain, one step per loop (the camera needs a moment between steps). */
    private void updateExposure(long now) {
        if (!MANUAL_EXPOSURE) {
            exposureStatus = "auto";
            return;
        }
        if (exposureStep >= 3 && (appliedExposureMs != EXPOSURE_MS || appliedGain != GAIN)) {
            exposureStep = 1;                                        // edited live in Panels
        }
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
                exposureNextNanos = now + 50_000_000L;
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

    /** Ids on the preview for balls seen this frame; the target group's balls are green. */
    private void pushLabels() {
        TrackedGroup target = getTargetGroup();
        List<Label> labels = new ArrayList<>();
        for (Track t : tracker.tracks) {
            if (!t.confirmed || t.misses != 0) continue;
            boolean inTarget = target != null && t.type == BallType.POLLEN && target.contains(t.id);
            labels.add(new Label(t.type.letter + "#" + t.id, t.px, t.py, t.r, t.type, inTarget));
        }
        processor.labels = labels;
    }

    private void writeTelemetry() {
        if (telemetry == null) return;
        if (calibrating) {
            telemetry.addData("Vision", "CALIBRATING: " + calStatus);
            return;
        }
        TrackedGroup g = getTargetGroup();
        telemetry.addData("Vision: pollen", String.format("%d balls, %d groups%s", pollen.size(), groups.size(),
                g == null ? "" : String.format(", target %d ball(s) %.1f in %+.0f deg",
                        g.count, g.distance(), g.headingDeg())));
        telemetry.addData("Vision: nectar", String.format("red %d, blue %d",
                getNectar(BallType.RED_NECTAR).size(), getNectar(BallType.BLUE_NECTAR).size()));
        telemetry.addData("Vision: camera", String.format("%s, %s, tilt %.2f focal %.1f",
                isFrameFresh() ? "frames ok" : "NO FRAMES", exposureStatus, TILT_DEG, FOCAL_PX));
    }

    // =====================================================================
    // ============================ GEOMETRY ===============================
    // Pixel -> floor position (robot frame: x = right, y = forward). Pure math.
    // =====================================================================

    static final class Geometry {
        private Geometry() {}

        /**
         * Floor position of a ball of this radius whose image center is (px, py).
         * Returns {x right (in), y forward (in), slant distance (in), expected radius (px)},
         * or null if that pixel looks at or above the horizon.
         */
        static double[] project(double px, double py, double radius, int w, int h) {
            double f = FOCAL_PX;
            double u = (px - w / 2.0) / f;                 // right +
            double v = (h / 2.0 - py) / f;                 // up +
            double a = Math.toRadians(TILT_DEG);

            // Ray through the pixel in robot axes {forward, right, up} (same as blobTrack)
            double dF = Math.cos(a) - v * Math.sin(a);
            double dR = u;
            double dU = Math.sin(a) + v * Math.cos(a);
            if (dU > -1e-3) return null;

            double drop = LENS_HEIGHT - radius;            // lens down to ball-center height
            if (drop <= 0.1) return null;
            double t = drop / -dU;
            double slant = t * Math.sqrt(dF * dF + dR * dR + dU * dU);
            return new double[] { t * dR + CAM_RIGHT_OFFSET, t * dF + CAM_FORWARD_OFFSET,
                    slant, expectedRadiusPx(slant, radius, u, v) };
        }

        /** Inverse of project(): where a ball at robot (x, y) appears. {px, py, rPx}, or null. */
        static double[] toPixel(double x, double y, double radius, int w, int h) {
            double fwd = y - CAM_FORWARD_OFFSET, right = x - CAM_RIGHT_OFFSET;
            double up = radius - LENS_HEIGHT;
            double a = Math.toRadians(TILT_DEG);
            double z = fwd * Math.cos(a) + up * Math.sin(a);         // along the optical axis
            if (z < 0.5) return null;
            double yc = -fwd * Math.sin(a) + up * Math.cos(a);
            double u = right / z, v = yc / z;
            double slant = Math.sqrt(fwd * fwd + right * right + up * up);
            return new double[] { w / 2.0 + FOCAL_PX * u, h / 2.0 - FOCAL_PX * v,
                    expectedRadiusPx(slant, radius, u, v) };
        }

        /**
         * Radius (px) the distance transform should measure for a real ball. Off-center balls
         * image as ellipses whose short axis is stretched by sqrt(1 + u^2 + v^2), so they look
         * up to ~10% bigger near the corners; blobTrack ignored that.
         */
        static double expectedRadiusPx(double slant, double radius, double u, double v) {
            double s2 = slant * slant - radius * radius;
            return FOCAL_PX * radius / Math.sqrt(Math.max(s2, 1e-6)) * Math.sqrt(1 + u * u + v * v);
        }

        /**
         * One ball at a known floor distance -> {tiltDeg, focalPx}, or null. Focal from its size,
         * tilt from its height in the image (blobTrack's formula, plus the off-center stretch).
         */
        static double[] solveCalibration(double px, double py, double rPx, double floorDist,
                                         double radius, int w, int h) {
            double drop = LENS_HEIGHT - radius;
            if (drop <= 0.1 || floorDist <= 0 || rPx <= 0) return null;
            double slant = Math.hypot(floorDist, drop);
            double k = Math.sqrt(slant * slant - radius * radius) / radius;

            double f = rPx * k;
            for (int i = 0; i < 5; i++) {                  // converges in a couple of steps
                double u = (px - w / 2.0) / f, v = (h / 2.0 - py) / f;
                f = rPx * k / Math.sqrt(1 + u * u + v * v);
            }
            double u = (px - w / 2.0) / f, v = (h / 2.0 - py) / f;
            double arg = -(drop / slant) * Math.sqrt(1 + u * u + v * v) / Math.sqrt(1 + v * v);
            if (arg < -1 || arg > 1) return null;
            return new double[] { Math.toDegrees(Math.asin(arg) - Math.atan(v)), f };
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
    }

    // =====================================================================
    // ============================ TRACKER ================================
    // PollenTracker's alpha-beta filter + greedy nearest matching; a track
    // only ever matches a measurement of its own color.
    // =====================================================================

    static final class Tracker {
        final List<Track> tracks = new ArrayList<>();
        private int nextId = 0;
        private long lastNanos = 0;

        void reset() {
            tracks.clear();
            lastNanos = 0;
        }

        void update(List<Measurement> meas, long nanos) {
            double dt = lastNanos == 0 ? 0.05 : (nanos - lastNanos) / 1e9;
            dt = Math.max(0.005, Math.min(0.5, dt));
            lastNanos = nanos;

            // Predict
            for (Track t : tracks) {
                t.x += t.vx * dt;
                t.y += t.vy * dt;
            }

            // Candidate pairs inside the gate, closest first, never across colors
            List<double[]> pairs = new ArrayList<>();          // {distance, trackIndex, measIndex}
            for (int ti = 0; ti < tracks.size(); ti++) {
                for (int mi = 0; mi < meas.size(); mi++) {
                    if (tracks.get(ti).type != meas.get(mi).type) continue;
                    double d = Math.hypot(tracks.get(ti).x - meas.get(mi).x, tracks.get(ti).y - meas.get(mi).y);
                    if (d < GATE) pairs.add(new double[] { d, ti, mi });
                }
            }
            Collections.sort(pairs, new Comparator<double[]>() {
                @Override
                public int compare(double[] a, double[] b) {
                    return Double.compare(a[0], b[0]);
                }
            });

            boolean[] tUsed = new boolean[tracks.size()];
            boolean[] mUsed = new boolean[meas.size()];

            // Correct matched tracks
            for (double[] p : pairs) {
                int ti = (int) p[1], mi = (int) p[2];
                if (tUsed[ti] || mUsed[mi]) continue;
                tUsed[ti] = true;
                mUsed[mi] = true;

                Track t = tracks.get(ti);
                Measurement m = meas.get(mi);
                double rx = m.x - t.x, ry = m.y - t.y;
                t.x += ALPHA * rx;
                t.y += ALPHA * ry;
                t.vx += (BETA / dt) * rx;
                t.vy += (BETA / dt) * ry;
                t.hits++;
                t.misses = 0;
                t.px = m.px;
                t.py = m.py;
                t.r = m.r;
                t.lastSeenNanos = nanos;
                if (t.hits >= CONFIRM_HITS) t.confirmed = true;
            }

            // Unmatched tracks
            for (int ti = 0; ti < tracks.size(); ti++) {
                if (tUsed[ti]) continue;
                Track t = tracks.get(ti);
                t.misses++;
                t.vx *= 0.8;
                t.vy *= 0.8;
            }

            // Drop dead tracks (unconfirmed ones die on their 2nd miss)
            Iterator<Track> it = tracks.iterator();
            while (it.hasNext()) {
                Track t = it.next();
                if (t.misses > MAX_MISSES || (!t.confirmed && t.misses > 1)) it.remove();
            }

            // New tracks
            for (int mi = 0; mi < meas.size(); mi++) {
                if (!mUsed[mi]) tracks.add(new Track(nextId++, meas.get(mi), nanos));
            }
        }
    }

    // =====================================================================
    // ========================= VISION PROCESSOR ==========================
    // Camera thread. For each color: mask -> cleanup -> distance transform
    // -> one peak per ball. Pixels only; inches happen in periodic().
    // =====================================================================

    static final class Processor implements VisionProcessor {
        private int width = IMG_W, height = IMG_H;
        private final Mat ycc = new Mat(), yellowRaw = new Mat(), mask = new Mat(),
                dist = new Mat(), inv = new Mat();
        private Mat openKernel, closeKernel;
        private int openKernelSize = -1, closeKernelSize = -1;
        private long seq = 0;

        volatile Frame latest = new Frame(Collections.<RawBall>emptyList(), 0, -1);
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

            // Convert and blur once, reuse for every color
            Imgproc.cvtColor(frame, ycc, Imgproc.COLOR_RGB2YCrCb);
            if (BLUR > 1) {
                int b = BLUR | 1;
                Imgproc.GaussianBlur(ycc, ycc, new org.opencv.core.Size(b, b), 0);
            }
            int top = (int) Math.max(0, Math.min(height - 1, height * ROI_TOP_FRAC));

            List<RawBall> found = new ArrayList<>();

            // Pollen (yellow). The raw yellow mask is kept so red can exclude it.
            Core.inRange(ycc, new Scalar(Y_MIN, CR_MIN, CB_MIN), new Scalar(Y_MAX, CR_MAX, CB_MAX), yellowRaw);
            yellowRaw.copyTo(mask);
            findBalls(BallType.POLLEN, MAX_BALLS, top, frame, found);

            if (DETECT_RED_NECTAR) {
                Core.inRange(ycc, new Scalar(R_Y_MIN, R_CR_MIN, R_CB_MIN),
                        new Scalar(R_Y_MAX, R_CR_MAX, R_CB_MAX), mask);
                if (RED_EXCLUDES_YELLOW) Core.subtract(mask, yellowRaw, mask);   // 255 - 255 = 0
                findBalls(BallType.RED_NECTAR, MAX_NECTAR_BALLS, top, frame, found);
            }
            if (DETECT_BLUE_NECTAR) {
                Core.inRange(ycc, new Scalar(B_Y_MIN, B_CR_MIN, B_CB_MIN),
                        new Scalar(B_Y_MAX, B_CR_MAX, B_CB_MAX), mask);
                findBalls(BallType.BLUE_NECTAR, MAX_NECTAR_BALLS, top, frame, found);
            }

            // Frame time on the System.nanoTime() clock, so it lines up with the pose history
            long now = System.nanoTime();
            long t = (captureTimeNanos > 0 && Math.abs(now - captureTimeNanos) < 1_000_000_000L)
                    ? captureTimeNanos : now;
            Frame f = new Frame(Collections.unmodifiableList(found), t, ++seq);
            latest = f;
            return f;
        }

        /** Cleans the current mask and pulls one circle per ball out of it. */
        private void findBalls(BallType type, int maxBalls, int top, Mat frame, List<RawBall> out) {
            if (top > 0) mask.submat(0, top, 0, width).setTo(ZERO);
            if (OPEN_SIZE > 1) Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, openKernel);
            if (CLOSE_SIZE > 1) Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, closeKernel);

            if (SHOW_MASK && type.letter.equalsIgnoreCase(MASK_COLOR)) {
                Core.bitwise_not(mask, inv);
                frame.setTo(new Scalar(0, 0, 0, 255), inv);
            }

            // Pixel value = distance to the nearest edge, so each ball's center is a peak
            Imgproc.distanceTransform(mask, dist, Imgproc.DIST_L2, 5);

            List<RawBall> mine = new ArrayList<>();
            for (int i = 0; i < 30 && mine.size() < maxBalls; i++) {
                Core.MinMaxLocResult mm = Core.minMaxLoc(dist);
                double r = mm.maxVal;
                if (r < MIN_RADIUS) break;
                Point p = mm.maxLoc;
                double cx = p.x + subPixel((int) p.x, (int) p.y, true, r);
                double cy = p.y + subPixel((int) p.x, (int) p.y, false, r);

                boolean ok = true;
                for (RawBall b : mine) {
                    double d = Math.hypot(cx - b.x, cy - b.y);
                    if (d < OVERLAP_FRAC * (r + b.r)) { ok = false; break; }              // same ball
                    if (d < (r + b.r) && r < SLIVER_RATIO * b.r) { ok = false; break; }    // sliver
                }
                if (cx - r < EDGE_MARGIN || cx + r > width - EDGE_MARGIN
                        || cy + r > height - EDGE_MARGIN || cy - r < top + EDGE_MARGIN) ok = false;   // cut off

                if (ok) mine.add(new RawBall(type, cx, cy, r));
                Imgproc.circle(dist, p, (int) Math.ceil(r), ZERO, -1);                   // erase, look again
            }
            out.addAll(mine);
        }

        /** Parabola fit through the peak and its 2 neighbors -> sub-pixel offset. */
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
            return Math.max(-0.5, Math.min(0.5, 0.5 * (l - rr) / den));
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
                    ringPaint.setColor(colorOf(b.type));
                    canvas.drawCircle((float) b.x * s, (float) b.y * s, (float) b.r * s, ringPaint);
                }
            }
            for (Label l : labels) {
                if (l.target) canvas.drawCircle(l.px * s, l.py * s, (l.r + 3) * s, targetPaint);
                textPaint.setColor(l.target ? Color.GREEN : colorOf(l.type));
                canvas.drawText(l.text, l.px * s, (l.py - l.r) * s - 5 * scaleCanvasDensity, textPaint);
            }
        }

        private static int colorOf(BallType t) {
            return t == BallType.POLLEN ? Color.YELLOW : t == BallType.RED_NECTAR ? Color.RED : Color.CYAN;
        }
    }

    // =====================================================================
    // ========================= SMALL DATA CLASSES ========================
    // =====================================================================

    /** One circle out of the distance transform. Pixels only. */
    static final class RawBall {
        final BallType type;
        final double x, y, r;
        RawBall(BallType type, double x, double y, double r) {
            this.type = type; this.x = x; this.y = y; this.r = r;
        }
    }

    static final class Frame {
        final List<RawBall> balls;   // per color, biggest first
        final long timeNanos;
        final long seq;
        Frame(List<RawBall> balls, long timeNanos, long seq) {
            this.balls = balls; this.timeNanos = timeNanos; this.seq = seq;
        }
    }

    static final class Measurement {
        final BallType type;
        final double x, y;          // tracking frame (field with a pose supplier, else robot), in
        final double px, py, r;     // where it was in the image (px)
        Measurement(BallType type, double x, double y, double px, double py, double r) {
            this.type = type; this.x = x; this.y = y; this.px = px; this.py = py; this.r = r;
        }
    }

    static final class Track {
        final int id;
        final BallType type;
        double x, y, vx, vy;        // tracking frame
        double px, py, r;           // last image position, for the preview label
        int hits = 1, misses = 0;
        boolean confirmed = false;
        long lastSeenNanos;
        Track(int id, Measurement m, long nanos) {
            this.id = id; this.type = m.type; this.x = m.x; this.y = m.y;
            this.px = m.px; this.py = m.py; this.r = m.r; this.lastSeenNanos = nanos;
        }
    }

    static final class Label {
        final String text;
        final float px, py, r;
        final BallType type;
        final boolean target;
        Label(String text, double px, double py, double r, BallType type, boolean target) {
            this.text = text; this.px = (float) px; this.py = (float) py; this.r = (float) r;
            this.type = type; this.target = target;
        }
    }

    static final class PoseStamp {
        final long nanos;
        final Pose pose;
        PoseStamp(long nanos, Pose pose) {
            this.nanos = nanos; this.pose = pose;
        }
    }
}
