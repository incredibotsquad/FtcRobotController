package org.firstinspires.ftc.teamcode.Vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.Size;

import com.bylazar.camerastream.PanelsCameraStream;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.utils.LoopTimer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Iterator;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * Same pipeline as PollenTracker, but the TARGET is picked by GROUP SIZE instead
 * of by distance.
 *
 * Touching balls come out of the distance transform as separate tracks, so a clump
 * is several tracks sitting close together. Tracks within CLUSTER_RADIUS of each
 * other are chained into one group and scored:
 *
 *     score = ballCount - DIST_PENALTY * distance
 *
 * Three balls at 50 in beat one ball at 20 in as long as the extra balls are worth
 * more than the extra driving. Raise DIST_PENALTY to favor close balls, lower it to
 * favor big piles.
 *
 * Panels: camera preview + telemetry. The field radar is gone; the ball positions
 * it drew are already in the groups/balls telemetry tables.
 */
@Configurable
@TeleOp(name = "Pollen Cluster Tracker", group = "Vision")
public class blobTrack extends LinearOpMode {

    // =====================================================================
    // ============================ TUNING =================================
    // =====================================================================

    // ---- Target selection: groups beat single balls ----
    public static double CLUSTER_RADIUS = 9.0;       // balls within this of each other are one group (in)
    public static double DIST_PENALTY = 0.03;        // score = count - DIST_PENALTY * distance
    //   0.03 -> 1 extra ball is worth ~33 in of driving
    public static boolean AIM_AT_CENTROID = true;    // aim at the middle of the group, else its nearest ball

    // ---- Camera / robot measurements (tape measure) ----
    public static double LENS_HEIGHT = 9.1;          // floor to lens center (in)  <- MEASURE
    public static double CAM_FORWARD_OFFSET = 0;     // lens forward of robot center (in), negative = behind
    public static double CAM_RIGHT_OFFSET = 2.25;    // lens right of robot center (in)

    // ---- Robot physical width (used by isPathBlocked) ----
    // Measure from the robot's center line to each outer edge.
    // CAM_RIGHT_OFFSET is already applied by Geometry.project(), so these are
    // independent of camera position — just the physical robot half-widths.
    public static double ROBOT_HALF_WIDTH_LEFT  = 9.0;  // center to left  edge (in) <- MEASURE
    public static double ROBOT_HALF_WIDTH_RIGHT = 9.0;  // center to right edge (in) <- MEASURE

    // ---- From calibration (press A, then copy the printed values here) ----
    public static double TILT_DEG = -25.59;          // negative = tilted down
    public static double FOCAL_PX = 427.2;

    // ---- Calibration ----
    public static boolean AUTO_CALIBRATE = true;     // true = calibrate as soon as the OpMode starts (init)
    public static double CAL_FLOOR_DIST = 30;        // under-lens point to ball's floor contact (in)
    public static int CAL_SAMPLES = 10;

    // ---- Color (YCrCb). 128 = neutral on Cr and Cb. Purple is Cb ~150+ ----
    public static int Y_MIN  = 32,  Y_MAX  = 255;    // brightness
    public static int CR_MIN = 128, CR_MAX = 185;    // red vs green
    public static int CB_MIN = 0,   CB_MAX = 105;    // blue vs yellow  <- main knob

    // ---- Mask cleanup ----
    public static int BLUR = 5;                      // odd number
    public static int OPEN_SIZE = 3;                 // removes specks / thin strips
    public static int CLOSE_SIZE = 5;                // fills the ball's holes
    public static boolean SHOW_MASK = false;         // true = preview shows only pixels that pass color

    // ---- Detection ----
    public static double ROI_TOP_FRAC = 0;           // ignore image above this fraction; 0 = whole frame
    public static double MIN_RADIUS = 4;             // px
    public static double OVERLAP_FRAC = 0.5;         // higher = one ball less likely to get 2 circles
    public static double SLIVER_RATIO = 0.4;         // reject small leftovers touching a bigger ball
    public static int EDGE_MARGIN = 2;               // reject balls cut off by image edge
    public static int MAX_BALLS = 6;

    // ---- Size check: measured radius / expected radius ----
    public static double SIZE_MIN = 0.6;
    public static double SIZE_MAX = 1.5;

    // ---- Tracker ----
    public static double GATE = 4.0;                 // in, max jump to count as same ball
    public static double ALPHA = 0.5;                // position smoothing (lower = smoother, laggier)
    public static double BETA = 0.1;                 // velocity smoothing
    public static int CONFIRM_HITS = 3;              // frames before a ball is trusted
    public static int MAX_MISSES = 5;                // frames a ball can vanish before being dropped

    // ---- Camera controls ----
    public static boolean MANUAL_EXPOSURE = true;
    public static long EXPOSURE_MS = 10;
    public static int GAIN = 100;

    // ---- Panels ----
    // Read once when the stream starts, so changing it live does nothing.
    private static final int PANELS_STREAM_FPS = 10;

    // =====================================================================

    private static final int IMG_W = 320;
    private static final int IMG_H = 240;
    private static final double BALL_RADIUS = 1.4;   // Pollen = 2.8" diameter

    @Override
    public void runOpMode() {
        PollenProcessor processor = new PollenProcessor();
        Geometry geo = new Geometry();
        Tracker tracker = new Tracker();

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(processor)
                .setCameraResolution(new Size(IMG_W, IMG_H))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        if (MANUAL_EXPOSURE) setManualExposure(portal);

        // Panels: camera preview + telemetry mirrored to the driver station
        PanelsCameraStream.INSTANCE.startStream(portal, PANELS_STREAM_FPS);
        LoopTimer loopTimer = new LoopTimer();
        Telemetry telem = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

        telem.setMsTransmissionInterval(100);
        telem.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        boolean calibrating = AUTO_CALIBRATE;            // starts collecting samples right away
        boolean lastA = false;
        List<double[]> calSamples = new ArrayList<>();
        long lastFrameTime = 0;
        List<String> rawLines = new ArrayList<>();
        List<Group> groups = new ArrayList<>();
        Group target = null;

        while (opModeIsActive() || opModeInInit()) {
            loopTimer.start();

            // Live edits from the Panels configurables panel
            geo.syncFromStatics();

            // A = start calibration
            boolean a = gamepad1.a;
            if (a && !lastA) {
                calibrating = true;
                calSamples.clear();
            }
            lastA = a;

            Frame frame = processor.latest;
            if (frame.timeNanos != lastFrameTime) {           // only process new frames
                lastFrameTime = frame.timeNanos;

                if (calibrating) {
                    if (frame.balls.size() == 1) {
                        Ball b = frame.balls.get(0);
                        calSamples.add(new double[] { b.x, b.y, b.r });
                    }
                    if (calSamples.size() >= CAL_SAMPLES) {
                        double sx = 0, sy = 0, sr = 0;
                        for (double[] s : calSamples) { sx += s[0]; sy += s[1]; sr += s[2]; }
                        int n = calSamples.size();
                        geo.calibrate(sx / n, sy / n, sr / n, CAL_FLOOR_DIST);
                        calibrating = false;
                        tracker.reset();
                    }
                } else {
                    List<Measurement> meas = new ArrayList<>();
                    rawLines.clear();
                    for (Ball b : frame.balls) {
                        Projection p = geo.project(b.x, b.y, b.r);
                        boolean pass = p != null && p.sizeRatio >= SIZE_MIN && p.sizeRatio <= SIZE_MAX;
                        if (pass) meas.add(new Measurement(p.x, p.y, b.x, b.y, b.r));
                        rawLines.add(p == null
                                ? String.format("r=%4.1f  above horizon  REJECT", b.r)
                                : String.format("r=%4.1f exp=%4.1f ratio=%4.2f %s",
                                b.r, p.expectedR, p.sizeRatio, pass ? "ok" : "REJECT"));
                    }
                    tracker.update(meas, frame.timeNanos);

                    // Group the confirmed balls, then pick the best-scoring group
                    groups = tracker.getGroups();
                    target = tracker.getTargetGroup(groups);

                    // Send IDs to the preview: every ball of the target group is green
                    List<Label> labels = new ArrayList<>();
                    for (Track t : tracker.getConfirmed()) {
                        if (t.misses != 0) continue;
                        boolean inTarget = target != null && target.contains(t.id);
                        labels.add(new Label(t.id, t.px, t.py, t.r, inTarget));
                    }
                    processor.labels = labels;
                }
            }

            // ---- Telemetry ----
            if (calibrating) {
                telem.addLine("CALIBRATING");
                telem.addLine(String.format("ONE ball straight ahead, %.1f\" along floor", CAL_FLOOR_DIST));
                telem.addData("samples", "%d / %d", calSamples.size(), CAL_SAMPLES);
                if (frame.balls.size() > 1) telem.addLine("More than one ball in view!");
                if (frame.balls.isEmpty()) telem.addLine("No ball in view");
            } else {
                if (target != null) {
                    telem.addData("TARGET", "%d ball(s)  x %.1f  y %.1f  dist %.1f  score %.2f",
                            target.balls.size(), target.x, target.y, target.distance(), target.score);
                    // Plain numbers so the Panels graph plugin can plot them
                    telem.addData("targetX", target.x);
                    telem.addData("targetY", target.y);
                    telem.addData("targetDist", target.distance());
                    telem.addData("targetCount", target.balls.size());
                } else {
                    telem.addData("TARGET", "none");
                }

                telem.addLine("-- groups --");
                telem.addLine("n    x      y     dist  score");
                for (Group g : groups) {
                    telem.addLine(String.format("%-3d %6.1f %6.1f %6.1f %6.2f%s",
                            g.balls.size(), g.x, g.y, g.distance(), g.score,
                            g == target ? "  <-- target" : ""));
                }

                telem.addLine("-- balls --");
                telem.addLine("id    x      y     dist");
                for (Track t : tracker.getConfirmed()) {
                    telem.addLine(String.format("%-3d %6.1f %6.1f %6.1f",
                            t.id, t.x, t.y, t.distance()));
                }

                telem.addLine("-- raw detections --");
                for (String s : rawLines) telem.addLine(s);
            }

            telem.addLine("-- camera (copy into TILT_DEG / FOCAL_PX) --");
            telem.addData("tiltDeg", "%.2f", geo.tiltDeg);
            telem.addData("focalPx", "%.1f", geo.focalPx);
            telem.addLine("Press A to recalibrate");
            telem.addData("trackedBalls", tracker.getConfirmed().size());
            telem.addData("groups", groups.size());
            telem.addData("loopHz", loopTimer.getHz());
            telem.update();
            loopTimer.end();
            sleep(20);
        }

        PanelsCameraStream.INSTANCE.stopStream();
        portal.close();
    }

    private void setManualExposure(VisionPortal portal) {
        while (!isStopRequested()
                && portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            sleep(20);
        }
        if (isStopRequested()) return;
        ExposureControl exp = portal.getCameraControl(ExposureControl.class);
        if (exp.getMode() != ExposureControl.Mode.Manual) {
            exp.setMode(ExposureControl.Mode.Manual);
            sleep(50);
        }
        exp.setExposure(EXPOSURE_MS, TimeUnit.MILLISECONDS);
        sleep(20);
        portal.getCameraControl(GainControl.class).setGain(GAIN);
        sleep(20);
    }

    // =====================================================================
    // ========================= DATA CLASSES ==============================
    // =====================================================================

    static class Ball {
        final double x, y, r;   // pixels
        Ball(double x, double y, double r) { this.x = x; this.y = y; this.r = r; }
    }

    static class Frame {
        final List<Ball> balls;   // biggest (closest) first
        final long timeNanos;
        Frame(List<Ball> balls, long timeNanos) { this.balls = balls; this.timeNanos = timeNanos; }
    }

    static class Projection {
        final double x, y, expectedR, sizeRatio;
        Projection(double x, double y, double expectedR, double sizeRatio) {
            this.x = x; this.y = y; this.expectedR = expectedR; this.sizeRatio = sizeRatio;
        }
    }

    static class Measurement {
        final double x, y;          // floor position (in)
        final double px, py, r;     // where it was in the image (px)
        Measurement(double x, double y, double px, double py, double r) {
            this.x = x; this.y = y; this.px = px; this.py = py; this.r = r;
        }
    }

    static class Track {
        final int id;
        double x, y, vx, vy;
        double px, py, r;           // last image position, for drawing the ID
        int hits = 1, misses = 0;
        boolean confirmed = false;
        Track(int id, Measurement m) {
            this.id = id; this.x = m.x; this.y = m.y;
            this.px = m.px; this.py = m.py; this.r = m.r;
        }
        double distance() { return Math.hypot(x, y); }
    }

    /** A clump of balls sitting close together, plus where to drive for it. */
    static class Group {
        final List<Track> balls = new ArrayList<>();
        double x, y;            // aim point (in)
        double score;
        int leadId = -1;        // nearest ball, identifies the group frame to frame
        double distance() { return Math.hypot(x, y); }
        boolean contains(int id) {
            for (Track t : balls) if (t.id == id) return true;
            return false;
        }
    }

    /** What the preview draws next to each tracked ball. */
    static class Label {
        final int id;
        final float px, py, r;
        final boolean target;
        Label(int id, double px, double py, double r, boolean target) {
            this.id = id; this.px = (float) px; this.py = (float) py; this.r = (float) r; this.target = target;
        }
    }

    // =====================================================================
    // ========================= VISION PROCESSOR ==========================
    // Color mask -> cleanup -> distance transform -> one peak per ball
    // =====================================================================

    static class PollenProcessor implements VisionProcessor {
        private int width, height;
        private final Mat ycc = new Mat(), mask = new Mat(), dist = new Mat(), inv = new Mat();
        private Mat openKernel, closeKernel;
        private int openKernelSize = -1, closeKernelSize = -1;
        volatile Frame latest = new Frame(new ArrayList<Ball>(), 0);
        volatile List<Label> labels = new ArrayList<>();

        private final Paint targetPaint = new Paint();
        private final Paint otherPaint = new Paint();
        private final Paint roiPaint = new Paint();
        private final Paint textPaint = new Paint();

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            this.width = width;
            this.height = height;
            rebuildKernels();

            targetPaint.setColor(Color.GREEN);
            targetPaint.setStyle(Paint.Style.STROKE);
            otherPaint.setColor(Color.YELLOW);
            otherPaint.setStyle(Paint.Style.STROKE);
            roiPaint.setColor(Color.WHITE);
            textPaint.setStyle(Paint.Style.FILL);
            textPaint.setTextAlign(Paint.Align.CENTER);
            textPaint.setAntiAlias(true);
        }

        /** OPEN_SIZE / CLOSE_SIZE are live-editable, so the kernels have to follow them. */
        private void rebuildKernels() {
            int o = Math.max(1, OPEN_SIZE);
            if (o != openKernelSize) {
                openKernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE,
                        new org.opencv.core.Size(o, o));
                openKernelSize = o;
            }
            int c = Math.max(1, CLOSE_SIZE);
            if (c != closeKernelSize) {
                closeKernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE,
                        new org.opencv.core.Size(c, c));
                closeKernelSize = c;
            }
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            rebuildKernels();

            // 1. Color mask
            Imgproc.cvtColor(frame, ycc, Imgproc.COLOR_RGB2YCrCb);
            if (BLUR > 1) {
                int b = BLUR | 1;
                Imgproc.GaussianBlur(ycc, ycc, new org.opencv.core.Size(b, b), 0);
            }
            Core.inRange(ycc,
                    new Scalar(Y_MIN, CR_MIN, CB_MIN),
                    new Scalar(Y_MAX, CR_MAX, CB_MAX),
                    mask);

            // 2. ROI: blank out the top of the image
            int top = (int) (height * ROI_TOP_FRAC);
            if (top > 0) mask.submat(0, top, 0, width).setTo(new Scalar(0));

            // 3. Cleanup
            if (OPEN_SIZE > 1)  Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, openKernel);
            if (CLOSE_SIZE > 1) Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, closeKernel);

            if (SHOW_MASK) {
                Core.bitwise_not(mask, inv);
                frame.setTo(new Scalar(0, 0, 0, 255), inv);
            }

            // 4. Distance transform: pixel value = distance to nearest edge
            Imgproc.distanceTransform(mask, dist, Imgproc.DIST_L2, 5);

            // 5. Peaks, biggest first
            List<Ball> found = new ArrayList<>();
            for (int i = 0; i < 30 && found.size() < MAX_BALLS; i++) {
                Core.MinMaxLocResult mm = Core.minMaxLoc(dist);
                double r = mm.maxVal;
                if (r < MIN_RADIUS) break;
                Point p = mm.maxLoc;

                double cx = p.x + subPixel((int) p.x, (int) p.y, true, r);
                double cy = p.y + subPixel((int) p.x, (int) p.y, false, r);

                boolean ok = true;
                for (Ball b : found) {
                    double d = Math.hypot(cx - b.x, cy - b.y);
                    if (d < OVERLAP_FRAC * (r + b.r)) { ok = false; break; }            // same ball
                    if (d < (r + b.r) && r < SLIVER_RATIO * b.r) { ok = false; break; }  // sliver
                }
                if (cx - r < EDGE_MARGIN || cx + r > width - EDGE_MARGIN
                        || cy + r > height - EDGE_MARGIN) ok = false;                   // cut off

                if (ok) found.add(new Ball(cx, cy, r));

                Imgproc.circle(dist, p, (int) Math.ceil(r), new Scalar(0), -1);
            }

            long t = captureTimeNanos != 0 ? captureTimeNanos : System.nanoTime();
            Frame f = new Frame(Collections.unmodifiableList(found), t);
            latest = f;
            return f;
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
            targetPaint.setStrokeWidth(3 * scaleCanvasDensity);
            otherPaint.setStrokeWidth(2 * scaleCanvasDensity);

            // Only draw the ROI line when something is actually being cut off
            if (ROI_TOP_FRAC > 0) {
                roiPaint.setStrokeWidth(2 * scaleCanvasDensity);
                float topY = (float) (height * ROI_TOP_FRAC) * s;
                canvas.drawLine(0, topY, width * s, topY, roiPaint);
            }

            if (!(userContext instanceof Frame)) return;
            List<Ball> balls = ((Frame) userContext).balls;
            for (Ball b : balls) {
                canvas.drawCircle((float) b.x * s, (float) b.y * s, (float) b.r * s, otherPaint);
            }

            // Tracker IDs; every ball of the target group is drawn green
            textPaint.setTextSize(16 * scaleCanvasDensity);
            for (Label l : labels) {
                if (l.target) {
                    canvas.drawCircle(l.px * s, l.py * s, l.r * s, targetPaint);
                }
                textPaint.setColor(l.target ? Color.GREEN : Color.YELLOW);
                canvas.drawText("#" + l.id, l.px * s, (l.py - l.r) * s - 6 * scaleCanvasDensity, textPaint);
            }
        }
    }

    // =====================================================================
    // ============================ GEOMETRY ===============================
    // Pixel -> floor position (robot frame: x = right, y = forward)
    // =====================================================================

    static class Geometry {
        double tiltDeg = TILT_DEG;
        double focalPx = FOCAL_PX;

        // Last values we saw in the static fields, so an edit in Panels is picked up
        // without a stale write clobbering a fresh calibration.
        private double seenTilt = TILT_DEG;
        private double seenFocal = FOCAL_PX;

        /** Pulls TILT_DEG / FOCAL_PX in if they were edited from the Panels config panel. */
        void syncFromStatics() {
            if (TILT_DEG != seenTilt) {
                tiltDeg = TILT_DEG;
                seenTilt = TILT_DEG;
            }
            if (FOCAL_PX != seenFocal) {
                focalPx = FOCAL_PX;
                seenFocal = FOCAL_PX;
            }
        }

        /** Camera ray (not normalized) in robot axes {forward, right, up}. */
        private double[] ray(double px, double py) {
            double u = (px - IMG_W / 2.0) / focalPx;   // right +
            double v = (IMG_H / 2.0 - py) / focalPx;   // up +
            double a = Math.toRadians(tiltDeg);
            return new double[] {
                    Math.cos(a) - v * Math.sin(a),
                    u,
                    Math.sin(a) + v * Math.cos(a)
            };
        }

        /** Returns null if the ray doesn't hit the floor. */
        Projection project(double px, double py, double rPx) {
            double[] d = ray(px, py);
            if (d[2] > -1e-3) return null;

            double t = (BALL_RADIUS - LENS_HEIGHT) / d[2];   // both negative -> t > 0
            double fwd = t * d[0];
            double right = t * d[1];
            double slant = t * Math.sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);

            double expR = expectedRadius(slant);
            return new Projection(right + CAM_RIGHT_OFFSET, fwd + CAM_FORWARD_OFFSET,
                    expR, rPx / expR);
        }

        double expectedRadius(double slant) {
            double s2 = slant * slant - BALL_RADIUS * BALL_RADIUS;
            return focalPx * BALL_RADIUS / Math.sqrt(Math.max(s2, 1e-6));
        }

        /** One ball straight ahead. Solves focal length (from size) and tilt (from height in image). */
        void calibrate(double px, double py, double rPx, double floorDist) {
            double drop = LENS_HEIGHT - BALL_RADIUS;
            double slant = Math.hypot(floorDist, drop);

            focalPx = rPx * Math.sqrt(slant * slant - BALL_RADIUS * BALL_RADIUS) / BALL_RADIUS;

            double u = (px - IMG_W / 2.0) / focalPx;
            double v = (IMG_H / 2.0 - py) / focalPx;
            double sinDepression = drop / slant;
            double arg = -sinDepression * Math.sqrt(1 + u * u + v * v) / Math.sqrt(1 + v * v);
            arg = Math.max(-1, Math.min(1, arg));
            tiltDeg = Math.toDegrees(Math.asin(arg) - Math.atan(v));

            // Push the result into the static fields so the tuned numbers show up in
            // the Panels config panel, ready to be copied into the source.
            TILT_DEG = tiltDeg;
            FOCAL_PX = focalPx;
            seenTilt = tiltDeg;
            seenFocal = focalPx;
        }
    }

    // =====================================================================
    // ============================ TRACKER ================================
    // Alpha-beta filter per ball, greedy nearest matching, grouped by proximity
    // =====================================================================

    static class Tracker {
        private final List<Track> tracks = new ArrayList<>();
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

            // Candidate pairs inside the gate, closest first
            List<double[]> pairs = new ArrayList<>();   // {distance, trackIndex, measIndex}
            for (int ti = 0; ti < tracks.size(); ti++) {
                for (int mi = 0; mi < meas.size(); mi++) {
                    double d = Math.hypot(tracks.get(ti).x - meas.get(mi).x,
                            tracks.get(ti).y - meas.get(mi).y);
                    if (d < GATE) pairs.add(new double[] { d, ti, mi });
                }
            }
            Collections.sort(pairs, new Comparator<double[]>() {
                @Override public int compare(double[] a, double[] b) { return Double.compare(a[0], b[0]); }
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
                if (!mUsed[mi]) tracks.add(new Track(nextId++, meas.get(mi)));
            }
        }

        List<Track> getConfirmed() {
            List<Track> out = new ArrayList<>();
            for (Track t : tracks) if (t.confirmed) out.add(t);
            Collections.sort(out, new Comparator<Track>() {
                @Override public int compare(Track a, Track b) { return Double.compare(a.distance(), b.distance()); }
            });
            return out;
        }

        /**
         * Chains confirmed balls into groups: any two within CLUSTER_RADIUS end up
         * together, so a line of touching balls stays one group.
         */
        List<Group> getGroups() {
            List<Track> confirmed = getConfirmed();
            List<Group> groups = new ArrayList<>();
            boolean[] used = new boolean[confirmed.size()];

            for (int i = 0; i < confirmed.size(); i++) {
                if (used[i]) continue;
                Group g = new Group();
                g.balls.add(confirmed.get(i));
                used[i] = true;

                boolean grew = true;
                while (grew) {
                    grew = false;
                    for (int j = 0; j < confirmed.size(); j++) {
                        if (used[j]) continue;
                        for (Track member : g.balls) {
                            double d = Math.hypot(member.x - confirmed.get(j).x,
                                    member.y - confirmed.get(j).y);
                            if (d <= CLUSTER_RADIUS) {
                                g.balls.add(confirmed.get(j));
                                used[j] = true;
                                grew = true;
                                break;
                            }
                        }
                    }
                }

                // Aim point, lead ball and score
                Track nearest = g.balls.get(0);
                double sx = 0, sy = 0;
                for (Track t : g.balls) {
                    sx += t.x;
                    sy += t.y;
                    if (t.distance() < nearest.distance()) nearest = t;
                }
                if (AIM_AT_CENTROID) {
                    g.x = sx / g.balls.size();
                    g.y = sy / g.balls.size();
                } else {
                    g.x = nearest.x;
                    g.y = nearest.y;
                }
                g.leadId = nearest.id;
                g.score = g.balls.size() - DIST_PENALTY * g.distance();
                groups.add(g);
            }

            Collections.sort(groups, new Comparator<Group>() {
                @Override public int compare(Group a, Group b) { return Double.compare(b.score, a.score); }
            });
            return groups;
        }

        /** Highest scoring group of the given list, or null. */
        Group getTargetGroup(List<Group> groups) {
            return groups.isEmpty() ? null : groups.get(0);
        }

        /** The group containing a given ball id, or null. */
        Group groupContaining(int id) {
            if (id < 0) return null;
            for (Group g : getGroups()) if (g.contains(id)) return g;
            return null;
        }
    }

    // =====================================================================
    // =================== PATH PLANNING UTILITIES =========================
    // Pure-geometry helpers; they have no side effects and no access to
    // camera state.  Feed obstacle positions from a separate nectar detector
    // (e.g. BioBuzzProcessor) and robot pose from OdometrySubsystem.
    // =====================================================================

    /**
     * Returned by {@link #isPathBlocked}.
     */
    public static class PathBlockResult {
        /** True when at least one obstacle occupies the corridor to the target. */
        public final boolean blocked;
        /**
         * Heading adjustment in degrees that steers the robot clear of the
         * nearest blocking obstacle.  Positive = turn RIGHT, negative = turn LEFT.
         * Zero when the path is unblocked.
         */
        public final double suggestedTurnDegrees;
        /** How many obstacles are left of the current heading. */
        public final int obstaclesLeft;
        /** How many obstacles are right of the current heading. */
        public final int obstaclesRight;

        PathBlockResult(boolean blocked, double suggestedTurnDeg, int left, int right) {
            this.blocked = blocked;
            this.suggestedTurnDegrees = suggestedTurnDeg;
            this.obstaclesLeft = left;
            this.obstaclesRight = right;
        }
    }

    /**
     * Tests whether any obstacle (typically nectar balls) falls inside the
     * straight-line corridor from the robot to a target pollen group.
     *
     * <p>The corridor width is the robot's actual physical footprint, defined by
     * {@link #ROBOT_HALF_WIDTH_LEFT} and {@link #ROBOT_HALF_WIDTH_RIGHT} — the
     * distance from the robot centre line to each outer edge (measure with a tape
     * measure and set those constants).  The camera offset is already baked into
     * obstacle coordinates by {@code Geometry.project()}, so no further adjustment
     * is needed here.
     *
     * <p>An obstacle is only considered "in the way" when its floor projection lies
     * between the robot and the target — not behind the robot or past the target.
     *
     * <p>When blocked, {@code suggestedTurnDegrees} gives the minimum heading change
     * that shifts the robot's path clear of the nearest blocker:
     * positive = turn right, negative = turn left.
     *
     * @param targetX   robot-relative floor X of the target, right-positive (in)
     * @param targetY   robot-relative floor Y of the target, forward-positive (in)
     * @param obstacles each element is {@code double[] {x, y}} — robot-relative floor
     *                  position of one obstacle (in), as returned by
     *                  {@code Geometry.project()} on nectar detections
     * @return a {@link PathBlockResult} with blocked flag and suggested turn direction
     */
    public static PathBlockResult isPathBlocked(
            double targetX, double targetY,
            List<double[]> obstacles) {

        double pathLen = Math.hypot(targetX, targetY);
        if (pathLen < 1e-3) {
            return new PathBlockResult(false, 0, 0, 0);
        }

        // Unit vector along the path (ux, uy).
        // Signed perpendicular distance convention: positive = obstacle is LEFT of path.
        double ux = targetX / pathLen;
        double uy = targetY / pathLen;

        boolean blocked = false;
        int leftCount = 0, rightCount = 0;

        // Track the blocking obstacle deepest inside the robot footprint so we
        // can compute the minimum turn needed to clear it.
        double worstPerp = 0;      // signed; 0 = no blocker found yet
        double worstAlong = pathLen * 0.5; // distance along path to worst blocker
        boolean foundBlocker = false;

        for (double[] obs : obstacles) {
            if (obs == null || obs.length < 2) continue;
            double ox = obs[0], oy = obs[1];

            // Scalar projection onto path — obstacle must be between robot and target.
            double along = ox * ux + oy * uy;
            if (along < 0 || along > pathLen) continue;

            // Signed perpendicular offset from the robot's centre-line path.
            // Positive = obstacle is to the LEFT.
            double perp = -ox * uy + oy * ux;

            if (perp >= 0) leftCount++;
            else rightCount++;

            // Check against the robot's actual physical half-widths.
            // perp > 0 (left side): blocking if perp < ROBOT_HALF_WIDTH_LEFT
            // perp < 0 (right side): blocking if |perp| < ROBOT_HALF_WIDTH_RIGHT
            boolean inFootprint = (perp >= 0)
                    ? (perp < ROBOT_HALF_WIDTH_LEFT)
                    : (-perp < ROBOT_HALF_WIDTH_RIGHT);

            if (inFootprint) {
                blocked = true;
                // Keep the obstacle that penetrates deepest (closest to centre line)
                // as the worst case — it needs the biggest turn to clear.
                if (!foundBlocker || Math.abs(perp) < Math.abs(worstPerp)) {
                    worstPerp = perp;
                    worstAlong = along;
                    foundBlocker = true;
                }
            }
        }

        double suggestedTurnDeg = 0;
        if (blocked) {
            // Minimum turn to put the robot edge just clear of the worst blocker.
            // The edge that matters is whichever side the blocker is on.
            double edgeOffset = (worstPerp >= 0)
                    ? ROBOT_HALF_WIDTH_LEFT    // blocker on left  → clear left edge
                    : ROBOT_HALF_WIDTH_RIGHT;  // blocker on right → clear right edge

            // New path centre must be offset by (edgeOffset - |worstPerp|) + 20% margin
            // away from the blocker. atan2 gives the heading change needed at distance worstAlong.
            double clearance = (edgeOffset - Math.abs(worstPerp)) + edgeOffset * 0.2;
            double turnMag = Math.toDegrees(Math.atan2(clearance, worstAlong));

            // worstPerp > 0 → blocker is LEFT → steer RIGHT → positive degrees
            suggestedTurnDeg = (worstPerp >= 0) ? turnMag : -turnMag;
        }

        return new PathBlockResult(blocked, suggestedTurnDeg, leftCount, rightCount);
    }

    /**
     * Tests whether the straight-line path to a target pollen group would
     * require the robot to cross into the opposing alliance's autonomous zone.
     *
     * <p>In BIOBuzz, each alliance must stay on their half of the field during
     * autonomous.  The field is 144 × 144 inches; the dividing line sits at
     * {@code boundaryFieldY} (typically 72 in from either alliance wall).
     *
     * <p>Coordinate frame used here: field X and Y are absolute field inches
     * as reported by OdometrySubsystem.  {@code robotHeadingDeg} is the robot's
     * current heading in degrees, measured counter-clockwise from the field +X
     * axis, matching the FTC odometry convention.
     *
     * @param targetX            robot-relative floor X of the pollen group, right-positive (in)
     * @param targetY            robot-relative floor Y of the pollen group, forward-positive (in)
     * @param robotFieldX        robot's current field X from OdometrySubsystem (in)
     * @param robotFieldY        robot's current field Y from OdometrySubsystem (in)
     * @param robotHeadingDeg    robot heading in degrees, CCW from field +X axis
     * @param boundaryFieldY     field Y coordinate of the autonomous zone boundary (in);
     *                           use 72.0 for the centre line of a standard 144"-wide field
     * @param robotBelowBoundary true if the robot started on the lower-Y half of the field
     *                           (field Y &lt; boundaryFieldY); false if on the upper-Y half
     * @return true if the target is on the opposing alliance's side, meaning the
     *         robot would have to cross the boundary to reach it
     */
    public static boolean requiresCrossField(
            double targetX, double targetY,
            double robotFieldX, double robotFieldY, double robotHeadingDeg,
            double boundaryFieldY, boolean robotBelowBoundary) {

        double heading = Math.toRadians(robotHeadingDeg);

        // Convert robot-relative (right=+x, forward=+y) to absolute field coordinates.
        // Robot forward unit vector in field frame = (cos θ, sin θ).
        // Robot right  unit vector in field frame  = (sin θ, -cos θ).
        double fieldTargetX = robotFieldX
                + targetY * Math.cos(heading)
                + targetX * Math.sin(heading);
        double fieldTargetY = robotFieldY
                + targetY * Math.sin(heading)
                - targetX * Math.cos(heading);

        // Suppress "unused" warning — fieldTargetX is kept for completeness but only
        // the Y axis determines which half of the field the target is on.
        @SuppressWarnings("unused") double _x = fieldTargetX;

        if (robotBelowBoundary) {
            return fieldTargetY > boundaryFieldY;
        } else {
            return fieldTargetY < boundaryFieldY;
        }
    }
}