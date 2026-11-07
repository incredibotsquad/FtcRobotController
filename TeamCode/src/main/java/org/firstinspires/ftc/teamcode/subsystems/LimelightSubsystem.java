package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.common.AllianceColors;
import org.firstinspires.ftc.teamcode.common.HiveCell;
import org.firstinspires.ftc.teamcode.common.HiveState;
import org.firstinspires.ftc.teamcode.common.HiveTags;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Limelight 3A looking at the HIVE AprilTags. Camera is fixed to the chassis.
 *
 * Why not Limelight's MegaTag field localization: the tags are on the CELLS and move every time a
 * HIVE TIPS, so a static field map is wrong for half the tags after the first TIP. Instead, every
 * loop this subsystem reads each tag's position in ROBOT space (Limelight does the camera-to-robot
 * transform) and:
 *  1. Classifies the tag's CELL as up or down from the tag height (about 50 in vs 36 in), and
 *     updates HiveState once a HIVE reads the same way for HIVE_CONFIRM_FRAMES frames in a row.
 *  2. Keeps the tag observations so {@link #estimateRobotPosition(double)} can work out the robot's
 *     field X/Y from the known tag positions and the odometry heading.
 *
 * Limelight setup (web UI, http://limelight.local:5801 while connected over USB):
 *  - Pipeline LL_PIPELINE: AprilTag / fiducial, family 36h11, tag size 82.55 mm (3.25 in).
 *  - Robot-space camera pose: the camera's position and angle relative to the robot center on the
 *    floor. It must be accurate, or tag heights and positions will be off.
 *  - No field map needed.
 *
 * Check ROBOT_SPACE_Y_IS_LEFT before trusting results: hold a tag to the robot's LEFT and look at
 * "Limelight: first tag (fwd, left, up)". "left" must be positive. If it's negative, flip the flag.
 *
 * Before it will work on the robot:
 * 1. In the Limelight web UI, set up pipeline 0 for AprilTags: family 36h11, tag size 82.55 mm. Also enter the camera's position and angle relative to the robot center on the floor. If that is off, the heights and positions will be off too.
 * 2. Name the device limelight in the Robot Controller configuration.
 * 3. Check one sign. Hold a tag to the robot's left and watch the "Limelight: first tag (fwd, left, up)" telemetry; left should be positive. I set the code up for Limelight's documented convention, but I'm not certain of it, so if left comes out negative, set ROBOT_SPACE_Y_IS_LEFT = true.
 * 4. The tag heights and Y positions in HiveTags were worked out from the manual's drawings (±1.5 in). Measure them on a real field. Tag height is also visible in that telemetry line.
 */
@Configurable
public class LimelightSubsystem extends SubsystemBase {

    public static String LIMELIGHT_NAME = "limelight";
    public static int LL_PIPELINE = 0;
    public static int POLL_RATE_HZ = 100;

    /**
     * Limelight's robot space is documented as X forward, Y right, Z up. Our frame (Pinpoint/FTCLib)
     * is X forward, Y LEFT. Leave false to negate Limelight's Y; set true if the check above shows
     * Limelight already reports +Y to the left.
     */
    public static boolean ROBOT_SPACE_Y_IS_LEFT = false;

    /** Ignore results older than this. */
    public static long MAX_RESULT_AGE_MS = 100;
    /** Ignore tags farther than this (horizontal distance); pose error grows with range. */
    public static double MAX_TAG_DISTANCE_IN = 110;
    /** Tags within this many inches of the up/down height threshold are ambiguous and ignored. */
    public static double UP_DOWN_MARGIN_IN = 3.0;
    /** Consecutive frames a HIVE must read the same before HiveState changes (filters noise and mid-TIP frames). */
    public static int HIVE_CONFIRM_FRAMES = 3;

    /** One usable tag sighting this frame. */
    public static class TagObservation {
        public final int id;
        public final boolean cellUp;
        /** Tag position relative to the robot center, inches (forward, left, up). */
        public final double forward, left, up;

        TagObservation(int id, boolean cellUp, double forward, double left, double up) {
            this.id = id;
            this.cellUp = cellUp;
            this.forward = forward;
            this.left = left;
            this.up = up;
        }
    }

    /** Robot position estimate from the current frame's tags. */
    public static class PositionEstimate {
        public final Translation2d position;
        public final int tagCount;
        /** Largest distance from any single tag's estimate to the average, inches. */
        public final double spreadIn;

        PositionEstimate(Translation2d position, int tagCount, double spreadIn) {
            this.position = position;
            this.tagCount = tagCount;
            this.spreadIn = spreadIn;
        }
    }

    private final Limelight3A limelight;
    private final TelemetryManager telemetry;

    private List<TagObservation> observations = Collections.emptyList();
    private long frameTimestampMs = 0;
    private LLResult latestResult = null;

    // Per-HIVE debounce: index 0 = RED, 1 = BLUE.
    private final HiveCell[] pendingUpCell = new HiveCell[2];
    private final int[] pendingCount = new int[2];

    // Per-HIVE diagnostics (index 0 = RED, 1 = BLUE): every tag of that HIVE in the last frame
    // that had any, including rejected ones, and when that frame was.
    private final String[] lastHiveView = {"never seen", "never seen"};
    private final long[] lastHiveSeenMs = new long[2];
    private String frameStatus = "no frame yet";

    public LimelightSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetry = telemetry;
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.setPollRateHz(POLL_RATE_HZ);
        limelight.pipelineSwitch(LL_PIPELINE);
        limelight.start();
    }

    /** Tags seen in the latest fresh frame (empty if none). */
    public List<TagObservation> getObservations() {
        return observations;
    }

    /** Control Hub time (System.currentTimeMillis) of the latest frame used, or 0 if none yet. */
    public long getFrameTimestampMs() {
        return frameTimestampMs;
    }

    public boolean isConnected() {
        return limelight.isConnected();
    }

    /**
     * What the camera last saw of this alliance's HIVE, for logs: each tag's height and verdict
     * (UP / DOWN / ambiguous / too far), how long ago, and the current frame status.
     */
    public String describeHive(AllianceColors alliance) {
        int i = alliance == AllianceColors.RED ? 0 : 1;
        String age = lastHiveSeenMs[i] == 0 ? ""
                : String.format(" (%d ms ago)", System.nanoTime() / 1_000_000L - lastHiveSeenMs[i]);
        return "last seen: " + lastHiveView[i] + age + "; frame now: " + frameStatus;
    }

    /**
     * Robot field X/Y from this frame's tags, using the given heading (radians, from odometry; the
     * Pinpoint IMU heading is more accurate than a single tag's orientation). Returns null if no
     * usable tags.
     *
     * For each tag: tagField = robot + R(heading) * tagRobot, so robot = tagField - R(heading) * tagRobot.
     */
    public PositionEstimate estimateRobotPosition(double headingRad) {
        List<TagObservation> obs = observations;
        if (obs.isEmpty()) return null;

        double cos = Math.cos(headingRad);
        double sin = Math.sin(headingRad);
        double[] xs = new double[obs.size()];
        double[] ys = new double[obs.size()];
        double sumX = 0, sumY = 0;

        for (int i = 0; i < obs.size(); i++) {
            TagObservation o = obs.get(i);
            double tagX = HiveTags.fieldX(o.id);
            double tagY = HiveTags.fieldY(o.id, o.cellUp);
            xs[i] = tagX - (o.forward * cos - o.left * sin);
            ys[i] = tagY - (o.forward * sin + o.left * cos);
            sumX += xs[i];
            sumY += ys[i];
        }

        double meanX = sumX / obs.size();
        double meanY = sumY / obs.size();
        double spread = 0;
        for (int i = 0; i < obs.size(); i++) {
            spread = Math.max(spread, Math.hypot(xs[i] - meanX, ys[i] - meanY));
        }
        return new PositionEstimate(new Translation2d(meanX, meanY), obs.size(), spread);
    }

    /** The raw Limelight result read by the last periodic() (may be null, invalid or stale). For test OpModes. */
    public LLResult getLatestResult() {
        return latestResult;
    }

    @Override
    public void periodic() {
        LLResult result = limelight.getLatestResult();
        latestResult = result;
        if (result == null || !result.isValid() || result.getStaleness() > MAX_RESULT_AGE_MS) {
            frameStatus = !isConnected() ? "NOT CONNECTED" : result == null ? "no result"
                    : !result.isValid() ? "invalid (no targets)" : "stale " + result.getStaleness() + " ms";
            observations = Collections.emptyList();
            telemetry.addData("Limelight: tags", isConnected() ? "none" : "NOT CONNECTED");
            return;
        }

        double threshold = HiveTags.upDownThresholdZ();
        List<TagObservation> fresh = new ArrayList<>();
        // Up/down votes per HIVE this frame: [alliance][0 = AUDIENCE up, 1 = FAR up]
        int[][] votes = new int[2][2];
        StringBuilder[] view = {new StringBuilder(), new StringBuilder()};
        int hiveTags = 0;

        for (LLResultTypes.FiducialResult f : result.getFiducialResults()) {
            int id = f.getFiducialId();
            if (!HiveTags.isHiveTag(id)) continue;
            hiveTags++;

            Position p = f.getTargetPoseRobotSpace().getPosition().toUnit(DistanceUnit.INCH);
            double forward = p.x;
            double left = ROBOT_SPACE_Y_IS_LEFT ? p.y : -p.y;
            double up = p.z;
            double dist = Math.hypot(forward, left);

            String verdict = dist > MAX_TAG_DISTANCE_IN ? "too far"
                    : Math.abs(up - threshold) < UP_DOWN_MARGIN_IN ? "ambiguous"
                    : up > threshold ? "UP" : "DOWN";
            StringBuilder v = view[HiveTags.allianceOf(id) == AllianceColors.RED ? 0 : 1];
            if (v.length() > 0) v.append(", ");
            v.append(String.format("%d %s z%.1f d%.0f %s", id, HiveTags.cellOf(id), up, dist, verdict));

            if (dist > MAX_TAG_DISTANCE_IN) continue;
            if (Math.abs(up - threshold) < UP_DOWN_MARGIN_IN) continue; // ambiguous, e.g. mid-TIP

            boolean cellUp = up > threshold;
            fresh.add(new TagObservation(id, cellUp, forward, left, up));

            HiveCell cell = HiveTags.cellOf(id);
            HiveCell upCell = cellUp ? cell : HiveState.other(cell);
            votes[HiveTags.allianceOf(id) == AllianceColors.RED ? 0 : 1][upCell == HiveCell.AUDIENCE ? 0 : 1]++;
        }

        observations = fresh;
        frameTimestampMs = result.getControlHubTimeStamp();
        frameStatus = hiveTags + " HIVE tag(s), threshold z" + round1(threshold);
        long now = System.nanoTime() / 1_000_000L;
        for (int i = 0; i < 2; i++) {
            if (view[i].length() > 0) {
                lastHiveView[i] = view[i].toString();
                lastHiveSeenMs[i] = now;
            }
        }

        updateHiveState(AllianceColors.RED, votes[0]);
        updateHiveState(AllianceColors.BLUE, votes[1]);

        telemetry.addData("Limelight: tags", fresh.size());
        if (!fresh.isEmpty()) {
            TagObservation n = fresh.get(0);
            telemetry.addData("Limelight: first tag (fwd, left, up)",
                    n.id + ": " + round1(n.forward) + ", " + round1(n.left) + ", " + round1(n.up));
        }
        telemetry.addData("Limelight: up CELL red / blue",
                HiveState.getUpCell(AllianceColors.RED) + " / " + HiveState.getUpCell(AllianceColors.BLUE));
    }

    /** votes[0] = tags saying AUDIENCE is up, votes[1] = FAR is up. Ignores frames where tags disagree. */
    private void updateHiveState(AllianceColors alliance, int[] votes) {
        int i = alliance == AllianceColors.RED ? 0 : 1;
        if (votes[0] == 0 && votes[1] == 0) return;   // HIVE not seen: keep the debounce as is
        if (votes[0] > 0 && votes[1] > 0) {           // conflicting tags: likely mid-TIP
            pendingCount[i] = 0;
            return;
        }

        HiveCell seenUp = votes[0] > 0 ? HiveCell.AUDIENCE : HiveCell.FAR;
        if (seenUp == pendingUpCell[i]) {
            pendingCount[i]++;
        } else {
            pendingUpCell[i] = seenUp;
            pendingCount[i] = 1;
        }

        if (pendingCount[i] >= HIVE_CONFIRM_FRAMES) {
            if (HiveState.getUpCell(alliance) != seenUp) {
                Log.i("LimelightSubsystem", alliance + " HIVE: " + seenUp + " CELL is now up");
            }
            HiveState.setUpCell(alliance, seenUp, System.nanoTime() / 1_000_000L);
        }
    }

    private static double round1(double v) {
        return Math.round(v * 10) / 10.0;
    }
}
