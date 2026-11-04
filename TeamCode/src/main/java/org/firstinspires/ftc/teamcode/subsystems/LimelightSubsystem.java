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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.common.AllianceColors;
import org.firstinspires.ftc.teamcode.common.HiveCell;
import org.firstinspires.ftc.teamcode.common.HiveState;
import org.firstinspires.ftc.teamcode.common.HiveTags;
import org.firstinspires.ftc.teamcode.common.LaunchTargets;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Limelight 3A looking at the HIVE AprilTags. Camera is fixed to the chassis.
 *
 * Why not Limelight's MegaTag field localization: the tags are on the CELLS and move every time a
 * HIVE TIPS, so a static field map is wrong for half the tags after the first TIP. Instead, every
 * new camera frame this subsystem reads each tag's position in ROBOT space (Limelight does the
 * camera-to-robot transform) and:
 *  1. Classifies the tag's CELL as up or down from the tag height (about 50 in vs 36 in), and
 *     updates HiveState once a HIVE reads the same way for HIVE_CONFIRM_FRAMES frames in a row.
 *  2. Only while LaunchTargets.SELECTOR is VISION_ROLL: also classifies each tag by its roll angle
 *     (|roll| < ROLL_THRESHOLD_DEG = up) and updates HiveState's separate roll state through the
 *     same confirm-frames filter. Lets the two methods be compared on the field.
 *  3. Keeps the tag observations so {@link #estimateRobotPosition(double)} can work out the robot's
 *     field X/Y from the known tag positions and the odometry heading.
 *
 * Frames are skipped when invalid, stale, from a different pipeline than LL_PIPELINE, or the same
 * camera frame as last loop (so HIVE_CONFIRM_FRAMES counts real camera frames, not loop ticks).
 * Tags with an all-zero pose (no 3D data, e.g. "Full 3D" off in the pipeline) are ignored.
 *
 * Limelight setup (web UI, http://limelight.local:5801 while connected over USB):
 *  - Pipeline LL_PIPELINE: AprilTag / fiducial, family 36h11, tag size 82.55 mm (3.25 in), Full 3D on.
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

    /** Roll method: a tag's CELL is up when |roll| is below this (degrees). */
    public static double ROLL_THRESHOLD_DEG = 90.0;
    /** Roll method: tags within this many degrees of ROLL_THRESHOLD_DEG are ambiguous and ignored. 0 = off. */
    public static double ROLL_MARGIN_DEG = 0.0;

    /** One usable tag sighting this frame. */
    public static class TagObservation {
        public final int id;
        /** Tag position relative to the robot center, inches (forward, left, up). */
        public final double forward, left, up;
        /** Tag roll from Limelight, degrees. NaN when the roll method is not running. */
        public final double rollDeg;
        /** Height method verdict: true = CELL up, false = down, null = ambiguous (too close to the threshold). */
        public final Boolean heightSaysUp;
        /** Roll method verdict: true = CELL up, false = down, null = ambiguous or roll method not running. */
        public final Boolean rollSaysUp;

        TagObservation(int id, double forward, double left, double up,
                       double rollDeg, Boolean heightSaysUp, Boolean rollSaysUp) {
            this.id = id;
            this.forward = forward;
            this.left = left;
            this.up = up;
            this.rollDeg = rollDeg;
            this.heightSaysUp = heightSaysUp;
            this.rollSaysUp = rollSaysUp;
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
    /** Limelight-side timestamp of the last camera frame processed, to skip repeats. */
    private double lastLimelightFrameTs = Double.NaN;

    // Per-HIVE debounce, height method: index 0 = RED, 1 = BLUE.
    private final HiveCell[] pendingUpCell = new HiveCell[2];
    private final int[] pendingCount = new int[2];
    // Per-HIVE debounce, roll method: index 0 = RED, 1 = BLUE.
    private final HiveCell[] rollPendingUpCell = new HiveCell[2];
    private final int[] rollPendingCount = new int[2];

    public LimelightSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetry = telemetry;
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.setPollRateHz(POLL_RATE_HZ);
        limelight.pipelineSwitch(LL_PIPELINE);
        limelight.start();
    }

    /** Usable HIVE tags seen in the latest fresh frame (empty if none), with each method's verdict. */
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
     * Robot field X/Y from this frame's tags, using the given heading (radians, from odometry; the
     * Pinpoint IMU heading is more accurate than a single tag's orientation). Returns null if no
     * usable tags. Uses the height method's up/down verdict to pick each tag's field position.
     *
     * For each tag: tagField = robot + R(heading) * tagRobot, so robot = tagField - R(heading) * tagRobot.
     */
    public PositionEstimate estimateRobotPosition(double headingRad) {
        List<TagObservation> obs = new ArrayList<>();
        for (TagObservation o : observations) {
            if (o.heightSaysUp != null) obs.add(o);
        }
        if (obs.isEmpty()) return null;

        double cos = Math.cos(headingRad);
        double sin = Math.sin(headingRad);
        double[] xs = new double[obs.size()];
        double[] ys = new double[obs.size()];
        double sumX = 0, sumY = 0;

        for (int i = 0; i < obs.size(); i++) {
            TagObservation o = obs.get(i);
            double tagX = HiveTags.fieldX(o.id);
            double tagY = HiveTags.fieldY(o.id, o.heightSaysUp);
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

    @Override
    public void periodic() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid() || result.getStaleness() > MAX_RESULT_AGE_MS) {
            observations = Collections.emptyList();
            telemetry.addData("Limelight: tags", isConnected() ? "none" : "NOT CONNECTED");
            return;
        }
        if (result.getPipelineIndex() != LL_PIPELINE) {
            observations = Collections.emptyList();
            telemetry.addData("Limelight: tags", "WRONG PIPELINE " + result.getPipelineIndex()
                    + " (expected " + LL_PIPELINE + ")");
            return;
        }

        // The Limelight is polled faster than it produces frames; only process each camera frame once.
        if (result.getTimestamp() != lastLimelightFrameTs) {
            lastLimelightFrameTs = result.getTimestamp();
            processFrame(result);
        }

        addTelemetry();
    }

    private void processFrame(LLResult result) {
        boolean rollEnabled = LaunchTargets.isRollSelectorActive();
        if (!rollEnabled) {
            // Start the roll debounce fresh whenever the roll method is switched back on.
            rollPendingUpCell[0] = rollPendingUpCell[1] = null;
            rollPendingCount[0] = rollPendingCount[1] = 0;
        }

        double threshold = HiveTags.upDownThresholdZ();
        List<TagObservation> fresh = new ArrayList<>();
        // Up/down votes per HIVE this frame: [alliance][0 = AUDIENCE up, 1 = FAR up]
        int[][] heightVotes = new int[2][2];
        int[][] rollVotes = new int[2][2];

        for (LLResultTypes.FiducialResult f : result.getFiducialResults()) {
            int id = f.getFiducialId();
            if (!HiveTags.isHiveTag(id)) continue;

            Pose3D pose = f.getTargetPoseRobotSpace();
            if (isZeroPose(pose)) continue; // no 3D data for this tag

            Position p = pose.getPosition().toUnit(DistanceUnit.INCH);
            double forward = p.x;
            double left = ROBOT_SPACE_Y_IS_LEFT ? p.y : -p.y;
            double up = p.z;

            if (Math.hypot(forward, left) > MAX_TAG_DISTANCE_IN) continue;

            Boolean heightSaysUp = classifyByHeight(up, threshold);
            double rollDeg = Double.NaN;
            Boolean rollSaysUp = null;
            if (rollEnabled) {
                rollDeg = pose.getOrientation().getRoll(AngleUnit.DEGREES);
                rollSaysUp = classifyByRoll(rollDeg);
            }

            fresh.add(new TagObservation(id, forward, left, up, rollDeg, heightSaysUp, rollSaysUp));
            if (heightSaysUp != null) addVote(heightVotes, id, heightSaysUp);
            if (rollSaysUp != null) addVote(rollVotes, id, rollSaysUp);
        }

        observations = fresh;
        frameTimestampMs = result.getControlHubTimeStamp();

        long nowMs = System.nanoTime() / 1_000_000L;
        updateHiveState(AllianceColors.RED, heightVotes[0], nowMs);
        updateHiveState(AllianceColors.BLUE, heightVotes[1], nowMs);
        if (rollEnabled) {
            updateRollHiveState(AllianceColors.RED, rollVotes[0], nowMs);
            updateRollHiveState(AllianceColors.BLUE, rollVotes[1], nowMs);
        }
    }

    private void addTelemetry() {
        telemetry.addData("Limelight: tags", observations.size());
        if (!observations.isEmpty()) {
            TagObservation n = observations.get(0);
            telemetry.addData("Limelight: first tag (fwd, left, up)",
                    n.id + ": " + round1(n.forward) + ", " + round1(n.left) + ", " + round1(n.up));
        }
        telemetry.addData("Limelight: up CELL red / blue (height)",
                HiveState.getUpCell(AllianceColors.RED) + " / " + HiveState.getUpCell(AllianceColors.BLUE));
        if (LaunchTargets.isRollSelectorActive()) {
            telemetry.addData("Limelight: up CELL red / blue (roll)",
                    HiveState.getRollUpCell(AllianceColors.RED) + " / " + HiveState.getRollUpCell(AllianceColors.BLUE));
        }
    }

    /** Height method: true = CELL up, false = down, null = within UP_DOWN_MARGIN_IN of the threshold. */
    private static Boolean classifyByHeight(double up, double threshold) {
        if (Math.abs(up - threshold) < UP_DOWN_MARGIN_IN) return null; // ambiguous, e.g. mid-TIP
        return up > threshold;
    }

    /** Roll method: true = CELL up (|roll| < ROLL_THRESHOLD_DEG), false = down, null = within ROLL_MARGIN_DEG. */
    private static Boolean classifyByRoll(double rollDeg) {
        double absRoll = Math.abs(rollDeg);
        if (Math.abs(absRoll - ROLL_THRESHOLD_DEG) < ROLL_MARGIN_DEG) return null;
        return absRoll < ROLL_THRESHOLD_DEG;
    }

    /** The SDK returns an all-zero pose (never null) when a tag has no 3D data. */
    private static boolean isZeroPose(Pose3D pose) {
        if (pose == null) return true;
        Position p = pose.getPosition();
        return p.x == 0 && p.y == 0 && p.z == 0;
    }

    /** A tag votes for which CELL of its HIVE is up: its own CELL if it reads up, else the other one. */
    private static void addVote(int[][] votes, int id, boolean cellUp) {
        HiveCell cell = HiveTags.cellOf(id);
        HiveCell upCell = cellUp ? cell : HiveState.other(cell);
        votes[HiveTags.allianceOf(id) == AllianceColors.RED ? 0 : 1][upCell == HiveCell.AUDIENCE ? 0 : 1]++;
    }

    /** Height method: votes[0] = tags saying AUDIENCE is up, votes[1] = FAR is up. */
    private void updateHiveState(AllianceColors alliance, int[] votes, long nowMs) {
        HiveCell confirmed = debounce(allianceIndex(alliance), votes, pendingUpCell, pendingCount);
        if (confirmed == null) return;
        if (HiveState.getUpCell(alliance) != confirmed) {
            Log.i("LimelightSubsystem", alliance + " HIVE: " + confirmed + " CELL is now up (height)");
        }
        HiveState.setUpCell(alliance, confirmed, nowMs);
    }

    /** Roll method: same filter as the height method, but writes HiveState's roll state. */
    private void updateRollHiveState(AllianceColors alliance, int[] votes, long nowMs) {
        HiveCell confirmed = debounce(allianceIndex(alliance), votes, rollPendingUpCell, rollPendingCount);
        if (confirmed == null) return;
        if (HiveState.getRollUpCell(alliance) != confirmed) {
            Log.i("LimelightSubsystem", alliance + " HIVE: " + confirmed + " CELL is now up (roll)");
        }
        HiveState.setRollUpCell(alliance, confirmed, nowMs);
    }

    /**
     * Returns the up CELL once it has read the same for HIVE_CONFIRM_FRAMES frames in a row, else null.
     * Ignores frames where tags disagree (likely mid-TIP) and frames where the HIVE isn't seen.
     */
    private static HiveCell debounce(int i, int[] votes, HiveCell[] pendingUp, int[] count) {
        if (votes[0] == 0 && votes[1] == 0) return null;   // HIVE not seen: keep the debounce as is
        if (votes[0] > 0 && votes[1] > 0) {                 // conflicting tags: likely mid-TIP
            count[i] = 0;
            return null;
        }

        HiveCell seenUp = votes[0] > 0 ? HiveCell.AUDIENCE : HiveCell.FAR;
        if (seenUp == pendingUp[i]) {
            count[i]++;
        } else {
            pendingUp[i] = seenUp;
            count[i] = 1;
        }
        return count[i] >= HIVE_CONFIRM_FRAMES ? seenUp : null;
    }

    private static int allianceIndex(AllianceColors alliance) {
        return alliance == AllianceColors.RED ? 0 : 1;
    }

    private static double round1(double v) {
        return Math.round(v * 10) / 10.0;
    }
}
