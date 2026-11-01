package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.common.AllianceColors;

import java.util.List;

/**
 * Manages the Limelight 3A for BIOBuzz HIVE detection.
 *
 * BIOBuzz has no fixed wall AprilTags. All 16 field tags (IDs 30-45) are 4-tag clusters
 * mounted on the underside of the four HIVE cells. One pipeline in Limelight Manager
 * handles all of them simultaneously via getFiducialResults().
 *
 * Roll threshold rule (from FTC game manual):
 *   |roll| < 90 degrees  →  cell is facing UP   → UPRIGHT_SCORABLE
 *   |roll| >= 90 degrees →  cell is facing DOWN  → TIPPED_NOT_SCORABLE
 */
public class HiveSubsystem extends SubsystemBase {

    private static final String TAG = "HiveSubsystem";
    private static final double SCORABLE_ROLL_THRESHOLD_DEGREES = 90.0;
    private static final double MAX_RESULT_STALENESS_MS = 250.0;

    // -------------------------------------------------------------------------
    // Public types
    // -------------------------------------------------------------------------

    public enum CellState { UPRIGHT_SCORABLE, TIPPED_NOT_SCORABLE, UNKNOWN }

    /**
     * The four physical HIVE cells on the BIOBuzz field.
     * Each carries a 4-tag AprilTag cluster; minId/maxId define that cluster.
     */
    public enum HiveCell {
        RED_FAR   (30, 33, AllianceColors.RED),
        RED_NEAR  (34, 37, AllianceColors.RED),
        BLUE_NEAR (38, 41, AllianceColors.BLUE),
        BLUE_FAR  (42, 45, AllianceColors.BLUE);

        public final int minId, maxId;
        public final AllianceColors alliance;

        HiveCell(int minId, int maxId, AllianceColors alliance) {
            this.minId = minId;
            this.maxId = maxId;
            this.alliance = alliance;
        }

        /** Returns the HiveCell whose cluster contains tagId, or null for unknown IDs. */
        public static HiveCell fromTagId(int tagId) {
            for (HiveCell c : values()) {
                if (tagId >= c.minId && tagId <= c.maxId) return c;
            }
            return null;
        }
    }

    // -------------------------------------------------------------------------
    // Internal per-cell snapshot (reset and rebuilt every periodic() tick)
    // -------------------------------------------------------------------------

    private static class CellSnapshot {
        CellState state    = CellState.UNKNOWN;
        CellState prevState = CellState.UNKNOWN;
        boolean   justTipped = false;
        int       visibleTagCount = 0;
        // Best (most centered) tag pose from this frame; null when no tags visible
        Pose3D    tagPoseRobotSpace = null;
        double    bearingDegrees   = 0;
        double    distanceInches   = 0;
    }

    // -------------------------------------------------------------------------
    // Fields
    // -------------------------------------------------------------------------

    private final Limelight3A      limelight;
    private final TelemetryManager telemetry;
    private       AllianceColors   alliance          = AllianceColors.RED;
    private       int              hivePipelineIndex = 0;

    // One snapshot per HiveCell, indexed by HiveCell.ordinal()
    private final CellSnapshot[] snapshots = new CellSnapshot[HiveCell.values().length];

    // -------------------------------------------------------------------------
    // Construction
    // -------------------------------------------------------------------------

    public HiveSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this(hardwareMap, telemetry, 0);
    }

    public HiveSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry, int hivePipelineIndex) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.telemetry        = telemetry;
        this.hivePipelineIndex = hivePipelineIndex;
        for (int i = 0; i < snapshots.length; i++) snapshots[i] = new CellSnapshot();
    }

    // -------------------------------------------------------------------------
    // Setup
    // -------------------------------------------------------------------------

    /**
     * Call once at OpMode init after the alliance color is known.
     * Switches the Limelight to the AprilTag pipeline and starts streaming.
     */
    public void start(AllianceColors alliance, int hivePipelineIndex) {
        this.alliance          = alliance;
        this.hivePipelineIndex = hivePipelineIndex;
        limelight.pipelineSwitch(hivePipelineIndex);
        limelight.start();
        Log.i(TAG, "Started — alliance=" + alliance + "  pipeline=" + hivePipelineIndex);
    }

    /** Convenience overload that reuses the pipeline index set at construction. */
    public void start(AllianceColors alliance) {
        start(alliance, this.hivePipelineIndex);
    }

    // -------------------------------------------------------------------------
    // Core update loop — called automatically every scheduler tick
    // -------------------------------------------------------------------------

    @Override
    public void periodic() {
        // Carry previous states forward before clearing this frame's data
        for (CellSnapshot s : snapshots) {
            s.prevState         = s.state;
            s.state             = CellState.UNKNOWN;
            s.tagPoseRobotSpace = null;
            s.visibleTagCount   = 0;
            s.justTipped        = false;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;
        if (result.getPipelineIndex() != hivePipelineIndex) return;
        if (result.getStaleness() > MAX_RESULT_STALENESS_MS) return;

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return;

        // Per-cluster accumulators
        double[] rollSum   = new double[snapshots.length];
        int[]    rollCount = new int[snapshots.length];
        // Track which tag is most centered (smallest squared angular offset) per cluster
        double[] bestScore = new double[snapshots.length];
        for (int i = 0; i < bestScore.length; i++) bestScore[i] = Double.MAX_VALUE;

        for (LLResultTypes.FiducialResult f : fiducials) {
            HiveCell cell = HiveCell.fromTagId(f.getFiducialId());
            if (cell == null) continue;
            int idx = cell.ordinal();

            snapshots[idx].visibleTagCount++;

            Pose3D pose = f.getTargetPoseRobotSpace();
            if (pose == null) continue;

            // Average roll across all visible tags in this cluster for a stable reading
            double roll = Math.abs(pose.getOrientation().getRoll(AngleUnit.DEGREES));
            rollSum[idx]   += roll;
            rollCount[idx] += 1;

            // Keep the tag closest to the camera's optical center for bearing/distance
            double score = f.getTargetXDegrees() * f.getTargetXDegrees()
                         + f.getTargetYDegrees() * f.getTargetYDegrees();
            if (score < bestScore[idx]) {
                bestScore[idx]                  = score;
                snapshots[idx].tagPoseRobotSpace = pose;
            }
        }

        // Resolve state for every cell with at least one visible tag
        for (HiveCell cell : HiveCell.values()) {
            int idx = cell.ordinal();
            CellSnapshot snap = snapshots[idx];
            if (rollCount[idx] == 0) continue;

            double avgRoll = rollSum[idx] / rollCount[idx];
            snap.state = (avgRoll < SCORABLE_ROLL_THRESHOLD_DEGREES)
                    ? CellState.UPRIGHT_SCORABLE
                    : CellState.TIPPED_NOT_SCORABLE;

            if (snap.tagPoseRobotSpace != null) {
                // Robot-space: X = lateral, Z = depth forward
                double x = snap.tagPoseRobotSpace.getPosition().toUnit(DistanceUnit.INCH).x;
                double z = snap.tagPoseRobotSpace.getPosition().toUnit(DistanceUnit.INCH).z;
                snap.bearingDegrees = Math.toDegrees(Math.atan2(x, z));
                snap.distanceInches = Math.hypot(x, z);
            }

            // Edge trigger: fires once when state transitions between two known states
            if (snap.prevState != CellState.UNKNOWN
                    && snap.state    != CellState.UNKNOWN
                    && snap.prevState != snap.state) {
                snap.justTipped = true;
                Log.i(TAG, cell.name() + " transitioned: " + snap.prevState + " → " + snap.state);
            }
        }
    }

    // -------------------------------------------------------------------------
    // Public API
    // -------------------------------------------------------------------------

    /** State of a specific physical cell. Returns UNKNOWN when that cell is out of view. */
    public CellState getCellState(HiveCell cell) {
        return snapshots[cell.ordinal()].state;
    }

    /**
     * True if at least one cell for the given alliance is currently visible.
     * Use before calling getActiveCellState() to avoid false UNKNOWN reads.
     */
    public boolean hasCellVisible(AllianceColors targetAlliance) {
        for (HiveCell cell : HiveCell.values()) {
            if (cell.alliance == targetAlliance
                    && snapshots[cell.ordinal()].state != CellState.UNKNOWN) {
                return true;
            }
        }
        return false;
    }

    /**
     * State of whichever alliance cell is currently visible.
     * In normal play only one cell per alliance is visible at a time.
     * Returns UNKNOWN when no cells for that alliance are in view.
     */
    public CellState getActiveCellState(AllianceColors targetAlliance) {
        for (HiveCell cell : HiveCell.values()) {
            if (cell.alliance == targetAlliance) {
                CellState s = snapshots[cell.ordinal()].state;
                if (s != CellState.UNKNOWN) return s;
            }
        }
        return CellState.UNKNOWN;
    }

    /**
     * True for exactly one scheduler tick the moment a cell transitions between
     * UPRIGHT_SCORABLE and TIPPED_NOT_SCORABLE (or vice versa).
     * Use this as an event trigger to re-aim immediately after a HIVE tip.
     */
    public boolean justTipped(AllianceColors targetAlliance) {
        for (HiveCell cell : HiveCell.values()) {
            if (cell.alliance == targetAlliance && snapshots[cell.ordinal()].justTipped)
                return true;
        }
        return false;
    }

    /**
     * Returns the upright-scorable cell for the given alliance, or null if none visible.
     * In BIOBuzz only one cell per alliance can be scorable at a time (bi-stable HIVE).
     */
    public HiveCell getActiveScoringCell(AllianceColors targetAlliance) {
        for (HiveCell cell : HiveCell.values()) {
            if (cell.alliance == targetAlliance
                    && snapshots[cell.ordinal()].state == CellState.UPRIGHT_SCORABLE) {
                return cell;
            }
        }
        return null;
    }

    /**
     * Horizontal bearing in degrees from robot forward to the active scorable cell
     * of the configured alliance. Negative = left, positive = right.
     * Returns null when no scorable cell is visible.
     */
    public Double getTargetBearingDegrees() {
        HiveCell active = getActiveScoringCell(alliance);
        if (active == null) return null;
        CellSnapshot snap = snapshots[active.ordinal()];
        return snap.tagPoseRobotSpace != null ? snap.bearingDegrees : null;
    }

    /**
     * Horizontal distance in inches from the robot to the active scorable cell
     * of the configured alliance.
     * Returns null when no scorable cell is visible.
     */
    public Double getTargetDistanceInches() {
        HiveCell active = getActiveScoringCell(alliance);
        if (active == null) return null;
        CellSnapshot snap = snapshots[active.ordinal()];
        return snap.tagPoseRobotSpace != null ? snap.distanceInches : null;
    }

    /**
     * Full 3D pose of the best visible tag on the active scorable cell,
     * expressed in robot-relative coordinates.
     * Returns null when no scorable cell is visible.
     * Use for precise turret + flywheel targeting beyond bearing/distance alone.
     */
    public Pose3D getActiveCellTagPose() {
        HiveCell active = getActiveScoringCell(alliance);
        return active != null ? snapshots[active.ordinal()].tagPoseRobotSpace : null;
    }

    /**
     * How many tags from a given cell's cluster were seen in the last frame.
     * 0 = cell not visible; 4 = full cluster visible (best accuracy).
     */
    public int getVisibleTagCount(HiveCell cell) {
        return snapshots[cell.ordinal()].visibleTagCount;
    }

    /** Switch to a different Limelight pipeline (e.g. the pollen/colour detector). */
    public void setPipeline(int index) {
        limelight.pipelineSwitch(index);
    }

    /** Push hive state to the Driver Station telemetry dashboard. */
    public void addTelemetry() {
        for (HiveCell cell : HiveCell.values()) {
            CellSnapshot s = snapshots[cell.ordinal()];
            telemetry.addData("HIVE " + cell.name(),
                    s.state + " [" + s.visibleTagCount + " tags]");
        }
        Double bearing  = getTargetBearingDegrees();
        Double distance = getTargetDistanceInches();
        telemetry.addData("Target Bearing",
                bearing  != null ? String.format("%.1f deg", bearing)  : "N/A");
        telemetry.addData("Target Distance",
                distance != null ? String.format("%.1f in",  distance) : "N/A");
    }

    // -------------------------------------------------------------------------
    // Backward-compat stubs (keep existing commands compiling unchanged)
    // -------------------------------------------------------------------------

    /**
     * BIOBuzz has no fixed wall tags, so field-pose localization via Limelight
     * is not available. Always returns null.
     * Kept so RelocalizeCommand and LaunchReadinessCommand compile without changes.
     */
    public com.pedropathing.geometry.Pose getLatestFieldPose() {
        return null;
    }

    /** Kept for call sites that still pass a boolean. Prefer start(AllianceColors). */
    public void setAlliance(boolean isRed) {
        start(isRed ? AllianceColors.RED : AllianceColors.BLUE, hivePipelineIndex);
    }
}
