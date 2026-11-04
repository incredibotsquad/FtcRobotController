package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.common.AllianceColors;
import org.firstinspires.ftc.teamcode.subsystems.HiveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HiveSubsystem.CellState;
import org.firstinspires.ftc.teamcode.subsystems.HiveSubsystem.HiveCell;

import java.util.List;
import java.util.Locale;

/**
 * Bench / field test for HiveSubsystem (BIOBuzz HIVE vision).
 *
 * Runs on Limelight pipeline 0 and shows, live in both INIT and RUN:
 *   - Verdict per alliance: SCORABLE / TIPPED / NOT VISIBLE
 *   - Per-cell state, tag count, average |roll| and margin to the 90 deg threshold
 *   - Active scoring cell, target bearing, target distance, full 3D tag pose
 *   - Tip events: subsystem justTipped() count AND a robust count that survives dropped frames
 *   - Limelight health: connected, pipeline, FPS, temp, staleness, why a frame was rejected
 *   - Distance cross-check: subsystem math vs robot-space X/Y vs camera-space X/Z
 *   - Raw per-tag readout (toggle)
 *
 * Controls (gamepad1):
 *   DPAD LEFT  = target RED alliance
 *   DPAD RIGHT = target BLUE alliance
 *   A          = reset tip counters
 *   Y          = toggle raw tag list
 */
@TeleOp(name = "Test: Hive Vision", group = "Test")
public class HiveVisionTest extends LinearOpMode {

    private static final int    HIVE_PIPELINE            = 1;
    private static final double ROLL_THRESHOLD_DEG       = 90.0;   // must match HiveSubsystem
    private static final double MAX_STALENESS_MS         = 250.0;  // must match HiveSubsystem

    private HiveSubsystem    hive;
    private Limelight3A      limelight;   // second handle to the same device, read-only diagnostics
    private TelemetryManager telemetryM;

    private AllianceColors alliance    = AllianceColors.RED;
    private boolean        showRawTags = true;

    // Tip tracking
    private int subsystemTipsRed  = 0;
    private int subsystemTipsBlue = 0;
    private final CellState[] lastKnownState = new CellState[HiveCell.values().length];
    private final int[]       robustTips     = new int[HiveCell.values().length];
    private String            lastTipEvent   = "none";

    private final ElapsedTime runtime   = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();

    // Gamepad edge detection
    private boolean prevLeft, prevRight, prevA, prevY;

    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        limelight  = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);

        hive = new HiveSubsystem(hardwareMap, telemetryM, HIVE_PIPELINE);
        CommandScheduler.getInstance().registerSubsystem(hive); // idempotent; ensures periodic() runs
        hive.start(alliance, HIVE_PIPELINE);

        resetTipTracking();
        runtime.reset();
        loopTimer.reset();

        // Vision runs during INIT too so you can check the hive before pressing play
        while (opModeInInit()) {
            loopBody("INIT");
        }

        while (opModeIsActive()) {
            loopBody("RUNNING");
        }

        limelight.stop();
        CommandScheduler.getInstance().reset();
    }

    // -------------------------------------------------------------------------
    // Main loop
    // -------------------------------------------------------------------------

    private void loopBody(String phase) {
        handleGamepad();

        // Runs HiveSubsystem.periodic() exactly like the real robot does
        CommandScheduler.getInstance().run();

        trackTips();

        LLResult result = limelight.getLatestResult();
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();

        telemetryM.addLine("===== HIVE VISION TEST (" + phase + ") =====");
        telemetryM.addData("Target alliance", alliance + "   (DPAD L=RED  R=BLUE)");
        telemetryM.addData("Loop", fmt("%.1f ms", loopMs));

        // ---- Verdicts ----
        telemetryM.addLine("");
        telemetryM.addLine("----- VERDICT -----");
        telemetryM.addData(">> " + alliance + " HIVE", verdict(alliance));
        AllianceColors other = (alliance == AllianceColors.RED) ? AllianceColors.BLUE : AllianceColors.RED;
        telemetryM.addData("   " + other + " HIVE", verdict(other));
        telemetryM.addData("Tipped since start (" + alliance + ")", tippedSinceStart(alliance));

        // ---- Targeting (configured alliance) ----
        telemetryM.addLine("");
        telemetryM.addLine("----- TARGET (" + alliance + ") -----");
        HiveCell scoring = hive.getActiveScoringCell(alliance);
        telemetryM.addData("Active scoring cell", scoring != null ? scoring.name() : "none");
        Double bearing  = hive.getTargetBearingDegrees();
        Double distance = hive.getTargetDistanceInches();
        telemetryM.addData("Bearing",  bearing  != null ? fmt("%.1f deg (%s)", bearing, bearing < 0 ? "LEFT" : "RIGHT") : "N/A");
        telemetryM.addData("Distance", distance != null ? fmt("%.1f in  (%.2f ft)", distance, distance / 12.0) : "N/A");
        Pose3D pose = hive.getActiveCellTagPose();
        if (pose != null) {
            Position p = pose.getPosition().toUnit(DistanceUnit.INCH);
            YawPitchRollAngles o = pose.getOrientation();
            telemetryM.addData("Tag pose XYZ (in)", fmt("%.1f, %.1f, %.1f", p.x, p.y, p.z));
            telemetryM.addData("Tag pose R/P/Y (deg)", fmt("%.1f, %.1f, %.1f",
                    o.getRoll(AngleUnit.DEGREES), o.getPitch(AngleUnit.DEGREES), o.getYaw(AngleUnit.DEGREES)));
        } else {
            telemetryM.addData("Tag pose", "N/A");
        }

        // ---- Per-cell detail ----
        telemetryM.addLine("");
        telemetryM.addLine("----- CELLS -----");
        hive.addTelemetry(); // the subsystem's own telemetry, so that method gets tested too
        List<LLResultTypes.FiducialResult> fiducials = usableFiducials(result);
        for (HiveCell cell : HiveCell.values()) {
            Double avgRoll = averageRoll(fiducials, cell);
            String rollText = (avgRoll == null) ? "--"
                    : fmt("|roll| %.1f deg, margin %.1f", avgRoll, Math.abs(ROLL_THRESHOLD_DEG - avgRoll));
            CellState known = lastKnownState[cell.ordinal()];
            telemetryM.addData("  " + cell.name(), rollText
                    + "  | last known " + (known != null ? shortState(known) : "--")
                    + "  | tips " + robustTips[cell.ordinal()]);
        }

        // ---- Tip events ----
        telemetryM.addLine("");
        telemetryM.addLine("----- TIP EVENTS (A = reset) -----");
        telemetryM.addData("justTipped() RED / BLUE", subsystemTipsRed + " / " + subsystemTipsBlue);
        telemetryM.addData("Robust tips RED / BLUE", robustTipsFor(AllianceColors.RED) + " / " + robustTipsFor(AllianceColors.BLUE));
        telemetryM.addData("Last tip", lastTipEvent);

        // ---- Distance cross-check ----
        addDistanceCrossCheck(fiducials);

        // ---- Limelight health ----
        telemetryM.addLine("");
        telemetryM.addLine("----- LIMELIGHT -----");
        telemetryM.addData("Connected / running", limelight.isConnected() + " / " + limelight.isRunning());
        LLStatus status = limelight.getStatus();
        if (status != null) {
            telemetryM.addData("Pipeline (status)", status.getPipelineIndex() + "  (expected " + HIVE_PIPELINE + ")");
            telemetryM.addData("FPS / Temp / CPU", status.getFps() + " / " + status.getTemp() + " C / " + status.getCpu());
        }
        telemetryM.addData("Frame", frameReason(result));
        if (result != null) {
            telemetryM.addData("Staleness", result.getStaleness() + " ms");
            List<LLResultTypes.FiducialResult> all = result.getFiducialResults();
            telemetryM.addData("Tags in frame", all != null ? all.size() : 0);
        }

        // ---- Raw tags ----
        if (showRawTags) {
            telemetryM.addLine("");
            telemetryM.addLine("----- RAW TAGS (Y = hide) -----");
            if (fiducials.isEmpty()) {
                telemetryM.addLine("  none");
            } else {
                for (LLResultTypes.FiducialResult f : fiducials) {
                    HiveCell cell = HiveCell.fromTagId(f.getFiducialId());
                    Pose3D rs = f.getTargetPoseRobotSpace();
                    String rollText = "--";
                    String posText  = "--";
                    if (rs != null) {
                        rollText = fmt("%.1f", rs.getOrientation().getRoll(AngleUnit.DEGREES));
                        Position p = rs.getPosition().toUnit(DistanceUnit.INCH);
                        posText = fmt("%.1f,%.1f,%.1f", p.x, p.y, p.z);
                    }
                    telemetryM.addData("  #" + f.getFiducialId(),
                            (cell != null ? cell.name() : "NOT HIVE")
                                    + " roll " + rollText
                                    + " rs(in) " + posText
                                    + fmt(" tx %.1f ty %.1f", f.getTargetXDegrees(), f.getTargetYDegrees()));
                }
            }
        } else {
            telemetryM.addLine("(Y = show raw tags)");
        }

        telemetryM.addData("Runtime", fmt("%.1f s", runtime.seconds()));
        telemetryM.update(telemetry);
    }

    // -------------------------------------------------------------------------
    // Gamepad
    // -------------------------------------------------------------------------

    private void handleGamepad() {
        boolean left  = gamepad1.dpad_left;
        boolean right = gamepad1.dpad_right;
        boolean a     = gamepad1.a;
        boolean y     = gamepad1.y;

        if (left && !prevLeft && alliance != AllianceColors.RED) {
            alliance = AllianceColors.RED;
            hive.start(alliance, HIVE_PIPELINE);
        }
        if (right && !prevRight && alliance != AllianceColors.BLUE) {
            alliance = AllianceColors.BLUE;
            hive.start(alliance, HIVE_PIPELINE);
        }
        if (a && !prevA) resetTipTracking();
        if (y && !prevY) showRawTags = !showRawTags;

        prevLeft  = left;
        prevRight = right;
        prevA     = a;
        prevY     = y;
    }

    // -------------------------------------------------------------------------
    // Tip tracking
    // -------------------------------------------------------------------------

    private void resetTipTracking() {
        subsystemTipsRed  = 0;
        subsystemTipsBlue = 0;
        for (int i = 0; i < lastKnownState.length; i++) {
            lastKnownState[i] = null;
            robustTips[i]     = 0;
        }
        lastTipEvent = "none";
    }

    /**
     * Two tip counters:
     *  - justTipped(): the subsystem's edge trigger. It needs two consecutive frames with
     *    known states, so a single dropped frame mid-tip makes it miss.
     *  - robust: remembers the last KNOWN state per cell through UNKNOWN gaps and counts
     *    any change. If robust > justTipped, the subsystem is missing tips.
     */
    private void trackTips() {
        if (hive.justTipped(AllianceColors.RED)) {
            subsystemTipsRed++;
            lastTipEvent = fmt("RED via justTipped() @ %.1fs", runtime.seconds());
        }
        if (hive.justTipped(AllianceColors.BLUE)) {
            subsystemTipsBlue++;
            lastTipEvent = fmt("BLUE via justTipped() @ %.1fs", runtime.seconds());
        }

        for (HiveCell cell : HiveCell.values()) {
            int idx = cell.ordinal();
            CellState now = hive.getCellState(cell);
            if (now == CellState.UNKNOWN) continue;
            CellState prev = lastKnownState[idx];
            if (prev != null && prev != now) {
                robustTips[idx]++;
                lastTipEvent = fmt("%s %s -> %s @ %.1fs", cell.name(), shortState(prev), shortState(now), runtime.seconds());
            }
            lastKnownState[idx] = now;
        }
    }

    private int robustTipsFor(AllianceColors a) {
        int total = 0;
        for (HiveCell cell : HiveCell.values()) {
            if (cell.alliance == a) total += robustTips[cell.ordinal()];
        }
        return total;
    }

    private String tippedSinceStart(AllianceColors a) {
        int n = Math.max(robustTipsFor(a), a == AllianceColors.RED ? subsystemTipsRed : subsystemTipsBlue);
        return n > 0 ? "YES (" + n + ")" : "NO";
    }

    // -------------------------------------------------------------------------
    // Verdict / state helpers
    // -------------------------------------------------------------------------

    private String verdict(AllianceColors a) {
        if (!hive.hasCellVisible(a)) return "NOT VISIBLE";
        HiveCell scoring = hive.getActiveScoringCell(a);
        if (scoring != null) {
            return "SCORABLE  -> " + scoring.name() + " [" + hive.getVisibleTagCount(scoring) + " tags]";
        }
        return "TIPPED / NOT SCORABLE";
    }

    private static String shortState(CellState s) {
        switch (s) {
            case UPRIGHT_SCORABLE:    return "UP";
            case TIPPED_NOT_SCORABLE: return "TIPPED";
            default:                  return "?";
        }
    }

    // -------------------------------------------------------------------------
    // Raw Limelight helpers (mirror HiveSubsystem's filtering)
    // -------------------------------------------------------------------------

    private String frameReason(LLResult r) {
        if (r == null)                                return "NULL (no data yet)";
        if (!r.isValid())                             return "INVALID (no targets)";
        if (r.getPipelineIndex() != HIVE_PIPELINE)    return "WRONG PIPELINE (" + r.getPipelineIndex() + ")";
        if (r.getStaleness() > MAX_STALENESS_MS)      return "STALE (" + r.getStaleness() + " ms)";
        return "OK";
    }

    private List<LLResultTypes.FiducialResult> usableFiducials(LLResult r) {
        if (!"OK".equals(frameReason(r))) return java.util.Collections.emptyList();
        List<LLResultTypes.FiducialResult> f = r.getFiducialResults();
        return f != null ? f : java.util.Collections.<LLResultTypes.FiducialResult>emptyList();
    }

    private Double averageRoll(List<LLResultTypes.FiducialResult> fiducials, HiveCell cell) {
        double sum = 0;
        int count = 0;
        for (LLResultTypes.FiducialResult f : fiducials) {
            if (HiveCell.fromTagId(f.getFiducialId()) != cell) continue;
            Pose3D pose = f.getTargetPoseRobotSpace();
            if (pose == null) continue;
            sum += Math.abs(pose.getOrientation().getRoll(AngleUnit.DEGREES));
            count++;
        }
        return count > 0 ? sum / count : null;
    }

    /** Most-centered tag of the target alliance's visible cell (same pick rule as the subsystem). */
    private LLResultTypes.FiducialResult bestTagFor(List<LLResultTypes.FiducialResult> fiducials, AllianceColors a) {
        LLResultTypes.FiducialResult best = null;
        double bestScore = Double.MAX_VALUE;
        for (LLResultTypes.FiducialResult f : fiducials) {
            HiveCell cell = HiveCell.fromTagId(f.getFiducialId());
            if (cell == null || cell.alliance != a) continue;
            double score = f.getTargetXDegrees() * f.getTargetXDegrees()
                    + f.getTargetYDegrees() * f.getTargetYDegrees();
            if (score < bestScore) {
                bestScore = score;
                best = f;
            }
        }
        return best;
    }

    /**
     * Shows three ways of computing bearing/distance so you can check against a tape measure.
     * Works even when the cell is tipped (the subsystem only reports distance for scorable cells).
     *   1) Subsystem math: robot-space X and Z (what HiveSubsystem does today)
     *   2) Robot-space X (forward) and Y (left): Limelight's documented robot-space axes
     *   3) Camera-space X (right) and Z (forward): no robot mounting offset
     */
    private void addDistanceCrossCheck(List<LLResultTypes.FiducialResult> fiducials) {
        telemetryM.addLine("");
        telemetryM.addLine("----- DISTANCE CROSS-CHECK (" + alliance + ") -----");
        LLResultTypes.FiducialResult best = bestTagFor(fiducials, alliance);
        if (best == null) {
            telemetryM.addLine("  no " + alliance + " tag visible");
            return;
        }
        telemetryM.addData("  Using tag", "#" + best.getFiducialId());

        Pose3D rs = best.getTargetPoseRobotSpace();
        if (rs != null) {
            Position p = rs.getPosition().toUnit(DistanceUnit.INCH);
            telemetryM.addData("  1) Subsystem (RS x,z)", fmt("%.1f in @ %.1f deg",
                    Math.hypot(p.x, p.z), Math.toDegrees(Math.atan2(p.x, p.z))));
            telemetryM.addData("  2) Robot space (x,y)", fmt("%.1f in @ %.1f deg",
                    Math.hypot(p.x, p.y), Math.toDegrees(Math.atan2(-p.y, p.x))));
        }

        Pose3D cs = best.getTargetPoseCameraSpace();
        if (cs != null) {
            Position p = cs.getPosition().toUnit(DistanceUnit.INCH);
            telemetryM.addData("  3) Camera space (x,z)", fmt("%.1f in @ %.1f deg",
                    Math.hypot(p.x, p.z), Math.toDegrees(Math.atan2(p.x, p.z))));
        }
    }

    private static String fmt(String format, Object... args) {
        return String.format(Locale.US, format, args);
    }
}