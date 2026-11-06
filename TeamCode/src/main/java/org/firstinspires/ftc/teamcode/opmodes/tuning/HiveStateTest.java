package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.common.AllianceColors;
import org.firstinspires.ftc.teamcode.common.HiveCell;
import org.firstinspires.ftc.teamcode.common.HiveState;
import org.firstinspires.ftc.teamcode.common.HiveTags;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Measures how well LimelightSubsystem detects which CELL of each HIVE is up.
 *
 * You tell the OpMode the TRUE state of each HIVE (it starts at the match-start state: red
 * AUDIENCE up, blue FAR up). It runs the real LimelightSubsystem and scores it against that truth:
 *  - per tag reading: correct / wrong / ambiguous (too close to the up/down height threshold) /
 *    too far (beyond MAX_TAG_DISTANCE_IN). Same filters and Y-sign handling as the subsystem.
 *  - per frame, per HIVE: did the tags vote for the right up CELL, the wrong one, or disagree?
 *  - debounced HiveState (what the launcher actually uses): % of frames it matched the truth,
 *    false flips (changed to the wrong CELL), and detection latency after a TIP.
 *  - measured tag heights, split by whether the CELL is really up or down. Use these to set
 *    HiveTags.UP_TAG_Z_IN / DOWN_TAG_Z_IN: the suggested threshold and the gap between the two
 *    groups are shown. If the groups overlap, the classification can't work from that spot.
 *
 * Procedure: put the robot where it will shoot from, start, check the HIVE is scored correctly.
 * TIP the HIVE, then press that HIVE's button the moment it lands: the truth flips and the
 * latency timer starts. Repeat from other positions / distances / angles; A resets the stats.
 *
 * Controls (gamepad 1):
 *   Dpad left   - red HIVE TIPPED (flip red truth, start latency timer)
 *   Dpad right  - blue HIVE TIPPED (flip blue truth, start latency timer)
 *   A           - reset all statistics (keeps truth and HiveState)
 *   X           - reset truth AND HiveState to the match-start state, and the stats
 *   Y           - toggle per-tag detail lines
 *
 * Nothing moves: no motors or servos are touched. Panels is not legal in matches (R704).
 */
@Configurable
@TeleOp(name = "Test: HIVE State (Limelight)", group = "Test")
public class HiveStateTest extends LinearOpMode {

    /** Most per-tag lines to show per frame. */
    public static int MAX_TAG_LINES = 8;

    private static final int RED = 0, BLUE = 1;
    private static final AllianceColors[] ALLIANCES = {AllianceColors.RED, AllianceColors.BLUE};

    /** Min / max / mean of a stream of values. */
    private static class Stats {
        int n;
        double sum, min = Double.POSITIVE_INFINITY, max = Double.NEGATIVE_INFINITY;

        void add(double v) {
            n++;
            sum += v;
            min = Math.min(min, v);
            max = Math.max(max, v);
        }

        String describe() {
            if (n == 0) return "no samples";
            return String.format("n=%d  mean %.1f  min %.1f  max %.1f", n, sum / n, min, max);
        }
    }

    /** Scores for one HIVE. */
    private static class HiveScore {
        int tagsCorrect, tagsWrong, tagsAmbiguous, tagsTooFar;
        int framesSeen, framesCorrect, framesWrong, framesConflict;
        int stateFrames, stateCorrectFrames, falseFlips;
        long tipAtMs = -1;          // truth flipped, waiting for HiveState to follow
        long lastLatencyMs = -1, maxLatencyMs = -1;
        int tipsDetected;

        void resetStats() {
            tagsCorrect = tagsWrong = tagsAmbiguous = tagsTooFar = 0;
            framesSeen = framesCorrect = framesWrong = framesConflict = 0;
            stateFrames = stateCorrectFrames = falseFlips = 0;
            tipAtMs = -1;
            lastLatencyMs = maxLatencyMs = -1;
            tipsDetected = 0;
        }
    }

    private final HiveCell[] truth = new HiveCell[2];
    private final HiveCell[] lastState = new HiveCell[2];
    private final HiveScore[] scores = {new HiveScore(), new HiveScore()};
    private Stats heightTrulyUp = new Stats();
    private Stats heightTrulyDown = new Stats();
    private int framesTotal, framesNoTags;

    @Override
    public void runOpMode() {
        TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
        LimelightSubsystem limelight = new LimelightSubsystem(hardwareMap, panels);

        resetToMatchStart();

        panels.addLine("HIVE state test ready. Truth starts at match start:");
        panels.addLine("red AUDIENCE up, blue FAR up. Dpad L/R = red/blue TIPPED.");
        panels.update(telemetry);
        waitForStart();

        boolean prevLeft = false, prevRight = false, prevA = false, prevX = false, prevY = false;
        boolean showTags = true;
        long lastFrameStamp = Long.MIN_VALUE;
        long fpsWindowStartMs = nowMs();
        int fpsFrames = 0;
        double fps = 0;
        List<String> tagLines = new ArrayList<>();

        while (opModeIsActive()) {
            long now = nowMs();

            // ---------------------------------------------------- buttons
            if (gamepad1.dpad_left && !prevLeft) tip(RED, now);
            if (gamepad1.dpad_right && !prevRight) tip(BLUE, now);
            if (gamepad1.a && !prevA) resetStats();
            if (gamepad1.x && !prevX) resetToMatchStart();
            if (gamepad1.y && !prevY) showTags = !showTags;
            prevLeft = gamepad1.dpad_left;
            prevRight = gamepad1.dpad_right;
            prevA = gamepad1.a;
            prevX = gamepad1.x;
            prevY = gamepad1.y;

            // ---------------------------------------------------- run the real subsystem
            limelight.periodic();
            LLResult result = limelight.getLatestResult();

            boolean fresh = result != null && result.isValid()
                    && result.getStaleness() <= LimelightSubsystem.MAX_RESULT_AGE_MS;
            boolean newFrame = fresh && result.getControlHubTimeStamp() != lastFrameStamp;

            if (newFrame) {
                lastFrameStamp = result.getControlHubTimeStamp();
                fpsFrames++;
                tagLines = scoreFrame(result);
            } else if (!fresh) {
                tagLines = Collections.emptyList();
            }

            if (now - fpsWindowStartMs >= 1000) {
                fps = fpsFrames * 1000.0 / (now - fpsWindowStartMs);
                fpsFrames = 0;
                fpsWindowStartMs = now;
            }

            // Debounced HiveState vs truth: scored every new frame, latency checked every loop.
            for (int i = 0; i < 2; i++) {
                HiveCell state = HiveState.getUpCell(ALLIANCES[i]);
                HiveScore s = scores[i];
                if (state != lastState[i]) {
                    if (state != truth[i]) s.falseFlips++;
                    lastState[i] = state;
                }
                if (newFrame) {
                    s.stateFrames++;
                    if (state == truth[i]) s.stateCorrectFrames++;
                }
                if (s.tipAtMs >= 0 && state == truth[i]) {
                    s.lastLatencyMs = now - s.tipAtMs;
                    s.maxLatencyMs = Math.max(s.maxLatencyMs, s.lastLatencyMs);
                    s.tipsDetected++;
                    s.tipAtMs = -1;
                }
            }

            // ---------------------------------------------------- telemetry
            panels.addData("Limelight", !limelight.isConnected() ? "NOT CONNECTED"
                    : result == null ? "no result"
                    : !result.isValid() ? "result invalid"
                    : String.format("staleness %d ms, %.0f new frames/s", result.getStaleness(), fps));
            panels.addData("Frames scored / with no HIVE tags", framesTotal + " / " + framesNoTags);

            for (int i = 0; i < 2; i++) addHiveLines(panels, i, now);

            panels.addLine("--- tag height (in), by TRUE CELL state ---");
            panels.addData("CELL really UP", heightTrulyUp.describe());
            panels.addData("CELL really DOWN", heightTrulyDown.describe());
            panels.addData("Threshold now", String.format("%.1f (UP_TAG_Z_IN %.1f, DOWN_TAG_Z_IN %.1f, margin +/-%.1f)",
                    HiveTags.upDownThresholdZ(), HiveTags.UP_TAG_Z_IN, HiveTags.DOWN_TAG_Z_IN,
                    LimelightSubsystem.UP_DOWN_MARGIN_IN));
            if (heightTrulyUp.n > 0 && heightTrulyDown.n > 0) {
                double gap = heightTrulyUp.min - heightTrulyDown.max;
                panels.addData("Suggested", gap > 0
                        ? String.format("threshold %.1f, gap %.1f (margin must be < %.1f)",
                                (heightTrulyUp.min + heightTrulyDown.max) / 2.0, gap, gap / 2.0)
                        : String.format("OVERLAP by %.1f in: up and down can't be told apart here", -gap));
            }

            if (showTags) {
                panels.addLine("--- tags this frame (Y hides) ---");
                for (String line : tagLines) panels.addLine(line);
            }

            panels.update(telemetry);
        }
    }

    /** Scores every HIVE tag in one new frame; returns the per-tag detail lines. */
    private List<String> scoreFrame(LLResult result) {
        framesTotal++;
        double threshold = HiveTags.upDownThresholdZ();
        int[][] votes = new int[2][2]; // [alliance][0 = AUDIENCE up, 1 = FAR up], accepted tags only
        List<String> lines = new ArrayList<>();
        boolean anyHiveTag = false;

        List<LLResultTypes.FiducialResult> fiducials = new ArrayList<>(result.getFiducialResults());
        fiducials.sort((a, b) -> Integer.compare(a.getFiducialId(), b.getFiducialId()));

        for (LLResultTypes.FiducialResult f : fiducials) {
            int id = f.getFiducialId();
            if (!HiveTags.isHiveTag(id)) continue;
            anyHiveTag = true;

            int a = HiveTags.allianceOf(id) == AllianceColors.RED ? RED : BLUE;
            HiveCell cell = HiveTags.cellOf(id);
            boolean trulyUp = truth[a] == cell;
            HiveScore s = scores[a];

            // Same transform and filters as LimelightSubsystem.periodic().
            Position p = f.getTargetPoseRobotSpace().getPosition().toUnit(DistanceUnit.INCH);
            double forward = p.x;
            double left = LimelightSubsystem.ROBOT_SPACE_Y_IS_LEFT ? p.y : -p.y;
            double up = p.z;
            double dist = Math.hypot(forward, left);

            String verdict;
            if (dist > LimelightSubsystem.MAX_TAG_DISTANCE_IN) {
                s.tagsTooFar++;
                verdict = "TOO FAR";
            } else {
                // Heights are recorded for every in-range tag, ambiguous ones included.
                (trulyUp ? heightTrulyUp : heightTrulyDown).add(up);

                if (Math.abs(up - threshold) < LimelightSubsystem.UP_DOWN_MARGIN_IN) {
                    s.tagsAmbiguous++;
                    verdict = "AMBIGUOUS";
                } else {
                    boolean readUp = up > threshold;
                    HiveCell votedUp = readUp ? cell : HiveState.other(cell);
                    votes[a][votedUp == HiveCell.AUDIENCE ? 0 : 1]++;
                    if (readUp == trulyUp) {
                        s.tagsCorrect++;
                        verdict = readUp ? "UP ok" : "DOWN ok";
                    } else {
                        s.tagsWrong++;
                        verdict = (readUp ? "UP" : "DOWN") + " WRONG";
                    }
                }
            }

            if (lines.size() < MAX_TAG_LINES) {
                lines.add(String.format("%d %s %s: z %.1f, dist %.0f -> %s (really %s)",
                        id, ALLIANCES[a], cell, up, dist, verdict, trulyUp ? "UP" : "DOWN"));
            }
        }

        if (!anyHiveTag) framesNoTags++;

        // Frame-level vote per HIVE, as the subsystem's debounce sees it.
        for (int i = 0; i < 2; i++) {
            int audience = votes[i][0], far = votes[i][1];
            if (audience == 0 && far == 0) continue;
            HiveScore s = scores[i];
            s.framesSeen++;
            if (audience > 0 && far > 0) {
                s.framesConflict++;
            } else {
                HiveCell seenUp = audience > 0 ? HiveCell.AUDIENCE : HiveCell.FAR;
                if (seenUp == truth[i]) s.framesCorrect++;
                else s.framesWrong++;
            }
        }
        return lines;
    }

    private void addHiveLines(TelemetryManager panels, int i, long now) {
        HiveScore s = scores[i];
        String name = ALLIANCES[i].toString();
        HiveCell state = HiveState.getUpCell(ALLIANCES[i]);

        panels.addLine("--- " + name + " HIVE ---");
        panels.addData(name + " truth / HiveState", truth[i] + " up / " + state + " up"
                + (state == truth[i] ? "  (MATCH)" : "  (MISMATCH)"));
        panels.addData(name + " tags ok / wrong / ambig / far",
                s.tagsCorrect + " / " + s.tagsWrong + " / " + s.tagsAmbiguous + " / " + s.tagsTooFar
                        + "  (" + pct(s.tagsCorrect, s.tagsCorrect + s.tagsWrong + s.tagsAmbiguous) + " ok)");
        panels.addData(name + " frames ok / wrong / conflict",
                s.framesCorrect + " / " + s.framesWrong + " / " + s.framesConflict
                        + "  of " + s.framesSeen + " seen");
        panels.addData(name + " HiveState correct", pct(s.stateCorrectFrames, s.stateFrames)
                + " of frames, false flips " + s.falseFlips);
        String latency = s.tipAtMs >= 0
                ? "WAITING " + (now - s.tipAtMs) + " ms"
                : s.lastLatencyMs >= 0 ? "last " + s.lastLatencyMs + " ms, max " + s.maxLatencyMs + " ms" : "no TIP yet";
        panels.addData(name + " TIP detect (" + s.tipsDetected + ")", latency);
    }

    private void tip(int i, long now) {
        truth[i] = HiveState.other(truth[i]);
        scores[i].tipAtMs = HiveState.getUpCell(ALLIANCES[i]) == truth[i] ? -1 : now;
    }

    private void resetStats() {
        for (HiveScore s : scores) s.resetStats();
        heightTrulyUp = new Stats();
        heightTrulyDown = new Stats();
        framesTotal = 0;
        framesNoTags = 0;
    }

    private void resetToMatchStart() {
        HiveState.resetToMatchStart();
        truth[RED] = HiveState.RED_START_UP_CELL;
        truth[BLUE] = HiveState.BLUE_START_UP_CELL;
        lastState[RED] = truth[RED];
        lastState[BLUE] = truth[BLUE];
        resetStats();
    }

    private static String pct(int part, int whole) {
        return whole == 0 ? "-" : String.format("%.0f%%", 100.0 * part / whole);
    }

    private static long nowMs() {
        return System.nanoTime() / 1_000_000L;
    }
}
