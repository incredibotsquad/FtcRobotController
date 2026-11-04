package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.AllianceColors;
import org.firstinspires.ftc.teamcode.common.HiveCell;
import org.firstinspires.ftc.teamcode.common.HiveState;
import org.firstinspires.ftc.teamcode.common.HiveTags;
import org.firstinspires.ftc.teamcode.common.LaunchTargets;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem.TagObservation;

import java.util.List;
import java.util.Locale;

/**
 * Bench / field test for LimelightSubsystem's two HIVE up/down methods: tag HEIGHT vs tag ROLL.
 *
 * The roll method only runs while LaunchTargets.SELECTOR is VISION_ROLL, so this OpMode sets it
 * to VISION_ROLL at start (both methods run) and restores the previous selector when it ends.
 *
 * Shows, live in both INIT and RUN:
 *   - Up CELL per HIVE from each method (after the confirm-frames filter) and whether they agree
 *   - Tip counts per method (changes of the confirmed up CELL)
 *   - Per-tag agreement between the methods on raw single frames
 *   - Per-CELL tag count, average height and |roll| this frame
 *   - Limelight health: connected, pipeline, FPS, temp, staleness
 *   - Raw per-tag readout (toggle)
 *
 * Controls (gamepad1):
 *   DPAD LEFT  = focus RED HIVE
 *   DPAD RIGHT = focus BLUE HIVE
 *   B          = roll method on/off (switches SELECTOR between VISION_ROLL and VISION_HEIGHT)
 *   A          = reset counters
 *   Y          = toggle raw tag list
 */
@TeleOp(name = "Test: Hive Vision", group = "Test")
public class HiveVisionTest extends LinearOpMode {

    private static final AllianceColors[] ALLIANCES = {AllianceColors.RED, AllianceColors.BLUE};

    private LimelightSubsystem limelightSubsystem;
    private Limelight3A        limelight;   // second handle to the same device, read-only diagnostics
    private TelemetryManager   telemetryM;

    private LaunchTargets.SelectorMode previousSelector;
    private AllianceColors focus       = AllianceColors.RED;
    private boolean        showRawTags = true;

    // Tip tracking, index 0 = RED, 1 = BLUE
    private final HiveCell[] lastHeightUp = new HiveCell[2];
    private final HiveCell[] lastRollUp   = new HiveCell[2];
    private final int[]      heightTips   = new int[2];
    private final int[]      rollTips     = new int[2];
    private String           lastTipEvent = "none";

    // Raw single-frame comparison of the two methods
    private long lastFrameMs = 0;
    private int  framesCompared, tagsCompared, tagsAgree, tagsDisagree, heightAmbiguous, rollAmbiguous;

    private final ElapsedTime runtime   = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();

    // Gamepad edge detection
    private boolean prevLeft, prevRight, prevA, prevB, prevY;

    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        previousSelector = LaunchTargets.SELECTOR;
        LaunchTargets.SELECTOR = LaunchTargets.SelectorMode.VISION_ROLL; // run both methods
        HiveState.resetToMatchStart();

        limelightSubsystem = new LimelightSubsystem(hardwareMap, telemetryM);
        CommandScheduler.getInstance().registerSubsystem(limelightSubsystem);
        limelight = hardwareMap.get(Limelight3A.class, LimelightSubsystem.LIMELIGHT_NAME);

        resetCounters();
        runtime.reset();
        loopTimer.reset();

        try {
            // Vision runs during INIT too so you can check the HIVE before pressing play
            while (opModeInInit()) {
                loopBody("INIT");
            }
            while (opModeIsActive()) {
                loopBody("RUNNING");
            }
        } finally {
            LaunchTargets.SELECTOR = previousSelector; // don't leak the test's selector into the next OpMode
            limelight.stop();
            CommandScheduler.getInstance().reset();
        }
    }

    // -------------------------------------------------------------------------
    // Main loop
    // -------------------------------------------------------------------------

    private void loopBody(String phase) {
        handleGamepad();

        // Runs LimelightSubsystem.periodic() exactly like the real robot does
        CommandScheduler.getInstance().run();

        trackTips();
        compareMethodsOnNewFrame();

        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();
        boolean rollOn = LaunchTargets.isRollSelectorActive();
        List<TagObservation> obs = limelightSubsystem.getObservations();

        telemetryM.addLine("===== HIVE VISION TEST (" + phase + ") =====");
        telemetryM.addData("Roll method", rollOn ? "ON  (B = off)" : "OFF (B = on)");
        telemetryM.addData("Focus HIVE", focus + "   (DPAD L=RED  R=BLUE)");
        telemetryM.addData("Loop", fmt("%.1f ms", loopMs));

        // ---- Confirmed up CELL per method ----
        telemetryM.addLine("");
        telemetryM.addLine("----- UP CELL (confirmed) -----");
        for (AllianceColors a : ALLIANCES) {
            HiveCell height = HiveState.getUpCell(a);
            String line = "height " + height + age(HiveState.getLastUpdateMs(a));
            if (rollOn) {
                HiveCell roll = HiveState.getRollUpCell(a);
                line += "  | roll " + roll + age(HiveState.getRollLastUpdateMs(a))
                        + (height == roll ? "  AGREE" : "  DISAGREE");
            }
            telemetryM.addData((a == focus ? ">> " : "   ") + a + " HIVE", line);
        }

        // ---- Tips ----
        telemetryM.addLine("");
        telemetryM.addLine("----- TIPS (A = reset) -----");
        telemetryM.addData("Height RED / BLUE", heightTips[0] + " / " + heightTips[1]);
        if (rollOn) {
            telemetryM.addData("Roll RED / BLUE", rollTips[0] + " / " + rollTips[1]);
        }
        telemetryM.addData("Last tip", lastTipEvent);

        // ---- Raw single-frame agreement ----
        if (rollOn) {
            telemetryM.addLine("");
            telemetryM.addLine("----- PER-TAG, SINGLE FRAMES (A = reset) -----");
            telemetryM.addData("Frames / tags compared", framesCompared + " / " + tagsCompared);
            telemetryM.addData("Agree / disagree", tagsAgree + " / " + tagsDisagree
                    + (tagsCompared > 0 ? fmt("  (%.1f%% agree)", 100.0 * tagsAgree / tagsCompared) : ""));
            telemetryM.addData("Ambiguous height / roll", heightAmbiguous + " / " + rollAmbiguous);
        }

        // ---- Per-CELL this frame ----
        telemetryM.addLine("");
        telemetryM.addLine("----- CELLS (this frame) -----");
        for (AllianceColors a : ALLIANCES) {
            for (HiveCell c : HiveCell.values()) {
                telemetryM.addData("  " + a + " " + c, cellSummary(obs, a, c, rollOn));
            }
        }

        // ---- Limelight health ----
        telemetryM.addLine("");
        telemetryM.addLine("----- LIMELIGHT -----");
        telemetryM.addData("Connected / running", limelight.isConnected() + " / " + limelight.isRunning());
        LLStatus status = limelight.getStatus();
        if (status != null) {
            telemetryM.addData("Pipeline (status)", status.getPipelineIndex()
                    + "  (expected " + LimelightSubsystem.LL_PIPELINE + ")");
            telemetryM.addData("FPS / Temp / CPU", status.getFps() + " / " + status.getTemp() + " C / " + status.getCpu());
        }
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            telemetryM.addData("Staleness", result.getStaleness() + " ms (max " + LimelightSubsystem.MAX_RESULT_AGE_MS + ")");
            telemetryM.addData("Tags in frame (all IDs)",
                    result.getFiducialResults() != null ? result.getFiducialResults().size() : 0);
        }
        telemetryM.addData("Usable HIVE tags", obs.size());

        // ---- Raw tags ----
        if (showRawTags) {
            telemetryM.addLine("");
            telemetryM.addLine("----- RAW TAGS (Y = hide) -----");
            if (obs.isEmpty()) {
                telemetryM.addLine("  none");
            } else {
                for (TagObservation o : obs) {
                    String line = HiveTags.allianceOf(o.id) + " " + HiveTags.cellOf(o.id)
                            + fmt("  fwd %.1f left %.1f up %.1f", o.forward, o.left, o.up)
                            + "  H:" + verdict(o.heightSaysUp);
                    if (rollOn) {
                        line += fmt("  |roll| %.1f", Math.abs(o.rollDeg)) + " R:" + verdict(o.rollSaysUp);
                    }
                    telemetryM.addData("  #" + o.id, line);
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
        boolean b     = gamepad1.b;
        boolean y     = gamepad1.y;

        if (left && !prevLeft)   focus = AllianceColors.RED;
        if (right && !prevRight) focus = AllianceColors.BLUE;
        if (a && !prevA) resetCounters();
        if (b && !prevB) {
            LaunchTargets.SELECTOR = LaunchTargets.isRollSelectorActive()
                    ? LaunchTargets.SelectorMode.VISION_HEIGHT
                    : LaunchTargets.SelectorMode.VISION_ROLL;
        }
        if (y && !prevY) showRawTags = !showRawTags;

        prevLeft  = left;
        prevRight = right;
        prevA     = a;
        prevB     = b;
        prevY     = y;
    }

    // -------------------------------------------------------------------------
    // Tracking
    // -------------------------------------------------------------------------

    private void resetCounters() {
        for (int i = 0; i < 2; i++) {
            lastHeightUp[i] = HiveState.getUpCell(ALLIANCES[i]);
            lastRollUp[i]   = HiveState.getRollUpCell(ALLIANCES[i]);
            heightTips[i]   = 0;
            rollTips[i]     = 0;
        }
        lastTipEvent = "none";
        framesCompared = tagsCompared = tagsAgree = tagsDisagree = heightAmbiguous = rollAmbiguous = 0;
    }

    /** Counts changes of each method's confirmed up CELL. */
    private void trackTips() {
        for (int i = 0; i < 2; i++) {
            AllianceColors a = ALLIANCES[i];
            HiveCell height = HiveState.getUpCell(a);
            if (height != lastHeightUp[i]) {
                heightTips[i]++;
                lastHeightUp[i] = height;
                lastTipEvent = fmt("%s height: %s up @ %.1fs", a, height, runtime.seconds());
            }
            HiveCell roll = HiveState.getRollUpCell(a);
            if (roll != lastRollUp[i]) {
                rollTips[i]++;
                lastRollUp[i] = roll;
                lastTipEvent = fmt("%s roll: %s up @ %.1fs", a, roll, runtime.seconds());
            }
        }
    }

    /** Once per new camera frame: does each tag's height verdict match its roll verdict? */
    private void compareMethodsOnNewFrame() {
        long frameMs = limelightSubsystem.getFrameTimestampMs();
        if (frameMs == lastFrameMs) return;
        lastFrameMs = frameMs;
        if (!LaunchTargets.isRollSelectorActive()) return;

        List<TagObservation> obs = limelightSubsystem.getObservations();
        if (obs.isEmpty()) return;
        framesCompared++;
        for (TagObservation o : obs) {
            if (o.heightSaysUp == null) heightAmbiguous++;
            if (o.rollSaysUp == null)   rollAmbiguous++;
            if (o.heightSaysUp == null || o.rollSaysUp == null) continue;
            tagsCompared++;
            if (o.heightSaysUp.equals(o.rollSaysUp)) tagsAgree++;
            else tagsDisagree++;
        }
    }

    // -------------------------------------------------------------------------
    // Formatting helpers
    // -------------------------------------------------------------------------

    private static String cellSummary(List<TagObservation> obs, AllianceColors a, HiveCell c, boolean rollOn) {
        int n = 0, heightUp = 0, heightDown = 0, rollUp = 0, rollDown = 0;
        double sumUp = 0, sumRoll = 0;
        for (TagObservation o : obs) {
            if (HiveTags.allianceOf(o.id) != a || HiveTags.cellOf(o.id) != c) continue;
            n++;
            sumUp += o.up;
            if (o.heightSaysUp != null) { if (o.heightSaysUp) heightUp++; else heightDown++; }
            if (rollOn) {
                sumRoll += Math.abs(o.rollDeg);
                if (o.rollSaysUp != null) { if (o.rollSaysUp) rollUp++; else rollDown++; }
            }
        }
        if (n == 0) return "--";
        String s = fmt("%d tags | height %.1f in, up/down %d/%d", n, sumUp / n, heightUp, heightDown);
        if (rollOn) s += fmt(" | |roll| %.1f, up/down %d/%d", sumRoll / n, rollUp, rollDown);
        return s;
    }

    private static String verdict(Boolean up) {
        if (up == null) return "?";
        return up ? "UP" : "DOWN";
    }

    /** HiveState timestamps use System.nanoTime() in ms. */
    private static String age(long updatedAtMs) {
        if (updatedAtMs == 0) return " (not seen)";
        return fmt(" (%.1fs ago)", (System.nanoTime() / 1_000_000L - updatedAtMs) / 1000.0);
    }

    private static String fmt(String format, Object... args) {
        return String.format(Locale.US, format, args);
    }
}
