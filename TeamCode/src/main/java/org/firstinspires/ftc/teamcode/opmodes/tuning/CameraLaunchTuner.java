package org.firstinspires.ftc.teamcode.opmodes.tuning;

import android.os.SystemClock;
import android.util.Log;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Incredibot;
import org.firstinspires.ftc.teamcode.commands.CloseGateCommand;
import org.firstinspires.ftc.teamcode.commands.LaunchBallsCommand;
import org.firstinspires.ftc.teamcode.commands.LaunchReadinessCommand;
import org.firstinspires.ftc.teamcode.commands.ReverseIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.SmartIntakeCommand;
import org.firstinspires.ftc.teamcode.common.CrossOpModeStorage;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchGateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchSubsystem;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

/**
 * CameraLaunchTuner: a stationary launch-tuning OpMode driven by the laptop program in
 * tools/camera_launch_tuning (see CAMERA_LAUNCH_TUNING.md at the repo root).
 *
 * Reuses the production mechanism code: Incredibot's subsystems (same hardware names, bulk
 * caching), SmartIntakeCommand (loading, the FULL run-on / eject / transfer stop, and the transfer
 * feed while the gate is open), LaunchBallsCommand (the firing state machine), CloseGateCommand
 * (gate default), ReverseIntakeCommand (jam clearing), IntakeSubsystem's laser/beam-break counting,
 * and LaunchSubsystem's flywheel controller (bang-bang + voltage-compensated feedforward + PID)
 * through updateFlywheelWithGains(), so a trial's gains are frozen without editing shared statics.
 *
 * Differs from TeleOp: the drivetrain, odometry and Limelight are not registered (the drive motors
 * are never commanded, so the robot stays still); there is no LaunchReadinessCommand (RPM, gains and
 * the turret position come from the trial settings instead of the distance tables and auto-aim);
 * the intake only runs after gamepad 1 A arms loading; launches fire on their own once both flywheel
 * motors are stable (and the laptop is recording, when the laptop link is required).
 *
 * Files (on the Control Hub): /sdcard/FIRST/data/camera_launch_tuning/
 *   status.json     robot -> laptop, rewritten (atomically) a few times a second
 *   command.json    laptop -> robot (settings, fire permission, refill authorization)
 *   trials/trial_RUN_NNNN.json / .csv   per-trial summary and per-loop telemetry
 *
 * Controls:
 *   gamepad 1 A          arm loading (refused until the robot is confirmed empty after a burst)
 *   gamepad 1 B          abort the current trial
 *   gamepad 1 X          acknowledge "physically empty" (after checking the dead zone)
 *   gamepad 1 LT / gamepad 2 BACK (hold)  reverse intake + transfer, flywheel off, gate closed
 *   gamepad 1 Y          (idle) toggle "laptop link required"
 *   gamepad 1 dpad       (idle, link off only) up/down RPM +/-RPM_STEP, left/right turret
 *
 * Panels is not legal in matches (R704): pit/practice tool only.
 */
@Configurable
@TeleOp(name = "CameraLaunchTuner", group = "Tests")
public class CameraLaunchTuner extends LinearOpMode {

    private static final String TAG = "CameraLaunchTuner";
    public static final int PROTOCOL_VERSION = 1;

    // ------------------------------------------------------------------ tunables (Panels)
    /** Both motors within this many RPM of the target... */
    public static double STABLE_TOLERANCE_RPM = 40;
    /** ...and within this many RPM of each other... */
    public static double MOTOR_MISMATCH_RPM = 60;
    /** ...continuously for this long (and not in bang-bang) = stable. */
    public static long STABLE_MS = 400;
    /** Give up spinning up after this long (no launch). */
    public static long SPINUP_TIMEOUT_MS = 8000;
    /** With the laptop link required: wait this long for the laptop's "recording, fire" permission. */
    public static long CAMERA_WAIT_TIMEOUT_MS = 30000;
    /** Keep logging (flywheel still running) this long after the launch command ends. */
    public static long POST_LOG_MS = 1500;
    /** Extra wait after the estimated turret travel time. */
    public static long TURRET_EXTRA_SETTLE_MS = 250;
    /** status.json rewrite interval (not during the burst itself). */
    public static long STATUS_INTERVAL_MS = 250;
    /** command.json poll interval. */
    public static long COMMAND_POLL_MS = 250;
    /** Telemetry row cap per trial (memory guard). */
    public static int MAX_ROWS = 30000;
    public static double RPM_STEP = 25;
    public static double TURRET_STEP = 0.005;

    // Absolute limits the robot enforces on any settings it receives (the laptop has its own,
    // usually tighter, limits). Settings outside these are rejected whole, never clamped.
    public static double LIMIT_RPM_MIN = 1000, LIMIT_RPM_MAX = 5500;
    public static double LIMIT_P_MAX = 0.5;
    public static double LIMIT_KS_MAX = 0.5;
    public static double LIMIT_KV_MAX = 0.002;
    public static double LIMIT_TRANSFER_MIN = 0.2, LIMIT_TRANSFER_MAX = 1.0;
    public static long LIMIT_MAX_FIRE_MS_MIN = 500, LIMIT_MAX_FIRE_MS_MAX = 10000;

    // ------------------------------------------------------------------ state
    enum Phase { INIT, IDLE, LOADING, SPINUP, LAUNCHING, POST, CLEARING, AWAIT_EMPTY_ACK }

    /** One frozen set of trial settings. */
    static final class Settings {
        String id = "robot-default";
        String source = "robot";
        double targetRpm = Double.NaN;
        double p, ks, kv, transferPower, turretPosition;
        long maxFireMs;

        Settings copy() {
            Settings s = new Settings();
            s.id = id; s.source = source; s.targetRpm = targetRpm; s.p = p; s.ks = ks; s.kv = kv;
            s.transferPower = transferPower; s.turretPosition = turretPosition; s.maxFireMs = maxFireMs;
            return s;
        }

        /** Empty string if valid, else the first problem. */
        String validate() {
            if (!finite(targetRpm) || targetRpm < LIMIT_RPM_MIN || targetRpm > LIMIT_RPM_MAX) return "target_rpm out of range";
            if (!finite(p) || p < 0 || p > LIMIT_P_MAX) return "p out of range";
            if (!finite(ks) || ks < 0 || ks > LIMIT_KS_MAX) return "ks out of range";
            if (!finite(kv) || kv < 0 || kv > LIMIT_KV_MAX) return "kv out of range";
            if (!finite(transferPower) || transferPower < LIMIT_TRANSFER_MIN || transferPower > LIMIT_TRANSFER_MAX) return "transfer_power out of range";
            if (maxFireMs < LIMIT_MAX_FIRE_MS_MIN || maxFireMs > LIMIT_MAX_FIRE_MS_MAX) return "max_fire_ms out of range";
            if (!finite(turretPosition) || turretPosition < LaunchSubsystem.TURRET_MIN || turretPosition > LaunchSubsystem.TURRET_MAX) return "turret_position out of range";
            return "";
        }

        JSONObject toJson() throws JSONException {
            JSONObject o = new JSONObject();
            o.put("settings_id", id);
            o.put("source", source);
            putNum(o, "target_rpm", targetRpm);
            o.put("p", p);
            o.put("ks", ks);
            o.put("kv", kv);
            o.put("transfer_power", transferPower);
            o.put("max_fire_ms", maxFireMs);
            o.put("turret_position", turretPosition);
            return o;
        }

        static Settings fromJson(JSONObject o) throws JSONException {
            Settings s = new Settings();
            s.id = o.getString("settings_id");
            s.source = "laptop";
            s.targetRpm = o.getDouble("target_rpm");
            s.p = o.getDouble("p");
            s.ks = o.getDouble("ks");
            s.kv = o.getDouble("kv");
            s.transferPower = o.getDouble("transfer_power");
            double mf = o.getDouble("max_fire_ms");
            if (!finite(mf) || mf != Math.rint(mf)) throw new JSONException("max_fire_ms must be a whole number");
            s.maxFireMs = (long) mf;
            s.turretPosition = o.getDouble("turret_position");
            return s;
        }
    }

    private Incredibot robot;
    private IntakeSubsystem intake;
    private LaunchSubsystem launcher;
    private LaunchGateSubsystem gate;
    private TelemetryManager panels;

    private SmartIntakeCommand smartIntake;
    private LaunchBallsCommand launchCommand;
    private ReverseIntakeCommand reverseCommand;

    private Phase phase = Phase.INIT;
    private Settings current;          // what the next trial will use
    private Settings trialSettings;    // frozen copy for the running trial
    private String settingsError = "";
    private boolean linkRequired = true;

    private final String runId = String.format(Locale.US, "%tY%<tm%<td-%<tH%<tM%<tS", System.currentTimeMillis());
    private int trialCounter = 0;
    private int trialId = 0;           // running or last trial
    private boolean trialOpen = false;

    /** A burst opened the gate since the robot was last confirmed empty (refill needs confirmation). */
    private boolean needsEmptyConfirmation = false;
    private int lastBurstTrialId = 0;
    private int refillAuthorizedTrial = 0;
    private final java.util.Set<Integer> revokedTrials = new java.util.HashSet<>();
    private int fireAuthorizedTrial = 0;
    private long lastCommandSeq = -1;
    private String message = "";

    // trial timing / evidence
    private long armedMs, loadedMs, spinupStartMs, stableSinceMs, stableReachedMs, fireMs, launchEndMs, postEndMs;
    private long turretReadyAtMs;
    private int countAtFire, countAtEnd;
    private double rpmLeftAtFire, rpmRightAtFire, batteryAtFire;
    private String completionReason = "";
    private boolean gateOpenedThisTrial = false;
    private double savedTransferPower = Double.NaN;
    private long savedMaxFireMs = -1;

    private StringBuilder rows = new StringBuilder();
    private int rowCount = 0;
    private long lastLoopNs = 0;

    private File baseDir, trialsDir;
    private long lastStatusMs = 0, lastCommandPollMs = 0;

    @Override
    public void runOpMode() {
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        robot = new Incredibot(hardwareMap, panels); // constructors don't move anything
        intake = robot.intakeSubsystem;
        launcher = robot.launchSubsystem;
        gate = robot.launchGateSubsystem;

        baseDir = new File(AppUtil.ROBOT_DATA_DIR, "camera_launch_tuning");
        trialsDir = new File(baseDir, "trials");
        //noinspection ResultOfMethodCallIgnored
        trialsDir.mkdirs();

        current = productionDefaults();
        Log.i(TAG, "run " + runId + " initial settings " + describe(current));

        // ---------------------------------------------------- INIT: sensors only, nothing commanded
        while (opModeInInit()) {
            long now = System.currentTimeMillis();
            pollCommand(now);
            double[] raw = intake.readRawSensorsForDisplay();
            telemetry.addLine("CameraLaunchTuner (INIT): no actuator is commanded until START.");
            telemetry.addData("Sensor 1 laser (mm / baseline)", String.format(Locale.US, "%.1f / %.1f", raw[0], intake.getSensor1BaselineMm()));
            telemetry.addData("Sensor 2 beam", raw[1] > 0 ? "BLOCKED" : "clear");
            telemetry.addData("Sensor 3 beam", raw[2] > 0 ? "BLOCKED" : "clear");
            telemetry.addData("Sensor 4 laser (mm / baseline)", String.format(Locale.US, "%.1f / %.1f", raw[3], intake.getSensor4BaselineMm()));
            telemetry.addData("Flywheel L / R RPM", String.format(Locale.US, "%.0f / %.0f", launcher.getLeftFlywheelRPM(), launcher.getRightFlywheelRPM()));
            telemetry.addData("Battery (V)", String.format(Locale.US, "%.2f", launcher.getBatteryVoltage()));
            telemetry.addData("Settings", describe(current) + (settingsError.isEmpty() ? "" : "  ERROR: " + settingsError));
            telemetry.addData("Files", baseDir.getAbsolutePath());
            telemetry.update();
            if (now - lastStatusMs >= STATUS_INTERVAL_MS) writeStatus(now, raw);
        }
        if (isStopRequested()) return;

        // ---------------------------------------------------- START
        CommandScheduler.getInstance().reset();
        // Drive, odometry and Limelight are deliberately not registered: the drive stays unpowered.
        CommandScheduler.getInstance().registerSubsystem(intake, robot.intakeBarSubsystem, gate, launcher);
        gate.setDefaultCommand(new CloseGateCommand(gate));
        smartIntake = new SmartIntakeCommand(intake, gate, robot.odometrySubsystem);
        reverseCommand = new ReverseIntakeCommand(intake);
        robot.intakeBarSubsystem.raise(); // as TeleOp does at START
        launcher.setTurretPosition(current.turretPosition);
        intake.stopIntakeAndTransfer();
        launcher.stop();
        phase = Phase.IDLE;
        message = "Press gamepad 1 A to load 4 balls.";

        try {
            while (opModeIsActive()) {
                long now = System.currentTimeMillis();
                long loopNs = System.nanoTime();
                double loopMs = lastLoopNs == 0 ? 0 : (loopNs - lastLoopNs) / 1e6;
                lastLoopNs = loopNs;

                CommandScheduler.getInstance().run(); // intake periodic (counting) + commands
                handleClearButtons(now);
                handleButtons(now);
                step(now);
                runFlywheel();
                if (phase == Phase.IDLE || phase == Phase.AWAIT_EMPTY_ACK || phase == Phase.SPINUP || phase == Phase.LOADING) {
                    pollCommand(now);
                }
                if (trialOpen && (phase == Phase.SPINUP || phase == Phase.LAUNCHING || phase == Phase.POST)) {
                    logRow(now, loopMs);
                }
                boolean inBurst = phase == Phase.LAUNCHING;
                if (!inBurst && now - lastStatusMs >= STATUS_INTERVAL_MS) writeStatus(now, null);
                showTelemetry(loopMs);
            }
        } finally {
            restoreLaunchStatics();
            CommandScheduler.getInstance().reset();
            launcher.stop();
            intake.stopIntakeAndTransfer();
            gate.closeGate();
            phase = Phase.INIT;
            writeStatus(System.currentTimeMillis(), null);
        }
    }

    // ------------------------------------------------------------------ main state machine

    private void step(long now) {
        switch (phase) {
            case LOADING:
                // Production FULL handling: run-on, full eject, transfer stop, all inside SmartIntakeCommand.
                if (smartIntake.getState() == SmartIntakeCommand.State.FULL
                        && !intake.isFullEjecting() && intake.isFullAndTransferDone()) {
                    loadedMs = now;
                    startSpinup(now);
                }
                break;
            case SPINUP:
                boolean stable = flywheelStable();
                if (stable) {
                    if (stableSinceMs < 0) stableSinceMs = now;
                } else {
                    stableSinceMs = -1;
                }
                boolean stableLongEnough = stableSinceMs >= 0 && now - stableSinceMs >= STABLE_MS;
                boolean turretReady = now >= turretReadyAtMs;
                if (stableLongEnough && turretReady && stableReachedMs < 0) stableReachedMs = now;
                boolean cameraOk = !linkRequired || fireAuthorizedTrial == trialId;
                if (stableLongEnough && turretReady && cameraOk) {
                    fire(now);
                } else if (stableReachedMs < 0 && now - spinupStartMs > SPINUP_TIMEOUT_MS) {
                    endTrialWithoutBurst(now, "spinup_timeout");
                } else if (stableReachedMs >= 0 && !cameraOk && now - stableReachedMs > CAMERA_WAIT_TIMEOUT_MS) {
                    endTrialWithoutBurst(now, "camera_not_ready");
                }
                break;
            case LAUNCHING:
                if (gate.isGateOpen()) gateOpenedThisTrial = true;
                if (!launchCommand.isScheduled()) {
                    launchEndMs = now;
                    countAtEnd = intake.getArtifactCount();
                    completionReason = launchCommand.getEndReason().name().toLowerCase(Locale.US);
                    restoreLaunchStatics();
                    // The gate is closed again: stop SmartIntakeCommand before it starts intaking
                    // (TeleOp would intake here; a trial only ever loads one set of balls).
                    CommandScheduler.getInstance().cancel(smartIntake);
                    phase = Phase.POST;
                }
                break;
            case POST:
                if (now - launchEndMs >= POST_LOG_MS) {
                    postEndMs = now;
                    finishTrial(now, completionReason);
                }
                break;
            default:
                break;
        }
    }

    private void runFlywheel() {
        Settings s = trialSettings;
        boolean spinning = s != null && (phase == Phase.SPINUP || phase == Phase.LAUNCHING || phase == Phase.POST);
        if (spinning) {
            launcher.updateFlywheelWithGains(s.targetRpm, s.p, LaunchSubsystem.FLYWHEEL_I, LaunchSubsystem.FLYWHEEL_D, s.ks, s.kv);
        } else if (launcher.getTargetRPM() != 0 || launcher.getLastFlywheelPower() != 0) {
            launcher.stop();
        }
    }

    private boolean flywheelStable() {
        double target = trialSettings.targetRpm;
        double l = launcher.getLeftFlywheelRPM(), r = launcher.getRightFlywheelRPM();
        return !launcher.isFlywheelInBangBang()
                && Math.abs(l - target) <= STABLE_TOLERANCE_RPM
                && Math.abs(r - target) <= STABLE_TOLERANCE_RPM
                && Math.abs(l - r) <= MOTOR_MISMATCH_RPM;
    }

    private void arm(long now) {
        if (phase != Phase.IDLE) return;
        if (needsEmptyConfirmation && !refillAuthorized()) {
            message = "Refill refused: confirm empty first (laptop review of trial " + lastBurstTrialId
                    + " must confirm every ball left, or check the dead zone and press X).";
            return;
        }
        if (intake.getArtifactCount() == 0 && (intake.isSensor1Blocked() || intake.isSensor4Blocked())) {
            message = String.format(Locale.US, "Can't arm: sensor %s reads blocked with the count at 0 (laser 1 %.0f/%.0f mm, laser 4 %.0f/%.0f mm, reading/baseline)."
                            + " Remove any ball there, or check the baseline; then try A again.",
                    intake.isSensor1Blocked() ? "1" : "4", intake.getSensor1DistanceMm(), intake.getSensor1BaselineMm(),
                    intake.getSensor4DistanceMm(), intake.getSensor4BaselineMm());
            return;
        }
        String err = current.validate();
        if (!err.isEmpty()) {
            message = "Can't arm: settings invalid (" + err + "). Send settings from the laptop"
                    + (linkRequired ? "." : " or set RPM with the dpad.");
            return;
        }
        if (needsEmptyConfirmation) { // authorized by the laptop review: the robot is empty
            needsEmptyConfirmation = false;
            if (intake.getArtifactCount() != 0) intake.setArtifactCount(0);
        }
        trialCounter++;
        trialId = trialCounter;
        trialOpen = true;
        trialSettings = current.copy();
        armedMs = now;
        loadedMs = spinupStartMs = stableSinceMs = stableReachedMs = fireMs = launchEndMs = postEndMs = -1;
        gateOpenedThisTrial = false;
        completionReason = "";
        countAtFire = -1;
        countAtEnd = -1;
        rows = new StringBuilder();
        rowCount = 0;
        rows.append("t_wall_ms,t_boot_ms,t_rel_ms,phase,target_rpm,left_rpm,right_rpm,mean_rpm,power,bang_bang,battery_v,")
                .append("count,s1_blocked,s1_mm,s2_blocked,s3_blocked,s4_blocked,s4_mm,gate_open,intake_power,transfer_power,")
                .append("smart_state,launch_running,loop_ms\n");
        CommandScheduler.getInstance().schedule(smartIntake);
        phase = Phase.LOADING;
        message = "Trial " + trialId + ": loading. Feed balls until the robot reads 4.";
        Log.i(TAG, "trial " + trialId + " armed with " + describe(trialSettings));
        writeStatus(now, null);
    }

    private void startSpinup(long now) {
        spinupStartMs = now;
        stableSinceMs = -1;
        stableReachedMs = -1;
        double from = launcher.getTurretPosition();
        launcher.setTurretPosition(trialSettings.turretPosition);
        double speed = LaunchReadinessCommand.turretSpeedPosPerSec();
        long travelMs = speed > 0 ? (long) (Math.abs(trialSettings.turretPosition - from) / speed * 1000) : 0;
        turretReadyAtMs = now + travelMs + LaunchReadinessCommand.TURRET_SETTLE_MS + TURRET_EXTRA_SETTLE_MS;
        phase = Phase.SPINUP;
        message = "Trial " + trialId + ": spinning up" + (linkRequired ? ", waiting for stable speed and the laptop camera." : ".");
        writeStatus(now, null);
    }

    private void fire(long now) {
        fireMs = now;
        countAtFire = intake.getArtifactCount();
        rpmLeftAtFire = launcher.getLeftFlywheelRPM();
        rpmRightAtFire = launcher.getRightFlywheelRPM();
        batteryAtFire = launcher.getBatteryVoltage();
        // Freeze the launch settings for the burst; restored when it ends.
        savedTransferPower = SmartIntakeCommand.LAUNCH_TRANSFER_POWER;
        savedMaxFireMs = LaunchBallsCommand.MAX_FIRE_MS;
        SmartIntakeCommand.LAUNCH_TRANSFER_POWER = trialSettings.transferPower;
        LaunchBallsCommand.MAX_FIRE_MS = trialSettings.maxFireMs;
        launchCommand = new LaunchBallsCommand(gate, intake, robot.driveSubsystem, robot.intakeBarSubsystem);
        CommandScheduler.getInstance().schedule(launchCommand);
        needsEmptyConfirmation = true; // the gate opens now: a ball can end up in the dead zone
        lastBurstTrialId = trialId;
        refillAuthorizedTrial = 0;
        phase = Phase.LAUNCHING;
        message = "Trial " + trialId + ": launching.";
        Log.i(TAG, String.format(Locale.US, "trial %d fire: count %d, L/R %.0f/%.0f RPM, %.2f V",
                trialId, countAtFire, rpmLeftAtFire, rpmRightAtFire, batteryAtFire));
        writeStatus(now, null);
    }

    private void restoreLaunchStatics() {
        if (!Double.isNaN(savedTransferPower)) SmartIntakeCommand.LAUNCH_TRANSFER_POWER = savedTransferPower;
        if (savedMaxFireMs >= 0) LaunchBallsCommand.MAX_FIRE_MS = savedMaxFireMs;
        savedTransferPower = Double.NaN;
        savedMaxFireMs = -1;
    }

    /** Spin-up timeout / no camera: the balls are still held below the closed gate. */
    private void endTrialWithoutBurst(long now, String reason) {
        countAtEnd = intake.getArtifactCount();
        finishTrial(now, reason);
    }

    private void finishTrial(long now, String reason) {
        if (!trialOpen) return;
        completionReason = reason;
        CommandScheduler.getInstance().cancel(smartIntake); // end() stops intake + transfer
        launcher.stop();
        if (countAtEnd < 0) countAtEnd = intake.getArtifactCount();
        writeTrialFiles(now);
        trialOpen = false;
        trialSettings = null;
        phase = Phase.IDLE;
        message = "Trial " + trialId + " done (" + reason + "). "
                + (needsEmptyConfirmation ? "Waiting for the laptop review or X (empty) before refilling." : "Press A to load.");
        Log.i(TAG, "trial " + trialId + " finished: " + reason);
        writeStatus(now, null);
    }

    private void abort(long now, String reason) {
        if (!trialOpen) return;
        if (launchCommand != null && launchCommand.isScheduled()) {
            CommandScheduler.getInstance().cancel(launchCommand); // end() closes the gate
            gateOpenedThisTrial = true;
        }
        restoreLaunchStatics();
        if (launchEndMs < 0 && fireMs >= 0) launchEndMs = now;
        countAtEnd = intake.getArtifactCount();
        gate.closeGate();
        finishTrial(now, reason);
    }

    // ------------------------------------------------------------------ buttons

    private void handleClearButtons(long now) {
        boolean held = gamepad1.left_trigger > 0.5 || gamepad2.back;
        if (held && phase != Phase.CLEARING) {
            abort(now, "operator_clear");
            CommandScheduler.getInstance().cancel(smartIntake);
            launcher.stop();
            gate.closeGate();
            CommandScheduler.getInstance().schedule(reverseCommand);
            phase = Phase.CLEARING;
            message = "Clearing: reversing intake and transfer.";
            writeStatus(now, null);
        } else if (!held && phase == Phase.CLEARING) {
            CommandScheduler.getInstance().cancel(reverseCommand); // end() stops intake + transfer
            needsEmptyConfirmation = true; // sensors clearing doesn't prove the dead zone is empty
            refillAuthorizedTrial = 0;
            phase = Phase.AWAIT_EMPTY_ACK;
            message = "Check the robot (dead zone too) is empty, then press X. Hold LT to reverse again.";
            writeStatus(now, null);
        }
    }

    private void handleButtons(long now) {
        if (gamepad1.bWasPressed()) {
            if (trialOpen) {
                abort(now, "aborted");
                message = "Trial " + trialId + " aborted." + (needsEmptyConfirmation ? " Confirm empty before refilling." : "");
            }
        }
        if (gamepad1.xWasPressed() && (phase == Phase.IDLE || phase == Phase.AWAIT_EMPTY_ACK)) {
            intake.setArtifactCount(0);
            needsEmptyConfirmation = false;
            refillAuthorizedTrial = 0;
            phase = Phase.IDLE;
            message = "Empty acknowledged. Press A to load.";
            Log.i(TAG, "operator acknowledged physically empty");
            writeStatus(now, null);
        }
        if (gamepad1.aWasPressed()) {
            if (phase == Phase.IDLE) arm(now);
            else if (phase == Phase.AWAIT_EMPTY_ACK) message = "Press X to confirm empty first.";
        }
        if (phase == Phase.IDLE) {
            if (gamepad1.yWasPressed()) {
                linkRequired = !linkRequired;
                message = "Laptop link " + (linkRequired ? "REQUIRED" : "off (fires without the camera)");
            }
            if (!linkRequired) {
                boolean changed = false;
                if (gamepad1.dpadUpWasPressed()) { current = manualCopy(); current.targetRpm = baseRpm() + RPM_STEP; changed = true; }
                if (gamepad1.dpadDownWasPressed()) { current = manualCopy(); current.targetRpm = baseRpm() - RPM_STEP; changed = true; }
                if (gamepad1.dpadRightWasPressed()) { current = manualCopy(); current.turretPosition = clampTurret(current.turretPosition + TURRET_STEP); changed = true; }
                if (gamepad1.dpadLeftWasPressed()) { current = manualCopy(); current.turretPosition = clampTurret(current.turretPosition - TURRET_STEP); changed = true; }
                if (changed) {
                    launcher.setTurretPosition(current.turretPosition);
                    message = "Manual settings: " + describe(current);
                }
            }
        }
    }

    private Settings manualCopy() {
        Settings s = current.copy();
        s.source = "manual";
        s.id = "manual-" + System.currentTimeMillis();
        return s;
    }

    private double baseRpm() {
        if (finite(current.targetRpm)) return current.targetRpm;
        double[] rpms = LaunchReadinessCommand.SHOT_RPMS;
        return rpms.length > 0 ? rpms[0] : LIMIT_RPM_MIN;
    }

    private static double clampTurret(double v) {
        return Math.max(LaunchSubsystem.TURRET_MIN, Math.min(LaunchSubsystem.TURRET_MAX, v));
    }

    private boolean refillAuthorized() {
        return refillAuthorizedTrial != 0 && refillAuthorizedTrial == lastBurstTrialId
                && !revokedTrials.contains(refillAuthorizedTrial)
                && intake.getArtifactCount() == 0 && !intake.isSensor1Blocked() && !intake.isSensor4Blocked();
    }

    // ------------------------------------------------------------------ laptop link

    private Settings productionDefaults() {
        Settings s = new Settings();
        s.p = LaunchSubsystem.FLYWHEEL_P;
        s.ks = LaunchSubsystem.FLYWHEEL_KS;
        s.kv = LaunchSubsystem.FLYWHEEL_KV;
        s.transferPower = SmartIntakeCommand.LAUNCH_TRANSFER_POWER;
        s.maxFireMs = LaunchBallsCommand.MAX_FIRE_MS;
        s.turretPosition = clampTurret(CrossOpModeStorage.turretPosition);
        s.targetRpm = Double.NaN; // must be chosen for the spot
        return s;
    }

    private void pollCommand(long now) {
        if (now - lastCommandPollMs < COMMAND_POLL_MS) return;
        lastCommandPollMs = now;
        File f = new File(baseDir, "command.json");
        if (!f.exists()) return;
        JSONObject cmd;
        try {
            cmd = new JSONObject(readFile(f));
        } catch (IOException | JSONException e) {
            settingsError = "command.json unreadable: " + e.getMessage();
            return;
        }
        try {
            if (cmd.optInt("protocol", -1) != PROTOCOL_VERSION) {
                settingsError = "command.json protocol mismatch";
                return;
            }
            long seq = cmd.optLong("seq", -1);
            if (!runId.equals(cmd.optString("robot_run_id", runId)) && cmd.has("robot_run_id")) {
                // Commands addressed to an earlier OpMode run: only settings carry over.
                cmd.remove("fire_trial");
                cmd.remove("refill_authorized_trial");
            }
            JSONArray revoked = cmd.optJSONArray("revoked_trials");
            if (revoked != null) {
                for (int i = 0; i < revoked.length(); i++) revokedTrials.add(revoked.getInt(i));
                if (revokedTrials.contains(refillAuthorizedTrial)) refillAuthorizedTrial = 0;
            }
            if (phase == Phase.SPINUP || phase == Phase.LOADING) {
                fireAuthorizedTrial = cmd.optInt("fire_trial", fireAuthorizedTrial);
            }
            if (phase == Phase.IDLE || phase == Phase.INIT || phase == Phase.AWAIT_EMPTY_ACK) {
                int refill = cmd.optInt("refill_authorized_trial", 0);
                if (refill != 0 && !revokedTrials.contains(refill)) refillAuthorizedTrial = refill;
                if (cmd.has("link_required")) linkRequired = cmd.getBoolean("link_required");
                JSONObject so = cmd.optJSONObject("settings");
                if (so != null && !so.optString("settings_id", "").equals(current.id)) {
                    try {
                        Settings s = Settings.fromJson(so);
                        String err = s.validate();
                        if (err.isEmpty()) {
                            current = s;
                            settingsError = "";
                            if (phase != Phase.INIT) launcher.setTurretPosition(current.turretPosition);
                            Log.i(TAG, "settings applied: " + describe(current));
                        } else {
                            settingsError = "rejected " + so.optString("settings_id") + ": " + err;
                        }
                    } catch (JSONException e) {
                        settingsError = "rejected settings: " + e.getMessage();
                    }
                }
            }
            lastCommandSeq = Math.max(lastCommandSeq, seq);
        } catch (JSONException e) {
            settingsError = "command.json invalid: " + e.getMessage();
        }
    }

    private void writeStatus(long now, double[] rawSensors) {
        lastStatusMs = now;
        try {
            JSONObject o = new JSONObject();
            o.put("protocol", PROTOCOL_VERSION);
            o.put("opmode", "CameraLaunchTuner");
            o.put("robot_run_id", runId);
            o.put("wall_ms", now);
            o.put("boot_ms", SystemClock.elapsedRealtime());
            o.put("phase", phase.name());
            o.put("trial_id", trialId);
            o.put("trial_open", trialOpen);
            o.put("last_burst_trial_id", lastBurstTrialId);
            o.put("needs_empty_confirmation", needsEmptyConfirmation);
            o.put("refill_authorized_trial", refillAuthorizedTrial);
            o.put("fire_authorized_trial", fireAuthorizedTrial);
            o.put("link_required", linkRequired);
            o.put("command_seq_seen", lastCommandSeq);
            o.put("settings", current.toJson());
            o.put("trial_settings", trialSettings != null ? trialSettings.toJson() : JSONObject.NULL);
            o.put("settings_error", settingsError);
            o.put("message", message);
            o.put("artifact_count", intake.getArtifactCount());
            o.put("left_rpm", launcher.getLeftFlywheelRPM());
            o.put("right_rpm", launcher.getRightFlywheelRPM());
            o.put("battery_v", launcher.getBatteryVoltage());
            if (rawSensors != null) {
                o.put("s1_mm", rawSensors[0]);
                o.put("s2_blocked", rawSensors[1] > 0);
                o.put("s3_blocked", rawSensors[2] > 0);
                o.put("s4_mm", rawSensors[3]);
            } else {
                o.put("s1_mm", intake.getSensor1DistanceMm());
                o.put("s1_blocked", intake.isSensor1Blocked());
                o.put("s2_blocked", intake.isSensor2Blocked());
                o.put("s3_blocked", intake.isSensor3Blocked());
                o.put("s4_mm", intake.getSensor4DistanceMm());
                o.put("s4_blocked", intake.isSensor4Blocked());
            }
            o.put("s1_baseline_mm", intake.getSensor1BaselineMm());
            o.put("s4_baseline_mm", intake.getSensor4BaselineMm());
            o.put("production", productionSnapshot());
            o.put("limits", limitsSnapshot());
            writeAtomic(new File(baseDir, "status.json"), o.toString(2));
        } catch (JSONException | IOException e) {
            Log.w(TAG, "status write failed: " + e);
        }
    }

    private JSONObject productionSnapshot() throws JSONException {
        JSONObject o = new JSONObject();
        o.put("shot_distances_in", toArray(LaunchReadinessCommand.SHOT_DISTANCES_IN));
        o.put("shot_rpms", toArray(LaunchReadinessCommand.SHOT_RPMS));
        o.put("use_gain_table", LaunchSubsystem.USE_GAIN_TABLE);
        o.put("gain_table_distances_in", toArray(LaunchSubsystem.GAIN_TABLE_DISTANCES_IN));
        o.put("gain_table_p", toArray(LaunchSubsystem.GAIN_TABLE_P));
        o.put("gain_table_ks", toArray(LaunchSubsystem.GAIN_TABLE_KS));
        o.put("gain_table_kv", toArray(LaunchSubsystem.GAIN_TABLE_KV));
        o.put("flywheel_p", LaunchSubsystem.FLYWHEEL_P);
        o.put("flywheel_i", LaunchSubsystem.FLYWHEEL_I);
        o.put("flywheel_d", LaunchSubsystem.FLYWHEEL_D);
        o.put("flywheel_ks", LaunchSubsystem.FLYWHEEL_KS);
        o.put("flywheel_kv", LaunchSubsystem.FLYWHEEL_KV);
        o.put("bang_bang_enabled", LaunchSubsystem.FLYWHEEL_BANG_BANG_ENABLED);
        o.put("bang_bang_threshold_rpm", LaunchSubsystem.FLYWHEEL_BANG_BANG_THRESHOLD_RPM);
        o.put("voltage_compensation", LaunchSubsystem.FLYWHEEL_VOLTAGE_COMPENSATION);
        o.put("nominal_voltage", LaunchSubsystem.NOMINAL_VOLTAGE);
        // While a burst runs the statics hold the trial values; report the production ones.
        o.put("launch_transfer_power", !Double.isNaN(savedTransferPower) ? savedTransferPower : SmartIntakeCommand.LAUNCH_TRANSFER_POWER);
        o.put("max_fire_ms", savedMaxFireMs >= 0 ? savedMaxFireMs : LaunchBallsCommand.MAX_FIRE_MS);
        o.put("clear_ms", LaunchBallsCommand.CLEAR_MS);
        o.put("simple_launch", LaunchBallsCommand.SIMPLE_LAUNCH);
        o.put("max_artifacts", IntakeSubsystem.MAX_ARTIFACTS);
        o.put("turret_min", LaunchSubsystem.TURRET_MIN);
        o.put("turret_max", LaunchSubsystem.TURRET_MAX);
        o.put("turret_mid", LaunchSubsystem.TURRET_MID);
        o.put("stored_turret_position", CrossOpModeStorage.turretPosition);
        return o;
    }

    private JSONObject limitsSnapshot() throws JSONException {
        JSONObject o = new JSONObject();
        o.put("target_rpm", new JSONArray().put(LIMIT_RPM_MIN).put(LIMIT_RPM_MAX));
        o.put("p", new JSONArray().put(0).put(LIMIT_P_MAX));
        o.put("ks", new JSONArray().put(0).put(LIMIT_KS_MAX));
        o.put("kv", new JSONArray().put(0).put(LIMIT_KV_MAX));
        o.put("transfer_power", new JSONArray().put(LIMIT_TRANSFER_MIN).put(LIMIT_TRANSFER_MAX));
        o.put("max_fire_ms", new JSONArray().put(LIMIT_MAX_FIRE_MS_MIN).put(LIMIT_MAX_FIRE_MS_MAX));
        o.put("turret_position", new JSONArray().put(LaunchSubsystem.TURRET_MIN).put(LaunchSubsystem.TURRET_MAX));
        o.put("stable_tolerance_rpm", STABLE_TOLERANCE_RPM);
        o.put("stable_ms", STABLE_MS);
        return o;
    }

    // ------------------------------------------------------------------ trial files

    private void logRow(long now, double loopMs) {
        if (rowCount >= MAX_ROWS) return;
        rowCount++;
        double l = launcher.getLeftFlywheelRPM(), r = launcher.getRightFlywheelRPM();
        rows.append(now).append(',').append(SystemClock.elapsedRealtime()).append(',').append(now - armedMs).append(',')
                .append(phase.name()).append(',')
                .append(fmt(trialSettings != null ? trialSettings.targetRpm : 0)).append(',')
                .append(fmt(l)).append(',').append(fmt(r)).append(',').append(fmt((l + r) / 2)).append(',')
                .append(fmt4(launcher.getLastFlywheelPower())).append(',').append(launcher.isFlywheelInBangBang() ? 1 : 0).append(',')
                .append(fmt(launcher.getBatteryVoltage())).append(',')
                .append(intake.getArtifactCount()).append(',')
                .append(intake.isSensor1Blocked() ? 1 : 0).append(',').append(fmt(intake.getSensor1DistanceMm())).append(',')
                .append(intake.isSensor2Blocked() ? 1 : 0).append(',').append(intake.isSensor3Blocked() ? 1 : 0).append(',')
                .append(intake.isSensor4Blocked() ? 1 : 0).append(',').append(fmt(intake.getSensor4DistanceMm())).append(',')
                .append(gate.isGateOpen() ? 1 : 0).append(',')
                .append(fmt(intake.getIntakeMotorPower())).append(',').append(fmt(intake.getTransferMotorPower())).append(',')
                .append(smartIntake.getState()).append(',')
                .append(launchCommand != null && launchCommand.isScheduled() ? 1 : 0).append(',')
                .append(fmt(loopMs)).append('\n');
    }

    private void writeTrialFiles(long now) {
        String base = String.format(Locale.US, "trial_%s_%04d", runId, trialId);
        try {
            writeAtomic(new File(trialsDir, base + ".csv"), rows.toString());
            JSONObject o = new JSONObject();
            o.put("protocol", PROTOCOL_VERSION);
            o.put("robot_run_id", runId);
            o.put("trial_id", trialId);
            o.put("telemetry_file", base + ".csv");
            o.put("telemetry_rows", rowCount);
            o.put("telemetry_truncated", rowCount >= MAX_ROWS);
            o.put("settings", trialSettings.toJson());
            o.put("completion_reason", completionReason);
            o.put("burst_started", fireMs >= 0);
            o.put("gate_opened", gateOpenedThisTrial || fireMs >= 0);
            o.put("count_at_fire", fireMs >= 0 ? countAtFire : JSONObject.NULL);
            o.put("count_at_end", countAtEnd);
            o.put("robot_counted_exits", fireMs >= 0 ? Math.max(0, countAtFire - countAtEnd) : 0);
            o.put("empty_confirmed_at_timeout", launchCommand != null && fireMs >= 0 && launchCommand.wasEmptyConfirmedAtTimeout());
            o.put("left_rpm_at_fire", fireMs >= 0 ? rpmLeftAtFire : JSONObject.NULL);
            o.put("right_rpm_at_fire", fireMs >= 0 ? rpmRightAtFire : JSONObject.NULL);
            o.put("battery_v_at_fire", fireMs >= 0 ? batteryAtFire : JSONObject.NULL);
            JSONObject t = new JSONObject();
            t.put("armed_ms", armedMs);
            putTime(t, "loaded_ms", loadedMs);
            putTime(t, "spinup_start_ms", spinupStartMs);
            putTime(t, "stable_ms", stableReachedMs);
            putTime(t, "fire_ms", fireMs);
            putTime(t, "launch_end_ms", launchEndMs);
            putTime(t, "post_end_ms", postEndMs);
            t.put("finished_ms", now);
            t.put("clock", "System.currentTimeMillis (wall)");
            o.put("timestamps", t);
            o.put("stable_tolerance_rpm", STABLE_TOLERANCE_RPM);
            o.put("stable_window_ms", STABLE_MS);
            o.put("link_required", linkRequired);
            o.put("sensors_at_end", sensorsJson());
            o.put("production", productionSnapshot());
            writeAtomic(new File(trialsDir, base + ".json"), o.toString(2));
        } catch (JSONException | IOException e) {
            Log.e(TAG, "trial file write failed: " + e);
        }
    }

    private JSONObject sensorsJson() throws JSONException {
        JSONObject s = new JSONObject();
        s.put("s1_blocked", intake.isSensor1Blocked());
        s.put("s1_mm", intake.getSensor1DistanceMm());
        s.put("s2_blocked", intake.isSensor2Blocked());
        s.put("s3_blocked", intake.isSensor3Blocked());
        s.put("s4_blocked", intake.isSensor4Blocked());
        s.put("s4_mm", intake.getSensor4DistanceMm());
        return s;
    }

    // ------------------------------------------------------------------ display

    private void showTelemetry(double loopMs) {
        telemetry.addData("Phase", phase + (trialOpen ? " (trial " + trialId + ")" : ""));
        telemetry.addLine(message);
        telemetry.addData("Settings", describe(trialSettings != null ? trialSettings : current));
        if (!settingsError.isEmpty()) telemetry.addData("Settings error", settingsError);
        telemetry.addData("Laptop link", linkRequired ? "required (gamepad 1 Y toggles in idle)" : "off");
        telemetry.addData("Count", intake.getArtifactCount() + (needsEmptyConfirmation ? "  (refill needs empty confirmation)" : ""));
        telemetry.addData("Flywheel L / R / target", String.format(Locale.US, "%.0f / %.0f / %.0f",
                launcher.getLeftFlywheelRPM(), launcher.getRightFlywheelRPM(), launcher.getTargetRPM()));
        telemetry.addData("Sensors 1-4", (intake.isSensor1Blocked() ? "X" : "-") + (intake.isSensor2Blocked() ? "X" : "-")
                + (intake.isSensor3Blocked() ? "X" : "-") + (intake.isSensor4Blocked() ? "X" : "-"));
        telemetry.addData("Battery / loop", String.format(Locale.US, "%.2f V / %.1f ms", launcher.getBatteryVoltage(), loopMs));
        telemetry.addLine("A load | B abort | X empty ack | hold LT (or gp2 BACK) reverse");
        telemetry.update();
    }

    private static String describe(Settings s) {
        return String.format(Locale.US, "%s: %s RPM, P %.4f, kS %.3f, kV %.6f, transfer %.2f, timeout %d ms, turret %.3f",
                s.id, finite(s.targetRpm) ? String.format(Locale.US, "%.0f", s.targetRpm) : "unset",
                s.p, s.ks, s.kv, s.transferPower, s.maxFireMs, s.turretPosition);
    }

    // ------------------------------------------------------------------ helpers

    private static boolean finite(double v) {
        return !Double.isNaN(v) && !Double.isInfinite(v);
    }

    private static void putNum(JSONObject o, String k, double v) throws JSONException {
        o.put(k, finite(v) ? (Object) v : JSONObject.NULL);
    }

    private static void putTime(JSONObject o, String k, long v) throws JSONException {
        o.put(k, v >= 0 ? (Object) v : JSONObject.NULL);
    }

    private static JSONArray toArray(double[] values) throws JSONException {
        JSONArray a = new JSONArray();
        for (double v : values) a.put(v);
        return a;
    }

    private static String fmt(double v) {
        return String.format(Locale.US, "%.2f", v);
    }

    private static String fmt4(double v) {
        return String.format(Locale.US, "%.4f", v);
    }

    private static String readFile(File f) throws IOException {
        StringBuilder sb = new StringBuilder();
        BufferedReader r = new BufferedReader(new FileReader(f));
        try {
            String line;
            while ((line = r.readLine()) != null) sb.append(line).append('\n');
        } finally {
            r.close();
        }
        return sb.toString();
    }

    /** Writes to a temp file and renames it over the target, so readers never see half a file. */
    private static void writeAtomic(File target, String content) throws IOException {
        File tmp = new File(target.getParentFile(), target.getName() + ".tmp");
        FileWriter w = new FileWriter(tmp, false);
        try {
            w.write(content);
        } finally {
            w.close();
        }
        if (!tmp.renameTo(target)) {
            //noinspection ResultOfMethodCallIgnored
            target.delete();
            if (!tmp.renameTo(target)) throw new IOException("rename failed for " + target);
        }
    }
}
