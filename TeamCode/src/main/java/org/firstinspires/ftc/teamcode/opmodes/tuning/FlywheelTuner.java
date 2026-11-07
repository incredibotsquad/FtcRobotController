package org.firstinspires.ftc.teamcode.opmodes.tuning;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Incredibot;
import org.firstinspires.ftc.teamcode.commands.LaunchBallsCommand;
import org.firstinspires.ftc.teamcode.commands.LaunchReadinessCommand;
import org.firstinspires.ftc.teamcode.commands.RelocalizeWithLimelightCommand;
import org.firstinspires.ftc.teamcode.common.CrossOpModeStorage;
import org.firstinspires.ftc.teamcode.subsystems.LaunchSubsystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Flywheel tuner that runs the real TeleOp: same Incredibot, same INIT/START sequence, same default
 * commands (LaunchReadinessCommand aims and picks RPM + gains from the distance tables,
 * SmartIntakeCommand, CloseGateCommand, DriveRobotCommand) and the same operator bindings on
 * gamepad 2, including RB = LaunchBallsCommand. What you tune here is what TeleOp does.
 * (For the first kS/kV tune at fixed RPMs, use FlywheelBenchTuner.)
 *
 * Tuning is per POSITION: drive (gamepad 1 sticks) so the robot is within ROW_SNAP_IN of one of
 * the table distances (LaunchReadinessCommand.SHOT_DISTANCES_IN / LaunchSubsystem.GAIN_TABLE_DISTANCES_IN),
 * then edit that row. Edits are ignored away from a row, since in between the values are
 * interpolated. The odometry pose must be right for the distance to be: start from a known pose, or
 * relocalize (operator Y, corner stance) before tuning.
 *
 * Controls, gamepad 1 (gamepad 2 is the normal operator):
 *   Sticks        - drive (as TeleOp)
 *   Start         - fire: LaunchBallsCommand, same as operator RB (for tuning alone)
 *   A / X         - shot RPM for this row +/- RPM_STEP (does it hit the CELL?)
 *   Dpad up/down  - P +/- step        (gain row for this distance, or FLYWHEEL_* if
 *   Dpad R/L      - kS +/- step        LaunchSubsystem.USE_GAIN_TABLE is off)
 *   RB / LB       - kV +/- step
 *   B             - cycle gain step size
 *   Back          - auto-adjust kV from the current speed (P = 0, bang-bang off, flywheel settled)
 *   Y             - log all tables to logcat (tag FlywheelTuner) to copy into the code
 *   Left stick    - (press) reset statistics
 *
 * Shots are detected from the RPM while the gate is open: a drop of SHOT_DETECT_DROP_RPM below the
 * recent peak is a shot.
 * For each launch (gate open to closed) it lists the RPM each artifact left at and whether the
 * flywheel recovered before the next one. Spread in those numbers = inconsistent shots.
 *
 * Edits change the live static values, so TeleOp uses them until the app restarts. Copy them into
 * the code (Y logs them) to keep them. Panels is not legal in matches (R704).
 */
@Configurable
@Disabled
@TeleOp(name = "FlywheelTuner", group = "Tests")
public class FlywheelTuner extends LinearOpMode {

    /** Edits only apply within this many inches of a table distance. */
    public static double ROW_SNAP_IN = 3.0;
    public static double RPM_STEP = 25;
    /** A drop of more than this below the recent peak counts as a shot. */
    public static double SHOT_DETECT_DROP_RPM = 75;
    /** After a shot, RPM must rise this much above the dip's low point before the next shot can be detected. */
    public static double SHOT_REARM_RPM = 30;

    private static final String TAG = "FlywheelTuner";

    private final double[] stepSizes = {0.1, 0.01, 0.001, 0.0001, 0.00001};
    private int stepIndex = 2;

    // ---- shot detection
    private boolean inDip = false;
    private double peakRpm = 0, dipMinRpm = 0;
    private long lastShotMs = -1;
    private boolean recoveredSinceShot = true;
    private long lastRecoveryMs = -1;

    // ---- current launch (gate open -> closed)
    private boolean gateWasOpen = false;
    private final List<String> burstShots = new ArrayList<>();
    private String burstHeader = "no launch yet";
    private int totalShots = 0;
    private double worstDipRpm = 0;

    private LaunchSubsystem launcher;

    @Override
    public void runOpMode() {
        TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
        GamepadEx driverGamepad = new GamepadEx(gamepad1);
        GamepadEx operatorGamepad = new GamepadEx(gamepad2);
        GamepadEx tuningPad = new GamepadEx(gamepad1); // separate instance: edge detection for tuning buttons

        // ---------------------------------------------------- INIT: same as IncredibotsMecanumDrive
        Incredibot incredibot = new Incredibot(hardwareMap, panels);
        launcher = incredibot.launchSubsystem;
        RelocalizeWithLimelightCommand relocalize = new RelocalizeWithLimelightCommand(
                incredibot.limelightSubsystem, incredibot.odometrySubsystem, incredibot.driveSubsystem);

        incredibot.intakeBarSubsystem.raise();
        incredibot.odometrySubsystem.resetPose(CrossOpModeStorage.currentPose.getX(), CrossOpModeStorage.currentPose.getY(),
                Math.toDegrees(CrossOpModeStorage.currentPose.getHeading()));
        if (!Incredibot.DRIVE_ONLY_MODE) relocalize.schedule();

        while (opModeInInit()) {
            CommandScheduler.getInstance().run();
            panels.addLine("FlywheelTuner: runs the real TeleOp. Flywheel spins up on START.");
            if (Incredibot.DRIVE_ONLY_MODE) panels.addLine("WARNING: DRIVE_ONLY_MODE is on, the launcher won't run.");
            panels.addData("Start pose", CrossOpModeStorage.currentPose);
            panels.update(telemetry);
        }
        waitForStart();

        if (!Incredibot.DRIVE_ONLY_MODE) {
            incredibot.launchSubsystem.setTurretPosition(CrossOpModeStorage.turretPosition);
        }
        incredibot.initialize(Incredibot.OpModeType.TELEOP, driverGamepad, operatorGamepad);

        long lastLoopNs = System.nanoTime();
        double loopMs = 0;

        // ---------------------------------------------------- main loop: TeleOp + tuning
        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();

            LaunchReadinessCommand readiness = incredibot.launchReadinessCommand;
            double distance = readiness == null ? Double.NaN : readiness.getLastDistanceIn();
            int shotRow = nearestRow(LaunchReadinessCommand.SHOT_DISTANCES_IN, distance);
            int gainRow = LaunchSubsystem.USE_GAIN_TABLE ? nearestRow(LaunchSubsystem.GAIN_TABLE_DISTANCES_IN, distance) : -1;
            boolean useTable = LaunchSubsystem.USE_GAIN_TABLE;
            boolean canEditGains = !useTable || gainRow >= 0;

            // ------------------------------------------------ tuning buttons
            tuningPad.readButtons();
            double step = stepSizes[stepIndex];

            if (tuningPad.wasJustPressed(GamepadKeys.Button.START)) {
                new LaunchBallsCommand(incredibot.launchGateSubsystem, incredibot.intakeSubsystem, incredibot.driveSubsystem,
                        incredibot.intakeBarSubsystem).schedule();
            }
            if (tuningPad.wasJustPressed(GamepadKeys.Button.B)) stepIndex = (stepIndex + 1) % stepSizes.length;
            if (tuningPad.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) resetStats();
            if (tuningPad.wasJustPressed(GamepadKeys.Button.Y)) logTables();

            if (shotRow >= 0) {
                if (tuningPad.wasJustPressed(GamepadKeys.Button.A)) LaunchReadinessCommand.SHOT_RPMS[shotRow] += RPM_STEP;
                if (tuningPad.wasJustPressed(GamepadKeys.Button.X)) {
                    LaunchReadinessCommand.SHOT_RPMS[shotRow] = Math.max(0, LaunchReadinessCommand.SHOT_RPMS[shotRow] - RPM_STEP);
                }
            }
            if (canEditGains) {
                if (tuningPad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) addGain(GainColumn.P, gainRow, step);
                if (tuningPad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) addGain(GainColumn.P, gainRow, -step);
                if (tuningPad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) addGain(GainColumn.KS, gainRow, step);
                if (tuningPad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) addGain(GainColumn.KS, gainRow, -step);
                if (tuningPad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) addGain(GainColumn.KV, gainRow, step);
                if (tuningPad.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) addGain(GainColumn.KV, gainRow, -step);

                // Back: rescale kV so feedforward alone gives the current speed (see FlywheelBenchTuner).
                if (tuningPad.wasJustPressed(GamepadKeys.Button.BACK)) {
                    double actualTPS = launcher.getFlywheelVelocityTPS();
                    double targetTPS = LaunchSubsystem.rpmToTps(launcher.getTargetRPM());
                    if (actualTPS > 100 && targetTPS > 0) {
                        double kv = getGain(GainColumn.KV, gainRow);
                        addGain(GainColumn.KV, gainRow, kv * targetTPS / actualTPS - kv);
                    }
                }
            }

            // ------------------------------------------------ measurements
            double rpm = launcher.getCurrentFlywheelRPM();
            double target = launcher.getTargetRPM();
            updateShots(rpm, target, incredibot.launchGateSubsystem.isGateOpen(), distance);

            long nowNs = System.nanoTime();
            loopMs = 0.9 * loopMs + 0.1 * ((nowNs - lastLoopNs) / 1e6);
            lastLoopNs = nowNs;

            // ------------------------------------------------ telemetry (every loop, like TeleOp)
            panels.addData("targetRPM", target);
            panels.addData("actualRPM", rpm);
            panels.addData("power", launcher.getLastFlywheelPower());

            panels.addLine("--- FlywheelTuner ---");
            panels.addData("Distance (in)", Double.isNaN(distance) ? "-" : String.format("%.1f", distance));
            panels.addData("Shot row", shotRow >= 0
                    ? String.format("%d (%.0f in): RPM %.0f  [A +%.0f / X -%.0f]", shotRow,
                            LaunchReadinessCommand.SHOT_DISTANCES_IN[shotRow], LaunchReadinessCommand.SHOT_RPMS[shotRow], RPM_STEP, RPM_STEP)
                    : "none: drive within " + ROW_SNAP_IN + " in of " + Arrays.toString(LaunchReadinessCommand.SHOT_DISTANCES_IN));
            panels.addData("Gain row", !useTable ? "USE_GAIN_TABLE off: editing FLYWHEEL_*"
                    : gainRow >= 0 ? String.format("%d (%.0f in)", gainRow, LaunchSubsystem.GAIN_TABLE_DISTANCES_IN[gainRow])
                    : "none: drive within " + ROW_SNAP_IN + " in of " + Arrays.toString(LaunchSubsystem.GAIN_TABLE_DISTANCES_IN));
            if (canEditGains) {
                panels.addData("P (dpad U/D) / kS (dpad R/L) / kV (RB/LB)", String.format("%.5f / %.4f / %.6f",
                        getGain(GainColumn.P, gainRow), getGain(GainColumn.KS, gainRow), getGain(GainColumn.KV, gainRow)));
            }
            panels.addData("Step (B)", step);
            panels.addData("Mode / at speed", (launcher.isFlywheelInBangBang() ? "BANG-BANG" : "PIDF")
                    + " / " + launcher.isFlywheelAtSpeed());
            panels.addData("Battery V / loop ms", String.format("%.2f / %.1f", launcher.getBatteryVoltage(), loopMs));
            panels.addData("Last launch", burstHeader);
            for (String shot : burstShots) panels.addLine("  " + shot);
            panels.addData("Shots total / worst dip", totalShots + " / " + Math.round(worstDipRpm));
            panels.addData("Recovery ms (last)", lastRecoveryMs >= 0 ? String.valueOf(lastRecoveryMs) : "-");

            panels.update(telemetry);
        }

        CommandScheduler.getInstance().reset();
    }

    // ------------------------------------------------------------------ shots

    /**
     * Peak/dip shot detection that works within a burst (shots can come before the flywheel has
     * recovered). Each shot records the RPM the artifact left at (the peak before the drop).
     */
    private void updateShots(double rpm, double target, boolean gateOpen, double distance) {
        long now = System.nanoTime() / 1_000_000L;

        if (gateOpen && !gateWasOpen) { // new launch
            burstShots.clear();
            burstHeader = String.format("target %.0f rpm at %.1f in, %.2f V", target, distance, launcher.getBatteryVoltage());
            peakRpm = rpm;
            inDip = false;
        }
        gateWasOpen = gateOpen;

        if (!gateOpen) {
            // Only count shots while the gate is open: with it closed, a drop is the target changing
            // (e.g. driving to a nearer row), not an artifact.
            peakRpm = rpm;
            inDip = false;
        } else if (!inDip) {
            peakRpm = Math.max(peakRpm, rpm);
            if (peakRpm - rpm > SHOT_DETECT_DROP_RPM) {
                if (lastShotMs >= 0 && !recoveredSinceShot && !burstShots.isEmpty()) {
                    int last = burstShots.size() - 1;
                    burstShots.set(last, burstShots.get(last) + ", next shot before recovery");
                }
                inDip = true;
                dipMinRpm = rpm;
                lastShotMs = now;
                recoveredSinceShot = false;
                totalShots++;
                burstShots.add(String.format("shot %d: left at %.0f rpm (%+.0f vs target)",
                        burstShots.size() + 1, peakRpm, peakRpm - target));
            }
        } else {
            dipMinRpm = Math.min(dipMinRpm, rpm);
            if (rpm - dipMinRpm > SHOT_REARM_RPM) {
                worstDipRpm = Math.max(worstDipRpm, peakRpm - dipMinRpm);
                inDip = false;
                peakRpm = rpm;
            }
        }

        if (!recoveredSinceShot && lastShotMs >= 0 && Math.abs(target - rpm) < LaunchSubsystem.TARGET_RPM_TOLERANCE) {
            recoveredSinceShot = true;
            lastRecoveryMs = now - lastShotMs;
        }
    }

    private void resetStats() {
        burstShots.clear();
        burstHeader = "no launch yet";
        totalShots = 0;
        worstDipRpm = 0;
        lastRecoveryMs = -1;
        lastShotMs = -1;
        recoveredSinceShot = true;
    }

    // ------------------------------------------------------------------ table editing

    private enum GainColumn { P, KS, KV }

    /** Index of the table distance within ROW_SNAP_IN of {@code distance}, or -1. */
    private static int nearestRow(double[] distances, double distance) {
        if (Double.isNaN(distance)) return -1;
        int best = -1;
        for (int i = 0; i < distances.length; i++) {
            if (Math.abs(distances[i] - distance) <= ROW_SNAP_IN
                    && (best < 0 || Math.abs(distances[i] - distance) < Math.abs(distances[best] - distance))) {
                best = i;
            }
        }
        return best;
    }

    /** Gain row {@code row} of the table, or the FLYWHEEL_* single set when the table is off (row ignored). */
    private static double getGain(GainColumn c, int row) {
        if (!LaunchSubsystem.USE_GAIN_TABLE) {
            switch (c) {
                case P: return LaunchSubsystem.FLYWHEEL_P;
                case KS: return LaunchSubsystem.FLYWHEEL_KS;
                default: return LaunchSubsystem.FLYWHEEL_KV;
            }
        }
        double[] col = column(c);
        return row >= 0 && row < col.length ? col[row] : Double.NaN;
    }

    private static void addGain(GainColumn c, int row, double delta) {
        if (!LaunchSubsystem.USE_GAIN_TABLE) {
            switch (c) {
                case P: LaunchSubsystem.FLYWHEEL_P = Math.max(0, LaunchSubsystem.FLYWHEEL_P + delta); break;
                case KS: LaunchSubsystem.FLYWHEEL_KS = Math.max(0, LaunchSubsystem.FLYWHEEL_KS + delta); break;
                default: LaunchSubsystem.FLYWHEEL_KV = Math.max(0, LaunchSubsystem.FLYWHEEL_KV + delta); break;
            }
            return;
        }
        double[] col = column(c);
        if (row >= 0 && row < col.length) col[row] = Math.max(0, col[row] + delta);
    }

    private static double[] column(GainColumn c) {
        switch (c) {
            case P: return LaunchSubsystem.GAIN_TABLE_P;
            case KS: return LaunchSubsystem.GAIN_TABLE_KS;
            default: return LaunchSubsystem.GAIN_TABLE_KV;
        }
    }

    /** Logs every table as Java initializers, ready to paste into the code. */
    private void logTables() {
        Log.i(TAG, "LaunchReadinessCommand.SHOT_DISTANCES_IN = " + javaArray(LaunchReadinessCommand.SHOT_DISTANCES_IN));
        Log.i(TAG, "LaunchReadinessCommand.SHOT_RPMS = " + javaArray(LaunchReadinessCommand.SHOT_RPMS));
        Log.i(TAG, "LaunchSubsystem.GAIN_TABLE_DISTANCES_IN = " + javaArray(LaunchSubsystem.GAIN_TABLE_DISTANCES_IN));
        Log.i(TAG, "LaunchSubsystem.GAIN_TABLE_P  = " + javaArray(LaunchSubsystem.GAIN_TABLE_P));
        Log.i(TAG, "LaunchSubsystem.GAIN_TABLE_I  = " + javaArray(LaunchSubsystem.GAIN_TABLE_I));
        Log.i(TAG, "LaunchSubsystem.GAIN_TABLE_D  = " + javaArray(LaunchSubsystem.GAIN_TABLE_D));
        Log.i(TAG, "LaunchSubsystem.GAIN_TABLE_KS = " + javaArray(LaunchSubsystem.GAIN_TABLE_KS));
        Log.i(TAG, "LaunchSubsystem.GAIN_TABLE_KV = " + javaArray(LaunchSubsystem.GAIN_TABLE_KV));
        Log.i(TAG, String.format("LaunchSubsystem.FLYWHEEL_P/I/D/KS/KV = %s / %s / %s / %s / %s",
                LaunchSubsystem.FLYWHEEL_P, LaunchSubsystem.FLYWHEEL_I, LaunchSubsystem.FLYWHEEL_D,
                LaunchSubsystem.FLYWHEEL_KS, LaunchSubsystem.FLYWHEEL_KV));
    }

    private static String javaArray(double[] values) {
        StringBuilder sb = new StringBuilder("{");
        for (int i = 0; i < values.length; i++) {
            if (i > 0) sb.append(", ");
            sb.append(values[i]);
        }
        return sb.append("};").toString();
    }
}
