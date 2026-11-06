package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchGateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchSubsystem;

/**
 * Bench tuner: fixed target RPMs, manual gate/transfer/intake. Use it for the first feedforward
 * (kS/kV) tune and sanity checks; use FlywheelTuner to tune per position with the real TeleOp commands.
 * Tunes the flywheel using real shots. The gains being tuned are LaunchSubsystem's own
 * FLYWHEEL_* fields, so match OpModes use whatever you tune here (until the app restarts).
 * When happy, copy the values shown into the defaults in LaunchSubsystem.
 *
 * Controls (gamepad 1):
 *   Y            - toggle between high and low target RPM
 *   A            - toggle gate (open = shoot)
 *   X            - toggle transfer
 *   Start        - toggle intake
 *   B            - cycle step size
 *   Dpad up/down - P +/- step
 *   Dpad L/R     - kS -/+ step
 *   LB / RB      - kV -/+ step
 *   Back         - auto-adjust kV from the current speed (use with P = 0, bang-bang off,
 *                  flywheel settled; repeat until actual RPM matches target)
 *   Left stick   - (press) reset shot statistics
 *
 * Suggested order:
 *   1. Bang-bang OFF (Panels: LaunchSubsystem.FLYWHEEL_BANG_BANG_ENABLED), P = 0.
 *      Tune kS, then kV (or Back) until actual RPM settles on target at both targets.
 *   2. Feed and shoot. Raise P until "recovery ms" stops improving or RPM oscillates, then back off.
 *   3. Bang-bang ON. Adjust FLYWHEEL_BANG_BANG_THRESHOLD_RPM for the lowest spin-up and
 *      recovery times without overshoot.
 *   Tune with a charged battery at about NOMINAL_VOLTAGE.
 */
@Configurable
@TeleOp(name = "FlywheelBenchTuner", group = "Tests")
public class FlywheelBenchTuner extends LinearOpMode {

    public static double highVelocityRPM = 1600;
    public static double lowVelocityRPM = 900;

    public static double transferMotorPower = 1;
    public static double intakeMotorPower = 1;

    /** A drop of more than this below target, while at speed, counts as a shot. */
    public static double SHOT_DETECT_DROP_RPM = 75;
    /** Telemetry rate. Control runs every loop; telemetry is the slow part. */
    public static long TELEMETRY_INTERVAL_MS = 50;

    private final double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001, 0.00001};
    private int stepIndex = 3;

    // shot / spin-up measurement
    private enum ShotState { SPINNING_UP, AT_SPEED, RECOVERING }
    private ShotState shotState = ShotState.SPINNING_UP;
    private long phaseStartMs;
    private double minRpmThisShot;
    private int shotCount = 0;
    private double lastDipRpm = 0, worstDipRpm = 0;
    private long lastRecoveryMs = 0, totalRecoveryMs = 0;
    private long lastSpinUpMs = 0;

    @Override
    public void runOpMode() {
        // Fast loops matter for recovery: read all hub data in one bulk call per loop.
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
        LaunchSubsystem launcher = new LaunchSubsystem(hardwareMap, panels);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, panels);
        LaunchGateSubsystem gate = new LaunchGateSubsystem(hardwareMap, panels);
        GamepadEx gp1 = new GamepadEx(gamepad1);

        boolean useHigh = true;
        boolean transferOn = false;
        boolean intakeOn = false;
        long lastTelemetryMs = 0;
        long lastLoopNs = System.nanoTime();
        double loopMs = 0;

        panels.addLine("Flywheel tuner ready. Flywheel spins up on START.");
        panels.update(telemetry);
        waitForStart();

        gate.closeGate();
        startSpinUp();

        while (opModeIsActive()) {
            gp1.readButtons();
            double step = stepSizes[stepIndex];

            // ---------------------------------------------------- buttons
            if (gp1.wasJustPressed(GamepadKeys.Button.Y)) {
                useHigh = !useHigh;
                startSpinUp();
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.B)) stepIndex = (stepIndex + 1) % stepSizes.length;
            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                if (gate.isGateOpen()) gate.closeGate(); else gate.openGate();
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.X)) transferOn = !transferOn;
            if (gp1.wasJustPressed(GamepadKeys.Button.START)) intakeOn = !intakeOn;
            if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) resetStats();

            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) LaunchSubsystem.FLYWHEEL_P += step;
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) LaunchSubsystem.FLYWHEEL_P = Math.max(0, LaunchSubsystem.FLYWHEEL_P - step);
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) LaunchSubsystem.FLYWHEEL_KS += step;
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) LaunchSubsystem.FLYWHEEL_KS = Math.max(0, LaunchSubsystem.FLYWHEEL_KS - step);
            if (gp1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) LaunchSubsystem.FLYWHEEL_KV += step;
            if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) LaunchSubsystem.FLYWHEEL_KV = Math.max(0, LaunchSubsystem.FLYWHEEL_KV - step);

            double targetRPM = useHigh ? highVelocityRPM : lowVelocityRPM;

            // Back: rescale kV so feedforward alone would produce the current speed at the target.
            // Steady state with P = 0: power = kS + kV_true * actualTPS, and power = kS + kV * targetTPS.
            if (gp1.wasJustPressed(GamepadKeys.Button.BACK)) {
                double actualTPS = launcher.getFlywheelVelocityTPS();
                double targetTPS = LaunchSubsystem.rpmToTps(targetRPM);
                if (actualTPS > 100 && targetTPS > 0) {
                    LaunchSubsystem.FLYWHEEL_KV = LaunchSubsystem.FLYWHEEL_KV * targetTPS / actualTPS;
                }
            }

            // ---------------------------------------------------- outputs
            launcher.updateFlywheel(targetRPM);
            intake.setTransferMotorPower(transferOn ? transferMotorPower : 0);
            intake.setIntakeMotorPower(intakeOn ? intakeMotorPower : 0);

            // ---------------------------------------------------- measurements
            double rpm = launcher.getCurrentFlywheelRPM();
            updateShotStats(rpm, targetRPM);

            long nowNs = System.nanoTime();
            loopMs = 0.9 * loopMs + 0.1 * ((nowNs - lastLoopNs) / 1e6);
            lastLoopNs = nowNs;

            // ---------------------------------------------------- telemetry (rate limited)
            long nowMs = nowMs();
            if (nowMs - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
                lastTelemetryMs = nowMs;

                // Graph keys (Panels "Graphs" tab)
                panels.addData("targetRPM", targetRPM);
                panels.addData("actualRPM", rpm);
                panels.addData("leftRPM", launcher.getLeftFlywheelRPM());
                panels.addData("rightRPM", launcher.getRightFlywheelRPM());
                panels.addData("error", targetRPM - rpm);
                panels.addData("power", launcher.getLastFlywheelPower());

                panels.addLine("--- Status ---");
                panels.addData("Mode", launcher.isFlywheelInBangBang() ? "BANG-BANG (full power)" : "PIDF");
                panels.addData("At speed", launcher.isFlywheelAtSpeed());
                panels.addData("Battery V", String.format("%.2f", launcher.getBatteryVoltage()));
                panels.addData("Loop ms", String.format("%.1f", loopMs));
                panels.addData("Gate (A) / Transfer (X) / Intake (Start)", (gate.isGateOpen() ? "OPEN" : "closed")
                        + " / " + (transferOn ? "ON" : "off") + " / " + (intakeOn ? "ON" : "off"));

                panels.addLine("--- Shots ---");
                panels.addData("Spin-up ms (last)", lastSpinUpMs);
                panels.addData("Shots", shotCount);
                panels.addData("Dip RPM last / worst", Math.round(lastDipRpm) + " / " + Math.round(worstDipRpm));
                panels.addData("Recovery ms last / avg", lastRecoveryMs + " / "
                        + (shotCount > 0 ? totalRecoveryMs / shotCount : 0));

                panels.addLine("--- Gains (LaunchSubsystem.FLYWHEEL_*) ---");
                panels.addData("Step (B)", stepSizes[stepIndex]);
                panels.addData("P (dpad U/D)", LaunchSubsystem.FLYWHEEL_P);
                panels.addData("kS (dpad L/R)", LaunchSubsystem.FLYWHEEL_KS);
                panels.addData("kV (LB/RB, Back=auto)", LaunchSubsystem.FLYWHEEL_KV);
                panels.addData("I / D (Panels)", LaunchSubsystem.FLYWHEEL_I + " / " + LaunchSubsystem.FLYWHEEL_D);
                panels.addData("Bang-bang", LaunchSubsystem.FLYWHEEL_BANG_BANG_ENABLED
                        ? "on, " + LaunchSubsystem.FLYWHEEL_BANG_BANG_THRESHOLD_RPM + " rpm" : "off");

                panels.update(telemetry);
            }
        }

        launcher.stop();
        intake.stopIntakeAndTransfer();
        gate.closeGate();
    }

    /**
     * SPINNING_UP: measures time from a target change until first at speed.
     * AT_SPEED:    a drop of SHOT_DETECT_DROP_RPM below target starts a shot.
     * RECOVERING:  tracks the lowest RPM (dip) and time until back within tolerance.
     */
    private void updateShotStats(double rpm, double targetRPM) {
        long now = nowMs();
        boolean atSpeed = Math.abs(targetRPM - rpm) < LaunchSubsystem.TARGET_RPM_TOLERANCE;

        switch (shotState) {
            case SPINNING_UP:
                if (atSpeed) {
                    lastSpinUpMs = now - phaseStartMs;
                    shotState = ShotState.AT_SPEED;
                }
                break;
            case AT_SPEED:
                if (targetRPM - rpm > SHOT_DETECT_DROP_RPM) {
                    shotState = ShotState.RECOVERING;
                    phaseStartMs = now;
                    minRpmThisShot = rpm;
                }
                break;
            case RECOVERING:
                minRpmThisShot = Math.min(minRpmThisShot, rpm);
                if (atSpeed) {
                    lastRecoveryMs = now - phaseStartMs;
                    lastDipRpm = targetRPM - minRpmThisShot;
                    worstDipRpm = Math.max(worstDipRpm, lastDipRpm);
                    totalRecoveryMs += lastRecoveryMs;
                    shotCount++;
                    shotState = ShotState.AT_SPEED;
                }
                break;
        }
    }

    private void startSpinUp() {
        shotState = ShotState.SPINNING_UP;
        phaseStartMs = nowMs();
    }

    private void resetStats() {
        shotCount = 0;
        lastDipRpm = worstDipRpm = 0;
        lastRecoveryMs = totalRecoveryMs = 0;
    }

    private static long nowMs() {
        return System.nanoTime() / 1_000_000L;
    }
}
