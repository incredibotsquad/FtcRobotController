package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchGateSubsystem;

import java.util.ArrayDeque;
import java.util.Deque;

/**
 * Validates the intake beam breaks and the artifact counting logic in IntakeSubsystem.
 * Runs intake.periodic() every loop, so the count, auto-stop and status light behave
 * exactly as they will in a real OpMode.
 *
 * Controls (gamepad 1):
 *   Right trigger (hold) - intake in (refuses / auto-stops when full)
 *   Left trigger (hold)  - reverse / eject
 *   Y                    - toggle launch mode (gate open + setLaunching(true))
 *   Right bumper (hold)  - in launch mode: run transfer forward to feed past sensor 4
 *   Dpad up/down         - your EXPECTED count (what you physically loaded)
 *   Dpad left/right      - force subsystem count -1/+1 (setArtifactCount)
 *   Back                 - reset subsystem count, expected count, and stats
 *
 * Sensor states shown are the subsystem's debounced values. "Edges" counts each
 * clear->blocked transition; one artifact passing a sensor should add exactly 1.
 * More than that means flicker: raise IntakeSubsystem.DEBOUNCE_MS.
 */
@TeleOp(name = "Test: Intake Count", group = "Test")
public class IntakeCountTest extends LinearOpMode {

    private static final double TRIGGER_THRESHOLD = 0.2;
    private static final int LOG_SIZE = 8;

    private final boolean[] prevBlocked = new boolean[4];
    private final int[] blockEdges = new int[4];
    private final Deque<String> eventLog = new ArrayDeque<>();

    @Override
    public void runOpMode() {
        TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, panels);
        LaunchGateSubsystem gate = new LaunchGateSubsystem(hardwareMap, panels);
        gate.closeGate();

        ElapsedTime timer = new ElapsedTime();
        int expected = 0;
        int increments = 0;
        int decrements = 0;
        int maxSeen = 0;
        boolean launchMode = false;
        boolean prevY = false, prevUp = false, prevDown = false;
        boolean prevLeft = false, prevRight = false, prevBack = false;

        intake.setArtifactCount(0);
        panels.addLine("Intake count test ready. Empty the intake before starting.");
        panels.addLine("RT = intake, LT = eject, Y = launch mode, RB = feed (launch mode)");
        panels.update(telemetry);
        waitForStart();
        timer.reset();
        snapshotSensors(intake, prevBlocked);

        while (opModeIsActive()) {
            // ---------------------------------------------------- buttons (edge triggered)
            if (gamepad1.y && !prevY) {
                launchMode = !launchMode;
                intake.setLaunching(launchMode);
                if (launchMode) gate.openGate(); else gate.closeGate();
                log(timer, "launch mode " + (launchMode ? "ON" : "OFF"), intake);
            }
            if (gamepad1.dpad_up && !prevUp) expected = Math.min(IntakeSubsystem.MAX_ARTIFACTS, expected + 1);
            if (gamepad1.dpad_down && !prevDown) expected = Math.max(0, expected - 1);
            if (gamepad1.dpad_right && !prevRight) {
                intake.setArtifactCount(intake.getArtifactCount() + 1);
                log(timer, "manual +1", intake);
            }
            if (gamepad1.dpad_left && !prevLeft) {
                intake.setArtifactCount(intake.getArtifactCount() - 1);
                log(timer, "manual -1", intake);
            }
            if (gamepad1.back && !prevBack) {
                intake.setArtifactCount(0);
                expected = 0;
                increments = decrements = maxSeen = 0;
                java.util.Arrays.fill(blockEdges, 0);
                eventLog.clear();
                timer.reset();
            }
            prevY = gamepad1.y;
            prevUp = gamepad1.dpad_up;
            prevDown = gamepad1.dpad_down;
            prevLeft = gamepad1.dpad_left;
            prevRight = gamepad1.dpad_right;
            prevBack = gamepad1.back;

            // ---------------------------------------------------- motors
            String action;
            if (gamepad1.left_trigger > TRIGGER_THRESHOLD) {
                intake.reverseIntakeAndTransfer();
                action = "EJECTING";
            } else if (launchMode && gamepad1.right_bumper) {
                intake.stopIntakeMotorOnly();
                intake.stopSweepers();
                intake.setTransferMotorPower(1);
                action = "FEEDING (launch)";
            } else if (gamepad1.right_trigger > TRIGGER_THRESHOLD) {
                intake.startIntakeAndTransfer();
                action = intake.isFull() ? "FULL - intake refused" : "INTAKING";
            } else {
                intake.stopIntakeAndTransfer();
                action = "IDLE";
            }

            // ---------------------------------------------------- run subsystem logic
            int before = intake.getArtifactCount();
            intake.periodic();
            int after = intake.getArtifactCount();

            if (after != before) {
                if (after > before) increments += after - before; else decrements += before - after;
                maxSeen = Math.max(maxSeen, after);
                log(timer, "count " + before + " -> " + after + " (" + action + ")", intake);
            }

            boolean[] now = new boolean[4];
            snapshotSensors(intake, now);
            for (int i = 0; i < 4; i++) {
                if (now[i] && !prevBlocked[i]) blockEdges[i]++;
                prevBlocked[i] = now[i];
            }

            // ---------------------------------------------------- telemetry
            panels.addLine("====== INTAKE COUNT TEST ======");
            panels.addData("Action", action);
            panels.addData("Launch mode", launchMode ? "ON (gate open)" : "off");
            panels.addData("Counted", after);
            panels.addData("Expected (dpad up/down)", expected);
            panels.addData("Result", after == expected ? "MATCH" : "MISMATCH");
            panels.addData("Full", intake.isFull());
            panels.addLine("------ sensors (1 = entrance, 4 = top) ------");
            for (int i = 0; i < 4; i++) {
                panels.addData("Sensor " + (i + 1),
                        (now[i] ? "BLOCKED" : "clear  ") + "   edges: " + blockEdges[i]);
            }
            panels.addLine("------ stats ------");
            panels.addData("+ events / - events", increments + " / " + decrements);
            panels.addData("Max count seen", maxSeen);
            panels.addLine("------ event log (newest first) ------");
            for (String entry : eventLog) panels.addLine(entry);
            panels.update(telemetry);
        }

        intake.stopIntakeAndTransfer();
        intake.setLaunching(false);
        gate.closeGate();
    }

    private static void snapshotSensors(IntakeSubsystem intake, boolean[] out) {
        out[0] = intake.isSensor1Blocked();
        out[1] = intake.isSensor2Blocked();
        out[2] = intake.isSensor3Blocked();
        out[3] = intake.isSensor4Blocked();
    }

    private void log(ElapsedTime timer, String message, IntakeSubsystem intake) {
        String sensors = (intake.isSensor1Blocked() ? "X" : "-")
                + (intake.isSensor2Blocked() ? "X" : "-")
                + (intake.isSensor3Blocked() ? "X" : "-")
                + (intake.isSensor4Blocked() ? "X" : "-");
        eventLog.addFirst(String.format("%6.2fs [%s] %s", timer.seconds(), sensors, message));
        while (eventLog.size() > LOG_SIZE) eventLog.removeLast();
    }
}
