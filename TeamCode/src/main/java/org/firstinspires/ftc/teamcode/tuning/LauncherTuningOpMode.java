package org.firstinspires.ftc.teamcode.tuning;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Actions.LaunchKickAction;
import org.firstinspires.ftc.teamcode.common.RobotHardware;

import java.util.ArrayList;
import java.util.List;

/**
 * LAUNCHER TUNING OPMODE
 * =======================
 * 
 * This OpMode helps you systematically tune the launcher for 3-ball shots from any distance.
 * 
 * TUNING WORKFLOW:
 * ----------------
 * 1. Position the robot at a known distance from the goal
 * 2. Set the distance category (CLOSE/MID/FAR) using gamepad
 * 3. Adjust initial flywheel velocity until Ball 1 hits consistently
 * 4. Press A to shoot Ball 1 and observe RPM drop
 * 5. Adjust visor position for Ball 2 to compensate for RPM drop
 * 6. Press A to shoot Ball 2 and observe RPM drop
 * 7. Adjust visor position for Ball 3 to compensate for further RPM drop
 * 8. Press A to shoot Ball 3
 * 9. Record all values shown in telemetry
 * 
 * CONTROLS:
 * ---------
 * Left Stick Y:     Adjust flywheel velocity (hold for continuous)
 * Right Stick Y:    Adjust visor/hood position (hold for continuous)
 * D-Pad Up/Down:    Change adjustment step size
 * D-Pad Left/Right: Change distance category (CLOSE/MID/FAR)
 * A:                Kick ball (shoots current ball)
 * B:                Reset to Ball 1 (start new sequence)
 * X:                Spin up flywheel to target velocity
 * Y:                Stop flywheel
 * Left Bumper:      Save current values to log
 * Right Bumper:     Toggle between coarse/fine adjustment
 * 
 * TELEMETRY OUTPUT:
 * -----------------
 * All values are displayed on both Driver Station and FTC Dashboard.
 * Use Dashboard graphs to visualize RPM drop over time.
 */
@Config
@TeleOp(name = "Launcher Tuning", group = "Tuning")
public class LauncherTuningOpMode extends LinearOpMode {

    // ===== TUNABLE PARAMETERS (adjustable via FTC Dashboard) =====
    
    // Distance categories in inches
    public static double DISTANCE_CLOSE = 36.0;
    public static double DISTANCE_MID = 72.0;
    public static double DISTANCE_FAR = 108.0;
    
    // Initial flywheel velocities (ticks per second)
    public static double VELOCITY_CLOSE = 1260;  // ~45% of max
    public static double VELOCITY_MID = 1400;    // ~50% of max
    public static double VELOCITY_FAR = 1624;    // ~58% of max
    
    // Visor positions for CLOSE distance
    public static double VISOR_CLOSE_BALL1 = 0.15;
    public static double VISOR_CLOSE_BALL2 = 0.15;
    public static double VISOR_CLOSE_BALL3 = 0.15;
    
    // Visor positions for MID distance
    public static double VISOR_MID_BALL1 = 0.24;
    public static double VISOR_MID_BALL2 = 0.24;
    public static double VISOR_MID_BALL3 = 0.24;
    
    // Visor positions for FAR distance
    public static double VISOR_FAR_BALL1 = 0.69;
    public static double VISOR_FAR_BALL2 = 0.65;
    public static double VISOR_FAR_BALL3 = 0.61;
    
    // Adjustment step sizes
    public static double VELOCITY_STEP_COARSE = 50.0;
    public static double VELOCITY_STEP_FINE = 10.0;
    public static double VISOR_STEP_COARSE = 0.02;
    public static double VISOR_STEP_FINE = 0.005;
    
    // ===== INTERNAL STATE =====
    private RobotHardware robotHardware;
    private FtcDashboard dashboard;
    
    private enum DistanceCategory { CLOSE, MID, FAR }
    private DistanceCategory currentDistance = DistanceCategory.MID;
    
    private int currentBallNumber = 1;  // 1, 2, or 3
    private boolean fineAdjustment = false;
    
    private double currentVelocity;
    private double currentVisorPosition;
    
    // RPM tracking
    private double rpmBeforeShot = 0;
    private double rpmAfterShot = 0;
    private double rpmDrop = 0;
    
    // Shot history for current sequence
    private List<ShotData> shotHistory = new ArrayList<>();
    
    private ElapsedTime rpmSampleTimer = new ElapsedTime();
    private List<Action> runningActions = new ArrayList<>();
    
    // Data class for shot recording
    private static class ShotData {
        int ballNumber;
        double distance;
        double targetVelocity;
        double actualRpmBefore;
        double actualRpmAfter;
        double rpmDrop;
        double visorPosition;
        String result;  // "HIT", "SHORT", "LONG", "LEFT", "RIGHT"
        
        @Override
        public String toString() {
            return String.format("Ball%d: Vel=%.0f, RPM_Before=%.0f, RPM_After=%.0f, Drop=%.0f, Visor=%.3f, Result=%s",
                    ballNumber, targetVelocity, actualRpmBefore, actualRpmAfter, rpmDrop, visorPosition, result);
        }
    }
    
    @Override
    public void runOpMode() {
        // Initialize hardware and dashboard
        robotHardware = new RobotHardware(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        
        // Set initial values based on MID distance
        loadValuesForDistance(currentDistance);
        
        telemetry.addLine("=== LAUNCHER TUNING MODE ===");
        telemetry.addLine("Press START when ready");
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            // Process gamepad inputs
            handleGamepadInputs();
            
            // Update telemetry
            updateTelemetry();
            
            // Process any running actions (kick actions)
            processActions();
            
            // Small delay to prevent CPU overload
            sleep(20);
        }
        
        // Stop flywheel on exit
        robotHardware.setFlywheelVelocityInTPS(0);
    }
    
    private void handleGamepadInputs() {
        // ===== Distance Category Selection (D-Pad Left/Right) =====
        if (gamepad1.dpad_left) {
            if (currentDistance == DistanceCategory.FAR) {
                currentDistance = DistanceCategory.MID;
            } else if (currentDistance == DistanceCategory.MID) {
                currentDistance = DistanceCategory.CLOSE;
            }
            loadValuesForDistance(currentDistance);
            sleep(200);  // Debounce
        }
        if (gamepad1.dpad_right) {
            if (currentDistance == DistanceCategory.CLOSE) {
                currentDistance = DistanceCategory.MID;
            } else if (currentDistance == DistanceCategory.MID) {
                currentDistance = DistanceCategory.FAR;
            }
            loadValuesForDistance(currentDistance);
            sleep(200);  // Debounce
        }
        
        // ===== Adjustment Mode Toggle (Right Bumper) =====
        if (gamepad1.right_bumper) {
            fineAdjustment = !fineAdjustment;
            sleep(200);  // Debounce
        }
        
        // ===== Flywheel Velocity Adjustment (Left Stick Y) =====
        double velocityStep = fineAdjustment ? VELOCITY_STEP_FINE : VELOCITY_STEP_COARSE;
        if (Math.abs(gamepad1.left_stick_y) > 0.3) {
            currentVelocity -= gamepad1.left_stick_y * velocityStep * 0.1;
            currentVelocity = Math.max(0, Math.min(2800, currentVelocity));
            updateDashboardValue();
        }
        
        // ===== Visor Position Adjustment (Right Stick Y) =====
        double visorStep = fineAdjustment ? VISOR_STEP_FINE : VISOR_STEP_COARSE;
        if (Math.abs(gamepad1.right_stick_y) > 0.3) {
            currentVisorPosition -= gamepad1.right_stick_y * visorStep * 0.1;
            currentVisorPosition = Math.max(0.0, Math.min(1.0, currentVisorPosition));
            robotHardware.setLaunchVisorPosition(currentVisorPosition);
            updateDashboardValue();
        }
        
        // ===== Spin Up Flywheel (X) =====
        if (gamepad1.x) {
            robotHardware.setFlywheelVelocityInTPS(currentVelocity);
            sleep(200);  // Debounce
        }
        
        // ===== Stop Flywheel (Y) =====
        if (gamepad1.y) {
            robotHardware.setFlywheelVelocityInTPS(0);
            sleep(200);  // Debounce
        }
        
        // ===== Kick Ball / Shoot (A) =====
        if (gamepad1.a) {
            shootBall();
            sleep(300);  // Debounce
        }
        
        // ===== Reset to Ball 1 (B) =====
        if (gamepad1.b) {
            resetSequence();
            sleep(200);  // Debounce
        }
        
        // ===== Save Current Values (Left Bumper) =====
        if (gamepad1.left_bumper) {
            logCurrentValues();
            sleep(200);  // Debounce
        }
    }
    
    private void loadValuesForDistance(DistanceCategory distance) {
        switch (distance) {
            case CLOSE:
                currentVelocity = VELOCITY_CLOSE;
                switch (currentBallNumber) {
                    case 1: currentVisorPosition = VISOR_CLOSE_BALL1; break;
                    case 2: currentVisorPosition = VISOR_CLOSE_BALL2; break;
                    case 3: currentVisorPosition = VISOR_CLOSE_BALL3; break;
                }
                break;
            case MID:
                currentVelocity = VELOCITY_MID;
                switch (currentBallNumber) {
                    case 1: currentVisorPosition = VISOR_MID_BALL1; break;
                    case 2: currentVisorPosition = VISOR_MID_BALL2; break;
                    case 3: currentVisorPosition = VISOR_MID_BALL3; break;
                }
                break;
            case FAR:
                currentVelocity = VELOCITY_FAR;
                switch (currentBallNumber) {
                    case 1: currentVisorPosition = VISOR_FAR_BALL1; break;
                    case 2: currentVisorPosition = VISOR_FAR_BALL2; break;
                    case 3: currentVisorPosition = VISOR_FAR_BALL3; break;
                }
                break;
        }
        robotHardware.setLaunchVisorPosition(currentVisorPosition);
    }
    
    private void updateDashboardValue() {
        // Update the static values that can be seen in FTC Dashboard
        switch (currentDistance) {
            case CLOSE:
                VELOCITY_CLOSE = currentVelocity;
                switch (currentBallNumber) {
                    case 1: VISOR_CLOSE_BALL1 = currentVisorPosition; break;
                    case 2: VISOR_CLOSE_BALL2 = currentVisorPosition; break;
                    case 3: VISOR_CLOSE_BALL3 = currentVisorPosition; break;
                }
                break;
            case MID:
                VELOCITY_MID = currentVelocity;
                switch (currentBallNumber) {
                    case 1: VISOR_MID_BALL1 = currentVisorPosition; break;
                    case 2: VISOR_MID_BALL2 = currentVisorPosition; break;
                    case 3: VISOR_MID_BALL3 = currentVisorPosition; break;
                }
                break;
            case FAR:
                VELOCITY_FAR = currentVelocity;
                switch (currentBallNumber) {
                    case 1: VISOR_FAR_BALL1 = currentVisorPosition; break;
                    case 2: VISOR_FAR_BALL2 = currentVisorPosition; break;
                    case 3: VISOR_FAR_BALL3 = currentVisorPosition; break;
                }
                break;
        }
    }
    
    private void shootBall() {
        // Record RPM before shot
        rpmBeforeShot = robotHardware.getFlywheelVelocityInTPS();
        
        // Set visor position
        robotHardware.setLaunchVisorPosition(currentVisorPosition);
        
        // Execute kick action
        runningActions.add(new SequentialAction(
                new LaunchKickAction(robotHardware)
        ));
        
        // Wait a moment for kick to complete and measure RPM drop
        sleep(150);
        
        // Record RPM after shot
        rpmAfterShot = robotHardware.getFlywheelVelocityInTPS();
        rpmDrop = rpmBeforeShot - rpmAfterShot;
        
        // Create shot record
        ShotData shot = new ShotData();
        shot.ballNumber = currentBallNumber;
        shot.distance = getDistanceValue();
        shot.targetVelocity = currentVelocity;
        shot.actualRpmBefore = rpmBeforeShot;
        shot.actualRpmAfter = rpmAfterShot;
        shot.rpmDrop = rpmDrop;
        shot.visorPosition = currentVisorPosition;
        shot.result = "PENDING";  // User can update this
        shotHistory.add(shot);
        
        // Log the shot
        Log.i("LAUNCHER_TUNING", shot.toString());
        
        // Advance to next ball
        if (currentBallNumber < 3) {
            currentBallNumber++;
            // Load the next ball's visor position
            loadVisorForCurrentBall();
        } else {
            // Sequence complete, log summary
            logSequenceSummary();
        }
    }
    
    private void loadVisorForCurrentBall() {
        switch (currentDistance) {
            case CLOSE:
                switch (currentBallNumber) {
                    case 1: currentVisorPosition = VISOR_CLOSE_BALL1; break;
                    case 2: currentVisorPosition = VISOR_CLOSE_BALL2; break;
                    case 3: currentVisorPosition = VISOR_CLOSE_BALL3; break;
                }
                break;
            case MID:
                switch (currentBallNumber) {
                    case 1: currentVisorPosition = VISOR_MID_BALL1; break;
                    case 2: currentVisorPosition = VISOR_MID_BALL2; break;
                    case 3: currentVisorPosition = VISOR_MID_BALL3; break;
                }
                break;
            case FAR:
                switch (currentBallNumber) {
                    case 1: currentVisorPosition = VISOR_FAR_BALL1; break;
                    case 2: currentVisorPosition = VISOR_FAR_BALL2; break;
                    case 3: currentVisorPosition = VISOR_FAR_BALL3; break;
                }
                break;
        }
        robotHardware.setLaunchVisorPosition(currentVisorPosition);
    }
    
    private double getDistanceValue() {
        switch (currentDistance) {
            case CLOSE: return DISTANCE_CLOSE;
            case MID: return DISTANCE_MID;
            case FAR: return DISTANCE_FAR;
            default: return 0;
        }
    }
    
    private void resetSequence() {
        currentBallNumber = 1;
        shotHistory.clear();
        rpmBeforeShot = 0;
        rpmAfterShot = 0;
        rpmDrop = 0;
        loadValuesForDistance(currentDistance);
        Log.i("LAUNCHER_TUNING", "=== SEQUENCE RESET ===");
    }
    
    private void logCurrentValues() {
        Log.i("LAUNCHER_TUNING", "=== SAVED VALUES ===");
        Log.i("LAUNCHER_TUNING", String.format("Distance: %s (%.0f inches)", currentDistance, getDistanceValue()));
        Log.i("LAUNCHER_TUNING", String.format("Velocity: %.0f TPS", currentVelocity));
        Log.i("LAUNCHER_TUNING", String.format("Ball %d Visor: %.3f", currentBallNumber, currentVisorPosition));
        Log.i("LAUNCHER_TUNING", String.format("Last RPM Drop: %.0f", rpmDrop));
        
        // Also output copy-paste friendly format
        Log.i("LAUNCHER_TUNING", "--- COPY TO LaunchSystem.java ---");
        switch (currentDistance) {
            case CLOSE:
                Log.i("LAUNCHER_TUNING", String.format("FLYWHEEL_POWER_COEFFICIENT_CLOSE = %.4f;", currentVelocity / 2800.0));
                Log.i("LAUNCHER_TUNING", String.format("VISOR_POSITION_CLOSE_1 = %.3f;", VISOR_CLOSE_BALL1));
                Log.i("LAUNCHER_TUNING", String.format("VISOR_POSITION_CLOSE_2 = %.3f;", VISOR_CLOSE_BALL2));
                Log.i("LAUNCHER_TUNING", String.format("VISOR_POSITION_CLOSE_3 = %.3f;", VISOR_CLOSE_BALL3));
                break;
            case MID:
                Log.i("LAUNCHER_TUNING", String.format("FLYWHEEL_POWER_COEFFICIENT_MID = %.4f;", currentVelocity / 2800.0));
                Log.i("LAUNCHER_TUNING", String.format("VISOR_POSITION_MID_1 = %.3f;", VISOR_MID_BALL1));
                Log.i("LAUNCHER_TUNING", String.format("VISOR_POSITION_MID_2 = %.3f;", VISOR_MID_BALL2));
                Log.i("LAUNCHER_TUNING", String.format("VISOR_POSITION_MID_3 = %.3f;", VISOR_MID_BALL3));
                break;
            case FAR:
                Log.i("LAUNCHER_TUNING", String.format("FLYWHEEL_POWER_COEFFICIENT_FAR = %.4f;", currentVelocity / 2800.0));
                Log.i("LAUNCHER_TUNING", String.format("VISOR_POSITION_FAR_1 = %.3f;", VISOR_FAR_BALL1));
                Log.i("LAUNCHER_TUNING", String.format("VISOR_POSITION_FAR_2 = %.3f;", VISOR_FAR_BALL2));
                Log.i("LAUNCHER_TUNING", String.format("VISOR_POSITION_FAR_3 = %.3f;", VISOR_FAR_BALL3));
                break;
        }
    }
    
    private void logSequenceSummary() {
        Log.i("LAUNCHER_TUNING", "=== 3-BALL SEQUENCE COMPLETE ===");
        for (ShotData shot : shotHistory) {
            Log.i("LAUNCHER_TUNING", shot.toString());
        }
        
        // Calculate average RPM drop
        double totalDrop = 0;
        for (ShotData shot : shotHistory) {
            totalDrop += shot.rpmDrop;
        }
        double avgDrop = totalDrop / shotHistory.size();
        Log.i("LAUNCHER_TUNING", String.format("Average RPM Drop: %.0f", avgDrop));
    }
    
    private void updateTelemetry() {
        double currentRpm = robotHardware.getFlywheelVelocityInTPS();
        double visorFromEncoder = robotHardware.getLaunchVisorPositionFromEncoder();
        
        telemetry.addLine("=== LAUNCHER TUNING ===");
        telemetry.addLine("");
        
        // Distance and ball info
        telemetry.addData("Distance", "%s (%.0f in)", currentDistance, getDistanceValue());
        telemetry.addData("Current Ball", "%d / 3", currentBallNumber);
        telemetry.addLine("");
        
        // Flywheel info
        telemetry.addLine("--- FLYWHEEL ---");
        telemetry.addData("Target Velocity", "%.0f TPS", currentVelocity);
        telemetry.addData("Actual Velocity", "%.0f TPS", currentRpm);
        telemetry.addData("Velocity Error", "%.0f TPS", currentVelocity - currentRpm);
        telemetry.addLine("");
        
        // Visor info
        telemetry.addLine("--- VISOR/HOOD ---");
        telemetry.addData("Target Position", "%.3f", currentVisorPosition);
        telemetry.addData("Encoder Position", "%.3f", visorFromEncoder);
        telemetry.addLine("");
        
        // Last shot info
        telemetry.addLine("--- LAST SHOT ---");
        telemetry.addData("RPM Before", "%.0f", rpmBeforeShot);
        telemetry.addData("RPM After", "%.0f", rpmAfterShot);
        telemetry.addData("RPM Drop", "%.0f (%.1f%%)", rpmDrop, (rpmDrop / Math.max(1, rpmBeforeShot)) * 100);
        telemetry.addLine("");
        
        // Adjustment mode
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addData("Adjustment Mode", fineAdjustment ? "FINE" : "COARSE");
        telemetry.addData("Velocity Step", "%.1f TPS", fineAdjustment ? VELOCITY_STEP_FINE : VELOCITY_STEP_COARSE);
        telemetry.addData("Visor Step", "%.4f", fineAdjustment ? VISOR_STEP_FINE : VISOR_STEP_COARSE);
        telemetry.addLine("");
        
        // Control hints
        telemetry.addLine("L-Stick: Velocity | R-Stick: Visor");
        telemetry.addLine("A: Shoot | B: Reset | X: Spin Up | Y: Stop");
        telemetry.addLine("D-Pad L/R: Distance | RB: Fine/Coarse");
        telemetry.addLine("LB: Save Values to Log");
        
        telemetry.update();
        
        // Send to FTC Dashboard for graphing
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Target Velocity", currentVelocity);
        packet.put("Actual Velocity", currentRpm);
        packet.put("Visor Position", currentVisorPosition);
        packet.put("RPM Drop", rpmDrop);
        packet.put("Ball Number", currentBallNumber);
        dashboard.sendTelemetryPacket(packet);
    }
    
    private void processActions() {
        if (!runningActions.isEmpty()) {
            TelemetryPacket packet = new TelemetryPacket();
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;
        }
    }
}
