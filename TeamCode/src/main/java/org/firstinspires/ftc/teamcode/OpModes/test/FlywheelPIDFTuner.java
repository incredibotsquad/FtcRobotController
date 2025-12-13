package org.firstinspires.ftc.teamcode.OpModes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * FLYWHEEL PIDF TUNER
 * 
 * Use this OpMode with FTC Dashboard to tune your flywheel PIDF values.
 * 
 * STEP 1: Run this OpMode and look at "DEFAULT PIDF VALUES" in telemetry
 * STEP 2: Set USE_CUSTOM_PIDF = true in Dashboard
 * STEP 3: Start with the default values and adjust from there
 * 
 * Controls:
 * - Right Trigger: Run flywheel at target velocity
 * - A Button: Reset timers
 * 
 * IMPORTANT: The motor uses DEFAULT SDK PIDF until you enable USE_CUSTOM_PIDF!
 */
@Config
@TeleOp(name = "Flywheel PIDF Tuner", group = "Tuning")
public class FlywheelPIDFTuner extends LinearOpMode {
    
    // Toggle this to TRUE to use custom PIDF values below
    // Keep FALSE to use SDK default values
    public static boolean USE_CUSTOM_PIDF = false;
    
    // Toggle this to control one or both motors
    // TRUE = Both motors (normal operation)
    // FALSE = Motor 2 only (for isolated testing - Motor 2 has encoder)
    public static boolean USE_BOTH_MOTORS = true;
    
    // PIDF Coefficients - Only used when USE_CUSTOM_PIDF = true
    // These should be set to values CLOSE to your defaults, then adjusted
    public static double FLYWHEEL_P = 40.0;
    public static double FLYWHEEL_I = 3.0;
    public static double FLYWHEEL_D = 2.0;
    public static double FLYWHEEL_F = 14.0;
    
    // Target velocity in ticks per second
    public static double TARGET_VELOCITY = 2000;
    
    // Tolerance for considering "at target"
    public static double VELOCITY_TOLERANCE = 30;
    
    private DcMotorEx flywheelMotor1;
    private DcMotorEx flywheelMotor2;
    
    // Store the SDK's default PIDF values
    private PIDFCoefficients defaultPIDF;
    
    private ElapsedTime spinUpTimer = new ElapsedTime();
    private ElapsedTime recoveryTimer = new ElapsedTime();
    
    private boolean wasRunning = false;
    private boolean wasAtTarget = false;
    private double lastSpinUpTime = 0;
    private double lastRecoveryTime = 0;
    private double peakVelocity = 0;
    private double minVelocity = 0;
    
    private FtcDashboard dashboard;
    
    @Override
    public void runOpMode() {
        // Initialize dashboard for graphing
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        
        // Initialize FlywheelMotor1 - runs without encoder, reversed
        flywheelMotor1 = hardwareMap.get(DcMotorEx.class, "FlywheelMotor1");
        flywheelMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Initialize FlywheelMotor2 - uses encoder for velocity control
        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "FlywheelMotor2");
        flywheelMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // IMPORTANT: Read the SDK's default PIDF values BEFORE changing anything
        defaultPIDF = flywheelMotor2.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Set Motor1 to also use encoder mode for velocity control
        flywheelMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        telemetry.addLine("╔═══════════════════════════════════════╗");
        telemetry.addLine("║     FLYWHEEL PIDF TUNER               ║");
        telemetry.addLine("╚═══════════════════════════════════════╝");
        telemetry.addLine("");
        telemetry.addLine(">>> SDK DEFAULT PIDF VALUES <<<");
        telemetry.addData("  Default P", "%.4f", defaultPIDF.p);
        telemetry.addData("  Default I", "%.4f", defaultPIDF.i);
        telemetry.addData("  Default D", "%.4f", defaultPIDF.d);
        telemetry.addData("  Default F", "%.4f", defaultPIDF.f);
        telemetry.addLine("");
        telemetry.addLine("WRITE DOWN THESE VALUES!");
        telemetry.addLine("Then set USE_CUSTOM_PIDF = true");
        telemetry.addLine("And start with these as your custom values");
        telemetry.addLine("");
        telemetry.addLine("Controls: Right Trigger = Run");
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            // Apply PIDF coefficients based on mode
            if (USE_CUSTOM_PIDF) {
                applyCustomPIDF();
            }
            // If USE_CUSTOM_PIDF is false, motor uses SDK defaults automatically
            
            // Get current velocity from motor 2 (has encoder)
            double currentVelocity = flywheelMotor2.getVelocity();
            boolean isRunning = gamepad1.right_trigger > 0.5;
            boolean atTarget = Math.abs(TARGET_VELOCITY - currentVelocity) < VELOCITY_TOLERANCE;
            
            // Track peak and min velocities
            if (isRunning) {
                peakVelocity = Math.max(peakVelocity, currentVelocity);
                if (currentVelocity > 100) {
                    if (minVelocity == 0) minVelocity = currentVelocity;
                    else minVelocity = Math.min(minVelocity, currentVelocity);
                }
            }
            
            // Spin-up timing
            if (isRunning && !wasRunning) {
                spinUpTimer.reset();
                peakVelocity = 0;
                minVelocity = 0;
            }
            
            if (isRunning && atTarget && !wasAtTarget) {
                lastSpinUpTime = spinUpTimer.milliseconds();
            }
            
            // Recovery timing
            if (isRunning && wasAtTarget && !atTarget) {
                recoveryTimer.reset();
            }
            
            if (isRunning && atTarget && !wasAtTarget && wasRunning) {
                lastRecoveryTime = recoveryTimer.milliseconds();
            }
            
            // Reset button
            if (gamepad1.a) {
                lastSpinUpTime = 0;
                lastRecoveryTime = 0;
                peakVelocity = 0;
                minVelocity = 0;
            }
            
            // Control the flywheel based on USE_BOTH_MOTORS setting
            if (isRunning) {
                if (USE_BOTH_MOTORS) {
                    flywheelMotor1.setVelocity(TARGET_VELOCITY);
                }
                flywheelMotor2.setVelocity(TARGET_VELOCITY);
            } else {
                if (USE_BOTH_MOTORS) {
                    flywheelMotor1.setVelocity(0);
                }
                flywheelMotor2.setVelocity(0);
                peakVelocity = 0;
                minVelocity = 0;
            }
            
            // Calculate error
            double error = TARGET_VELOCITY - currentVelocity;
            double errorPercent = (TARGET_VELOCITY > 0) ? (error / TARGET_VELOCITY) * 100 : 0;
            
            // Send data to Dashboard graph
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Target Velocity", TARGET_VELOCITY);
            packet.put("Current Velocity", currentVelocity);
            packet.put("Error", error);
            dashboard.sendTelemetryPacket(packet);
            
            // Get current PIDF being used
            PIDFCoefficients currentPIDF = flywheelMotor2.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            
            // Display telemetry
            telemetry.addLine("═══ SDK DEFAULT VALUES (Reference) ═══");
            telemetry.addData("Default P", "%.4f", defaultPIDF.p);
            telemetry.addData("Default I", "%.4f", defaultPIDF.i);
            telemetry.addData("Default D", "%.4f", defaultPIDF.d);
            telemetry.addData("Default F", "%.4f", defaultPIDF.f);
            
            telemetry.addLine("");
            telemetry.addLine("═══ CURRENT PIDF (Active) ═══");
            telemetry.addData("Mode", USE_CUSTOM_PIDF ? "CUSTOM" : "SDK DEFAULT");
            telemetry.addData("P", "%.4f", currentPIDF.p);
            telemetry.addData("I", "%.4f", currentPIDF.i);
            telemetry.addData("D", "%.4f", currentPIDF.d);
            telemetry.addData("F", "%.4f", currentPIDF.f);
            
            if (USE_CUSTOM_PIDF) {
                telemetry.addLine("");
                telemetry.addLine("═══ CUSTOM VALUES (Dashboard) ═══");
                telemetry.addData("Custom P", "%.4f", FLYWHEEL_P);
                telemetry.addData("Custom I", "%.4f", FLYWHEEL_I);
                telemetry.addData("Custom D", "%.4f", FLYWHEEL_D);
                telemetry.addData("Custom F", "%.4f", FLYWHEEL_F);
            }
            
            telemetry.addLine("");
            telemetry.addLine("═══ VELOCITY ═══");
            telemetry.addData("Target", "%.0f TPS", TARGET_VELOCITY);
            telemetry.addData("Current", "%.0f TPS", currentVelocity);
            telemetry.addData("Error", "%.0f TPS (%.1f%%)", error, errorPercent);
            telemetry.addData("At Target?", atTarget ? "✓ YES" : "✗ NO");
            
            telemetry.addLine("");
            telemetry.addLine("═══ TIMING ═══");
            telemetry.addData("Spin-Up Time", "%.0f ms", lastSpinUpTime);
            telemetry.addData("Recovery Time", "%.0f ms", lastRecoveryTime);
            
            telemetry.addLine("");
            telemetry.addLine("═══ STATUS ═══");
            telemetry.addData("Flywheel", isRunning ? "▶ RUNNING" : "■ STOPPED");
            telemetry.addData("Motors Active", USE_BOTH_MOTORS ? "BOTH (1 & 2)" : "MOTOR 2 ONLY");
            telemetry.addData("Motor 1 Velocity", USE_BOTH_MOTORS ? String.format("%.0f TPS", flywheelMotor1.getVelocity()) : "DISABLED");
            telemetry.addData("Motor 2 Velocity", "%.0f TPS", currentVelocity);
            
            telemetry.update();
            
            wasRunning = isRunning;
            wasAtTarget = atTarget;
        }
        
        flywheelMotor1.setVelocity(0);
        flywheelMotor2.setVelocity(0);
    }
    
    private void applyCustomPIDF() {
        PIDFCoefficients customPIDF = new PIDFCoefficients(FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D, FLYWHEEL_F);
        flywheelMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, customPIDF);
        flywheelMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, customPIDF);
    }
}