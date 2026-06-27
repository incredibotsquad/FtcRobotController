package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.LaunchSubsystem;

@Disabled
@TeleOp(name = "FlywheelTuner", group = "Tests")
public class FlywheelTuner extends LinearOpMode {

    // ===== PIDF tuning values =====
    public static double P = 1.0;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 12.5; // Start with F = 32767 / MaxTPS

    // ===== Velocity targets =====
    public static double highVelocityRPM = 1650;   // ticks/sec (inferred) 1302
    public static double lowVelocityRPM = 1200;   // ticks/sec (inferred) 1204
    double curTargetVelocity = highVelocityRPM;

    // ===== Step sizes for tuning =====
    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001};

    //    double[] stepSizes = {0.0005, 0.001, 0.005, 0.01, 0.05};
    int stepIndex = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the subsystem
        LaunchSubsystem launcher = new LaunchSubsystem(hardwareMap, telemetry);
        waitForStart();

        while (opModeIsActive()) {

            // ===============================
            // Target velocity toggle
            // ===============================
            if (gamepad1.yWasPressed()) {
                if (curTargetVelocity == highVelocityRPM) {
                    curTargetVelocity = lowVelocityRPM;
                } else {
                    curTargetVelocity = highVelocityRPM;
                }
            }

            // ===============================
            // Step size cycling
            // ===============================
            if (gamepad1.bWasPressed()) {
                stepIndex = (stepIndex + 1) % stepSizes.length;
            }

            // ===============================
            // PIDF adjustments
            // ===============================
            if (gamepad1.dpadLeftWasPressed()) {
                F -= stepSizes[stepIndex];
            }

            if (gamepad1.dpadRightWasPressed()) {
                F += stepSizes[stepIndex];
            }

            if (gamepad1.dpadUpWasPressed()) {
                P += stepSizes[stepIndex];
            }

            if (gamepad1.dpadDownWasPressed()) {
                P -= stepSizes[stepIndex];
            }

            launcher.setFlywheelPIDF(P, I, D, F);

            // ===============================
            // Set velocity
            // ===============================
            launcher.spinUpFlywheelToRPM(curTargetVelocity);

            double curVelocity = launcher.getCurrentFlywheelRPM();
            double error = curTargetVelocity - curVelocity;

            // ===============================
            // Telemetry
            // ===============================
            telemetry.addData("Target Velocity", curTargetVelocity);
            telemetry.addData("Current Velocity", "%.2f", curVelocity);
            telemetry.addData("Error", "%.2f", error);

            telemetry.addLine("-----------------------------------------");

            telemetry.addData("Tuning P (D-Pad U/D)", "%.4f", P);
            telemetry.addData("Tuning F (D-Pad L/R)", "%.4f", F);
            telemetry.addData("Step Size (B Button)", "%.4f", stepSizes[stepIndex]);

            telemetry.update();
        }
    }
}