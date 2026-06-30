import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.LaunchSubsystem;

@TeleOp(name = "FlywheelTuner", group = "Tests")
public class FlywheelTuner extends LinearOpMode {

    // ===== PIDF tuning values =====
    public static double P = 0; //35
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 0; // Start with F = 32767 / MaxTPS 12.5

    public static double kS = 0; //
    public static double kV = 0; //


    // ===== Velocity targets =====
    public static double highVelocityRPM = 4500;   // ticks/sec (inferred) 1302
    public static double lowVelocityRPM = 3100;   // ticks/sec (inferred) 1204
    double curTargetVelocity = highVelocityRPM;

    // ===== Step sizes for tuning =====
    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001};

    //    double[] stepSizes = {0.0005, 0.001, 0.005, 0.01, 0.05};
    int stepIndex = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the subsystem
        LaunchSubsystem launcher = new LaunchSubsystem(hardwareMap, telemetry);
        GamepadEx gp1 = new GamepadEx(gamepad1);
        waitForStart();

        while (opModeIsActive()) {
            gp1.readButtons();

            // ===============================
            // Target velocity toggle
            // ===============================
            if (gp1.wasJustPressed(GamepadKeys.Button.Y)) {
                if (curTargetVelocity == highVelocityRPM) {
                    curTargetVelocity = lowVelocityRPM;
                } else {
                    curTargetVelocity = highVelocityRPM;
                }
            }

            // ===============================
            // Step size cycling
            // ===============================
            if (gp1.wasJustPressed(GamepadKeys.Button.B)) {
                stepIndex = (stepIndex + 1) % stepSizes.length;
            }

            // ===============================
            // PIDF adjustments
            // ===============================
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
//                F -= stepSizes[stepIndex];

                kS -= stepSizes[stepIndex];
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
//                F += stepSizes[stepIndex];
                kS += stepSizes[stepIndex];
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                P += stepSizes[stepIndex];
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                P -= stepSizes[stepIndex];
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER))
            {
                kV -= stepSizes[stepIndex];

            }

            if (gp1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER))
            {
                kV += stepSizes[stepIndex];

            }

            // ===============================
            // AUTO-OPTIMIZE kV (RB + X)
            // ===============================
            if (gp1.isDown(GamepadKeys.Button.RIGHT_BUMPER) && gp1.wasJustPressed(GamepadKeys.Button.X)) {
                double targetTPS = (curTargetVelocity * Motor.GoBILDA.BARE.getCPR()) / 60.0;
                double currentTPS = launcher.getFlywheelVelocityTPS();
                
                if (targetTPS > 0 && currentTPS > 100) {
                    double currentError = targetTPS - currentTPS;
                    double currentPower = (P * currentError) + (kS + (kV * targetTPS));
                    kV = Math.max(0, (currentPower - kS) / currentTPS);
                }
            }


            //launcher.setFlywheelPIDF(P, I, D, F);

            launcher.setFlywheelPID(P, I, D);
            launcher.updateFeedforward(kS, kV);

            // ===============================
            // Set velocity
            // ===============================
//            launcher.spinUpFlywheelToRPM(curTargetVelocity);

            launcher.updateFlywheel(curTargetVelocity);

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
            telemetry.addData("Tuning kS (D-Pad L/R)", "%.4f", kS);
            telemetry.addData("Tuning kV (Trigger L/R)", "%.4f", kV);
//            telemetry.addData("Tuning F (D-Pad L/R)", "%.4f", F);
            telemetry.addData("Step Size (B Button)", "%.4f", stepSizes[stepIndex]);

            telemetry.update();
        }
    }
}