package org.firstinspires.ftc.teamcode.OpModes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "FlywheelTuner", group = "Tests")
public class FlywheelTuner extends OpMode {

    // ===== Hardware =====
    private DcMotorEx flywheelMotor1;
    private DcMotorEx flywheelMotor2;

    // ===== PIDF tuning values =====
    double P = 270.0;
    double I = 0.0;
    double D = 0.0;
    double F = 14.0; //15.1

    // ===== Velocity targets =====
    double highVelocity = 1650;   // ticks/sec (inferred)
    double lowVelocity  = 1200;   // ticks/sec (inferred)
    double curTargetVelocity = highVelocity;

    // ===== Step sizes for tuning =====
    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001};

//    double[] stepSizes = {0.0005, 0.001, 0.005, 0.01, 0.05};
    int stepIndex = 0;

    @Override
    public void init() {
        flywheelMotor1 = hardwareMap.get(DcMotorEx.class, "FlywheelMotor1");
        flywheelMotor1.setDirection(DcMotor.Direction.REVERSE);

        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "FlywheelMotor2");

        flywheelMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfCoefficients =
                new PIDFCoefficients(P, I, D, F);

        flywheelMotor1.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                pidfCoefficients
        );

        flywheelMotor2.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                pidfCoefficients
        );

        telemetry.addLine("Init complete");
        telemetry.update();
    }

    @Override
    public void loop() {

        // ===============================
        // Target velocity toggle
        // ===============================
        if (gamepad1.yWasPressed()) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = highVelocity;
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

        // ===============================
        // Apply PIDF coefficients
        // ===============================
        PIDFCoefficients pidfCoefficients =
                new PIDFCoefficients(P, I, D, F);

        flywheelMotor1.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                pidfCoefficients
        );

        flywheelMotor2.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                pidfCoefficients
        );

        // ===============================
        // Set velocity
        // ===============================
        flywheelMotor1.setVelocity(curTargetVelocity);
        flywheelMotor2.setVelocity(curTargetVelocity);

        double curVelocity = flywheelMotor2.getVelocity();
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