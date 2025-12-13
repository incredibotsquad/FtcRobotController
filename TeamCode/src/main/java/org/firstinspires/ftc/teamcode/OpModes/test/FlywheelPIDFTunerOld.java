package org.firstinspires.ftc.teamcode.OpModes.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Disabled
@Config  // This allows FTC Dashboard to change values live!
@TeleOp(name = "Flywheel PIDF Tuner Old", group = "Tests")
public class FlywheelPIDFTunerOld extends LinearOpMode {
    
    public static double FLYWHEEL_P = 10.0;
    public static double FLYWHEEL_I = 3.0;
    public static double FLYWHEEL_D = 0.0;
    public static double FLYWHEEL_F = 12.0;
    public static double TARGET_VELOCITY = 2000;  // TPS
    
    @Override
    public void runOpMode() {

        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "FlywheelMotor2");
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfOld = flywheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("P old", pidfOld.p);
        telemetry.addData("I old", pidfOld.i);
        telemetry.addData("D old", pidfOld.d);
        telemetry.addData("F old", pidfOld.f);
        telemetry.update();

        waitForStart();
        
        while (opModeIsActive()) {
            // Update PIDF coefficients in real-time from Dashboard
            PIDFCoefficients pidf = new PIDFCoefficients(FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D, FLYWHEEL_F);
            flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            
            // Control flywheel with triggers
            if (gamepad1.right_trigger > 0.5) {
                telemetry.addData("Setting Velocity", TARGET_VELOCITY);
                flywheel.setVelocity(TARGET_VELOCITY);
            } else {
                flywheel.setVelocity(0);
            }
            
            // Telemetry
            double currentVel = flywheel.getVelocity();
            double error = TARGET_VELOCITY - currentVel;
            
            telemetry.addData("Target", TARGET_VELOCITY);
            telemetry.addData("Current", currentVel);
            telemetry.addData("Error", error);
            telemetry.addData("P", FLYWHEEL_P);
            telemetry.addData("I", FLYWHEEL_I);
            telemetry.addData("D", FLYWHEEL_D);
            telemetry.addData("F", FLYWHEEL_F);
            telemetry.update();
        }
    }
}