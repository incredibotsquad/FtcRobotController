package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@TeleOp(name = "Test: Dual Motor Intake", group = "Testing")
public class IntakeMotorTestOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the subsystem
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, null);

        telemetry.addLine("--- Dual Motor Intake Test ---");
        telemetry.addLine("Hold A: Run BOTH Motors");
        telemetry.addLine("Hold X: Run INTAKE Motor Only");
        telemetry.addLine("Hold B: Run TRANSFER Motor Only");
        telemetry.addLine("------------------------------");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            
            if (gamepad1.a) {
                // Run both at the same time
                intake.startIntake(); 
                telemetry.addData("Status", "Running BOTH");
            } 
            else if (gamepad1.x) {
                // Stop everything
                intake.stopIntake();
                telemetry.addData("Status", "IDLE");
            }

            // Also show sensor data so you can see if balls 
            // are actually reaching the sensors during the test
            telemetry.addData("Low Sensor", intake.isLowSensorBlocked());
            telemetry.addData("Mid Sensor", intake.isMidSensorBlocked());
            telemetry.addData("High Sensor", intake.isHighSensorBlocked());
            telemetry.addData("Ball Count", intake.getArtifactCount());
            
            telemetry.update();
        }
    }
}