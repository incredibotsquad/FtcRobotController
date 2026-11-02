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
                intake.startIntakeAndTransfer();
                telemetry.addData("Status", "Running BOTH");
            } 
            else if (gamepad1.x) {
                // Stop everything
                intake.stopIntakeAndTransfer();
                telemetry.addData("Status", "IDLE");
            }

            // Also show sensor data so you can see if balls 
            // are actually reaching the sensors during the test
            telemetry.addData("Sensor 1", intake.isSensor1Blocked());
            telemetry.addData("Sensor 2", intake.isSensor2Blocked());
            telemetry.addData("Sensor 3", intake.isSensor3Blocked());
            telemetry.addData("Sensor 4", intake.isSensor4Blocked());

            telemetry.addData("Ball Count", intake.getArtifactCount());
            
            telemetry.update();
        }
    }
}