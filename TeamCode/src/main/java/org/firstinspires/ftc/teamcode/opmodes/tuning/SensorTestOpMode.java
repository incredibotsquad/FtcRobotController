package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import com.bylazar.telemetry.PanelsTelemetry;

@TeleOp(name = "Test: Beam Break Sensors", group = "Testing")
public class SensorTestOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the subsystem
        // We pass null for telemetry if your constructor allows, 
        // or just use the standard PanelsTelemetry instance
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, null);

        telemetry.addLine(">> Sensors Initialized");
        telemetry.addLine(">> Place a ball in front of each sensor to test");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Read raw states from the subsystem methods
            boolean one = intake.isSensor1Blocked();
            boolean two = intake.isSensor2Blocked();
            boolean three = intake.isSensor3Blocked();
            boolean four = intake.isSensor4Blocked();
            int count = intake.getArtifactCount();

            // Display results
            telemetry.addData("--- SENSOR STATES ---", "");
            telemetry.addData("Sensor 1", one ? "BLOCKED (TRUE)" : "CLEAN (FALSE)");
            telemetry.addData("Sensor 2", two  ? "BLOCKED (TRUE)" : "CLEAN (FALSE)");
            telemetry.addData("Sensor 3", three  ? "BLOCKED (TRUE)" : "CLEAN (FALSE)");
            telemetry.addData("Sensor 4", four  ? "BLOCKED (TRUE)" : "CLEAN (FALSE)");
            telemetry.addLine("-----------------------");
            telemetry.addData("Calculated Ball Count", count);
            
            // Helper for physical debugging
            if (intake.isFull()) {
                telemetry.addLine("STATUS: FULL (3 BALLS)");
            } else if (intake.getArtifactCount() == 0) {
                telemetry.addLine("STATUS: EMPTY");
            }

            telemetry.update();
        }
    }
}