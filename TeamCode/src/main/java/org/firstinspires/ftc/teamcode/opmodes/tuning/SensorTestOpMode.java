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
            boolean low = intake.isLowSensorBlocked();
            boolean mid = intake.isMidSensorBlocked();
            boolean high = intake.isHighSensorBlocked();
            int count = intake.getArtifactCount();

            // Display results
            telemetry.addData("--- SENSOR STATES ---", "");
            telemetry.addData("[TOP]    High Sensor", high ? "BLOCKED (TRUE)" : "CLEAN (FALSE)");
            telemetry.addData("[MIDDLE] Mid Sensor ", mid  ? "BLOCKED (TRUE)" : "CLEAN (FALSE)");
            telemetry.addData("[BOTTOM] Low Sensor ", low  ? "BLOCKED (TRUE)" : "CLEAN (FALSE)");
            telemetry.addLine("-----------------------");
            telemetry.addData("Calculated Ball Count", count);
            
            // Helper for physical debugging
            if (high && mid && low) {
                telemetry.addLine("STATUS: FULL (3 BALLS)");
            } else if (!high && !mid && !low) {
                telemetry.addLine("STATUS: EMPTY");
            }

            telemetry.update();
        }
    }
}
