package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * Shows the intake sensors as IntakeSubsystem sees them (1 and 4: laser distance and baseline; 2 and 3:
 * debounced beam breaks, currently ignored for counting) and the artifact count. Use it to measure
 * IntakeSubsystem.SENSOR1_BASELINE_MM / SENSOR4_BASELINE_MM with the sensors clear.
 * Motors are never commanded. intake.periodic() runs every loop, since that is what reads the sensors.
 * For testing the counting logic with the intake running, use IntakeCountTest.
 */
@Disabled
@TeleOp(name = "Test: Beam Break Sensors", group = "Test")
public class SensorTestOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, panels);

        panels.addLine(">> Sensors initialized");
        panels.addLine(">> Place a ball in front of each sensor to test");
        panels.update(telemetry);

        waitForStart();

        while (opModeIsActive()) {
            // Reads and debounces the sensors, updates the count and status light.
            intake.periodic();

            panels.addLine("--- SENSOR STATES ---");
            panels.addData("Sensor 1 (laser)", String.format("%s  %.1f mm (baseline %.1f)",
                    intake.isSensor1Blocked() ? "BLOCKED" : "CLEAR",
                    intake.getSensor1DistanceMm(), intake.getSensor1BaselineMm()));
            panels.addData("Sensor 2 (ignored)", intake.isSensor2Blocked() ? "BLOCKED" : "CLEAR");
            panels.addData("Sensor 3 (ignored)", intake.isSensor3Blocked() ? "BLOCKED" : "CLEAR");
            panels.addData("Sensor 4 (laser)", String.format("%s  %.1f mm (baseline %.1f)",
                    intake.isSensor4Blocked() ? "BLOCKED" : "CLEAR",
                    intake.getSensor4DistanceMm(), intake.getSensor4BaselineMm()));
            panels.addData("Artifact count", intake.getArtifactCount());
            panels.addLine("With both lasers clear, set IntakeSubsystem.SENSOR1/4_BASELINE_MM to the distances above.");
            panels.addLine("-----------------------");

            if (intake.isFull()) {
                panels.addLine("STATUS: FULL (" + IntakeSubsystem.MAX_ARTIFACTS + " BALLS)");
            } else if (intake.getArtifactCount() == 0) {
                panels.addLine("STATUS: EMPTY");
            }

            panels.update(telemetry);
        }
    }
}
