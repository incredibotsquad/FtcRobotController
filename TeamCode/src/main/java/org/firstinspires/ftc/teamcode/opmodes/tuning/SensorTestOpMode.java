package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * Shows the four intake beam breaks (debounced, as IntakeSubsystem sees them) and the artifact count.
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
            panels.addData("Sensor 1", intake.isSensor1Blocked() ? "BLOCKED" : "CLEAR");
            panels.addData("Sensor 2", intake.isSensor2Blocked() ? "BLOCKED" : "CLEAR");
            panels.addData("Sensor 3", intake.isSensor3Blocked() ? "BLOCKED" : "CLEAR");
            panels.addData("Sensor 4", intake.isSensor4Blocked() ? "BLOCKED" : "CLEAR");
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
