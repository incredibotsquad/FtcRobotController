package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Incredibot;

@TeleOp(name = "MainTeleOp", group = "TeleOp")
public class MainTeleOp extends CommandOpMode {
    private Incredibot incredibot;
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;

    @Override
    public void initialize() {

        //Set up controllers
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        // Initialize the robot container
        incredibot = new Incredibot(
                hardwareMap,
                Incredibot.OpModeType.TELEOP,
                driverGamepad,
                operatorGamepad,
                PanelsTelemetry.INSTANCE.getTelemetry());

        incredibot.odometrySubsystem.resetPose(new Pose2d(72, 72,  Rotation2d.fromDegrees(90)));
    }

    @Override
    public void run() {
        // Run the scheduler first so all subsystems execute their periodic() blocks
        super.run();

        // One unified update call to push everyone's data to the screen at once!
        PanelsTelemetry.INSTANCE.getTelemetry().update(telemetry);
    }
}