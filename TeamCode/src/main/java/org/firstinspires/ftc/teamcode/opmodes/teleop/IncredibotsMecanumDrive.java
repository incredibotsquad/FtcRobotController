package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Incredibot;

@TeleOp(name = "IncredibotsMecanumDrive", group = "TeleOp")
public class IncredibotsMecanumDrive extends CommandOpMode {
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

        //TODO: get this from CrossOpModeStorage
        incredibot.odometrySubsystem.resetPose(72, 72,0);
    }

    @Override
    public void run() {
        // Run the scheduler first so all subsystems execute their periodic() blocks
        super.run();

        // One unified update call to push everyone's data to the screen at once!
        PanelsTelemetry.INSTANCE.getTelemetry().update(telemetry);
    }
}