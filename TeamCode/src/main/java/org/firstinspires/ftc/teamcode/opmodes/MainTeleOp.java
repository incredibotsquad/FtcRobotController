package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Incredibot;

@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends CommandOpMode {
    private Incredibot incredibot;
    private GamepadEx driverGamepad;

    @Override
    public void initialize() {
        // 1. Initialize the robot container
        incredibot = new Incredibot(hardwareMap, Incredibot.OpModeType.TELEOP);
        
        // 2. Set up controllers
        driverGamepad = new GamepadEx(gamepad1);

        // 3. Assign default commands or button bindings
        incredibot.driveSubsystem.setDefaultCommand(new InstantCommand(
                () -> incredibot.driveSubsystem.drive(
                        driverGamepad.getLeftY(),
                        driverGamepad.getLeftX(),
                        driverGamepad.getRightX()
        )));
    }
}