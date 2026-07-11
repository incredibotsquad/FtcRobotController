package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LaunchKickSubsystem;

public class ResetKickCommand extends CommandBase {
    private final LaunchKickSubsystem kickSubsystem;
    public ResetKickCommand(LaunchKickSubsystem launchKickSubsystem) {
        this.kickSubsystem = launchKickSubsystem;
        addRequirements(launchKickSubsystem); // This is what allows the interrupt logic to work
    }

    @Override
    public void execute() {
        kickSubsystem.retractKicker();
    }
}