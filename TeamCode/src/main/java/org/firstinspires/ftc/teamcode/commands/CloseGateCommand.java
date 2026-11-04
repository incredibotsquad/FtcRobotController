package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LaunchGateSubsystem;

public class CloseGateCommand extends CommandBase {
    private final LaunchGateSubsystem gate;
    public CloseGateCommand(LaunchGateSubsystem launchGateSubsystem) {
        this.gate = launchGateSubsystem;
        addRequirements(launchGateSubsystem); // This is what allows the interrupt logic to work
    }

    @Override
    public void execute() {

        Log.i("CloseGateCommand", "Execute: closing gate");
        gate.closeGate();
    }
}