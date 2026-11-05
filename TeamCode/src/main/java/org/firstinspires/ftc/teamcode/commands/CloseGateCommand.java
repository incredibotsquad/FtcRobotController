package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LaunchGateSubsystem;

/**
 * Default command for the gate: closes it whenever no launch command owns the gate.
 * Requiring the gate lets LaunchBallsCommand interrupt this; the scheduler restarts it
 * (and initialize() closes the gate again) when the launch ends.
 */
public class CloseGateCommand extends CommandBase {
    private final LaunchGateSubsystem gate;

    public CloseGateCommand(LaunchGateSubsystem launchGateSubsystem) {
        this.gate = launchGateSubsystem;
        addRequirements(launchGateSubsystem);
    }

    @Override
    public void initialize() {
        Log.i("CloseGateCommand", "Closing gate");
        gate.closeGate();
    }

    @Override
    public void execute() {
        // Re-close only if something opened the gate without owning it (no servo write or log otherwise).
        if (gate.isGateOpen()) {
            gate.closeGate();
        }
    }
}
