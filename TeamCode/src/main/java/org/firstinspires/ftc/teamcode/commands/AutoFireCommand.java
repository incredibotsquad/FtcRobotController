package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LaunchGateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;

/**
 * STUB: placeholder so the project compiles. Does nothing and finishes immediately.
 * Intentionally declares no requirements, so it never interrupts LaunchReadinessCommand
 * (flywheel/turret) or CloseGateCommand.
 */
public class AutoFireCommand extends CommandBase {

    public AutoFireCommand(LaunchSubsystem launchSubsystem, LaunchGateSubsystem launchGateSubsystem,
                           OdometrySubsystem odometrySubsystem) {
    }

    @Override
    public void initialize() {
        Log.w("AutoFireCommand", "STUB: not implemented yet, not firing");
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
