package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;

/**
 * STUB: placeholder so the project compiles. Does not change the pose; finishes immediately.
 * Declares no requirements, so scheduling it never interrupts anything.
 */
public class RelocalizeWithLimelightCommand extends CommandBase {

    public RelocalizeWithLimelightCommand(LimelightSubsystem limelightSubsystem,
                                          OdometrySubsystem odometrySubsystem,
                                          LaunchSubsystem launchSubsystem,
                                          DriveSubsystem driveSubsystem) {
    }

    @Override
    public void initialize() {
        Log.w("RelocalizeWithLimelight", "STUB: not implemented yet, pose unchanged");
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
