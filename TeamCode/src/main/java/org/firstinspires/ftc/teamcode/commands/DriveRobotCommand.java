package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class DriveRobotCommand extends CommandBase {
    private final DriveSubsystem driveSubsystem;

    private final GamepadEx driverGamepad;

    public DriveRobotCommand(DriveSubsystem driveSubsystem, GamepadEx driverGamepad) {
        this.driveSubsystem = driveSubsystem;
        this.driverGamepad = driverGamepad;

        // This command strictly controls the drive train
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
//        Log.i("DriveRobotCommand", "execute. left y:" + driverGamepad.getLeftY() + " left x: " + driverGamepad.getLeftX() + " right X: " +  driverGamepad.getRightX());
        driveSubsystem.drive(-driverGamepad.getLeftX(), -driverGamepad.getLeftY(), -driverGamepad.getRightX());
    }

    @Override
    public boolean isFinished() {
        return false; // Returns false so it runs continuously in the background
    }

}