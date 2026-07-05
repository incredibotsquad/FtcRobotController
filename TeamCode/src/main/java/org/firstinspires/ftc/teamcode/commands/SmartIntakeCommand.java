package org.firstinspires.ftc.teamcode.commands;


import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchGateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchSubsystem;

public class SmartIntakeCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private final LaunchGateSubsystem launchGate;

    public SmartIntakeCommand(IntakeSubsystem intake, LaunchGateSubsystem launchGate) {
        this.intake = intake;
        this.launchGate = launchGate;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        // Condition A: Launch gate is open (Resetting/Launching)
        // Condition B: We have fewer than 3 artifacts
        boolean isLaunching = launchGate.isGateOpen();
        boolean needsMoreArtifacts = intake.getArtifactCount() < 3;

//        Log.i("SmartIntakeCommand", "isLaunching: " + isLaunching);
//        Log.i("SmartIntakeCommand", "needsMoreArtifacts: " + needsMoreArtifacts);

        if (isLaunching || needsMoreArtifacts) {
//            Log.i("SmartIntakeCommand", "Staring intake: ");
            intake.startIntake();
        } else {
            // We have 3 and we aren't launching
            Log.i("SmartIntakeCommand", "Stopping intake: ");
            //TODO: keep running the intake for another half a second.
            // Cannot just add a timer here - it would block the main loop.
            intake.stopIntake();
        }
    }

    @Override
    public boolean isFinished() {
        return false; // This stays active throughout the match
    }
}