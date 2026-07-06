package org.firstinspires.ftc.teamcode.commands;


import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchGateSubsystem;

@Configurable
public class SmartIntakeCommand extends CommandBase {
    public  static double DELAYED_STOP_DURATION_MILLIS = 1500;
    private final IntakeSubsystem intake;
    private final LaunchGateSubsystem launchGate;
    private ElapsedTime delayedStoptimer;
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
            delayedStoptimer = null;

        } else {
            // We have 3 and we aren't launching
            Log.i("SmartIntakeCommand", "Stopping intake: ");

            //keep running the intake for another half a second.
            if (delayedStoptimer == null) {
                delayedStoptimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            }
            else if (delayedStoptimer.milliseconds() > DELAYED_STOP_DURATION_MILLIS) {
                intake.stopIntake();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false; // This stays active throughout the match
    }
}