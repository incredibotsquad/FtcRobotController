package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class ReverseIntakeCommand extends CommandBase {
    private final IntakeSubsystem intake;
    public ReverseIntakeCommand(IntakeSubsystem intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void execute() {

        // Assuming your intake subsystem has a method for reverse 
        // or a method where you can pass negative power.
        // If it only has startIntake(), you might need to add reverseIntake() to the subsystem.
        intake.reverseIntake();

        Log.i("ReverseIntakeCommand", "Executing");
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Log.i("ReverseIntakeCommand", "Ended");
        intake.stopIntake();
    }
}