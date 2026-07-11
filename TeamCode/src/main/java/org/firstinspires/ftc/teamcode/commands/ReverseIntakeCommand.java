package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class ReverseIntakeCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private final ElapsedTime timer = new ElapsedTime();
    private final double durationSeconds;

    public ReverseIntakeCommand(IntakeSubsystem intake, double durationSeconds) {
        this.intake = intake;
        this.durationSeconds = durationSeconds;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        timer.reset();
        // Assuming your intake subsystem has a method for reverse 
        // or a method where you can pass negative power.
        // If it only has startIntake(), you might need to add reverseIntake() to the subsystem.
        intake.reverseIntake(); 
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return timer.seconds() >= durationSeconds;
    }
}