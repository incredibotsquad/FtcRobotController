package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * Runs the intake, transfer and sweepers in reverse to eject artifacts.
 * Bind with whileHeld(): it runs until the button is released, then stops the intake.
 * Requiring the intake interrupts the default intake command while held; the scheduler
 * restarts the default command on release.
 */
public class ReverseIntakeCommand extends CommandBase {
    private final IntakeSubsystem intake;

    public ReverseIntakeCommand(IntakeSubsystem intakeSubsystem) {
        this.intake = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intake.reverseIntakeAndTransfer();
    }

    @Override
    public void execute() {
        // Re-apply every loop in case something else touched the motors
        intake.reverseIntakeAndTransfer();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntakeAndTransfer();
    }

    @Override
    public boolean isFinished() {
        return false; // runs until the button is released (whileHeld cancels it)
    }
}
