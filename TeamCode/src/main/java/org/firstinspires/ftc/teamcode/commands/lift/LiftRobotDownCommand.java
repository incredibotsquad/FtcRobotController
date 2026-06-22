package org.firstinspires.ftc.teamcode.commands.lift;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class LiftRobotDownCommand extends CommandBase {
    private final LiftSubsystem liftSubsystem;
    ElapsedTime liftTimer;
    private static final double LIFT_DURATION = 3000;

    public LiftRobotDownCommand(LiftSubsystem liftSubsystem) {
        this.liftSubsystem = liftSubsystem;

        // This command strictly controls the lift subsystem
        addRequirements(liftSubsystem);
        //TODO: ADD CODE TO
    }

    @Override
    public void initialize() {
        liftTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    @Override
    public void execute() {
        liftSubsystem.startLiftDown();
    }

    @Override
    public boolean isFinished() {
        return liftTimer.milliseconds() >= LIFT_DURATION; // Returns false so it runs continuously in the background
    }

    @Override
    public void end(boolean interrupted) {
        liftSubsystem.stopLift();
    }
}