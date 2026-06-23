package org.firstinspires.ftc.teamcode.commands.lift;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class LiftRobotUpCommand extends CommandBase {
    private final LiftSubsystem liftSubsystem;

//    ElapsedTime liftTimer;
//    private static final double LIFT_DURATION = 3000;

    // Define your target position in "Voltage Units"
    // Example: If 0 is top, maybe 10.0 is the bottom (approx 3 full turns)
    private static final double TARGET_TOP_POSITION = 10.0;

    public LiftRobotUpCommand(LiftSubsystem liftSubsystem) {
        this.liftSubsystem = liftSubsystem;

        // This command strictly controls the lift subsystem
        addRequirements(liftSubsystem);
    }

    @Override
    public void initialize() {
//        liftTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    @Override
    public void execute() {
        liftSubsystem.startLiftUp();
    }

    @Override
    public boolean isFinished() {
        // The command finishes when the current position reaches or exceeds the target
        return liftSubsystem.getPosition() >= TARGET_TOP_POSITION;

//        return liftTimer.milliseconds() >= LIFT_DURATION; // Returns false so it runs continuously in the background
    }

    @Override
    public void end(boolean interrupted) {
        liftSubsystem.stopLift();
    }
}