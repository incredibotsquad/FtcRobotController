package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.LaunchGateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchSubsystem;

public class LaunchBallsCommand extends CommandBase {
    private final LaunchSubsystem launcher;
    private final LaunchGateSubsystem gateSubsystem;
    private final ElapsedTime timer;
    private static final double LAUNCH_TIMEOUT = 1500; // 1.5 seconds to clear all balls

    public LaunchBallsCommand(LaunchSubsystem launcher, LaunchGateSubsystem gateSubsystem) {
        this.launcher = launcher;
        this.gateSubsystem = gateSubsystem;
        this.timer = new ElapsedTime();
        
        // We require the GATE so no other command moves it.
        // We do NOT require the LAUNCHER so the AutoAimCommand 
        // can keep adjusting the aim while we are firing.
        addRequirements(gateSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        // Only open the gate if the flywheel and turret are ready
        if (launcher.isReadyToLaunch()) {
            gateSubsystem.openGate();
        } else {
            // If we lose aim (e.g. robot bumped), close gate immediately 
            // to stop firing mid-air
            gateSubsystem.closeGate();
            timer.reset(); // Reset timer so we get a full 1.5s once ready again
        }
    }

    @Override
    public boolean isFinished() {
        // Finish once the gate has been open and ready for 1.5 seconds
        return launcher.isReadyToLaunch() && timer.milliseconds() > LAUNCH_TIMEOUT;
    }

    @Override
    public void end(boolean interrupted) {
        // Safety: Always close the gate when the command ends
        gateSubsystem.closeGate();
    }
}