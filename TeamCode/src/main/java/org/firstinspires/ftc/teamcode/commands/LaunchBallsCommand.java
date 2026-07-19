package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.LaunchGateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchSubsystem;

public class LaunchBallsCommand extends CommandBase {
    private final LaunchSubsystem launcher;
    private final LaunchGateSubsystem launchGate;
    private boolean initialized = false;
    private final ElapsedTime timer;
    private static final double LAUNCH_DURATION_SLOW = 1500; // 1.5 second to clear all balls
    private static final double LAUNCH_DURATION_FAST = 500; // 1.5 second to clear all balls

    private double LAUNCH_DURATION = LAUNCH_DURATION_SLOW;

    public LaunchBallsCommand(LaunchSubsystem launchSubsystem, LaunchGateSubsystem launchGateSubsystem) {
        this.launcher = launchSubsystem;
        this.launchGate = launchGateSubsystem;
        this.initialized = false;
        this.timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        // We require the GATE so no other command moves it.
        // We do NOT require the LAUNCHER so the AutoAimCommand 
        // can keep adjusting the aim while we are firing.
        addRequirements(launchGateSubsystem);
    }

    public LaunchBallsCommand(LaunchSubsystem launchSubsystem, LaunchGateSubsystem launchGateSubsystem, boolean fast) {
        this.launcher = launchSubsystem;
        this.launchGate = launchGateSubsystem;
        this.initialized = false;
        this.timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        LAUNCH_DURATION = LAUNCH_DURATION_FAST;

        // We require the GATE so no other command moves it.
        // We do NOT require the LAUNCHER so the AutoAimCommand
        // can keep adjusting the aim while we are firing.
        addRequirements(launchGateSubsystem);
    }


    @Override
    public void execute() {
        // Only open the gate if the flywheel and turret are ready
//        if (launcher.isReadyToLaunch()) {
//            Log.i("LaunchBallsCommand", "Execute: launcher ready: opening gate");
//            launchGate.openGate();
//        } else {
//            Log.i("LaunchBallsCommand", "Execute: launcher not ready to launch");
//            // If we lose aim (e.g. robot bumped), close gate immediately
//            // to stop firing mid-air
//            launchGate.closeGate();
//            timer.reset(); // Reset timer so we get a full LAUNCH_DURATION once ready again
//        }
        if (!initialized) {
            Log.i("LaunchBallsCommand", "Initialize - resetting timer");
            timer.reset();
            initialized = true;
        }

        Log.i("LaunchBallsCommand", "Execute: opening gate without any checks");
        launchGate.openGate();

    }

    @Override
    public boolean isFinished() {
        boolean finished = timer.milliseconds() > LAUNCH_DURATION;
        // Finish once the gate has been open and ready for 1 seconds

        Log.i("LaunchBallsCommand", "Finished returned: " + finished);

        return finished;
    }

    /**
     * Called when the command ends - either normally via finished or if interrupted / cancelled.
     * this ensures the gate closes always
     * @param interrupted true if the command was cancelled**/
    @Override
    public void end(boolean interrupted) {
        Log.i("LaunchBallsCommand", "End: command interrupted: " + interrupted);
        // Safety: Always close the gate when the command ends
        initialized = false;
        launchGate.closeGate();
    }
}