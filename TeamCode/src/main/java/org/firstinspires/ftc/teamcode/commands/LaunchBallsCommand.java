package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.LaunchGateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchSubsystem;

public class LaunchBallsCommand extends CommandBase {
    private final LaunchSubsystem launcher;
    private final LaunchGateSubsystem launchGate;

    public LaunchBallsCommand(LaunchSubsystem launchSubsystem, LaunchGateSubsystem launchGateSubsystem) {
        this.launcher = launchSubsystem;
        this.launchGate = launchGateSubsystem;

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

        Log.i("LaunchBallsCommand", "Execute: opening gate without any checks");
        launchGate.openGate();

    }

    @Override
    public boolean isFinished() {
        return true;
    }


    /**
     * Called when the command ends - either normally via finished or if interrupted / cancelled.
     * this ensures the gate closes always
     * @param interrupted true if the command was cancelled**/
    @Override
    public void end(boolean interrupted) {
        Log.i("LaunchBallsCommand", "End: command interrupted: " + interrupted);
        // Safety: Always close the gate when the command ends
        launchGate.closeGate();
    }
}