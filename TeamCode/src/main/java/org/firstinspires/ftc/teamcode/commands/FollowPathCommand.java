package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.FileDescriptor;

@Configurable
public class FollowPathCommand extends CommandBase {
    private final Follower follower;
    private final PathChain path;
    private final boolean holdEnd;

    // Stall detection variables
    private ElapsedTime stallTimer = new ElapsedTime();
    private static final double STALL_VELOCITY_THRESHOLD = 0.5; // inches per second
    private static final double STALL_TIMEOUT = 750; // milliseconds before giving up

    public FollowPathCommand(Follower follower, PathChain path, boolean holdEnd) {
        this.follower = follower;
        this.path = path;
        this.holdEnd = holdEnd;
    }

    @Override
    public void initialize() {
        follower.followPath(path, holdEnd);
        stallTimer.reset();

        /*
        * In some versions of Pedro Pathing, if isFinished() is checked before the very first
        * follower.update() has a chance to run, there is a tiny window where the follower
        * hasn't fully "latched" into the path.To make your command 100% stable,
        * it is a best practice to call follower.update() once at the end of initialize.
        * This ensures the state is locked before the scheduler even asks if the command is finished.
        * */
        follower.update();
    }

    @Override
    public void execute() {

        follower.update();

        // Pedro Pathing update is usually handled in the OpMode's run()         // but we can check velocity here
        double currentVelocity = follower.getVelocity().getMagnitude();

        // If we are moving faster than the threshold, reset the timer
        if (currentVelocity > STALL_VELOCITY_THRESHOLD) {
            stallTimer.reset();
        }
    }

    @Override
    public boolean isFinished() {
        // Finish if:
        // 1. Pedro says we are done
        // 2. We have been stuck (velocity < threshold) for too long
        return !follower.isBusy() || stallTimer.milliseconds() > STALL_TIMEOUT;
    }

    @Override
    public void end(boolean interrupted) {
        if (stallTimer.milliseconds() > STALL_TIMEOUT) {
            Log.w("Auto", "Path Stalled! Moving to next command.");
            follower.breakFollowing(); // Stop the motors immediately
        }
    }
}
