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
    private final ElapsedTime initializationTimer = new ElapsedTime();
    private static final double STALL_VELOCITY_THRESHOLD = 0.5; // inches per second
    private static final double STALL_TIMEOUT = 750; // milliseconds before giving up
    private static final double END_TOLERANCE = 3.0; // Finish when 1 inch away
    private static final double MINIMUM_RUN_TIME = 50.0; // Ensure at least 50ms of run time

    public FollowPathCommand(Follower follower, PathChain path, boolean holdEnd) {
        this.follower = follower;
        this.path = path;
        this.holdEnd = holdEnd;
    }

    @Override
    public void initialize() {
        follower.followPath(path, holdEnd);
        stallTimer.reset();
        initializationTimer.reset();

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

        // Pedro Pathing update is usually handled in the OpMode's run()
        // but we can check velocity here

//        follower.update();

        double currentVelocity = follower.getVelocity().getMagnitude();

        // If we are moving faster than the threshold, reset the timer
        if (currentVelocity > STALL_VELOCITY_THRESHOLD) {
            stallTimer.reset();
        }
    }

    @Override
    public boolean isFinished() {
        // SAFETY: Do not allow the Compiler.command to finish if it hasn't been running for 50ms.
        // This prevents race conditions where isBusy() is checked before the path latches.
        if (initializationTimer.milliseconds() < MINIMUM_RUN_TIME) {
            return false;
        }

        // Finish if:
        // 1. Pedro says done
        // 2. We are "close enough" (Optimal for speed)
        // 3. Stall Detection

        double distanceRemaining = follower.getCurrentPath().getDistanceRemaining();
        boolean closeEnough = distanceRemaining < END_TOLERANCE;

        return !follower.isBusy() || closeEnough || stallTimer.milliseconds() > STALL_TIMEOUT;
    }

    @Override
    public void end(boolean interrupted) {
        if (stallTimer.milliseconds() > STALL_TIMEOUT) {
            Log.w("FollowPathCommand", "Path Stalled! Moving to next command.");
            follower.breakFollowing(); // Stop the motors immediately
        }
    }
}
