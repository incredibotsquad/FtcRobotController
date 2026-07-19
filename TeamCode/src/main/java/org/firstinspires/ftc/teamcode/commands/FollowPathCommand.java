package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

public class FollowPathCommand extends CommandBase {
    private final Follower follower;
    private final PathChain path;
    private final boolean holdEnd;

    public FollowPathCommand(Follower follower, PathChain path, boolean holdEnd) {
        this.follower = follower;
        this.path = path;
        this.holdEnd = holdEnd;
    }

    @Override
    public void initialize() {
        follower.followPath(path, holdEnd);

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
    }

    @Override
    public boolean isFinished() {
        // Command finishes when the follower is no longer busy

        Log.i("FollowPathCommand", "Finished returned: " + !follower.isBusy());
        return !follower.isBusy();
    }
}
