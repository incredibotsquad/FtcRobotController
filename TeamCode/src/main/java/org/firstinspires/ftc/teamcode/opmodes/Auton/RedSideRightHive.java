package org.firstinspires.ftc.teamcode.opmodes.Auton;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.common.AllianceColors;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RedSideRightHive extends CommandOpMode {
    private final PathConstants paths = new PathConstants(AllianceColors.RED);
    private Follower follower;
    private boolean isScheduled = false;
    @Override
    public void initialize() {
        follower = Constants.create(this.hardwareMap);
        follower.setPose(paths.startingPoseRightHive);

    }
    @Override
    public void run(){
        if(!isScheduled){
            schedule(getAutoRoutine());
            isScheduled = true;
        }
        super.run();
        follower.update(); // update the follower
    }
    public Command getAutoRoutine() {
        return new SequentialCommandGroup(
                // --- STEP 1: Main Autonomous Pathing ---
                new SequentialCommandGroup(

                        // 1. Run Near Preloads
                        new FollowPathCommand(follower, paths.intakeGarden(), true),

                        // 2. Run First Spike
                        new FollowPathCommand(follower, paths.moveToLeftHive(), true),
                        new FollowPathCommand(follower, paths.intakeFarFlower(), true),
                        new FollowPathCommand(follower, paths.intakeCloseFlower(), true),

                        // 3. Run Second Spike
                        new FollowPathCommand(follower, paths.moveToShootRightHive(), true),
                        new FollowPathCommand(follower, paths.parkRightSide(), true)
                )
        );
    }
}