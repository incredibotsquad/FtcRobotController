package org.firstinspires.ftc.teamcode.opmodes.Auton;

import static org.firstinspires.ftc.teamcode.opmodes.Auton.PathConstants.intakeGarden;
import static org.firstinspires.ftc.teamcode.opmodes.Auton.PathConstants.moveToLeftHive;
import static org.firstinspires.ftc.teamcode.opmodes.Auton.PathConstants.moveToShootRightHive;
import static org.firstinspires.ftc.teamcode.opmodes.Auton.PathConstants.parkRightSide;
import static org.firstinspires.ftc.teamcode.opmodes.Auton.PathConstants.startingPoseRightHive;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.Commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RedSideRightHive extends CommandOpMode {
    private Follower follower;
    private boolean isScheduled = false;
    @Override
    public void initialize() {
        follower = Constants.create(this.hardwareMap);
        follower.setPose(startingPoseRightHive);

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
                        new FollowPathCommand(follower, intakeGarden(), true),

                        // 2. Run First Spike
                        new FollowPathCommand(follower, moveToLeftHive(), true),
                        new FollowPathCommand(follower, PathConstants.intakeFarFlower(), true),
                        new FollowPathCommand(follower, PathConstants.intakeCloseFlower(), true),

                        // 3. Run Second Spike
                        new FollowPathCommand(follower, moveToShootRightHive(), true),
                        new FollowPathCommand(follower, parkRightSide(), true)
                )
        );
    }
}