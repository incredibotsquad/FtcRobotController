package org.firstinspires.ftc.teamcode.opmodes.Auton;

import static org.firstinspires.ftc.teamcode.opmodes.Auton.PathConstants.intakeFarFlowerLeftSide;
import static org.firstinspires.ftc.teamcode.opmodes.Auton.PathConstants.parkLeftSide;
import static org.firstinspires.ftc.teamcode.opmodes.Auton.PathConstants.startingPoseLeftHive;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.Commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RedSideLeftHive extends CommandOpMode {
    private Follower follower;
    private boolean isScheduled = false;
    @Override
    public void initialize() {
        follower = Constants.create(this.hardwareMap);
        follower.setPose(startingPoseLeftHive);

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
                        new FollowPathCommand(follower, intakeFarFlowerLeftSide(), true),
                        new FollowPathCommand(follower,parkLeftSide(), true)
                )
        );
    }
}