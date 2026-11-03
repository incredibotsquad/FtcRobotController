package org.firstinspires.ftc.teamcode.opmodes.Auton;

import static org.firstinspires.ftc.teamcode.opmodes.Auton.PathConstants.intakeFlower1;
import static org.firstinspires.ftc.teamcode.opmodes.Auton.PathConstants.intakeFlower2;
import static org.firstinspires.ftc.teamcode.opmodes.Auton.PathConstants.intakeGarden;
import static org.firstinspires.ftc.teamcode.opmodes.Auton.PathConstants.moveToHive1;
import static org.firstinspires.ftc.teamcode.opmodes.Auton.PathConstants.moveToHive2;
import static org.firstinspires.ftc.teamcode.opmodes.Auton.PathConstants.startingPose;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.Commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RedHive1 extends CommandOpMode {
    private Follower follower;
    private boolean isScheduled = false;
    @Override
    public void initialize() {
        follower = Constants.create(this.hardwareMap);
        follower.setPose(startingPose);

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
                        new FollowPathCommand(follower, moveToHive2(), true),
                        new FollowPathCommand(follower, intakeFlower1(), true),
                        new FollowPathCommand(follower, intakeFlower2(), true),

                        // 3. Run Second Spike
                        new FollowPathCommand(follower, moveToHive1(), true)
                )
        );
    }
}