package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.util.Log;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Incredibot;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.commands.LaunchBallsCommand;
import org.firstinspires.ftc.teamcode.common.AllianceColors;
import org.firstinspires.ftc.teamcode.common.CrossOpModeStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class ConfigAuto extends CommandOpMode {
    private Incredibot robot;
    private Follower follower;
    private Poses poses;
    private Paths paths;
    private double cycleTimer;
    
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private enum AutoPath {
        PRELOADS, HUMAN_PLAYER, TUNNEL, SPIKE_3, SPIKE_1, SPIKE_2, GATE
    }
    private List<AutoPath> pathOrder = new ArrayList<>();

    private boolean resetTimer = false;

    public String alliance = "red";

    public String position = "near";

    @Override
    public void initialize() {
        Log.i("Run Opmode", "Init");

        // Initialize the robot first. This creates the OdometrySubsystem, which calls pinpoint.resetPosAndIMU() (resetting to 0,0).
        robot = new Incredibot(hardwareMap, PanelsTelemetry.INSTANCE.getTelemetry());
        robot.initialize(Incredibot.OpModeType.AUTO, null, null);

        // Now initialize the follower and set the pose. This will correctly override the reset.
        follower = Constants.createFollower(this.hardwareMap);

        while(opModeInInit()){
            if(gamepad1.xWasReleased()){
                alliance = "blue";
                CrossOpModeStorage.allianceColor = AllianceColors.BLUE;
            }
            if(gamepad1.bWasReleased()){
                alliance = "red";
                CrossOpModeStorage.allianceColor = AllianceColors.RED;
            }
            if(gamepad1.aWasReleased()){
                position = "far";
            }
            if(gamepad1.yWasReleased()){
                position = "near";
            }
            if (gamepad2.xWasReleased()) {
                pathOrder.add(AutoPath.HUMAN_PLAYER);
            }
            if (gamepad2.bWasReleased()) {
                pathOrder.add(AutoPath.SPIKE_3);
            }
            if (gamepad2.aWasReleased()) {
                pathOrder.add(AutoPath.TUNNEL);
            }
            if (gamepad2.dpadLeftWasReleased()) {
                pathOrder.add(AutoPath.SPIKE_1);
            }
            if (gamepad2.dpadUpWasReleased()) {
                pathOrder.add(AutoPath.SPIKE_2);
            }
            if (gamepad2.dpadRightWasReleased()) {
                pathOrder.add(AutoPath.GATE);
            }
            if (gamepad2.yWasReleased()) {
                pathOrder.clear();
            }

            telemetry.addData("Selected Alliance", alliance);
            telemetry.addData("Selected Position", position);
            telemetry.addData("Selected Order", pathOrder.toString());
            telemetry.addLine("Gamepad 1 A: Far | Gamepad 1 B: Red | Gamepad X: Blue | Gamepad 1 Y: Near");
            telemetry.addLine("Gamepad 2 X: Human | Gamepad 2 B: Spike 3 | Gampad 2 A: Tunnel | Gamepad 2 Y: Clear");
            telemetry.addLine("Gamepad 2 Dpad Left: Spike 1 | Gamepad 2 Dpad Up: Spike 2 | Gamepad 2 Dpad Right: Gate");
            telemetry.update();
        }

        poses = new Poses(alliance.equals("red"));
        if(position.equals("far")){
            follower.setStartingPose(poses.FAR_START_POSE);
            cycleTimer = 27;
        }else{
            follower.setStartingPose(poses.NEAR_INIT);
            cycleTimer = 25;
        }

        paths = new Paths(follower, poses);
        paths.buildNearPaths();
        paths.buildFarPaths();

        Log.i("Run Opmode", "Follower pose: " + follower.getPose());

        schedule(getAutoRoutine());
    }

    @Override
    public void run() {
        if(!resetTimer){
            timer.reset();
            resetTimer = true;
        }
        super.run();
        follower.update();
        Log.i("Run Opmode", "Follower pose: " + follower.getPose());
    }

    public Command getAutoRoutine() {
        SequentialCommandGroup group = new SequentialCommandGroup();
        group.addCommands(getPathSequence(AutoPath.PRELOADS));

        for (AutoPath path : pathOrder) {
            group.addCommands(new ConditionalCommand(
                    getPathSequence(path),
                    new InstantCommand(),
                    () -> timer.seconds() < cycleTimer
            ));
        }

        // Keep cycling if there's time
        AutoPath cyclePath = position.equals("far") ? AutoPath.TUNNEL : AutoPath.GATE;
        for (int i = 0; i < 10; i++) {
            group.addCommands(new ConditionalCommand(
                    getPathSequence(cyclePath),
                    new InstantCommand(),
                    () -> timer.seconds() < cycleTimer
            ));
        }

        group.addCommands(new FollowPathCommand(follower, position.equals("far") ? paths.FAR_LEAVE : paths.NEAR_LEAVE, true));

        return group;
    }

    private Command getPathSequence(AutoPath path) {
        if (position.equals("far")) {
            switch (path) {
                case PRELOADS:
                    return new LaunchBallsCommand(robot.launchSubsystem, robot.launchGateSubsystem);
                case HUMAN_PLAYER:
                    return new SequentialCommandGroup(
                            new FollowPathCommand(follower, paths.FAR_HUMAN_PLAYER, true),
                            new FollowPathCommand(follower, paths.FAR_SCORE_HUMAN, true),
                            new LaunchBallsCommand(robot.launchSubsystem, robot.launchGateSubsystem)
                    );
                case SPIKE_1:
                    return new SequentialCommandGroup(
                            new FollowPathCommand(follower, paths.FAR_SPIKE_1, true),
                            new FollowPathCommand(follower, paths.FAR_SCORE_SPIKE_1, true),
                            new LaunchBallsCommand(robot.launchSubsystem, robot.launchGateSubsystem)
                    );
                case SPIKE_2:
                    return new SequentialCommandGroup(
                            new FollowPathCommand(follower, paths.FAR_SPIKE_2, true),
                            new FollowPathCommand(follower, paths.FAR_SCORE_SPIKE_2, true),
                            new LaunchBallsCommand(robot.launchSubsystem, robot.launchGateSubsystem)
                    );
                case SPIKE_3:
                    return new SequentialCommandGroup(
                            new FollowPathCommand(follower, paths.FAR_SPIKE_3, true),
                            new FollowPathCommand(follower, paths.FAR_SCORE_SPIKE_3, true),
                            new LaunchBallsCommand(robot.launchSubsystem, robot.launchGateSubsystem)
                    );
                case TUNNEL:
                    return new SequentialCommandGroup(
                            new FollowPathCommand(follower, paths.FAR_TUNNEL, true),
                            new FollowPathCommand(follower, paths.FAR_SCORE_TUNNEL, true),
                            new LaunchBallsCommand(robot.launchSubsystem, robot.launchGateSubsystem)
                    );
                case GATE:
                    return new SequentialCommandGroup(
                            new FollowPathCommand(follower, paths.FAR_GATE, true),
                            new FollowPathCommand(follower, paths.FAR_GATE_2, true),
                            new FollowPathCommand(follower, paths.FAR_SCORE_GATE, true),
                            new LaunchBallsCommand(robot.launchSubsystem, robot.launchGateSubsystem)
                    );
                default:
                    return new InstantCommand();
            }
        } else {
            switch (path) {
                case PRELOADS:
                    return new SequentialCommandGroup(
                            new FollowPathCommand(follower, paths.NEAR_PRELOADS, true),

                            new LaunchBallsCommand(robot.launchSubsystem, robot.launchGateSubsystem)
                    );
                case HUMAN_PLAYER:
                    return new SequentialCommandGroup(
                            new FollowPathCommand(follower, paths.NEAR_HUMAN_PLAYER, true),
                            new FollowPathCommand(follower, paths.NEAR_SCORE_HUMAN, true),
                            new LaunchBallsCommand(robot.launchSubsystem, robot.launchGateSubsystem)
                    );
                case SPIKE_1:
                    return new SequentialCommandGroup(
                            new FollowPathCommand(follower, paths.NEAR_SPIKE_1, true),
                            new FollowPathCommand(follower, paths.NEAR_SCORE_SPIKE_1, true),
                            new LaunchBallsCommand(robot.launchSubsystem, robot.launchGateSubsystem)
                    );
                case SPIKE_2:
                    return new SequentialCommandGroup(
                            new FollowPathCommand(follower, paths.NEAR_SPIKE_2, true),
                            new FollowPathCommand(follower, paths.NEAR_SCORE_SPIKE_2, true),
                            new LaunchBallsCommand(robot.launchSubsystem, robot.launchGateSubsystem)
                    );
                case SPIKE_3:
                    return new SequentialCommandGroup(
                            new FollowPathCommand(follower, paths.NEAR_SPIKE_3, true),
                            new FollowPathCommand(follower, paths.NEAR_SCORE_SPIKE_3, true),
                            new LaunchBallsCommand(robot.launchSubsystem, robot.launchGateSubsystem)
                    );
                case TUNNEL:
                    return new SequentialCommandGroup(
                            new FollowPathCommand(follower, paths.NEAR_TUNNEL, true),
                            new FollowPathCommand(follower, paths.NEAR_SCORE_TUNNEL, true),
                            new LaunchBallsCommand(robot.launchSubsystem, robot.launchGateSubsystem)
                    );
                case GATE:
                    return new SequentialCommandGroup(
                            new FollowPathCommand(follower, paths.NEAR_GATE, true),
                            new FollowPathCommand(follower, paths.NEAR_GATE_2, true),
                            new FollowPathCommand(follower, paths.NEAR_SCORE_GATE, true),
                            new LaunchBallsCommand(robot.launchSubsystem, robot.launchGateSubsystem)
                    );
                default:
                    return new InstantCommand();
            }
        }
    }
}