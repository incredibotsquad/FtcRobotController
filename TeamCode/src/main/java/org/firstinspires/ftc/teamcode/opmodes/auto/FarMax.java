package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.util.Log;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.RepeatCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Incredibot;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.commands.LaunchBallsCommand;
import org.firstinspires.ftc.teamcode.common.AllianceColors;
import org.firstinspires.ftc.teamcode.common.CrossOpModeStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "Far Max", group = "Autonomous")
public class FarMax extends CommandOpMode {
    private Incredibot robot;
    private Follower follower;
    private Poses poses;
    private Paths paths;
    public static long INTAKE_WAIT = 1500;
    private final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private boolean resetTimer = false;
    public AllianceColors alliance = AllianceColors.RED; // Default

    @Override
    public void initialize() {
        // 1. Subsystem Initialization
        robot = new Incredibot(hardwareMap, PanelsTelemetry.INSTANCE.getTelemetry());
        robot.initialize(Incredibot.OpModeType.AUTO, null, null);

        robot.launchGateSubsystem.closeGate();

        follower = Constants.createFollower(this.hardwareMap);

        // 2. Init Loop for Alliance Selection
        while (opModeInInit()) {
            if (gamepad2.xWasReleased()) {
                alliance = AllianceColors.BLUE;
            }
            if (gamepad2.bWasReleased()) {
                alliance = AllianceColors.RED;
            }

            CrossOpModeStorage.allianceColor = alliance;

            telemetry.addData("STATUS", "READY - NEAR START");
            telemetry.addData("Selected Alliance: ", alliance.toString());
            telemetry.addLine("------------------------------------");
//            telemetry.addLine("Gamepad 1 X: BLUE Alliance");
//            telemetry.addLine("Gamepad 1 B: RED Alliance");
            telemetry.update();
        }

        // 3. Pose and Path Setup
        poses = new Poses(alliance == AllianceColors.RED);
        follower.setStartingPose(poses.FAR_INIT);

        paths = new Paths(follower, poses);
        paths.buildFarPaths(); // Only build near paths

        // 4. Schedule the State Machine
        schedule(getAutoRoutine());
    }

    @Override
    public void run() {
        if (!resetTimer) {
            timer.reset();
            resetTimer = true;
        }
        super.run();
        follower.update();
        Pose currentFollowerPose = follower.getPose();

        if (currentFollowerPose.getX() != 0 && currentFollowerPose.getY() != 0)
            CrossOpModeStorage.currentPose = new Pose2d(currentFollowerPose.getX(), currentFollowerPose.getY(), new Rotation2d(currentFollowerPose.getHeading()));
    }

    public Command getAutoRoutine() {
        return new SequentialCommandGroup(
                // --- STEP 1: Main Autonomous Pathing ---
                new SequentialCommandGroup(
                        new InstantCommand(() -> Log.i("Far Max", "Starting Preload and Spikes")),

                        new FollowPathCommand(follower, paths.FAR_PRELOADS, true),
                        new LaunchBallsCommand(robot.launchSubsystem, robot.launchGateSubsystem, false, true),

                        new FollowPathCommand(follower, paths.FAR_SPIKE_3, true),
                        new FollowPathCommand(follower, paths.FAR_SCORE_SPIKE_3, true),
                        new LaunchBallsCommand(robot.launchSubsystem, robot.launchGateSubsystem, false, true),

                        //human player repeat sequence
                        new FollowPathCommand(follower, paths.FAR_HUMAN_PLAYER, true),
                        new ParallelRaceGroup(
                                new WaitCommand(INTAKE_WAIT),
                                new WaitUntilCommand(()-> robot.intakeSubsystem.getArtifactCount() == 3),
                                new RepeatCommand(
                                        new SequentialCommandGroup(
                                                new FollowPathCommand(follower, paths.FAR_JIGGLE_GATE_PATH_1, false),
                                                new FollowPathCommand(follower, paths.FAR_JIGGLE_GATE_PATH_2, false)))
                        ),
                        new FollowPathCommand(follower, paths.FAR_SCORE_HUMAN, true),
                        new LaunchBallsCommand(robot.launchSubsystem, robot.launchGateSubsystem, false, true),

                        //human player sequence 2
                        new FollowPathCommand(follower, paths.FAR_HUMAN_PLAYER, true),
                        new ParallelRaceGroup(
                                new WaitCommand(INTAKE_WAIT),
                                new WaitUntilCommand(()-> robot.intakeSubsystem.getArtifactCount() == 3),
                                new RepeatCommand(
                                        new SequentialCommandGroup(
                                                new FollowPathCommand(follower, paths.FAR_JIGGLE_GATE_PATH_1, false),
                                                new FollowPathCommand(follower, paths.FAR_JIGGLE_GATE_PATH_2, false)))
                        ),
                        new FollowPathCommand(follower, paths.FAR_SCORE_HUMAN, true),
                        new LaunchBallsCommand(robot.launchSubsystem, robot.launchGateSubsystem, false, true),

                        //human player sequence 3
                        new FollowPathCommand(follower, paths.FAR_HUMAN_PLAYER, true),
                        new ParallelRaceGroup(
                                new WaitCommand(INTAKE_WAIT),
                                new WaitUntilCommand(()-> robot.intakeSubsystem.getArtifactCount() == 3),
                                new RepeatCommand(
                                        new SequentialCommandGroup(
                                                new FollowPathCommand(follower, paths.FAR_JIGGLE_GATE_PATH_1, false),
                                                new FollowPathCommand(follower, paths.FAR_JIGGLE_GATE_PATH_2, false)))
                        ),
                        new FollowPathCommand(follower, paths.FAR_SCORE_HUMAN, true),
                        new LaunchBallsCommand(robot.launchSubsystem, robot.launchGateSubsystem, false, true),

                        //human player sequence 4
                        new FollowPathCommand(follower, paths.FAR_HUMAN_PLAYER, true),
                        new ParallelRaceGroup(
                                new WaitCommand(INTAKE_WAIT),
                                new WaitUntilCommand(()-> robot.intakeSubsystem.getArtifactCount() == 3),
                                new RepeatCommand(
                                        new SequentialCommandGroup(
                                                new FollowPathCommand(follower, paths.FAR_JIGGLE_GATE_PATH_1, false),
                                                new FollowPathCommand(follower, paths.FAR_JIGGLE_GATE_PATH_2, false)))
                        ),
                        new FollowPathCommand(follower, paths.FAR_SCORE_HUMAN, true),
                        new LaunchBallsCommand(robot.launchSubsystem, robot.launchGateSubsystem, false, true),

                        new FollowPathCommand(follower, paths.FAR_PARK, true),

                        new InstantCommand(() -> Log.i("Far Max", "Seconds " + timer.seconds()))
                )

                // --- STEP 2: 25-Second "End Game" / Gate Logic ---
                // If we finish the spikes and time is > 25s, we immediately go to the gate/park.
//                new ConditionalCommand(
//                        // Logic to run if time is running out: Go to the Gate
//                        new SequentialCommandGroup(
//                                new InstantCommand(() -> Log.i("Auto", "Time low: Navigating to Gate")),
//                                new FollowPathCommand(follower, paths.NEAR_PARK, true)
//                        ),
//                        // Logic to run if we still have time:
//                        // You could add a third spike here or another action
//                        new InstantCommand(() -> Log.i("Auto", "Spikes complete with time to spare")),
//
//                        () -> timer.seconds() > 25.0
//                )
        );
    }
}
