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
@Autonomous(name = "Near 12", group = "Autonomous")
public class Near12 extends CommandOpMode {
    private Incredibot robot;
    private Follower follower;
    private Poses poses;
    private Paths paths;
    public static long INTAKE_WAIT = 2000;
    private final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private boolean resetTimer = false;
    public AllianceColors alliance = AllianceColors.RED; // Default

    private final ElapsedTime intakeTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void initialize() {
        // 1. Subsystem Initialization
        robot = new Incredibot(hardwareMap, PanelsTelemetry.INSTANCE.getTelemetry());
        robot.initialize(Incredibot.OpModeType.AUTO, null, null);
        follower = Constants.createFollower(this.hardwareMap);

        robot.launchGateSubsystem.closeGate();

        // 2. Init Loop for Alliance Selection
        while (opModeInInit()) {
            if (gamepad1.xWasReleased()) {
                alliance = AllianceColors.BLUE;
            }
            if (gamepad1.bWasReleased()) {
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
        follower.setStartingPose(poses.NEAR_INIT);

        paths = new Paths(follower, poses);
        paths.buildNearPaths(); // Only build near paths

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
                        new InstantCommand(() -> Log.i("Near12", "Starting Preload and Spikes")),

                        // 1. Run Near Preloads
                        new FollowPathCommand(follower, paths.NEAR_PRELOADS, true),
                        new WaitCommand(200),
                        new LaunchBallsCommand(robot.launchSubsystem, robot.launchGateSubsystem, true, true),

                        // 2. Run First Spike
                        new FollowPathCommand(follower, paths.NEAR_SPIKE_1, true),
                        new FollowPathCommand(follower, paths.NEAR_OPEN_GATE, true),
                        new FollowPathCommand(follower, paths.NEAR_SCORE_SPIKE_1_AFTER_GATE, true),
                        new LaunchBallsCommand(robot.launchSubsystem, robot.launchGateSubsystem),

                        // 3. Run Second Spike
                        new FollowPathCommand(follower, paths.NEAR_SPIKE_2, true),
                        new FollowPathCommand(follower, paths.NEAR_SCORE_SPIKE_2, true),
                        new LaunchBallsCommand(robot.launchSubsystem, robot.launchGateSubsystem, true),

                        //gate cycle 1
                        new FollowPathCommand(follower, paths.NEAR_GATE_1_PATH, true),
                        new ParallelRaceGroup(
                                new WaitCommand(INTAKE_WAIT),
                                new WaitUntilCommand(()-> robot.intakeSubsystem.getArtifactCount() == 3),
                                new RepeatCommand(
                                        new SequentialCommandGroup(
                                                new FollowPathCommand(follower, paths.NEAR_JIGGLE_GATE_PATH_1, false),
                                                new FollowPathCommand(follower, paths.NEAR_JIGGLE_GATE_PATH_2, false)))
                                ),
                        new FollowPathCommand(follower, paths.NEAR_SCORE_GATE, true),
                        new LaunchBallsCommand(robot.launchSubsystem, robot.launchGateSubsystem, true),

                        // Gate cycle 2
                        new FollowPathCommand(follower, paths.NEAR_GATE_1_PATH, true),
                        new ParallelRaceGroup(
                                new WaitCommand(INTAKE_WAIT),
                                new WaitUntilCommand(()-> robot.intakeSubsystem.getArtifactCount() == 3),
                                new RepeatCommand(
                                        new SequentialCommandGroup(
                                                new FollowPathCommand(follower, paths.NEAR_JIGGLE_GATE_PATH_1, false),
                                                new FollowPathCommand(follower, paths.NEAR_JIGGLE_GATE_PATH_2, false)))
                        ),
                        new FollowPathCommand(follower, paths.NEAR_SCORE_GATE, true),
                        new LaunchBallsCommand(robot.launchSubsystem, robot.launchGateSubsystem, true),

                        //gate cycle 3
                        new FollowPathCommand(follower, paths.NEAR_GATE_1_PATH, true),
                        new ParallelRaceGroup(
                                new WaitCommand(INTAKE_WAIT),
                                new WaitUntilCommand(()-> robot.intakeSubsystem.getArtifactCount() == 3),
                                new RepeatCommand(
                                        new SequentialCommandGroup(
                                                new FollowPathCommand(follower, paths.NEAR_JIGGLE_GATE_PATH_1, false),
                                                new FollowPathCommand(follower, paths.NEAR_JIGGLE_GATE_PATH_2, false)))
                        ),
                        new FollowPathCommand(follower, paths.NEAR_SCORE_GATE, true),
                        new LaunchBallsCommand(robot.launchSubsystem, robot.launchGateSubsystem, true),

                        // move away from launch area
                        new FollowPathCommand(follower, paths.NEAR_LEAVE, true),




                        //5. Run Gate cycle
//                new FollowPathCommand(follower, paths.NEAR_GATE, true),
//                new FollowPathCommand(follower, paths.NEAR_GATE_2, true),
//                new FollowPathCommand(follower, paths.NEAR_SCORE_GATE, true),


                        new InstantCommand(() -> Log.i("Near12", "Seconds " + timer.seconds()))
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