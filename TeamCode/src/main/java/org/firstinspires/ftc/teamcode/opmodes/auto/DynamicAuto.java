package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.RepeatCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Incredibot;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.commands.LaunchBallsCommand;
import org.firstinspires.ftc.teamcode.common.AllianceColors;
import org.firstinspires.ftc.teamcode.common.CrossOpModeStorage;
import org.firstinspires.ftc.teamcode.common.RobotPosition;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Configurable Dynamic Auto", group = "Autonomous")
public class DynamicAuto extends CommandOpMode {
    private Incredibot robot;
    private Follower follower;
    private Poses poses;
    private Pose lastPathEndPose;

    private RobotPosition robotPosition = RobotPosition.NEAR;

    private AllianceColors alliance = AllianceColors.RED; // Default

    // The sequence the operator builds during init
    private final List<AutoPathPositions> pathSequence = new ArrayList<>();

    public static long INTAKE_DURATION_MS = 1500;
    private boolean resetTimer = false;
    private final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private enum AutoPathPositions {
        LAUNCH, SPIKE_1, SPIKE_2, SPIKE_3, GATE_OPEN_ONLY, GATE_OPEN_INTAKE, HUMAN_PLAYER, PARK, WAIT_1_SEC;

        public AutoPathPositions next() {
            return values()[(this.ordinal() + 1) % values().length];
        }

        public AutoPathPositions previous() {
            int previousOrdinal = this.ordinal() - 1;
            if (previousOrdinal < 0) {
                return values()[values().length - 1];
            }
            return values()[previousOrdinal];
        }
    }

    @Override
    public void initialize() {
        robot = new Incredibot(hardwareMap,  PanelsTelemetry.INSTANCE.getTelemetry());
        robot.initialize(Incredibot.OpModeType.AUTO, null, null);
        follower = Constants.createFollower(this.hardwareMap);

        robot.launchGateSubsystem.closeGate();

        AutoPathPositions selector = AutoPathPositions.LAUNCH;

        // --- INIT LOOP FOR OPERATOR CONFIGURATION ---
        while (opModeInInit()) {
            if(gamepad2.xWasPressed()){
                alliance = AllianceColors.BLUE;
            }

            if(gamepad2.bWasPressed()){
                alliance = AllianceColors.RED;
            }

            // DPAD UP: NEAR
            if (gamepad2.dpadUpWasPressed()) {
                robotPosition = RobotPosition.NEAR;
            }

            // DPAD DOWN: FAR
            if (gamepad2.dpadDownWasPressed()) {
                robotPosition = RobotPosition.FAR;
            }

            // A Button: Add current selector to sequence
            if (gamepad2.aWasPressed()) {
                pathSequence.add(selector);
            }

            // DPAD: Cycle through available positions
            if (gamepad2.dpadRightWasPressed()) {
                selector = selector.next();
            }

            // DPAD: Cycle through available positions
            if (gamepad2.dpadLeftWasPressed()) {
                selector = selector.previous();
            }

            // Y Button: Clear sequence
            if (gamepad2.yWasPressed()) {
                pathSequence.clear();
            }

            // Back: remove last entry
            if (gamepad2.backWasPressed()){
                if (!pathSequence.isEmpty())
                    pathSequence.remove(pathSequence.size() - 1);
            }

            telemetry.addLine("=== AUTO CONFIGURATION ===");
            telemetry.addLine("(X) for Blue, (B) for Red, DPAD UP for Near, DPAD DOWN for Far");
            telemetry.addData("Alliance Color: ", alliance);
            telemetry.addData("Robot Position: ", robotPosition);
            telemetry.addLine("-------------------------------");
            telemetry.addData("Currently Selecting", selector);
            telemetry.addLine("(DPAD L/R) to Cycle, (A) to Add, (Y) to Clear, Back to remove last");
            telemetry.addLine("-------------------------------");
            telemetry.addData("Sequence Size: ", pathSequence.size());
            telemetry.addData("Current Route: ", pathSequence.toString());
            PanelsTelemetry.INSTANCE.getTelemetry().update(telemetry);

        }

        poses = new Poses(alliance == AllianceColors.RED); // Red Alliance default

        if (robotPosition == RobotPosition.NEAR)
            lastPathEndPose = poses.NEAR_INIT;
        else
            lastPathEndPose = poses.FAR_INIT;

        follower.setStartingPose(lastPathEndPose);

        schedule(buildDynamicRoutine());
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

    /**
     * Iterates through the selected sequence and builds a combined command group.
     */
    public Command buildDynamicRoutine() {
        SequentialCommandGroup mainRoutine = new SequentialCommandGroup();

        for (AutoPathPositions pos : pathSequence) {
            switch (pos) {
                case LAUNCH:

                    Pose launchPose = robotPosition == RobotPosition.NEAR ? poses.NEAR_LAUNCH : poses.FAR_LAUNCH;

                    mainRoutine.addCommands(
                            moveTo(launchPose),
                            new LaunchBallsCommand(robot.launchSubsystem, robot.launchGateSubsystem, robotPosition == RobotPosition.NEAR, robotPosition == RobotPosition.FAR)
                    );
                    break;

                case SPIKE_1:
                    Pose spike1Pose = poses.FIRST_SPIKE;

                    if (robotPosition == RobotPosition.NEAR) {
                        mainRoutine.addCommands(moveTo(spike1Pose));
                    } else {
                        mainRoutine.addCommands(curveTo(spike1Pose, poses.FAR_SPIKE_1_CONTROL));
                    }
                    break;

                case SPIKE_2:
                    // Spike 2 uses a Bezier Curve with a control point
                    Pose spike2Control = robotPosition == RobotPosition.NEAR ? poses.NEAR_SPIKE_2_CONTROL : poses.FAR_SPIKE_2_CONTROL;
                    mainRoutine.addCommands(
                            curveTo(poses.SECOND_SPIKE, spike2Control)
                    );
                    break;

                case SPIKE_3:

                    Pose spike3Control = robotPosition == RobotPosition.NEAR ? poses.NEAR_SPIKE_3_CONTROL : poses.FAR_SPIKE_3_CONTROL;
                    mainRoutine.addCommands(
                            curveTo(poses.THIRD_SPIKE, spike3Control)
                    );
                    break;

                case GATE_OPEN_ONLY:
                        mainRoutine.addCommands(
                                curveTo(poses.OPEN_GATE, poses.OPEN_GATE_CONTROL)
                        );
                    break;

                case GATE_OPEN_INTAKE:
                    if (robotPosition == RobotPosition.NEAR) {
                        mainRoutine.addCommands(
                                curveTo(poses.ONE_PATH_GATE_INTAKE, poses.NEAR_1_PATH_GATE_CONTROL_1, poses.NEAR_1_PATH_GATE_CONTROL_2)
                        );
                    } else {
                        mainRoutine.addCommands(
                                curveTo(poses.ONE_PATH_GATE_INTAKE, poses.FAR_1_PATH_GATE_CONTROL_1, poses.FAR_1_PATH_GATE_CONTROL_2)
                        );
                    }

                    mainRoutine.addCommands(
                        new ParallelRaceGroup(
                                new WaitUntilCommand(() -> robot.intakeSubsystem.getArtifactCount() == 3),
                                new WaitCommand(INTAKE_DURATION_MS),
                                new RepeatCommand(
                                        new SequentialCommandGroup(
                                                moveTo(poses.GATE_INTAKE_JIGGLE_BACKWORD_POSE),
                                                moveTo(poses.ONE_PATH_GATE_INTAKE)
                                        )
                                )
                        )
                    );

                    break;

                case HUMAN_PLAYER:

                    if (robotPosition == RobotPosition.NEAR) {
                        mainRoutine.addCommands(curveTo(poses.HUMAN_PLAYER_POSE, poses.NEAR_HUMAN_CONTROL));
                    }
                    else {
                        mainRoutine.addCommands(moveTo(poses.HUMAN_PLAYER_POSE));
                    }

                    mainRoutine.addCommands(
                        new ParallelRaceGroup(
                                new WaitUntilCommand(()-> robot.intakeSubsystem.getArtifactCount() == 3),
                                new WaitCommand(INTAKE_DURATION_MS),
                                new RepeatCommand(
                                        new SequentialCommandGroup(
                                                moveTo(poses.HUMAN_JIGGLE_BACKWARD),
                                                moveTo(poses.HUMAN_PLAYER_POSE)
                                        )
                                )
                        )
                    );

                    break;

                case PARK:
                    Pose parkPose = robotPosition == RobotPosition.NEAR ? poses.NEAR_PARK : poses.FAR_PARK;

                    mainRoutine.addCommands(
                            moveTo(parkPose)
                    );
                    break;
                case WAIT_1_SEC:
                    mainRoutine.addCommands(
                            new WaitCommand(1000)
                    );
                    break;
            }
        }

        return mainRoutine;
    }

    // --- DYNAMIC PATH BUILDERS ---

    private Command moveTo(Pose targetPose) {
        PathChain pc = follower.pathBuilder()
                .addPath(new BezierLine(lastPathEndPose, targetPose))
                .setLinearHeadingInterpolation(lastPathEndPose.getHeading(), targetPose.getHeading())
                .build();
        lastPathEndPose = targetPose;
        return new FollowPathCommand(follower, pc, true);
    }

    private Command curveTo(Pose targetPose, Pose... controlPoints) {
        PathChain pc = follower.pathBuilder()
                .addPath(new BezierCurve(combinePoses(lastPathEndPose, targetPose, controlPoints)))
                .setLinearHeadingInterpolation(lastPathEndPose.getHeading(), targetPose.getHeading())
                .build();
        lastPathEndPose = targetPose;
        return new FollowPathCommand(follower, pc, true);
    }

    private Pose[] combinePoses(Pose start, Pose end, Pose[] controls) {
        Pose[] combined = new Pose[controls.length + 2];
        combined[0] = start;
        System.arraycopy(controls, 0, combined, 1, controls.length);
        combined[combined.length - 1] = end;
        return combined;
    }
}
