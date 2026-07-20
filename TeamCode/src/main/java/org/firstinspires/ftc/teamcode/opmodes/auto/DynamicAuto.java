package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Incredibot;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.commands.LaunchBallsCommand;
import org.firstinspires.ftc.teamcode.common.AllianceColors;
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

    public static long INTAKE_DURATION_MS = 750;

    private enum AutoPathPositions {
        LAUNCH, SPIKE_1, SPIKE_2, SPIKE_3, TUNNEL, GATE_OPEN_ONLY, GATE_OPEN_INTAKE, HUMAN_PLAYER, PARK, WAIT_1_SEC;

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
        robot = new Incredibot(hardwareMap, null);
        follower = Constants.createFollower(this.hardwareMap);

        AutoPathPositions selector = AutoPathPositions.LAUNCH;
        lastPathEndPose = poses.NEAR_INIT;

        // --- INIT LOOP FOR OPERATOR CONFIGURATION ---
        while (opModeInInit()) {
            if(gamepad1.xWasPressed()){
                alliance = AllianceColors.BLUE;
            }

            if(gamepad1.bWasPressed()){
                alliance = AllianceColors.RED;
            }

            // DPAD UP: NEAR
            if (gamepad1.dpadUpWasPressed()) {
                robotPosition = RobotPosition.NEAR;
                lastPathEndPose = poses.NEAR_INIT;
            }

            // DPAD DOWN: FAR
            if (gamepad1.dpadDownWasPressed()) {
                robotPosition = RobotPosition.FAR;
                lastPathEndPose = poses.FAR_INIT;
            }

            // A Button: Add current selector to sequence
            if (gamepad1.aWasPressed()) {
                pathSequence.add(selector);
            }

            // DPAD: Cycle through available positions
            if (gamepad1.dpadRightWasPressed()) {
                selector = selector.next();
            }

            // DPAD: Cycle through available positions
            if (gamepad1.dpadLeftWasPressed()) {
                selector = selector.previous();
            }

            // X Button: Clear sequence
            if (gamepad1.xWasPressed()) {
                pathSequence.clear();
            }

            telemetry.addLine("=== AUTO CONFIGURATION ===");
            telemetry.addData("Alliance Color: ", alliance);
            telemetry.addData("Robot Position: ", robotPosition);
            telemetry.addLine("---------------------------");
            telemetry.addData("Currently Selecting", selector);
            telemetry.addLine("Press (A) to Add, (DPAD L/R) to Cycle, (X) to Clear");
            telemetry.addLine("---------------------------");
            telemetry.addData("Sequence Size: ", pathSequence.size());
            telemetry.addData("Current Route: ", pathSequence.toString());
            telemetry.update();
        }

        poses = new Poses(alliance == AllianceColors.RED); // Red Alliance default

        follower.setStartingPose(lastPathEndPose);

        schedule(buildDynamicRoutine());
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
                            new LaunchBallsCommand(robot.launchSubsystem, robot.launchGateSubsystem, true)
                    );
                    break;

                case SPIKE_1:
                    mainRoutine.addCommands(
                            moveTo(poses.NEAR_FIRST_SPIKE)
                            // Add intaking command here if needed
                    );
                    break;

                case SPIKE_2:
                    // Spike 2 uses a Bezier Curve with a control point
                    mainRoutine.addCommands(
                            curveTo(poses.NEAR_SECOND_SPIKE, poses.NEAR_SECOND_SPIKE_CONTROL)
                    );
                    break;

                case SPIKE_3:

                    break;

                case TUNNEL:
                    break;

                case GATE_OPEN_ONLY:
                    break;

                case GATE_OPEN_INTAKE:
                    mainRoutine.addCommands(
                            curveTo(poses.NEAR_1_PATH_GATE_INTAKE, poses.NEAR_1_PATH_GATE_CONTROL_1, poses.NEAR_1_PATH_GATE_CONTROL_2),
                            new ParallelRaceGroup(
                                    //TODO: add a sequential command to move back and forth for intake
                                    new WaitUntilCommand(() -> robot.intakeSubsystem.getArtifactCount() == 3),
                                    new WaitCommand(INTAKE_DURATION_MS)
                            )
                    );
                    break;

                case HUMAN_PLAYER:
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