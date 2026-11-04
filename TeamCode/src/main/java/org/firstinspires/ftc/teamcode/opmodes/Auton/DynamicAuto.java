package org.firstinspires.ftc.teamcode.opmodes.Auton;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Incredibot;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.commands.LaunchBallsCommand;
import org.firstinspires.ftc.teamcode.common.AllianceColors;
import org.firstinspires.ftc.teamcode.common.CrossOpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.ArrayList;
import java.util.List;

/**
 * Autonomous built by the operator during INIT. Gamepad 2 picks the alliance, the start position
 * and an ordered list of steps; after START the steps run one after another.
 *
 * Poses come from {@link PathConstants}.
 *
 * The Follower belongs to DriveSubsystem, whose periodic() already calls follower.update(), so this
 * OpMode never updates it itself. The intake (SmartIntakeCommand) and the launcher
 * (LaunchReadinessCommand) run as default commands, so intake steps only drive and wait, and
 * launch steps only wait for the launcher and open the gate.
 */
@Configurable
@Autonomous(name = "Configurable Dynamic Auto", group = "Autonomous")
public class DynamicAuto extends CommandOpMode {

    /** Max path speed, as used by PathConstants. */
    public static double MAX_PATH_SPEED = 0.6;
    /** Longest time an intake step waits at a FLOWER / GARDEN for the robot to fill up. */
    public static long INTAKE_DURATION_MS = 1500;
    /** Longest time a launch step waits for the flywheel and turret before firing anyway. */
    public static long LAUNCH_READY_TIMEOUT_MS = 1500;

    /** A move shorter than this is skipped: a zero-length path has no direction to follow. */
    private static final double MIN_PATH_LENGTH_IN = 1.0;

    private enum StartPosition { RIGHT_HIVE, LEFT_HIVE }

    private enum AutoStep {
        LAUNCH_HERE, INTAKE_GARDEN, LAUNCH_LEFT_HIVE, INTAKE_FAR_FLOWER, INTAKE_CLOSE_FLOWER,
        LAUNCH_RIGHT_HIVE, PARK_LEFT_SIDE, PARK_RIGHT_SIDE, WAIT_1_SEC;

        public AutoStep next() {
            return values()[(this.ordinal() + 1) % values().length];
        }

        public AutoStep previous() {
            int previousOrdinal = this.ordinal() - 1;
            if (previousOrdinal < 0) {
                return values()[values().length - 1];
            }
            return values()[previousOrdinal];
        }
    }

    private Incredibot robot;
    private Follower follower;
    private TelemetryManager panels;

    private AllianceColors alliance = AllianceColors.RED;
    private StartPosition startPosition = StartPosition.RIGHT_HIVE;

    // The sequence the operator builds during init
    private final List<AutoStep> stepSequence = new ArrayList<>();

    private Pose lastPathEndPose;
    private boolean routineScheduled = false;

    // Alliance-adjusted copies of the PathConstants poses, filled in once the alliance is chosen.
    private Pose startRightHive, startLeftHive;
    private Pose intakeGarden, intakeFarFlower, farFlowerControl;
    private Pose intakeCloseFlower, closeFlowerControl1, closeFlowerControl2;
    private Pose launchLeftHive, launchRightHive;
    private Pose parkLeftSide, parkRightSide;

    @Override
    public void initialize() {
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        robot = new Incredibot(hardwareMap, panels);
        robot.initialize(Incredibot.OpModeType.AUTO, null, null);
        follower = robot.driveSubsystem.getFollower();

        AutoStep selector = AutoStep.LAUNCH_HERE;

        // --- INIT LOOP FOR OPERATOR CONFIGURATION ---
        // Nothing is commanded here: no motors or servos may move before START.
        while (opModeInInit()) {
            if (gamepad2.xWasPressed()) {
                alliance = AllianceColors.BLUE;
            }

            if (gamepad2.bWasPressed()) {
                alliance = AllianceColors.RED;
            }

            // DPAD UP: start at the left hive (far from the audience)
            if (gamepad2.dpadUpWasPressed()) {
                startPosition = StartPosition.LEFT_HIVE;
            }

            // DPAD DOWN: start at the right hive (audience side)
            if (gamepad2.dpadDownWasPressed()) {
                startPosition = StartPosition.RIGHT_HIVE;
            }

            // A Button: Add current selector to sequence
            if (gamepad2.aWasPressed()) {
                stepSequence.add(selector);
            }

            // DPAD: Cycle through available steps
            if (gamepad2.dpadRightWasPressed()) {
                selector = selector.next();
            }

            if (gamepad2.dpadLeftWasPressed()) {
                selector = selector.previous();
            }

            // Y Button: Clear sequence
            if (gamepad2.yWasPressed()) {
                stepSequence.clear();
            }

            // Back: remove last entry
            if (gamepad2.backWasPressed()) {
                if (!stepSequence.isEmpty())
                    stepSequence.remove(stepSequence.size() - 1);
            }

            telemetry.addLine("=== AUTO CONFIGURATION ===");
            telemetry.addLine("(X) Blue, (B) Red, DPAD UP Left Hive start, DPAD DOWN Right Hive start");
            telemetry.addData("Alliance Color: ", alliance);
            telemetry.addData("Start Position: ", startPosition);
            telemetry.addLine("-------------------------------");
            telemetry.addData("Currently Selecting", selector);
            telemetry.addLine("(DPAD L/R) to Cycle, (A) to Add, (Y) to Clear, Back to remove last");
            telemetry.addLine("-------------------------------");
            telemetry.addData("Sequence Size: ", stepSequence.size());
            telemetry.addData("Current Route: ", stepSequence.toString());
            panels.update(telemetry);
        }

        // START was pressed (or the OpMode was stopped during INIT).
        CrossOpModeStorage.allianceColor = alliance; // LaunchReadinessCommand aims using this
        buildPoses(new PathConstants(alliance));

        lastPathEndPose = startPosition == StartPosition.RIGHT_HIVE ? startRightHive : startLeftHive;
        robot.odometrySubsystem.resetPose(lastPathEndPose.x(), lastPathEndPose.y(), Math.toDegrees(lastPathEndPose.heading()));
    }

    @Override
    public void run() {
        if (!routineScheduled) {
            schedule(buildDynamicRoutine());
            routineScheduled = true;
        }
        super.run(); // runs every subsystem's periodic(), including follower.update() in DriveSubsystem
        panels.update(telemetry);
    }

    /**
     * Iterates through the selected sequence and builds a combined command group.
     */
    private Command buildDynamicRoutine() {
        SequentialCommandGroup mainRoutine = new SequentialCommandGroup();

        // The gate servo can't be commanded during INIT, so close it now. The routine requires the
        // gate (LaunchBallsCommand), which keeps CloseGateCommand from running while it is active.
        mainRoutine.addCommands(new InstantCommand(robot.launchGateSubsystem::closeGate));

        for (AutoStep step : stepSequence) {
            switch (step) {
                case LAUNCH_HERE:
                    mainRoutine.addCommands(launch());
                    break;

                case INTAKE_GARDEN:
                    mainRoutine.addCommands(
                            moveTo(intakeGarden),
                            waitForIntake()
                    );
                    break;

                case LAUNCH_LEFT_HIVE:
                    mainRoutine.addCommands(
                            moveTo(launchLeftHive),
                            launch()
                    );
                    break;

                case INTAKE_FAR_FLOWER:
                    // From the left-hive start, curve around like PathConstants.intakeFarFlowerLeftSide()
                    if (lastPathEndPose == startLeftHive) {
                        mainRoutine.addCommands(curveTo(intakeFarFlower, farFlowerControl));
                    } else {
                        mainRoutine.addCommands(moveTo(intakeFarFlower));
                    }
                    mainRoutine.addCommands(waitForIntake());
                    break;

                case INTAKE_CLOSE_FLOWER:
                    // From the far flower, curve like PathConstants.intakeCloseFlower()
                    if (lastPathEndPose == intakeFarFlower) {
                        mainRoutine.addCommands(curveTo(intakeCloseFlower, closeFlowerControl1, closeFlowerControl2));
                    } else {
                        mainRoutine.addCommands(moveTo(intakeCloseFlower));
                    }
                    mainRoutine.addCommands(waitForIntake());
                    break;

                case LAUNCH_RIGHT_HIVE:
                    mainRoutine.addCommands(
                            moveTo(launchRightHive),
                            launch()
                    );
                    break;

                case PARK_LEFT_SIDE:
                    mainRoutine.addCommands(moveTo(parkLeftSide));
                    break;

                case PARK_RIGHT_SIDE:
                    mainRoutine.addCommands(moveTo(parkRightSide));
                    break;

                case WAIT_1_SEC:
                    mainRoutine.addCommands(new WaitCommand(1000));
                    break;
            }
        }

        return mainRoutine;
    }

    // --- STEP BUILDERS ---

    /** Waits for the flywheel and turret (or the timeout), then empties the robot. */
    private Command launch() {
        return new SequentialCommandGroup(
                new ParallelRaceGroup(
                        new WaitUntilCommand(robot.launchSubsystem::isReadyToLaunch),
                        new WaitCommand(LAUNCH_READY_TIMEOUT_MS)
                ),
                new LaunchBallsCommand(robot.launchGateSubsystem, robot.intakeSubsystem, robot.driveSubsystem)
        );
    }

    /** SmartIntakeCommand runs the intake on its own; just wait until full or out of time. */
    private Command waitForIntake() {
        return new ParallelRaceGroup(
                new WaitUntilCommand(robot.intakeSubsystem::isFull),
                new WaitCommand(INTAKE_DURATION_MS)
        );
    }

    // --- DYNAMIC PATH BUILDERS ---

    private Command moveTo(Pose targetPose) {
        Pose start = lastPathEndPose;
        lastPathEndPose = targetPose;

        // e.g. LAUNCH_RIGHT_HIVE right after a right-hive start: same spot, nothing to drive
        if (Math.hypot(targetPose.x() - start.x(), targetPose.y() - start.y()) < MIN_PATH_LENGTH_IN) {
            return new InstantCommand(() -> { });
        }

        Path path = Paths.line(start, targetPose)
                .linear(start, targetPose)
                .with(Constants.foresightConfig.maxPathSpeed.at(MAX_PATH_SPEED));
        return new FollowPathCommand(follower, path, true);
    }

    private Command curveTo(Pose targetPose, Pose... controlPoints) {
        Pose start = lastPathEndPose;
        lastPathEndPose = targetPose;

        Path path = Paths.curve(combinePoses(start, targetPose, controlPoints))
                .linear(start, targetPose)
                .with(Constants.foresightConfig.maxPathSpeed.at(MAX_PATH_SPEED));
        return new FollowPathCommand(follower, path, true);
    }

    private Pose[] combinePoses(Pose start, Pose end, Pose[] controls) {
        Pose[] combined = new Pose[controls.length + 2];
        combined[0] = start;
        System.arraycopy(controls, 0, combined, 1, controls.length);
        combined[combined.length - 1] = end;
        return combined;
    }

    // --- POSES ---

    private void buildPoses(PathConstants paths) {
        startRightHive = paths.startingPoseRightHive;
        startLeftHive = paths.startingPoseLeftHive;

        intakeGarden = paths.intakeGarden;
        intakeFarFlower = paths.intakeFarFlower;
        farFlowerControl = paths.farFlowerControlPoint;
        intakeCloseFlower = paths.intakeCloseFlower;
        closeFlowerControl1 = paths.intakeCloseFlowerControl1;
        closeFlowerControl2 = paths.intakeCloseFlowerControl2;

        launchLeftHive = paths.shotOnLeftHive;
        launchRightHive = paths.shotOnRightHive;

        parkLeftSide = paths.parkPoseLeftSide;
        parkRightSide = paths.parkPoseRightSide;
    }
}
