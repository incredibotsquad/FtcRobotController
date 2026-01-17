package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import static org.firstinspires.ftc.teamcode.Actions.LaunchFlywheelAction.FLYWHEEL_FULL_TICKS_PER_SEC;
import static org.firstinspires.ftc.teamcode.subsystems.LaunchSystem.FLYWHEEL_POWER_COEFFICIENT_CLOSE;
import static org.firstinspires.ftc.teamcode.subsystems.LaunchSystem.TURRET_CENTERED_POSITION;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Actions.LaunchFlywheelAction;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.common.AllianceColors;
import org.firstinspires.ftc.teamcode.common.CrossOpModeStorage;
import org.firstinspires.ftc.teamcode.common.GamePattern;
import org.firstinspires.ftc.teamcode.common.LimelightAprilTagHelper;
import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchSystem;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;

import java.util.ArrayList;
import java.util.List;

@Config
@Autonomous(name = "Red_Near_Auto_9", group = "Autonomous")
public class RedNearAuto_9 extends BaseAuto {

    private static final int multiplier = 1;    //used to flip coordinates between red (1), Blue (-1)

    public double robotHeading = Math.toRadians(135 * multiplier);
    public double goalHeading = Math.toRadians(115 * multiplier);

    public double artifactHeading = Math.toRadians(90 * multiplier);
    public double reverseArtifactHeading = Math.toRadians(270 * multiplier);

    public double obeliskHeading = Math.toRadians(200); //this is same for both sides - do NOT need a multiplier
    public double splineHeading = Math.toRadians(0);

    public int LINE_DEPTH_SHALLOW = 25;

    public int LINE_DEPTH = 33;

    public Pose2d INIT_POS = new Pose2d(-54, 54 * multiplier, robotHeading);
    public Pose2d TAG_READ_POS = new Pose2d(-36, 15 * multiplier, obeliskHeading);

    public Pose2d LAUNCH_POS = new Pose2d(-48, 36 * multiplier, goalHeading);

    public Pose2d COLLECT_LINE_1_START = new Pose2d(-10, 38 * multiplier, artifactHeading);
    public Pose2d COLLECT_LINE_1_END = new Pose2d(COLLECT_LINE_1_START.position.x,  COLLECT_LINE_1_START.position.y + (LINE_DEPTH_SHALLOW * multiplier), artifactHeading);
    public Pose2d COLLECT_LINE_2_START = new Pose2d(16, 38 * multiplier, artifactHeading);
    public Pose2d COLLECT_LINE_2_END = new Pose2d(COLLECT_LINE_2_START.position.x,  COLLECT_LINE_2_START.position.y + (LINE_DEPTH * multiplier), artifactHeading);

    public Pose2d MOVE_OFF_LINE = new Pose2d(-48, 24  * multiplier, artifactHeading);

    private static double ALIGNMENT_WAIT_SECONDS = 1;

    // ========== TIMEOUTS (tunable via FTC Dashboard) ==========
    public static double TIMEOUT_INIT_MS = 0;      // INIT state timeout
    public static double TIMEOUT_INTAKE_MS = 6000;   // Intake states timeout (longer to collect balls)
    public static double STATE_TIMEOUT_MS = 6000;     // All other states timeout

    // ========== STATE MACHINE ==========
    private enum AutoState {
        INIT,
        DRIVE_TO_READ_SEQUENCE,
        DRIVE_TO_LAUNCH_PRELOAD,
        LAUNCH_ALL_PRELOAD,
        DRIVE_TO_INTAKE_1,
        INTAKE_BALLS_1,
        DRIVE_TO_LAUNCH_1,
        LAUNCH_ALL_1,
        DRIVE_TO_INTAKE_2,
        INTAKE_BALLS_2,
        DRIVE_TO_LAUNCH_2,
        LAUNCH_ALL_2,
        MOVE_AWAY_FROM_LINE,
        DONE
    }

    private AutoState currentState = AutoState.INIT;
    private boolean stateTransitionInProgress = false;
    private List<Action> runningActions = new ArrayList<>();
    private ElapsedTime stateTimer = new ElapsedTime();

    // ========== SUBSYSTEMS (same as TeleOp) ==========
    private FtcDashboard dashboard;

    private ElapsedTime totalTime;

    GamePattern pattern;

    @Override
    public void runOpMode() {
        // ========== INIT ==========
        robotHardware = new RobotHardware(hardwareMap);

        if (multiplier == 1) {
            CrossOpModeStorage.allianceColor = AllianceColors.RED;
        }
        else {
            CrossOpModeStorage.allianceColor = AllianceColors.BLUE;
        }

        robotHardware.startLimelight();
        robotHardware.setLimelightPipeline(CrossOpModeStorage.allianceColor);
        mecanumDrive = new MecanumDrive(hardwareMap, INIT_POS);

        spindex = new Spindex(robotHardware);
        spindex.initializeWithUnknowns();

        limelightAprilTagHelper = new LimelightAprilTagHelper(robotHardware);
        intakeSystem = new IntakeSystem(robotHardware, spindex);
        launchSystem = new LaunchSystem(robotHardware, spindex, limelightAprilTagHelper);

        dashboard = FtcDashboard.getInstance();

        // Alliance selection during init
        while (opModeInInit()) {}

        waitForStart();

        stateTimer.reset();
        currentState = AutoState.INIT;

        robotHardware.setLaunchTurretPosition(TURRET_CENTERED_POSITION);

        totalTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        // ========== MAIN LOOP (Non-Blocking) ==========
        while (opModeIsActive() && currentState != AutoState.DONE) {

            // Background tasks (same as TeleOp)
            getBackgroundTasks().run();

            // Process current state
            processState();

            //add auxiliary actions
            CheckForBallsToIntake();

            // Run actions (non-blocking)
            processActions();

            // Telemetry
            telemetry.addData("State", currentState);
            telemetry.addData("Actions Running", runningActions.size());
            telemetry.addData("Balls in Spindex", spindex.fullSlotCount());
            telemetry.update();
        }

        Log.w("RedNearAuto_9", "total time: " + totalTime.milliseconds());
        // Cleanup
        robotHardware.stopRobotAndMechanisms();
//        robotHardware.stopLimelight();
    }

    private void CheckForBallsToIntake() {
        if ((currentState == AutoState.INTAKE_BALLS_1 || currentState == AutoState.INTAKE_BALLS_2) && intakeSystem.isOn && !spindex.isFull()) {
            runningActions.add(intakeSystem.checkForBallIntakeAndGetActionTeleop());
        }
    }

    /**
     * State machine - creates actions based on current state
     */
    private void processState() {
        // Don't process new state if actions are running
        if (stateTransitionInProgress) return;

        // Timeout protection - uses per-state timeout values
        if (stateTimer.milliseconds() > getTimeoutForState(currentState)) {
            Log.w("RedNearAuto_9", "State timeout! Advancing from: " + currentState);
            runningActions.clear();
            stateTransitionInProgress = false;
            advanceToNextState();
            return;
        }

        switch (currentState) {
            case DRIVE_TO_READ_SEQUENCE:
                Log.i("RedNearAuto_9", "Starting DRIVE_TO_READ_SEQUENCE");
                runningActions.add(
                        new SequentialAction(
                                new LaunchFlywheelAction(robotHardware, FLYWHEEL_FULL_TICKS_PER_SEC * FLYWHEEL_POWER_COEFFICIENT_CLOSE, false),
                                mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose())
                                    .strafeToLinearHeading(LAUNCH_POS.position, obeliskHeading)
                                    .build(),
                                new InstantAction(() -> {
                                    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
                                    while (pattern == null && timer.milliseconds() < 20) {
                                        pattern = launchSystem.readGamePattern();   //read for 200 ms
                                        Log.i("RedNearAuto_9", "Trying to read pattern " + pattern != null? "found it" : "null");
                                    }
                                })
                        )
                );
                stateTransitionInProgress = true;
                break;
                
            case DRIVE_TO_LAUNCH_PRELOAD:
                Log.i("RedNearAuto_9", "Starting DRIVE_TO_LAUNCH_PRELOAD");
                runningActions.add(
                        new SequentialAction(
                                new LaunchFlywheelAction(robotHardware, FLYWHEEL_FULL_TICKS_PER_SEC * FLYWHEEL_POWER_COEFFICIENT_CLOSE, false),
                                mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose())
//                                        .turnTo(LAUNCH_POS.heading)
                                        .splineToLinearHeading(LAUNCH_POS, LAUNCH_POS.heading)
                                        .build()
                        ));
                stateTransitionInProgress = true;
                break;

            case LAUNCH_ALL_PRELOAD:
                Log.i("RedNearAuto_9", "Starting LAUNCH_ALL_PRELOAD");
                // Same as TeleOp LAUNCH_ALL state
                runningActions.add(
                        new SequentialAction(
                                new SleepAction(ALIGNMENT_WAIT_SECONDS),
                                launchSystem.getPerformLaunchOnAllSlots()
                        )
                );
                stateTransitionInProgress = true;
                break;

            case DRIVE_TO_INTAKE_1:
                Log.i("RedNearAuto_9", "Starting DRIVE_TO_INTAKE_1");
                runningActions.add(
                        new ParallelAction(
                                mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose())
                                        .setTangent(splineHeading)
                                        .splineToLinearHeading(COLLECT_LINE_1_START, COLLECT_LINE_1_START.heading)
                                        .build(),
                                intakeSystem.getTurnOnAction()
                        )
                );
                stateTransitionInProgress = true;
                break;

            case INTAKE_BALLS_1:
                Log.i("RedNearAuto_9", "Starting INTAKE_BALLS_1");
                // Just wait and check for balls - intake already on from previous state
                if (spindex.isFull()) {
                    advanceToNextState();
                } else {
                    // Add ball check action (same as TeleOp)
                    runningActions.add(
                            new ParallelAction(
                                    mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose())
                                            .lineToY(COLLECT_LINE_1_END.position.y, new TranslationalVelConstraint(20), new ProfileAccelConstraint(-10, 10))
                                            .build(),
                                    intakeSystem.checkForBallIntakeAndGetActionTeleop()
                            ));
                    stateTransitionInProgress = true;
                }
                break;

            case DRIVE_TO_LAUNCH_1:
                Log.i("RedNearAuto_9", "Starting DRIVE_TO_LAUNCH_1");
                runningActions.add(
                        new ParallelAction(
                                spindex.moveToNextFullSlotAction(), //move to full slot so we don't end up spitting out a ball that we took in
                                mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose())
                                        .strafeToLinearHeading(LAUNCH_POS.position, LAUNCH_POS.heading)
                                        .build(),
                                intakeSystem.getReverseIntakeAction(false),
                                intakeSystem.updateBallColorsAction(),
                                new InstantAction(() -> robotHardware.setLaunchTurretPosition(TURRET_CENTERED_POSITION))
                        )
                );
                stateTransitionInProgress = true;
                break;

            case LAUNCH_ALL_1:
                Log.i("RedNearAuto_9", "Starting LAUNCH_ALL_1");
                runningActions.add(
                        new SequentialAction(
                                new SleepAction(ALIGNMENT_WAIT_SECONDS),
                                launchSystem.getPerformLaunchOnAllSlots()
                        )
                );
                stateTransitionInProgress = true;
                break;

            case DRIVE_TO_INTAKE_2:
                Log.i("RedNearAuto_9", "Starting DRIVE_TO_INTAKE_2");
                runningActions.add(
                        new ParallelAction(
                                intakeSystem.getTurnOnAction(),
                                mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose())
                                        .setTangent(splineHeading)
                                        .splineToLinearHeading(COLLECT_LINE_2_START, COLLECT_LINE_2_START.heading)
                                        .build()
                        ));

                stateTransitionInProgress = true;
                break;
            case INTAKE_BALLS_2:
                Log.i("RedNearAuto_9", "Starting INTAKE_BALLS_2");
                if (spindex.isFull()) {
                    advanceToNextState();
                } else {
                    // Add ball check action (same as TeleOp)
                    runningActions.add(
                            new ParallelAction(
                                    mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose())
                                            .lineToY(COLLECT_LINE_2_END.position.y, new TranslationalVelConstraint(20), new ProfileAccelConstraint(-10, 10))
                                            .build(),
                                    intakeSystem.checkForBallIntakeAndGetActionTeleop()
                            ));
                    stateTransitionInProgress = true;
                }
                break;
            case DRIVE_TO_LAUNCH_2:
                Log.i("RedNearAuto_9", "Starting DRIVE_TO_LAUNCH_2");
                runningActions.add(
                        new ParallelAction(
                                spindex.moveToNextFullSlotAction(), //move to full slot so we don't end up spitting out a ball that we took in
                                mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose())
                                        .setTangent(reverseArtifactHeading)
                                        .splineToLinearHeading(LAUNCH_POS, LAUNCH_POS.heading)
                                        .build(),
                                intakeSystem.getReverseIntakeAction(false),
                                intakeSystem.updateBallColorsAction(),
                                new InstantAction(() -> robotHardware.setLaunchTurretPosition(TURRET_CENTERED_POSITION))
                        )
                );
                stateTransitionInProgress = true;
                break;

            case LAUNCH_ALL_2:
                Log.i("RedNearAuto_9", "Starting LAUNCH_ALL_2");
                runningActions.add(
                        new SequentialAction(
                                new SleepAction(ALIGNMENT_WAIT_SECONDS),
                                launchSystem.getPerformLaunchOnAllSlots()
                        )
                );
                stateTransitionInProgress = true;
                break;

            case MOVE_AWAY_FROM_LINE:
                Log.i("RedNearAuto_9", "Starting MOVE_AWAY_FROM_LINE");
                runningActions.add(
                        mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose())
                                .splineToConstantHeading(MOVE_OFF_LINE.position, MOVE_OFF_LINE.heading)
                                .build());
                stateTransitionInProgress = true;
                break;

            case DONE:
                Log.i("RedNearAuto_9", "Auto complete!");
                break;
        }
    }

    /**
     * Process actions - same pattern as TeleOp MechanismControl.ProcessActions()
     */
    private void processActions() {
        if (runningActions.isEmpty()) return;

        TelemetryPacket packet = new TelemetryPacket();
        List<Action> newActions = new ArrayList<>();

        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);  // Keep running
            }
        }

        dashboard.sendTelemetryPacket(packet);
        runningActions = newActions;

        // All actions complete - advance state
        if (runningActions.isEmpty() && stateTransitionInProgress) {
            stateTransitionInProgress = false;
            advanceToNextState();
        }
    }

    /**
     * Simple state progression
     */
    private void advanceToNextState() {
        stateTimer.reset();

        switch (currentState) {
            case INIT:
                currentState = AutoState.DRIVE_TO_LAUNCH_PRELOAD;
                break;
            case DRIVE_TO_READ_SEQUENCE:
                currentState = AutoState.DRIVE_TO_LAUNCH_PRELOAD;
                break;
            case DRIVE_TO_LAUNCH_PRELOAD:
                currentState = AutoState.LAUNCH_ALL_PRELOAD;
                break;
            case LAUNCH_ALL_PRELOAD:
                currentState = AutoState.DRIVE_TO_INTAKE_1;
                break;
            case DRIVE_TO_INTAKE_1:
                currentState = AutoState.INTAKE_BALLS_1;
                break;
            case INTAKE_BALLS_1:
                currentState = AutoState.DRIVE_TO_LAUNCH_1;
                break;
            case DRIVE_TO_LAUNCH_1:
                currentState = AutoState.LAUNCH_ALL_1;
                break;
            case LAUNCH_ALL_1:
                currentState = AutoState.DRIVE_TO_INTAKE_2;
                break;
            case DRIVE_TO_INTAKE_2:
                currentState = AutoState.INTAKE_BALLS_2;
                break;
            case INTAKE_BALLS_2:
                currentState = AutoState.DRIVE_TO_LAUNCH_2;
                break;
            case DRIVE_TO_LAUNCH_2:
                currentState = AutoState.LAUNCH_ALL_2;
                break;
            case LAUNCH_ALL_2:
                currentState = AutoState.MOVE_AWAY_FROM_LINE;
                break;
            default:
                currentState = AutoState.DONE;
        }

        Log.i("RedNearAuto_9", "Advanced to state: " + currentState);
    }

    private double getTimeoutForState(AutoState state) {
        switch (state) {
            case INIT:
                return TIMEOUT_INIT_MS;
            case INTAKE_BALLS_1:
            case INTAKE_BALLS_2:
                // Add more intake states here if needed (e.g., INTAKE_BALLS_2)
                return TIMEOUT_INTAKE_MS;
            default:
                return STATE_TIMEOUT_MS;
        }
    }
}
