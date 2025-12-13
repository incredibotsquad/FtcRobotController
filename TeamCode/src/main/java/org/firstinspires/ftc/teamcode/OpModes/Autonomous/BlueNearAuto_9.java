package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import static org.firstinspires.ftc.teamcode.Actions.LaunchFlywheelAction.FLYWHEEL_FULL_TICKS_PER_SEC;
import static org.firstinspires.ftc.teamcode.subsystems.LaunchSystem.FLYWHEEL_POWER_COEFFICIENT_CLOSE;
import static org.firstinspires.ftc.teamcode.subsystems.LaunchSystem.FLYWHEEL_POWER_COEFFICIENT_MID;
import static org.firstinspires.ftc.teamcode.subsystems.LaunchSystem.TURRET_SERVO_CENTERED;

import android.util.Log;

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
import org.firstinspires.ftc.teamcode.Actions.ResetSpindexerAction;
import org.firstinspires.ftc.teamcode.Actions.SpindexAction;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.common.AllianceColors;
import org.firstinspires.ftc.teamcode.common.GameColors;
import org.firstinspires.ftc.teamcode.common.GamePattern;
import org.firstinspires.ftc.teamcode.common.LimelightAprilTagHelper;
import org.firstinspires.ftc.teamcode.common.CrossOpModeStorage;
import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchSystem;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;

/**
 * BLUE NEAR AUTO - 9 BALL STRATEGY
 * 
 * Starting from near position (closer to goal), this auto:
 * 1. Moves to read obelisk pattern
 * 2. Moves to close launch position and launches 3 preloaded balls
 * 3. Collects 3 balls from spike line 1 (near - X = -7)
 * 4. Returns to launch position, launches 3 balls
 * 5. Collects 3 balls from spike line 2 (middle - X = 12)
 * 6. Returns to launch position, launches 3 balls
 * 7. Parks off spike marks
 * 
 * OPTIMIZATIONS:
 * - Pre-spin flywheel during robot movement
 * - Parallel actions for spindexer rotation during transit
 * - Optimized path planning with spline curves
 * - Reduced sleep times where safe
 * - Higher velocity for return-to-launch movements
 * - Color detection during transit when possible
 * - Uses closer launch position for faster cycle times
 */
@Autonomous(name = "Blue_Near_Auto_9", group = "Autonomous")
public class BlueNearAuto_9 extends BaseAuto {

    private static final int multiplier = -1;    // Red = 1, Blue = -1
    private static final String TAG = "BLUE_NEAR_AUTO_9";

    // Headings
    public double robotHeading = Math.toRadians(135 * multiplier);
    public double goalHeading = Math.toRadians(135 * multiplier);
    public double splineHeading = Math.toRadians(0);
    public double obeliskHeading = Math.toRadians(180);
    public double artifactHeading = Math.toRadians(90 * multiplier);

    // Key Positions
    public Pose2d INIT_POS = new Pose2d(-54, 54 * multiplier, robotHeading);
    public Pose2d TAG_READ_POS = new Pose2d(-36, 15 * multiplier, obeliskHeading);
    public Pose2d LAUNCH_POS = new Pose2d(-26, 26 * multiplier, goalHeading);

    // Ball collection step size (optimized for reliable intake)
    public int FORWARD_DELTA_TO_COLLECT_BALL_STEP = 7;

    // Spike Line 1 (Near - X = -7)
    public Pose2d COLLECT_LINE_1_START = new Pose2d(-7, 38 * multiplier, artifactHeading);
    public Pose2d COLLECT_LINE_1_BALL_1 = new Pose2d(-7, (38 + FORWARD_DELTA_TO_COLLECT_BALL_STEP) * multiplier, artifactHeading);
    public Pose2d COLLECT_LINE_1_BALL_2 = new Pose2d(-7, (38 + 2 * FORWARD_DELTA_TO_COLLECT_BALL_STEP - 2) * multiplier, artifactHeading);
    public Pose2d COLLECT_LINE_1_BALL_3 = new Pose2d(-7, (38 + 3 * FORWARD_DELTA_TO_COLLECT_BALL_STEP - 4) * multiplier, artifactHeading);

    // Spike Line 2 (Middle - X = 12)
    public Pose2d COLLECT_LINE_2_START = new Pose2d(12, 38 * multiplier, artifactHeading);
    public Pose2d COLLECT_LINE_2_BALL_1 = new Pose2d(12, (38 + FORWARD_DELTA_TO_COLLECT_BALL_STEP) * multiplier, artifactHeading);
    public Pose2d COLLECT_LINE_2_BALL_2 = new Pose2d(12, (38 + 2 * FORWARD_DELTA_TO_COLLECT_BALL_STEP - 2) * multiplier, artifactHeading);
    public Pose2d COLLECT_LINE_2_BALL_3 = new Pose2d(12, (38 + 3 * FORWARD_DELTA_TO_COLLECT_BALL_STEP - 4) * multiplier, artifactHeading);

    // Park position (off spike marks)
    public Pose2d PARK_POS = new Pose2d(-48, 24 * multiplier, artifactHeading);

    // OPTIMIZATION: Velocity/acceleration constraints tuned for speed vs reliability
    private static final double FAST_VEL = 55;
    private static final double FAST_ACCEL = 55;
    private static final double INTAKE_VEL = 18;  // Slower for reliable ball pickup
    private static final double INTAKE_ACCEL = 12;
    
    // OPTIMIZATION: Reduced wait times (tuned for reliability)
    private static final double BALL_DETECT_TIMEOUT_MS = 400;
    private static final double SPINDEX_SETTLE_TIME = 0.35;

    @Override
    public void runOpMode() throws InterruptedException {

        robotHardware = new RobotHardware(this.hardwareMap);
        CrossOpModeStorage.allianceColor = AllianceColors.BLUE;

        this.limelightAprilTagHelper = new LimelightAprilTagHelper(robotHardware);
        this.spindex = new Spindex(robotHardware);
        this.spindex.initializeWithPPG();
        this.intakeSystem = new IntakeSystem(robotHardware, this.spindex);
        this.launchSystem = new LaunchSystem(robotHardware, this.spindex, this.limelightAprilTagHelper);

        mecanumDrive = new MecanumDrive(this.hardwareMap, INIT_POS);

        robotHardware.startLimelight();
        robotHardware.setLimelightPipeline(6);

        // ========== BUILD ALL TRAJECTORIES DURING INIT (OPTIMIZATION) ==========
        
        Action moveToReadTag = mecanumDrive.actionBuilder(INIT_POS)
                .strafeToLinearHeading(TAG_READ_POS.position, TAG_READ_POS.heading)
                .build();

        Action launchFromTagRead = mecanumDrive.actionBuilder(TAG_READ_POS)
                .strafeToLinearHeading(LAUNCH_POS.position, LAUNCH_POS.heading)
                .build();

        // Line 1 trajectories
        Action toLine1Start = mecanumDrive.actionBuilder(LAUNCH_POS)
                .setTangent(splineHeading)
                .splineToLinearHeading(COLLECT_LINE_1_START, COLLECT_LINE_1_START.heading)
                .build();

        Action line1Ball1 = mecanumDrive.actionBuilder(COLLECT_LINE_1_START)
                .setTangent(artifactHeading)
                .lineToY(COLLECT_LINE_1_BALL_1.position.y, 
                        new TranslationalVelConstraint(INTAKE_VEL), 
                        new ProfileAccelConstraint(-INTAKE_ACCEL, INTAKE_ACCEL))
                .build();

        Action line1Ball2 = mecanumDrive.actionBuilder(COLLECT_LINE_1_BALL_1)
                .setTangent(artifactHeading)
                .lineToY(COLLECT_LINE_1_BALL_2.position.y, 
                        new TranslationalVelConstraint(INTAKE_VEL), 
                        new ProfileAccelConstraint(-INTAKE_ACCEL, INTAKE_ACCEL))
                .build();

        Action line1Ball3 = mecanumDrive.actionBuilder(COLLECT_LINE_1_BALL_2)
                .lineToY(COLLECT_LINE_1_BALL_3.position.y, 
                        new TranslationalVelConstraint(INTAKE_VEL), 
                        new ProfileAccelConstraint(-INTAKE_ACCEL, INTAKE_ACCEL))
                .build();

        Action line1ToLaunch = mecanumDrive.actionBuilder(COLLECT_LINE_1_BALL_3)
                .strafeToLinearHeading(LAUNCH_POS.position, LAUNCH_POS.heading, 
                        new TranslationalVelConstraint(FAST_VEL), 
                        new ProfileAccelConstraint(-FAST_ACCEL, FAST_ACCEL))
                .build();

        // Line 2 trajectories
        Action launchToLine2 = mecanumDrive.actionBuilder(LAUNCH_POS)
                .setTangent(splineHeading)
                .splineToLinearHeading(COLLECT_LINE_2_START, COLLECT_LINE_2_START.heading)
                .build();

        Action line2Ball1 = mecanumDrive.actionBuilder(COLLECT_LINE_2_START)
                .setTangent(artifactHeading)
                .lineToY(COLLECT_LINE_2_BALL_1.position.y, 
                        new TranslationalVelConstraint(INTAKE_VEL), 
                        new ProfileAccelConstraint(-INTAKE_ACCEL, INTAKE_ACCEL))
                .build();

        Action line2Ball2 = mecanumDrive.actionBuilder(COLLECT_LINE_2_BALL_1)
                .setTangent(artifactHeading)
                .lineToY(COLLECT_LINE_2_BALL_2.position.y, 
                        new TranslationalVelConstraint(INTAKE_VEL), 
                        new ProfileAccelConstraint(-INTAKE_ACCEL, INTAKE_ACCEL))
                .build();

        Action line2Ball3 = mecanumDrive.actionBuilder(COLLECT_LINE_2_BALL_2)
                .lineToY(COLLECT_LINE_2_BALL_3.position.y, 
                        new TranslationalVelConstraint(INTAKE_VEL), 
                        new ProfileAccelConstraint(-INTAKE_ACCEL, INTAKE_ACCEL))
                .build();

        Action line2ToLaunch = mecanumDrive.actionBuilder(COLLECT_LINE_2_BALL_3)
                .strafeToLinearHeading(LAUNCH_POS.position, LAUNCH_POS.heading, 
                        new TranslationalVelConstraint(FAST_VEL), 
                        new ProfileAccelConstraint(-FAST_ACCEL, FAST_ACCEL))
                .build();

        Action toPark = mecanumDrive.actionBuilder(LAUNCH_POS)
                .strafeToLinearHeading(PARK_POS.position, PARK_POS.heading)
                .build();

        // ========== WAIT FOR START ==========
        while (opModeInInit()) {
            telemetry.addLine("BLUE NEAR AUTO - 9 BALL");
            telemetry.addLine("Ready to start");
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            ElapsedTime totalTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

            // ========== PHASE 1: MOVE TO READ TAG, READ PATTERN ==========
            runBlockingWithBackground(
                    new SequentialAction(
                            new ParallelAction(
                                    spindex.moveToNextFullSlotAction(),
                                    moveToReadTag,
                                    new InstantAction(() -> robotHardware.setFlywheelVelocityInTPS(FLYWHEEL_FULL_TICKS_PER_SEC * FLYWHEEL_POWER_COEFFICIENT_CLOSE)),
                                    new InstantAction(() -> robotHardware.setLaunchTurretPosition(TURRET_SERVO_CENTERED))
                            )
                    )
            );

            GamePattern pattern = launchSystem.readGamePattern();
            Log.i(TAG, "Pattern detected: " + (pattern != null ? pattern.tagId : "null"));

            // ========== PHASE 2: MOVE TO LAUNCH & LAUNCH PRELOADED ==========
            runBlockingWithBackground(
                    new ParallelAction(
                            intakeSystem.getTurnOffAction(),
                            launchFromTagRead,
                            new LaunchFlywheelAction(robotHardware, FLYWHEEL_FULL_TICKS_PER_SEC * FLYWHEEL_POWER_COEFFICIENT_CLOSE)
                    )
            );

            runBlockingWithBackground(launchSystem.getBallPatternLaunchAction(pattern));
            Log.i(TAG, "Phase 1-2 complete - 3 balls launched. Time: " + totalTimer.seconds());

            // ========== PHASE 3: COLLECT LINE 1 (3 balls) ==========
            runBlockingWithBackground(
                    new SequentialAction(
                            new ParallelAction(
                                    toLine1Start,
                                    intakeSystem.getTurnOnAction(false),
                                    new SpindexAction(robotHardware, spindex.storedColors.get(0).intakePosition),
                                    // OPTIMIZATION: Pre-spin flywheel during transit
                                    new LaunchFlywheelAction(robotHardware, FLYWHEEL_FULL_TICKS_PER_SEC * FLYWHEEL_POWER_COEFFICIENT_CLOSE, false)
                            ),
                            line1Ball1,
                            createBallDetectionAction(0),
                            new SpindexAction(robotHardware, spindex.storedColors.get(1).intakePosition),
                            new SleepAction(SPINDEX_SETTLE_TIME)
                    )
            );

            runBlockingWithBackground(
                    new SequentialAction(
                            line1Ball2,
                            createBallDetectionWithColorAction(1, 0),
                            new SpindexAction(robotHardware, spindex.storedColors.get(2).intakePosition),
                            new SleepAction(SPINDEX_SETTLE_TIME + 0.15)
                    )
            );

            runBlockingWithBackground(
                    new SequentialAction(
                            line1Ball3,
                            createBallDetectionWithColorAction(2, 1)
                    )
            );

            // Return to launch position with color detection
            runBlockingWithBackground(
                    new ParallelAction(
                            intakeSystem.getReverseIntakeAction(),
                            line1ToLaunch,
                            new SequentialAction(
                                    new SpindexAction(robotHardware, spindex.storedColors.get(0).intakePosition),
                                    new SleepAction(0.2),
                                    new InstantAction(() -> detectColorForSlot(2))
                            ),
                            new LaunchFlywheelAction(robotHardware, FLYWHEEL_FULL_TICKS_PER_SEC * FLYWHEEL_POWER_COEFFICIENT_CLOSE)
                    )
            );

            runBlockingWithBackground(launchSystem.getBallPatternLaunchAction(pattern));
            Log.i(TAG, "Phase 3 complete - 6 balls launched. Time: " + totalTimer.seconds());

            // ========== PHASE 4: COLLECT LINE 2 (3 balls) ==========
            runBlockingWithBackground(
                    new SequentialAction(
                            new ParallelAction(
                                    launchToLine2,
                                    intakeSystem.getTurnOnAction(false),
                                    new SpindexAction(robotHardware, spindex.storedColors.get(0).intakePosition),
                                    new LaunchFlywheelAction(robotHardware, FLYWHEEL_FULL_TICKS_PER_SEC * FLYWHEEL_POWER_COEFFICIENT_MID, false)
                            ),
                            line2Ball1,
                            createBallDetectionAction(0),
                            new SpindexAction(robotHardware, spindex.storedColors.get(1).intakePosition),
                            new SleepAction(SPINDEX_SETTLE_TIME)
                    )
            );

            runBlockingWithBackground(
                    new SequentialAction(
                            line2Ball2,
                            createBallDetectionWithColorAction(1, 0),
                            new SpindexAction(robotHardware, spindex.storedColors.get(2).intakePosition),
                            new SleepAction(SPINDEX_SETTLE_TIME + 0.15)
                    )
            );

            runBlockingWithBackground(
                    new SequentialAction(
                            line2Ball3,
                            createBallDetectionWithColorAction(2, 1)
                    )
            );

            // Return to launch position
            runBlockingWithBackground(
                    new ParallelAction(
                            intakeSystem.getReverseIntakeAction(),
                            line2ToLaunch,
                            new SequentialAction(
                                    new SpindexAction(robotHardware, spindex.storedColors.get(0).intakePosition),
                                    new SleepAction(0.2),
                                    new InstantAction(() -> detectColorForSlot(2))
                            ),
                            new LaunchFlywheelAction(robotHardware, FLYWHEEL_FULL_TICKS_PER_SEC * FLYWHEEL_POWER_COEFFICIENT_MID)
                    )
            );

            runBlockingWithBackground(launchSystem.getBallPatternLaunchAction(pattern));
            Log.i(TAG, "Phase 4 complete - 9 balls launched. Time: " + totalTimer.seconds());

            // ========== PHASE 5: PARK ==========
            runBlockingWithBackground(
                    new ParallelAction(
                            toPark,
                            new ResetSpindexerAction(robotHardware)
                    )
            );

            Log.i(TAG, "AUTO COMPLETE! Total time: " + totalTimer.seconds() + " seconds");
            
            break;
        }
    }

    /**
     * OPTIMIZATION: Reusable ball detection action with timeout
     */
    private Action createBallDetectionAction(int slotIndex) {
        return new InstantAction(() -> {
            ElapsedTime intakeTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            while (intakeTimer.milliseconds() < BALL_DETECT_TIMEOUT_MS 
                    && spindex.storedColors.get(slotIndex).ballColor == GameColors.NONE) {
                if (robotHardware.didBallDetectionBeamBreak()) {
                    spindex.storedColors.get(slotIndex).ballColor = GameColors.UNKNOWN;
                    Log.i(TAG, "Ball detected at slot " + slotIndex);
                    break;
                }
            }
        });
    }

    /**
     * OPTIMIZATION: Combined ball detection and color sensing for previous ball
     */
    private Action createBallDetectionWithColorAction(int currentSlot, int previousSlot) {
        return new InstantAction(() -> {
            ElapsedTime intakeTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            while (intakeTimer.milliseconds() < BALL_DETECT_TIMEOUT_MS 
                    && spindex.storedColors.get(currentSlot).ballColor == GameColors.NONE) {
                if (robotHardware.didBallDetectionBeamBreak()) {
                    spindex.storedColors.get(currentSlot).ballColor = GameColors.UNKNOWN;
                    Log.i(TAG, "Ball detected at slot " + currentSlot);
                }
                // OPTIMIZATION: Detect color of previous ball while waiting
                if (spindex.storedColors.get(previousSlot).ballColor == GameColors.UNKNOWN) {
                    GameColors color = robotHardware.getDetectedBallColorFromLeftSensor();
                    spindex.storedColors.get(previousSlot).ballColor = color;
                    Log.i(TAG, "Color detected for slot " + previousSlot + ": " + color);
                }
            }
        });
    }

    /**
     * Helper to detect color for a specific slot
     */
    private void detectColorForSlot(int slot) {
        if (spindex.storedColors.get(slot).ballColor == GameColors.UNKNOWN) {
            GameColors color = robotHardware.getDetectedBallColorFromLeftSensor();
            spindex.storedColors.get(slot).ballColor = color;
            Log.i(TAG, "Color detected for slot " + slot + ": " + color);
        }
    }
}
