package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Actions.LaunchFlywheelAction;
import org.firstinspires.ftc.teamcode.Actions.LaunchKickAction;
import org.firstinspires.ftc.teamcode.Actions.LaunchVisorAction;
import org.firstinspires.ftc.teamcode.Actions.SpindexAction;
import org.firstinspires.ftc.teamcode.common.AllianceColors;
import org.firstinspires.ftc.teamcode.common.BallEntry;
import org.firstinspires.ftc.teamcode.common.CrossOpModeStorage;
import org.firstinspires.ftc.teamcode.common.GameColors;
import org.firstinspires.ftc.teamcode.common.GamePattern;
import org.firstinspires.ftc.teamcode.common.LimelightLaunchParameters;
import org.firstinspires.ftc.teamcode.common.LimelightAprilTagHelper;
import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.common.BallLaunchParameters;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

@Config
public class LaunchSystem {

    private RobotHardware robotHardware;
    private  Spindex spindex;

    private LimelightAprilTagHelper limelightAprilTagHelper;
    private AllianceColors allianceColor;
    private ElapsedTime turretAlignmentThrottleTimer;
    private ElapsedTime turretTagNotFoundTimer;
    public static int TURRET_VELOCITY = 4000;
    public static int TURRET_VELOCITY_FINE = 1500;  // Slower velocity for fine adjustments

    public static int TURRET_CENTERED_POSITION = 0;
    public static double TURRET_ALIGNMENT_THROTTLE_MILLIS = 20;
    public static double TURRET_ALIGNMENT_TOLERANCE_DEGREES_NEAR = 2;
    public static double TURRET_ALIGNMENT_TOLERANCE_DEGREES_FAR = 2;
    public static double TURRET_COARSE_TOLERANCE_DEGREES = 3;
    public static double TURRET_FRACTION_OF_DIFFERENCE_TO_COVER = 0.9;
    public static double TURRET_TAG_NOT_FOUND_TIMER_MILLIS_TELEOP = 5000;
    public static double TURRET_TAG_NOT_FOUND_TIMER_MILLIS_AUTO = 500;

    public static double TURRET_POWER_KP = 0.05;
    public static double TURRET_POWER_MIN = 0.12;
    public static double TURRET_POWER_MAX = 0.9;
    public static boolean TURRET_POWER_SOFT_LIMITS_ENABLED = true;
    public static double TURRET_PIVOT_OFFSET_X = -12;
    public static double TURRET_PIVOT_OFFSET_Y = 0;

    public static double GOAL_BLUE_X = -64;
    public static double GOAL_BLUE_Y = -64;
    public static double GOAL_RED_X = -64;
    public static double GOAL_RED_Y = 64;

    public static double TURRET_PULSES_PER_REV = 751.8; //PPR at the output shaft depending on the motor that is used.

    public static double TURRET_ANGLE_PER_PULLEY_ROTATION = 360.0 / (123.0 / 24.0);
    public static double TURRET_DEGREES_PER_TICK = TURRET_ANGLE_PER_PULLEY_ROTATION / TURRET_PULSES_PER_REV;
    public static int TURRET_MAX_TICKS_BEFORE_CLAMP = (int) (90.0 / TURRET_DEGREES_PER_TICK);


    // ========== NEW ALIGNMENT SYSTEM PARAMETERS ==========
    // Thresholds for alignment state machine
    public static double TURRET_LOCK_THRESHOLD_DEGREES = 1.5;      // Below this = locked (aligned)
    public static double TURRET_UNLOCK_THRESHOLD_DEGREES = 3.0;    // Above this = needs realignment (hysteresis)
    
    // Robot movement thresholds to trigger turret realignment
    public static double ROBOT_POSITION_CHANGE_THRESHOLD_INCHES = 2.0;   // Robot position change to unlock
    public static double ROBOT_HEADING_CHANGE_THRESHOLD_DEGREES = 3.0;   // Robot heading change to unlock
    
    // Limelight fine adjustment parameters
    public static double LIMELIGHT_FINE_ADJUST_MAX_DEGREES = 8.0;    // Only use limelight below this error
    public static double LIMELIGHT_MIN_ADJUST_THRESHOLD_DEGREES = 0.5; // Ignore limelight below this (noise)
    public static double LIMELIGHT_WEIGHT = 0.7;  // How much to trust limelight vs odometry (0-1)
    
    // Stability parameters
    public static int ALIGNMENT_STABLE_CYCLES_REQUIRED = 3;  // Cycles aligned before locking
    public static double TURRET_DEADBAND_DEGREES = 0.5;  // Don't move if error is below this

    // ========== POSITION-ONLY ALIGNMENT PARAMETERS ==========
    // Tighter tolerances for position-only mode (no limelight fallback)
    public static double POS_ONLY_LOCK_THRESHOLD_DEGREES = 0.8;      // Tighter than limelight mode
    public static double POS_ONLY_UNLOCK_THRESHOLD_DEGREES = 2.0;    // Hysteresis
    public static double POS_ONLY_DEADBAND_DEGREES = 0.3;            // Smaller deadband for precision
    public static int POS_ONLY_STABLE_CYCLES_REQUIRED = 5;           // More cycles for stability
    public static double POS_ONLY_MOVEMENT_THRESHOLD_INCHES = 1.0;   // More sensitive to movement
    public static double POS_ONLY_HEADING_THRESHOLD_DEGREES = 1.5;   // More sensitive to rotation
    public static int POS_ONLY_TURRET_VELOCITY_COARSE = 3000;        // Slightly slower for accuracy
    public static int POS_ONLY_TURRET_VELOCITY_FINE = 800;           // Very slow for fine adjustment
    public static int POS_ONLY_TURRET_VELOCITY_ULTRA_FINE = 400;     // Ultra slow for final approach
    public static double POS_ONLY_FRACTION_COARSE = 1.0;             // Full movement for coarse
    public static double POS_ONLY_FRACTION_FINE = 0.8;               // 80% for fine
    public static double POS_ONLY_FRACTION_ULTRA_FINE = 0.5;         // 50% for ultra fine
    public static boolean POS_ONLY_USE_PIVOT_OFFSET = true;          // Account for turret pivot offset
    public static double POS_ONLY_ERROR_FILTER_ALPHA = 0.3;          // Low-pass filter (0-1, lower = more filtering)
    
    // Position-only state machine
    private enum PositionOnlyAlignmentState {
        COARSE,         // Large error, fast movement
        FINE,           // Medium error, slow movement  
        ULTRA_FINE,     // Small error, very slow movement
        LOCKED          // Aligned and stable
    }
    
    private PositionOnlyAlignmentState posOnlyState = PositionOnlyAlignmentState.COARSE;
    private Pose2d posOnlyLastLockedPose = null;
    private int posOnlyLastLockedTurretPosition = 0;
    private int posOnlyStableCounter = 0;
    private double posOnlyFilteredError = 0;  // Low-pass filtered error for stability
    private double posOnlyLastError = 0;      // For derivative term
    private double posOnlyErrorIntegral = 0;  // For integral term
    // ========== END POSITION-ONLY PARAMETERS ==========


    // ========== LIMELIGHT-ONLY ALIGNMENT PARAMETERS ==========
    // This mode uses ONLY limelight for alignment - no odometry
    // If tag is not visible for TAG_LOST_TIMEOUT, centers turret
    public static double LL_ONLY_TAG_LOST_TIMEOUT_MILLIS = 2000;     // 2 seconds before centering
    public static double LL_ONLY_LOCK_THRESHOLD_DEGREES = 1.5;       // Below this = locked
    public static double LL_ONLY_UNLOCK_THRESHOLD_DEGREES = 3.0;     // Above this = realign
    public static double LL_ONLY_DEADBAND_DEGREES = 0.5;             // Ignore error below this
    public static int LL_ONLY_STABLE_CYCLES_REQUIRED = 3;            // Cycles before locking
    public static int LL_ONLY_TURRET_VELOCITY_COARSE = 3500;         // Fast for large errors
    public static int LL_ONLY_TURRET_VELOCITY_FINE = 1200;           // Slow for small errors
    public static double LL_ONLY_COARSE_THRESHOLD_DEGREES = 8.0;     // Above this = coarse mode
    public static double LL_ONLY_FRACTION_COARSE = 0.9;              // Movement fraction for coarse
    public static double LL_ONLY_FRACTION_FINE = 0.7;                // Movement fraction for fine
    public static double LL_ONLY_ERROR_FILTER_ALPHA = 0.4;           // Low-pass filter (0-1)
    public static double LL_ONLY_MOVEMENT_THRESHOLD_INCHES = 2.0;    // Robot movement to unlock (uses last limelight pose)
    public static double LL_ONLY_HEADING_THRESHOLD_DEGREES = 3.0;    // Robot heading change to unlock

    // Limelight-only state machine
    private enum LimelightOnlyAlignmentState {
        SEARCHING,      // Tag not visible, centering turret
        COARSE,         // Large error, fast movement
        FINE,           // Small error, slow movement
        LOCKED          // Aligned and stable
    }

    private LimelightOnlyAlignmentState llOnlyState = LimelightOnlyAlignmentState.SEARCHING;
    private ElapsedTime llOnlyTagLostTimer = null;
    private int llOnlyLastLockedTurretPosition = 0;
    private int llOnlyStableCounter = 0;
    private double llOnlyFilteredError = 0;
    private Pose3D llOnlyLastKnownPose = null;  // Last known robot pose from limelight (for movement detection)
    // ========== END LIMELIGHT-ONLY PARAMETERS ==========


    // Alignment state tracking
    private enum TurretAlignmentState {
        COARSE_ALIGNING,    // Large error, using odometry only
        FINE_ALIGNING,      // Small error, can use limelight for fine tuning
        LOCKED              // Aligned and stable, don't move unless robot moves
    }
    
    private TurretAlignmentState currentAlignmentState = TurretAlignmentState.COARSE_ALIGNING;
    private Pose2d lastLockedPose = null;           // Robot pose when turret was locked
    private int lastLockedTurretPosition = 0;       // Turret position when locked
    private int alignmentStableCounter = 0;         // Counts cycles at low error before locking
    private double lastOdometryTargetDegrees = 0;   // Last calculated target from odometry
    // ========== END NEW ALIGNMENT SYSTEM PARAMETERS ==========
    public static double ROBOT_NOT_ALIGNED_TO_SHOOT_LIGHT = 0.3;    //RED
    public static double ROBOT_ALIGNED_TO_SHOOT_LIGHT = 0.5;    //GREEN


    public static double FLYWHEEL_WARM_THROTTLE_MILLIS = 50;
    public static double FLYWHEEL_POWER_BUCKET_THRESHOLD_CLOSE = 0;
    public static double FLYWHEEL_POWER_BUCKET_THRESHOLD_MID = 60;
    public static double FLYWHEEL_POWER_BUCKET_THRESHOLD_FAR = 100;

    public static double FLYWHEEL_POWER_COEFFICIENT_CLOSE = 0.41;
    public static double FLYWHEEL_POWER_COEFFICIENT_MID = 0.45;
    public static double FLYWHEEL_POWER_COEFFICIENT_FAR = 0.555;

    public static double DEFAULT_FLYWHEEL_POWER_COEFFICIENT = FLYWHEEL_POWER_COEFFICIENT_MID;

    public static double VISOR_POSITION_CLOSE_1 = 0.01;
    public static double VISOR_POSITION_CLOSE_2 = 0.01;
    public static double VISOR_POSITION_CLOSE_3 = 0.01;

    public static double VISOR_POSITION_MID_1 = 0.35;
    public static double VISOR_POSITION_MID_2 = 0.3;
    public static double VISOR_POSITION_MID_3 = 0.3;

    public static double VISOR_POSITION_FAR_1 = 0.71;
    public static double VISOR_POSITION_FAR_2 = 0.71;
    public static double VISOR_POSITION_FAR_3 = 0.71;

    BallLaunchParameters oldLaunchParameters;
    Map<Double, BallLaunchParameters> distancePowerVisorMap = Map.of(
            FLYWHEEL_POWER_BUCKET_THRESHOLD_CLOSE, new BallLaunchParameters(
                    FLYWHEEL_POWER_BUCKET_THRESHOLD_CLOSE,
                    LaunchFlywheelAction.FLYWHEEL_FULL_TICKS_PER_SEC * FLYWHEEL_POWER_COEFFICIENT_CLOSE,
                    VISOR_POSITION_CLOSE_1,
                    VISOR_POSITION_CLOSE_2,
                    VISOR_POSITION_CLOSE_3),
            FLYWHEEL_POWER_BUCKET_THRESHOLD_MID, new BallLaunchParameters(
                    FLYWHEEL_POWER_BUCKET_THRESHOLD_MID,
                    LaunchFlywheelAction.FLYWHEEL_FULL_TICKS_PER_SEC * FLYWHEEL_POWER_COEFFICIENT_MID,
                    VISOR_POSITION_MID_1,
                    VISOR_POSITION_MID_2,
                    VISOR_POSITION_MID_3),
            FLYWHEEL_POWER_BUCKET_THRESHOLD_FAR, new BallLaunchParameters(
                    FLYWHEEL_POWER_BUCKET_THRESHOLD_FAR,
                    LaunchFlywheelAction.FLYWHEEL_FULL_TICKS_PER_SEC * FLYWHEEL_POWER_COEFFICIENT_FAR,
                    VISOR_POSITION_FAR_1,
                    VISOR_POSITION_FAR_2,
                    VISOR_POSITION_FAR_3)
    );

    public LaunchSystem(RobotHardware robotHardware, Spindex spindex, LimelightAprilTagHelper limelightAprilTagHelper) {
        this.robotHardware = robotHardware;
        this.spindex = spindex;
        this.limelightAprilTagHelper = limelightAprilTagHelper;
        this.turretAlignmentThrottleTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        this.turretTagNotFoundTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        this.allianceColor = CrossOpModeStorage.allianceColor;

        this.oldLaunchParameters = distancePowerVisorMap.get(FLYWHEEL_POWER_BUCKET_THRESHOLD_MID);
    }

    //make default constructor private
    private LaunchSystem() {}

    public Action getKeepWarmAction() {
        Log.i("== LAUNCH SYSTEM ==", "Keep warm");
        return new ParallelAction(
                new LaunchFlywheelAction(robotHardware, getRobotLaunchParametersBasedOnDistance().flywheelVelocity, false)
        );
    }

    public Action getTurnOffAction() {
        Log.i("== LAUNCH SYSTEM ==", "Turned Off");
        return new LaunchFlywheelAction(robotHardware, 0, false);
    }

    private Action getLaunchBallAction(BallLaunchParameters ballLaunchParameters) {
        Log.i("== LAUNCH SYSTEM ==", "getLaunchBallAction");
        return new SequentialAction(
                new ParallelAction(
                        new LaunchVisorAction(robotHardware, ballLaunchParameters.visorPositions.get(0)),
                        new LaunchFlywheelAction(robotHardware, ballLaunchParameters.flywheelVelocity)
                ),
                new InstantAction(() -> Log.i("== LAUNCH SYSTEM ==", "RPM Before kick:" + robotHardware.getFlywheelVelocityInTPS())),
                new LaunchKickAction(robotHardware),
                new LaunchVisorAction(robotHardware, VISOR_POSITION_CLOSE_1, false)
        );
    }

    public Action getLaunchNextBallAction() {
        Log.i("== LAUNCH SYSTEM ==", "Launch Next Ball Action");
        return getLaunchNextBallAction(getRobotLaunchParametersBasedOnDistance());
    }

    public Action getLaunchNextBallAction(BallLaunchParameters ballLaunchParameters) {
        Log.i("== LAUNCH SYSTEM ==", "Launch Next Ball Action");

        return spindex.isEmpty() ? new NullAction() :
                new SequentialAction(
                spindex.moveToNextFullSlotAction(),
                getLaunchBallAction(ballLaunchParameters),
                new InstantAction(() -> spindex.clearBallAtCurrentIndex())
        );
    }

    public Action getLaunchNextBallCloseAction() {
        Log.i("== LAUNCH SYSTEM ==", "Launch Next Ball Close Action");
        return getLaunchNextBallAction(distancePowerVisorMap.get(FLYWHEEL_POWER_BUCKET_THRESHOLD_CLOSE));
    }

    public Action getLaunchNextBallMidAction() {
        Log.i("== LAUNCH SYSTEM ==", "Launch Next Ball Mid Action");
        return getLaunchNextBallAction(distancePowerVisorMap.get(FLYWHEEL_POWER_BUCKET_THRESHOLD_MID));
    }

    public Action getLaunchNextBallFarAction() {
        Log.i("== LAUNCH SYSTEM ==", "Launch Next Ball Far Action");
        return getLaunchNextBallAction(distancePowerVisorMap.get(FLYWHEEL_POWER_BUCKET_THRESHOLD_FAR));
    }

    public Action getLaunchGreenBallAction() {
        Log.i("== LAUNCH SYSTEM ==", "Launch Green Ball Action");
        return getLaunchGreenBallAction(getRobotLaunchParametersBasedOnDistance());
    }

    public Action getLaunchGreenBallAction(BallLaunchParameters ballLaunchParameters) {
        Log.i("== LAUNCH SYSTEM ==", "Launch Green Ball Action");

        //this will be null if there is no known green slot
        Action spindexAction = spindex.moveToNextGreenSlotAction();
        if (spindexAction.getClass() == NullAction.class)
            return new NullAction();

        return  new SequentialAction(
                spindexAction,
                getLaunchBallAction(ballLaunchParameters),
                new InstantAction(() -> spindex.clearBallAtCurrentIndex())
        );
    }

    public Action getLaunchPurpleBallAction() {
        Log.i("== LAUNCH SYSTEM ==", "Launch Purple Ball Action");
        return getLaunchPurpleBallAction(getRobotLaunchParametersBasedOnDistance());
    }

    public Action getLaunchPurpleBallAction(BallLaunchParameters ballLaunchParameters) {
        Log.i("== LAUNCH SYSTEM ==", "Launch Purple Ball Action");

        //this will be null if there is no known purple slot
        Action spindexAction = spindex.moveToNextPurpleSlotAction();
        if (spindexAction.getClass() == NullAction.class)
            return new NullAction();

        return new SequentialAction(
                spindexAction,
                getLaunchBallAction(ballLaunchParameters),
                new InstantAction(() -> spindex.clearBallAtCurrentIndex())
        );
    }

    public GamePattern readGamePattern() {

        GamePattern pattern = limelightAprilTagHelper.getGamePatternFromObelisk();

        if (pattern == null) {
            Log.i("== LAUNCH SYSTEM ==", "readGamePattern. found null ");
        }
        else {
            Log.i("== LAUNCH SYSTEM ==", "readGamePattern. found " + pattern.tagId);
        }

        return pattern;
    }

    public Action getBallPatternLaunchAction(GamePattern pattern) {
        Log.i("== LAUNCH SYSTEM ==", "getPatternBallLaunchAction");

        if (pattern == null) return getPerformLaunchOnAllSlots();

        Log.i("== LAUNCH SYSTEM ==", "getPatternBallLaunchAction. Detected ID: " + pattern.tagId);

        if (pattern.tagId == 21) {
            return getLaunchGPPAction();
        }

        if (pattern.tagId == 22) {
            return getLaunchPGPAction();
        }

        if (pattern.tagId == 23) {
            return getLaunchPPGAction();
        }

        return getPerformLaunchOnAllSlots();
    }

    public Action getLaunchGPPAction() {
        List<BallEntry> greenSlots = spindex.storedColors.stream().filter(entry -> entry.ballColor == GameColors.GREEN).collect(Collectors.toList());
        List<BallEntry> purpleSlots = spindex.storedColors.stream().filter(entry -> entry.ballColor == GameColors.PURPLE).collect(Collectors.toList());

        if ((greenSlots.isEmpty()) || purpleSlots.size() < 2)
            return getPerformLaunchOnAllSlots();

        return getLaunchAllBallsInSequenceAction(List.of(greenSlots.get(0).index, purpleSlots.get(0).index, purpleSlots.get(1).index));
    }

    public Action getLaunchPGPAction() {

        List<BallEntry> greenSlots = spindex.storedColors.stream().filter(entry -> entry.ballColor == GameColors.GREEN).collect(Collectors.toList());
        List<BallEntry> purpleSlots = spindex.storedColors.stream().filter(entry -> entry.ballColor == GameColors.PURPLE).collect(Collectors.toList());

        Log.i("== LAUNCH SYSTEM ==", "getLaunchPGPAction");

        if ((greenSlots.isEmpty()) || purpleSlots.size() < 2)
            return getPerformLaunchOnAllSlots();


        Log.i("== LAUNCH SYSTEM ==", "getLaunchPGPAction. Green slot: " + greenSlots.get(0).index);
        Log.i("== LAUNCH SYSTEM ==", "getLaunchPGPAction. Purple slot 1: " + purpleSlots.get(0).index);
        Log.i("== LAUNCH SYSTEM ==", "getLaunchPGPAction. Purple slot 2: " + purpleSlots.get(1).index);

        return getLaunchAllBallsInSequenceAction(List.of(purpleSlots.get(0).index, greenSlots.get(0).index, purpleSlots.get(1).index));
    }

    public Action getLaunchPPGAction() {

        List<BallEntry> greenSlots = spindex.storedColors.stream().filter(entry -> entry.ballColor == GameColors.GREEN).collect(Collectors.toList());
        List<BallEntry> purpleSlots = spindex.storedColors.stream().filter(entry -> entry.ballColor == GameColors.PURPLE).collect(Collectors.toList());

        if ((greenSlots.isEmpty()) || purpleSlots.size() < 2)
            return getPerformLaunchOnAllSlots();

        return getLaunchAllBallsInSequenceAction(List.of(purpleSlots.get(0).index, purpleSlots.get(1).index, greenSlots.get(0).index));
    }

    private Action getLaunchAllBallsInSequenceAction(List<Integer> sequence) {
        Log.i("== LAUNCH SYSTEM ==", "getLaunchAllBallsInSequenceAction ");

        if (sequence.isEmpty())
            return new NullAction();

        BallLaunchParameters ballLaunchParameters = getRobotLaunchParametersBasedOnDistance();

        //we want to wait for flywheel to ramp up only when launching from far
        boolean waitForFlywheelBetweenLaunches = (ballLaunchParameters.distanceThreshold == FLYWHEEL_POWER_BUCKET_THRESHOLD_FAR);
        if (waitForFlywheelBetweenLaunches)
            Log.i("== LAUNCH SYSTEM ==", "getLaunchAllBallsInSequenceAction. Will wait for flywheel between launches: ");

        List<Action> actionsToRun = new ArrayList<>();

        //flywheel and spindex for first one can be kicked off without waiting for the first time
        actionsToRun.add(
                new ParallelAction(
                        waitForFlywheelBetweenLaunches ? new LaunchFlywheelAction(robotHardware, ballLaunchParameters.flywheelVelocity) : new NullAction(),
                        new SpindexAction(robotHardware, spindex.storedColors.get(sequence.get(0)).launchPosition)
                )
        );

        for (int index = 0; index < sequence.size(); index++) {
            BallEntry entry = spindex.storedColors.get(sequence.get(index));
            Log.i("== LAUNCH SYSTEM ==", "getLaunchAllBallsInSequenceAction. entry: " + index + " Color: " + entry.ballColor);

            actionsToRun.add(new SequentialAction(
                    new ParallelAction(
                            new LaunchVisorAction(robotHardware, ballLaunchParameters.visorPositions.get(index)),
                            new SpindexAction(robotHardware, entry.launchPosition),
                            waitForFlywheelBetweenLaunches ? new LaunchFlywheelAction(robotHardware, ballLaunchParameters.flywheelVelocity) : new NullAction()),
                        new InstantAction(() -> Log.i("== LAUNCH SYSTEM ==", "RPM Before kick:" + robotHardware.getFlywheelVelocityInTPS())),
                    new LaunchKickAction(robotHardware),
                    new InstantAction(() -> spindex.clearBallAtIndex(entry.index))
            ));
        }

        //bring the visor back
        actionsToRun.add(new LaunchVisorAction(robotHardware, VISOR_POSITION_CLOSE_1, false));

        return new SequentialAction(actionsToRun);
    }

    public Action getPerformLaunchOnAllSlots() {
        Log.i("== LAUNCH SYSTEM ==", "getPerformLaunchOnAllSlots Action");

        return getLaunchAllBallsInSequenceAction(List.of(0, 2, 1));     //this order is the most efficient
    }

    private BallLaunchParameters getRobotLaunchParametersBasedOnDistance() {
        LimelightLaunchParameters ydt = limelightAprilTagHelper.getGoalYawDistanceToleranceFromCurrentPosition();

        BallLaunchParameters launchParameters = oldLaunchParameters;

        if (ydt != null) {
//            Log.i("== LAUNCH SYSTEM ==", "getRobotLaunchParametersBasedOnDistance: YAW: " + ydt.yaw);
            Log.i("== LAUNCH SYSTEM ==", "getRobotLaunchParametersBasedOnDistance: DISTANCE: " + ydt.distance);

            if (ydt.distance < FLYWHEEL_POWER_BUCKET_THRESHOLD_MID) {
                launchParameters = new BallLaunchParameters(
                        FLYWHEEL_POWER_BUCKET_THRESHOLD_CLOSE,
                        LaunchFlywheelAction.FLYWHEEL_FULL_TICKS_PER_SEC * FLYWHEEL_POWER_COEFFICIENT_CLOSE,
                        VISOR_POSITION_CLOSE_1,
                        VISOR_POSITION_CLOSE_2,
                        VISOR_POSITION_CLOSE_3);

            } else if (ydt.distance < FLYWHEEL_POWER_BUCKET_THRESHOLD_FAR) {

                launchParameters = new BallLaunchParameters(
                        FLYWHEEL_POWER_BUCKET_THRESHOLD_MID,
                        LaunchFlywheelAction.FLYWHEEL_FULL_TICKS_PER_SEC * FLYWHEEL_POWER_COEFFICIENT_MID,
                        VISOR_POSITION_MID_1,
                        VISOR_POSITION_MID_2,
                        VISOR_POSITION_MID_3);

            } else {

                launchParameters = new BallLaunchParameters(
                        FLYWHEEL_POWER_BUCKET_THRESHOLD_FAR,
                        LaunchFlywheelAction.FLYWHEEL_FULL_TICKS_PER_SEC * FLYWHEEL_POWER_COEFFICIENT_FAR,
                        VISOR_POSITION_FAR_1,
                        VISOR_POSITION_FAR_2,
                        VISOR_POSITION_FAR_3);
            }
        }

        oldLaunchParameters = launchParameters;     //update the older parameters

//        Log.i("== LAUNCH SYSTEM ==", "getRobotLaunchParametersBasedOnDistance: FLYWHEEL POWER: " + launchParameters.flywheelVelocity / LaunchFlywheelAction.FLYWHEEL_FULL_TICKS_PER_SEC);
//        Log.i("== LAUNCH SYSTEM ==", "getRobotLaunchParametersBasedOnDistance: VISOR: " + launchParameters.visorPositions.get(0));

        return launchParameters;
    }

    public void KeepLauncherWarm() {
//        if (flywheelWarmerThrottleTimer.milliseconds() < FLYWHEEL_WARM_THROTTLE_MILLIS)
//            return;
//
//        flywheelWarmerThrottleTimer.reset();
//
//        //call to set if the velocity is more than 10% off
        double currentVelocity = robotHardware.getFlywheelVelocityInTPS();
        double targetVelocity = getRobotLaunchParametersBasedOnDistance().flywheelVelocity;
//
//        if (Math.abs(targetVelocity - currentVelocity) > LaunchFlywheelAction.FLYWHEEL_TARGET_VELOCITY_TOLERANCE_TPS) {

//            Log.i("LAUNCH SYSTEM", "WARMING UP FLYWHEEL: CURRENT VELOCITY: " + currentVelocity + " TARGET VELOCITY: " + targetVelocity);
            robotHardware.setFlywheelVelocityInTPS(targetVelocity);
//        }
    }


    public void AlignTurretToGoalTry() {

        if (turretAlignmentThrottleTimer.milliseconds() < TURRET_ALIGNMENT_THROTTLE_MILLIS) {
            return;
        }
        turretAlignmentThrottleTimer.reset();

//                Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal : Tag NOT found");

        double goalX = (allianceColor == AllianceColors.RED) ? GOAL_RED_X : GOAL_BLUE_X;
        double goalY = (allianceColor == AllianceColors.RED) ? GOAL_RED_Y : GOAL_BLUE_Y;

//                Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal : Goal X:" + goalX + " Goal Y: " + goalY);

        double currentTurretRadians = Math.toRadians(robotHardware.getLaunchTurretPosition() * TURRET_DEGREES_PER_TICK);
        double desiredTurretDeg = computeTurretDegreesToFaceGoal(CrossOpModeStorage.currentPose, goalX, goalY, currentTurretRadians);

//                Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal : currentTurretDeg:" + Math.toDegrees(currentTurretRadians) + " desiredTurretDeg: " + desiredTurretDeg);

        double errorDeg = normalizeToTurretRange(desiredTurretDeg, TURRET_DEGREES_PER_TICK, TURRET_MAX_TICKS_BEFORE_CLAMP);

        if (Math.abs(errorDeg) <= TURRET_COARSE_TOLERANCE_DEGREES) {
//                    Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal : tag not found error in tolerance range - stopping turret");
            robotHardware.setLaunchTurretPower(0);
            robotHardware.setAlignmentLightColor(ROBOT_ALIGNED_TO_SHOOT_LIGHT);    //turn off the alignment light
            return;
        }

        robotHardware.setAlignmentLightColor(ROBOT_NOT_ALIGNED_TO_SHOOT_LIGHT);    //turn off the alignment light

        double power = Math.max(-TURRET_POWER_MAX, Math.min(TURRET_POWER_MAX, TURRET_POWER_KP * errorDeg));
        if (Math.abs(power) < TURRET_POWER_MIN) {
            power = Math.copySign(TURRET_POWER_MIN, power);
        }

        if (TURRET_POWER_SOFT_LIMITS_ENABLED) {
            int currentPos = robotHardware.getLaunchTurretPosition();
            if ((power > 0 && currentPos >= TURRET_MAX_TICKS_BEFORE_CLAMP) || (power < 0 && currentPos <= -TURRET_MAX_TICKS_BEFORE_CLAMP)) {
                power = 0;
            }
        }

//        Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal : setLaunchTurretPower:" + power);

        robotHardware.setLaunchTurretPower(power);
    }
//
//    public void AlignTurretToGoal(boolean inAutonomous) {
//        if (turretAlignmentThrottleTimer.milliseconds() < TURRET_ALIGNMENT_THROTTLE_MILLIS) {
//            return;
//        }
//        turretAlignmentThrottleTimer.reset();
//
//        double turretAnglePerPulleyRotation = 360.0 / (123.0 / 24.0);
//        double degreesPerTick = turretAnglePerPulleyRotation / TURRET_PULSES_PER_REV;
//        int maxTicksBeforeClamp = (int) (90.0 / degreesPerTick);
//
//        LimelightLaunchParameters ydt = limelightAprilTagHelper.getGoalYawDistanceToleranceFromCurrentPosition();
//
//        //tag not found
//        if (ydt != null) {
//
//            turretTagNotFoundTimer.reset();
//
////            Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal : Tag found");
//
//            double turretTolerance = (ydt.distance > FLYWHEEL_POWER_BUCKET_THRESHOLD_FAR)
//                    ? TURRET_ALIGNMENT_TOLERANCE_DEGREES_FAR
//                    : TURRET_ALIGNMENT_TOLERANCE_DEGREES_NEAR;
//
////            Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal : Tag found. Tolerance: " + turretTolerance);
//
////            int isRobotToLeftOfCenterLine = isRobotToLeftOfCenterLine();
////            if (isRobotToLeftOfCenterLine == 1) {
//////                Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal : Tag found - adding left of line bias");
////                ydt.yaw = ydt.yaw - turretTolerance;
////            }
////            if (isRobotToLeftOfCenterLine == -1) {
//////                Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal : Tag found - adding right of line bias");
////                ydt.yaw = ydt.yaw + turretTolerance;
////            }
//
//            double difference = Math.abs(ydt.yaw) - turretTolerance;
//
////            Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal : Tag found - yaw difference: " + difference);
//
//            if (difference <= 0) {
////                Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal : Tag found - yaw difference within tolerance - aligned");
//                robotHardware.setAlignmentLightColor(ROBOT_ALIGNED_TO_SHOOT_LIGHT);
//                robotHardware.setLaunchTurretPower(0);
//                return;
//            }
//
//            robotHardware.setAlignmentLightColor(ROBOT_NOT_ALIGNED_TO_SHOOT_LIGHT);
//
//            int differenceToCoverInTicks = (int)((difference * TURRET_FRACTION_OF_DIFFERENCE_TO_COVER) / degreesPerTick);
//
//            int previousPos = robotHardware.getLaunchTurretPosition();
//
////            Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal: Tag found: old turret position: " + previousPos);
//
//            if (ydt.yaw < 0)
//                differenceToCoverInTicks = -1 * differenceToCoverInTicks;
//
//            int newMotorPosition = previousPos + differenceToCoverInTicks;
//
////            Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal: Tag found: new turret position: " + newMotorPosition);
//
//            if (newMotorPosition < 0)
//                newMotorPosition = Math.max( -1 * maxTicksBeforeClamp, newMotorPosition);
//            else
//                newMotorPosition = Math.min(maxTicksBeforeClamp, newMotorPosition);
//
////            Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal: Tag found: clamped motor position: " + newMotorPosition);
//
//            robotHardware.setLaunchTurretPosition(newMotorPosition);
//        }
//
//        if (TURRET_POWER_SOFT_LIMITS_ENABLED) {
//            int currentPos = robotHardware.getLaunchTurretPosition();
//            double power = robotHardware.getLaunchTurretPower();
//            if ((power > 0 && currentPos >= maxTicksBeforeClamp) || (power < 0 && currentPos <= -maxTicksBeforeClamp)) {
////                Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal: stopping motor since it reached clamp");
//
//                robotHardware.setLaunchTurretPower(0);
//            }
//        }
//    }

public void AlignTurretToGoalPower() {
    if (turretAlignmentThrottleTimer.milliseconds() < TURRET_ALIGNMENT_THROTTLE_MILLIS) {
        return;
    }
    turretAlignmentThrottleTimer.reset();

    double turretAnglePerPulleyRotation = 360.0 / (123.0 / 24.0);
    double degreesPerTick = turretAnglePerPulleyRotation / TURRET_PULSES_PER_REV;
    int maxTicksBeforeClamp = (int) (180.0 / degreesPerTick);

    LimelightLaunchParameters ydt = limelightAprilTagHelper.getGoalYawDistanceToleranceFromCurrentPosition();
    if (ydt == null) {
        robotHardware.setAlignmentLightColor(ROBOT_NOT_ALIGNED_TO_SHOOT_LIGHT);
        robotHardware.setLaunchTurretPower(0);
        return;
    }

    double turretTolerance = (ydt.distance > FLYWHEEL_POWER_BUCKET_THRESHOLD_FAR)
            ? TURRET_ALIGNMENT_TOLERANCE_DEGREES_FAR
            : TURRET_ALIGNMENT_TOLERANCE_DEGREES_NEAR;

    int isRobotToLeftOfCenterLine = isRobotToLeftOfCenterLine();
    if (isRobotToLeftOfCenterLine == 1) {
        ydt.yaw = ydt.yaw - turretTolerance;
    }
    if (isRobotToLeftOfCenterLine == -1) {
        ydt.yaw = ydt.yaw + turretTolerance;
    }

    double difference = Math.abs(ydt.yaw) - turretTolerance;
    if (difference <= 0) {
        robotHardware.setAlignmentLightColor(ROBOT_ALIGNED_TO_SHOOT_LIGHT);
        robotHardware.setLaunchTurretPower(0);
        return;
    }

    robotHardware.setAlignmentLightColor(ROBOT_NOT_ALIGNED_TO_SHOOT_LIGHT);

    double power = Math.max(-TURRET_POWER_MAX, Math.min(TURRET_POWER_MAX, TURRET_POWER_KP * ydt.yaw));
    if (Math.abs(power) < TURRET_POWER_MIN) {
        power = Math.copySign(TURRET_POWER_MIN, power);
    }

    if (TURRET_POWER_SOFT_LIMITS_ENABLED) {
        int currentPos = robotHardware.getLaunchTurretPosition();
        if ((power > 0 && currentPos >= maxTicksBeforeClamp) || (power < 0 && currentPos <= -maxTicksBeforeClamp)) {
            power = 0;
        }
    }

    robotHardware.setLaunchTurretPower(power);
}


    public double computeTurretDegreesToFaceGoal(Pose2d robotPose, double goalX, double goalY, double currentTurretRadians) {
        double heading = robotPose.heading.toDouble();

//        double cos = Math.cos(heading);
//        double sin = Math.sin(heading);

//        double offsetWorldX = TURRET_PIVOT_OFFSET_X * cos - TURRET_PIVOT_OFFSET_Y * sin;
//        double offsetWorldY = TURRET_PIVOT_OFFSET_X * sin + TURRET_PIVOT_OFFSET_Y * cos;
//
//        double turretWorldX = robotPose.position.x + offsetWorldX;
//        double turretWorldY = robotPose.position.y + offsetWorldY;

        double deltaX = goalX - robotPose.position.x;
        double deltaY = goalY - robotPose.position.y;

//        double deltaX = goalX - turretWorldX;
//        double deltaY = goalY - turretWorldY;


        Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal : robot pose: X: " + robotPose.position.x + " Y: " + robotPose.position.y + " Heading:" + Math.toDegrees(heading));

//        Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal : turret pose: X: " + turretWorldX + " Y: " + turretWorldY);

//        Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal : current Turret Degrees: " + Math.toDegrees(currentTurretRadians));

        double targetRadiansRelativeToRobotPosition = Math.atan2(deltaY, deltaX);

//        Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal : target Angle Relative To Robot X Y only: " + Math.toDegrees(targetRadiansRelativeToRobotPosition));

        double turretHeadingInFieldSpace = heading - currentTurretRadians;

        //For limelight, this is the robot's orientation - go ahead and update this in limelight
        robotHardware.updateLimelightYawDegrees(Math.toDegrees(turretHeadingInFieldSpace));

        Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal : turret heading in field space: " + Math.toDegrees(turretHeadingInFieldSpace));

        double degreesTurretHasToMove = Math.toDegrees(turretHeadingInFieldSpace - targetRadiansRelativeToRobotPosition);
//        Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal : degrees turret has to  move: " + degreesTurretHasToMove);

        return degreesTurretHasToMove;
    }

    private double normalizeToTurretRange(double degreesTurretHasToMove, double degreesPerTick, double maxTicksBeforeClamp) {
        double currentPos = robotHardware.getLaunchTurretPosition();

        double targetPos = currentPos + (degreesTurretHasToMove / degreesPerTick);

//        Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal : target position: " + targetPos);

        if (targetPos >= -maxTicksBeforeClamp && targetPos <= maxTicksBeforeClamp)  //within range - move
        {
//            Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal : target position reachable by moving " + degreesTurretHasToMove + " degrees");
            return degreesTurretHasToMove;
        }

        // flip the angle and try
        if (degreesTurretHasToMove > 0)
            degreesTurretHasToMove = 360 - degreesTurretHasToMove;
        if (degreesTurretHasToMove < 0)
            degreesTurretHasToMove = 360 + degreesTurretHasToMove;

        targetPos = currentPos + (degreesTurretHasToMove / degreesPerTick);

        if (targetPos >= -maxTicksBeforeClamp && targetPos <= maxTicksBeforeClamp)  //within range - move
        {
//            Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal : Flipped target position reachable by moving " + degreesTurretHasToMove + " degrees");
            return degreesTurretHasToMove;
        }

//        Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal : target position not reachable:");
        return 0; // dont move at all
    }

    /**
     * NEW ROBUST TURRET ALIGNMENT METHOD
     * 
     * Uses odometry-based positioning as the primary source of truth, with limelight
     * only for fine adjustments. Implements a state machine with hysteresis to prevent
     * oscillation and unnecessary movement.
     * 
     * State Machine:
     * - COARSE_ALIGNING: Large error (>TURRET_UNLOCK_THRESHOLD_DEGREES), uses odometry only
     * - FINE_ALIGNING: Small error, uses odometry + limelight for fine tuning
     * - LOCKED: Aligned and stable, turret holds position unless robot moves significantly
     */
    public void AlignTurretToGoalRobust() {
        // Throttle updates for stability
        if (turretAlignmentThrottleTimer.milliseconds() < TURRET_ALIGNMENT_THROTTLE_MILLIS) {
            return;
        }
        turretAlignmentThrottleTimer.reset();

        // Calculate turret conversion constants
        double turretAnglePerPulleyRotation = 360.0 / (123.0 / 24.0);
        double degreesPerTick = turretAnglePerPulleyRotation / TURRET_PULSES_PER_REV;
        int maxTicksBeforeClamp = (int) (90.0 / degreesPerTick);

        // Get current robot pose from odometry (primary source of truth)
        Pose2d currentPose = CrossOpModeStorage.currentPose;
        
        // Get goal coordinates based on alliance
        double goalX = (allianceColor == AllianceColors.RED) ? GOAL_RED_X : GOAL_BLUE_X;
        double goalY = (allianceColor == AllianceColors.RED) ? GOAL_RED_Y : GOAL_BLUE_Y;

        // ============ CHECK IF ROBOT HAS MOVED (to unlock turret if needed) ============
        boolean robotHasMoved = hasRobotMovedSignificantly(currentPose);
        
        if (robotHasMoved && currentAlignmentState == TurretAlignmentState.LOCKED) {
            // Robot moved - unlock turret and go back to coarse alignment
            Log.i("== LAUNCH SYSTEM ==", "AlignTurretRobust: Robot moved - unlocking turret");
            currentAlignmentState = TurretAlignmentState.COARSE_ALIGNING;
            alignmentStableCounter = 0;
        }

        // ============ CALCULATE ODOMETRY-BASED TARGET ============
        int currentTurretPosition = robotHardware.getLaunchTurretPosition();
        double currentTurretRadians = Math.toRadians(currentTurretPosition * degreesPerTick);
        
        // Calculate desired turret angle using odometry
        double odometryTargetDegrees = computeTurretDegreesToFaceGoal(currentPose, goalX, goalY, currentTurretRadians);
        odometryTargetDegrees = normalizeToTurretRange(odometryTargetDegrees, degreesPerTick, maxTicksBeforeClamp);
        lastOdometryTargetDegrees = odometryTargetDegrees;

        // ============ GET LIMELIGHT DATA (if available) ============
        LimelightLaunchParameters limelightData = limelightAprilTagHelper.getGoalYawDistanceToleranceFromCurrentPosition();
        double limelightAdjustmentDegrees = 0;
        boolean limelightAvailable = (limelightData != null);
        
        if (limelightAvailable) {
            turretTagNotFoundTimer.reset();  // Reset timeout since we found tag
            limelightAdjustmentDegrees = limelightData.yaw;
            
            // Apply center line bias if needed
//            int isLeftOfCenter = isRobotToLeftOfCenterLineUsingOdometry(currentPose);
//            double turretTolerance = (limelightData.distance > FLYWHEEL_POWER_BUCKET_THRESHOLD_FAR)
//                    ? TURRET_ALIGNMENT_TOLERANCE_DEGREES_FAR
//                    : TURRET_ALIGNMENT_TOLERANCE_DEGREES_NEAR;
//
//            if (isLeftOfCenter == 1) {
//                limelightAdjustmentDegrees -= turretTolerance;
//            } else if (isLeftOfCenter == -1) {
//                limelightAdjustmentDegrees += turretTolerance;
//            }
        }

        // ============ STATE MACHINE LOGIC ============
        double effectiveError;
        int targetTurretPosition;
        int turretVelocity = TURRET_VELOCITY;

        switch (currentAlignmentState) {
            case LOCKED:
                // In LOCKED state, only move if:
                // 1. Robot has moved (already handled above - would transition to COARSE)
                // 2. Limelight shows significant error
                
                if (limelightAvailable && Math.abs(limelightAdjustmentDegrees) > TURRET_UNLOCK_THRESHOLD_DEGREES) {
                    // Limelight shows we're misaligned - unlock
                    Log.i("== LAUNCH SYSTEM ==", "AlignTurretRobust: Limelight shows error " + limelightAdjustmentDegrees + " - unlocking");
                    currentAlignmentState = TurretAlignmentState.FINE_ALIGNING;
                    alignmentStableCounter = 0;
                } else {
                    // Stay locked - don't move turret at all
                    robotHardware.setAlignmentLightColor(ROBOT_ALIGNED_TO_SHOOT_LIGHT);
                    // Maintain position by setting to last locked position
//                    robotHardware.setLaunchTurretPositionAndVelocity(lastLockedTurretPosition, TURRET_VELOCITY_FINE);
                    return;
                }
                // Fall through to FINE_ALIGNING if we unlocked
                
            case FINE_ALIGNING:
                // Use combination of odometry and limelight
                if (limelightAvailable && Math.abs(limelightAdjustmentDegrees) < LIMELIGHT_FINE_ADJUST_MAX_DEGREES) {
                    // Weighted combination: prefer limelight for small adjustments
                    if (Math.abs(limelightAdjustmentDegrees) > LIMELIGHT_MIN_ADJUST_THRESHOLD_DEGREES) {
                        effectiveError = limelightAdjustmentDegrees * LIMELIGHT_WEIGHT 
                                       + odometryTargetDegrees * (1.0 - LIMELIGHT_WEIGHT);
                    } else {
                        // Limelight error is very small - use it directly
                        effectiveError = limelightAdjustmentDegrees;
                    }
                } else {
                    // No limelight or error too large - use odometry only
                    effectiveError = odometryTargetDegrees;
                }
                
                // Use slower velocity for fine adjustments
                turretVelocity = TURRET_VELOCITY_FINE;
                
                // Check if we should lock
                if (Math.abs(effectiveError) <= TURRET_LOCK_THRESHOLD_DEGREES) {
                    alignmentStableCounter++;
                    if (alignmentStableCounter >= ALIGNMENT_STABLE_CYCLES_REQUIRED) {
                        // Stable for enough cycles - lock the turret
                        currentAlignmentState = TurretAlignmentState.LOCKED;
                        lastLockedPose = currentPose;
                        lastLockedTurretPosition = currentTurretPosition;
                        robotHardware.setAlignmentLightColor(ROBOT_ALIGNED_TO_SHOOT_LIGHT);
                        Log.i("== LAUNCH SYSTEM ==", "AlignTurretRobust: LOCKED at position " + lastLockedTurretPosition);
                        return;
                    }
                } else {
                    alignmentStableCounter = 0;  // Reset counter if error increased
                }
                
                // Check if error grew too large (go back to coarse)
                if (Math.abs(effectiveError) > TURRET_UNLOCK_THRESHOLD_DEGREES * 2) {
                    currentAlignmentState = TurretAlignmentState.COARSE_ALIGNING;
                    alignmentStableCounter = 0;
                }
                break;
                
            case COARSE_ALIGNING:
            default:
                // Use odometry only for coarse alignment
                effectiveError = odometryTargetDegrees;
                turretVelocity = TURRET_VELOCITY;
                
                // Check if we're close enough to switch to fine alignment
                if (Math.abs(effectiveError) < LIMELIGHT_FINE_ADJUST_MAX_DEGREES) {
                    currentAlignmentState = TurretAlignmentState.FINE_ALIGNING;
                    alignmentStableCounter = 0;
                }
                break;
        }

        // ============ APPLY DEADBAND ============
        if (Math.abs(effectiveError) < TURRET_DEADBAND_DEGREES) {
            // Within deadband - don't move, but not yet stable enough to lock
            robotHardware.setAlignmentLightColor(ROBOT_ALIGNED_TO_SHOOT_LIGHT);
            return;
        }

        // ============ CALCULATE AND APPLY TURRET MOVEMENT ============
        robotHardware.setAlignmentLightColor(ROBOT_NOT_ALIGNED_TO_SHOOT_LIGHT);
        
        // Convert error to ticks, applying fraction for smooth approach
        double fractionToCover = (currentAlignmentState == TurretAlignmentState.FINE_ALIGNING) 
                ? TURRET_FRACTION_OF_DIFFERENCE_TO_COVER 
                : 1.0;  // Full movement for coarse alignment
        
        int ticksToMove = (int)((effectiveError * fractionToCover) / degreesPerTick);
        targetTurretPosition = currentTurretPosition + ticksToMove;

        // Apply soft limits
        if (targetTurretPosition < -maxTicksBeforeClamp) {
            targetTurretPosition = -maxTicksBeforeClamp;
        } else if (targetTurretPosition > maxTicksBeforeClamp) {
            targetTurretPosition = maxTicksBeforeClamp;
        }

        // Additional soft limit check - don't command movement that would exceed limits
        if (TURRET_POWER_SOFT_LIMITS_ENABLED) {
            if ((ticksToMove > 0 && currentTurretPosition >= maxTicksBeforeClamp) ||
                (ticksToMove < 0 && currentTurretPosition <= -maxTicksBeforeClamp)) {
                // At limit - don't move further
                Log.i("== LAUNCH SYSTEM ==", "AlignTurretRobust: At soft limit, stopping");
                robotHardware.setLaunchTurretPower(0);
                return;
            }
        }

        // Command the turret to move
        robotHardware.setLaunchTurretPositionAndVelocity(targetTurretPosition, turretVelocity);
        
//        Log.i("== LAUNCH SYSTEM ==", "AlignTurretRobust: State=" + currentAlignmentState + 
//              " Error=" + effectiveError + " Target=" + targetTurretPosition + " Velocity=" + turretVelocity);
    }

    /**
     * Check if the robot has moved significantly from the last locked pose.
     * Used to determine if we should unlock the turret.
     */
    private boolean hasRobotMovedSignificantly(Pose2d currentPose) {
        if (lastLockedPose == null) {
            return true;  // Never locked, consider as "moved"
        }

        // Check position change
        double dx = currentPose.position.x - lastLockedPose.position.x;
        double dy = currentPose.position.y - lastLockedPose.position.y;
        double positionChange = Math.sqrt(dx * dx + dy * dy);
        
        if (positionChange > ROBOT_POSITION_CHANGE_THRESHOLD_INCHES) {
            return true;
        }

        // Check heading change
        double currentHeadingDeg = Math.toDegrees(currentPose.heading.toDouble());
        double lastHeadingDeg = Math.toDegrees(lastLockedPose.heading.toDouble());
        double headingChange = Math.abs(normalizeAngle(currentHeadingDeg - lastHeadingDeg));
        
        if (headingChange > ROBOT_HEADING_CHANGE_THRESHOLD_DEGREES) {
            return true;
        }

        return false;
    }

    /**
     * Normalize angle to -180 to 180 range
     */
    private double normalizeAngle(double angleDegrees) {
        while (angleDegrees > 180) angleDegrees -= 360;
        while (angleDegrees < -180) angleDegrees += 360;
        return angleDegrees;
    }

    /**
     * Check if robot is to left of center line using odometry (doesn't need limelight)
     * Returns: 1 = left, -1 = right, 0 = unknown/on line
     */
    private int isRobotToLeftOfCenterLineUsingOdometry(Pose2d robotPose) {
        // Center line coordinates
        int x1 = 72;
        int y1 = -72;
        int x2 = -72;
        int y2 = 72;

        if (allianceColor == AllianceColors.BLUE) {
            y1 = 72;
            y2 = -72;
        }

        double currX = robotPose.position.x;
        double currY = robotPose.position.y;

        double d = (x2 - x1) * (currY - y1) - (y2 - y1) * (currX - x1);

        // If d > 0, point is to the left of the line
        if (Math.abs(d) < 1.0) return 0;  // Too close to line to tell
        return (d > 0) ? 1 : -1;
    }

    /**
     * Reset the turret alignment state machine.
     * Call this when starting a new match or when odometry is reset.
     */
    public void resetTurretAlignment() {
        currentAlignmentState = TurretAlignmentState.COARSE_ALIGNING;
        lastLockedPose = null;
        lastLockedTurretPosition = 0;
        alignmentStableCounter = 0;
        lastOdometryTargetDegrees = 0;
        Log.i("== LAUNCH SYSTEM ==", "AlignTurretRobust: State machine reset");
    }

    /**
     * Initialize alignment state from CrossOpModeStorage.
     * Call this at the start of TeleOp after Auto to preserve alignment state.
     * This seeds the last locked pose from the stored pose, allowing the alignment
     * to determine if the robot has moved since Auto ended.
     */
    public void initializeAlignmentFromStorage() {
        // Seed the last locked pose from storage so we can detect if robot moved
        lastLockedPose = CrossOpModeStorage.currentPose;
        lastLockedTurretPosition = robotHardware.getLaunchTurretPosition();
        
        // Start in FINE_ALIGNING to quickly verify alignment
        // If turret is still aligned, it will lock quickly
        // If turret is misaligned, it will correct with fine movements
        currentAlignmentState = TurretAlignmentState.FINE_ALIGNING;
        alignmentStableCounter = 0;
        
        Log.i("== LAUNCH SYSTEM ==", "AlignTurretRobust: Initialized from storage. " +
              "Pose: (" + lastLockedPose.position.x + ", " + lastLockedPose.position.y + ") " +
              "Turret: " + lastLockedTurretPosition);
    }

    /**
     * Get current alignment state for telemetry/debugging
     */
    public String getTurretAlignmentState() {
        return currentAlignmentState.toString();
    }

    /**
     * Check if turret is currently locked (aligned and stable)
     */
    public boolean isTurretLocked() {
        return currentAlignmentState == TurretAlignmentState.LOCKED;
    }

    // =====================================================================
    // LIMELIGHT-ONLY TURRET ALIGNMENT (NO ODOMETRY)
    // =====================================================================

    /**
     * LIMELIGHT-ONLY TURRET ALIGNMENT
     *
     * This method aligns the turret using ONLY limelight data - no odometry required.
     * Designed for situations where odometry is unavailable or unreliable.
     *
     * Features:
     * - 4-state machine: SEARCHING → COARSE → FINE → LOCKED
     * - If tag is not visible for 2 seconds, turret centers to initial position
     * - Locks turret when aligned, unlocks if robot moves (detected via limelight pose change)
     * - Low-pass filtering to reduce noise
     * - Center line bias compensation
     *
     * Call this method repeatedly in your control loop when you want limelight-only alignment.
     */
    public void AlignTurretToGoalLimelightOnly() {
        // Initialize tag lost timer if needed
        if (llOnlyTagLostTimer == null) {
            llOnlyTagLostTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        }

        // Throttle updates for stability
        if (turretAlignmentThrottleTimer.milliseconds() < TURRET_ALIGNMENT_THROTTLE_MILLIS) {
            return;
        }
        turretAlignmentThrottleTimer.reset();

        // Calculate turret conversion constants
        double turretAnglePerPulleyRotation = 360.0 / (123.0 / 24.0);
        double degreesPerTick = turretAnglePerPulleyRotation / TURRET_PULSES_PER_REV;
        int maxTicksBeforeClamp = (int) (90.0 / degreesPerTick);

        int currentTurretPosition = robotHardware.getLaunchTurretPosition();

        // ============ GET LIMELIGHT DATA ============
        LimelightLaunchParameters limelightData = limelightAprilTagHelper.getGoalYawDistanceToleranceFromCurrentPosition();
        Pose3D currentLimelightPose = limelightAprilTagHelper.getRobotPoseFromAprilTags();

        boolean tagVisible = (limelightData != null);

        if (tagVisible) {
            // Tag found - reset the lost timer
            llOnlyTagLostTimer.reset();

            // Check if robot has moved significantly (using limelight pose)
            if (llOnlyState == LimelightOnlyAlignmentState.LOCKED &&
                    hasRobotMovedUsingLimelight(currentLimelightPose)) {
                Log.i("== LAUNCH SYSTEM ==", "LLOnlyAlign: Robot moved - unlocking");
                llOnlyState = LimelightOnlyAlignmentState.COARSE;
                llOnlyStableCounter = 0;
            }

            // Update last known pose
            llOnlyLastKnownPose = currentLimelightPose;

            // If we were searching, transition to coarse alignment
            if (llOnlyState == LimelightOnlyAlignmentState.SEARCHING) {
                Log.i("== LAUNCH SYSTEM ==", "LLOnlyAlign: Tag found - starting alignment");
                llOnlyState = LimelightOnlyAlignmentState.COARSE;
                llOnlyStableCounter = 0;
                llOnlyFilteredError = limelightData.yaw;  // Initialize filter
            }
        } else {
            // Tag not visible
            if (llOnlyTagLostTimer.milliseconds() > LL_ONLY_TAG_LOST_TIMEOUT_MILLIS) {
                // Tag lost for too long - center the turret
                if (llOnlyState != LimelightOnlyAlignmentState.SEARCHING) {
                    Log.i("== LAUNCH SYSTEM ==", "LLOnlyAlign: Tag lost for " +
                            LL_ONLY_TAG_LOST_TIMEOUT_MILLIS + "ms - centering turret");
                    llOnlyState = LimelightOnlyAlignmentState.SEARCHING;
                }

                // Move turret to center position
                robotHardware.setAlignmentLightColor(ROBOT_NOT_ALIGNED_TO_SHOOT_LIGHT);
                robotHardware.setLaunchTurretPositionAndVelocity(TURRET_CENTERED_POSITION, LL_ONLY_TURRET_VELOCITY_COARSE);
                return;
            }

            // Tag temporarily lost but within timeout - hold current position
            if (llOnlyState == LimelightOnlyAlignmentState.LOCKED) {
                robotHardware.setLaunchTurretPositionAndVelocity(llOnlyLastLockedTurretPosition, LL_ONLY_TURRET_VELOCITY_FINE);
            }
            // If not locked, just wait for tag to reappear
            return;
        }

        // ============ CALCULATE ERROR WITH BIAS ============
        double rawError = limelightData.yaw;

        // Apply center line bias if applicable
        int isLeftOfCenter = isRobotToLeftOfCenterLine();
        double turretTolerance = (limelightData.distance > FLYWHEEL_POWER_BUCKET_THRESHOLD_FAR)
                ? TURRET_ALIGNMENT_TOLERANCE_DEGREES_FAR
                : TURRET_ALIGNMENT_TOLERANCE_DEGREES_NEAR;

        if (isLeftOfCenter == 1) {
            rawError -= turretTolerance;
        } else if (isLeftOfCenter == -1) {
            rawError += turretTolerance;
        }

        // Apply low-pass filter
        llOnlyFilteredError = LL_ONLY_ERROR_FILTER_ALPHA * rawError
                + (1.0 - LL_ONLY_ERROR_FILTER_ALPHA) * llOnlyFilteredError;

        double effectiveError = llOnlyFilteredError;

        // ============ STATE MACHINE ============
        int turretVelocity;
        double fractionToCover;

        switch (llOnlyState) {
            case LOCKED:
                // Check if error grew too large
                if (Math.abs(effectiveError) > LL_ONLY_UNLOCK_THRESHOLD_DEGREES) {
                    Log.i("== LAUNCH SYSTEM ==", "LLOnlyAlign: Error " + effectiveError + " - unlocking");
                    llOnlyState = LimelightOnlyAlignmentState.FINE;
                    llOnlyStableCounter = 0;
                } else {
                    // Stay locked
                    robotHardware.setAlignmentLightColor(ROBOT_ALIGNED_TO_SHOOT_LIGHT);
                    robotHardware.setLaunchTurretPositionAndVelocity(llOnlyLastLockedTurretPosition, LL_ONLY_TURRET_VELOCITY_FINE);
                    return;
                }
                // Fall through if unlocked

            case FINE:
                turretVelocity = LL_ONLY_TURRET_VELOCITY_FINE;
                fractionToCover = LL_ONLY_FRACTION_FINE;

                // Check for lock
                if (Math.abs(effectiveError) <= LL_ONLY_LOCK_THRESHOLD_DEGREES) {
                    llOnlyStableCounter++;
                    if (llOnlyStableCounter >= LL_ONLY_STABLE_CYCLES_REQUIRED) {
                        llOnlyState = LimelightOnlyAlignmentState.LOCKED;
                        llOnlyLastLockedTurretPosition = currentTurretPosition;
                        robotHardware.setAlignmentLightColor(ROBOT_ALIGNED_TO_SHOOT_LIGHT);
                        Log.i("== LAUNCH SYSTEM ==", "LLOnlyAlign: LOCKED at " + llOnlyLastLockedTurretPosition);
                        return;
                    }
                } else {
                    llOnlyStableCounter = 0;
                }

                // Check if error grew too large (back to coarse)
                if (Math.abs(effectiveError) > LL_ONLY_COARSE_THRESHOLD_DEGREES) {
                    llOnlyState = LimelightOnlyAlignmentState.COARSE;
                    llOnlyStableCounter = 0;
                }
                break;

            case COARSE:
            default:
                turretVelocity = LL_ONLY_TURRET_VELOCITY_COARSE;
                fractionToCover = LL_ONLY_FRACTION_COARSE;

                // Check for transition to fine
                if (Math.abs(effectiveError) < LL_ONLY_COARSE_THRESHOLD_DEGREES) {
                    llOnlyState = LimelightOnlyAlignmentState.FINE;
                    llOnlyStableCounter = 0;
                }
                break;

            case SEARCHING:
                // Should not reach here if tag is visible - handled above
                return;
        }

        // ============ APPLY DEADBAND ============
        if (Math.abs(effectiveError) < LL_ONLY_DEADBAND_DEGREES) {
            robotHardware.setAlignmentLightColor(ROBOT_ALIGNED_TO_SHOOT_LIGHT);
            return;
        }

        // ============ CALCULATE AND APPLY TURRET MOVEMENT ============
        robotHardware.setAlignmentLightColor(ROBOT_NOT_ALIGNED_TO_SHOOT_LIGHT);

        int ticksToMove = (int)((effectiveError * fractionToCover) / degreesPerTick);
        int targetTurretPosition = currentTurretPosition + ticksToMove;

        // Apply soft limits
        targetTurretPosition = Math.max(-maxTicksBeforeClamp,
                Math.min(maxTicksBeforeClamp, targetTurretPosition));

        // Check if at limit
        if (TURRET_POWER_SOFT_LIMITS_ENABLED) {
            if ((ticksToMove > 0 && currentTurretPosition >= maxTicksBeforeClamp) ||
                    (ticksToMove < 0 && currentTurretPosition <= -maxTicksBeforeClamp)) {
                Log.i("== LAUNCH SYSTEM ==", "LLOnlyAlign: At soft limit");
                robotHardware.setLaunchTurretPower(0);
                return;
            }
        }

        // Command turret movement
        robotHardware.setLaunchTurretPositionAndVelocity(targetTurretPosition, turretVelocity);
    }


    /**
     * Check if robot has moved significantly using limelight pose data.
     * Used when odometry is not available.
     */
    private boolean hasRobotMovedUsingLimelight(Pose3D currentPose) {
        if (llOnlyLastKnownPose == null || currentPose == null) {
            return false;  // Can't determine movement without previous pose
        }

        // Get positions in inches (limelight returns meters)
        double currentX = currentPose.getPosition().x * 39.37;
        double currentY = currentPose.getPosition().y * 39.37;
        double lastX = llOnlyLastKnownPose.getPosition().x * 39.37;
        double lastY = llOnlyLastKnownPose.getPosition().y * 39.37;

        double dx = currentX - lastX;
        double dy = currentY - lastY;
        double positionChange = Math.sqrt(dx * dx + dy * dy);

        if (positionChange > LL_ONLY_MOVEMENT_THRESHOLD_INCHES) {
            return true;
        }

        // Check heading change
        double currentHeading = currentPose.getOrientation().getYaw(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES);
        double lastHeading = llOnlyLastKnownPose.getOrientation().getYaw(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES);
        double headingChange = Math.abs(normalizeAngle(currentHeading - lastHeading));

        return headingChange > LL_ONLY_HEADING_THRESHOLD_DEGREES;
    }

    /**
     * Reset the limelight-only alignment state machine.
     */
    public void resetLimelightOnlyAlignment() {
        llOnlyState = LimelightOnlyAlignmentState.SEARCHING;
        llOnlyLastLockedTurretPosition = 0;
        llOnlyStableCounter = 0;
        llOnlyFilteredError = 0;
        llOnlyLastKnownPose = null;
        if (llOnlyTagLostTimer != null) {
            llOnlyTagLostTimer.reset();
        }
        Log.i("== LAUNCH SYSTEM ==", "LLOnlyAlign: Reset");
    }

    /**
     * Get the limelight-only alignment state for telemetry.
     */
    public String getLimelightOnlyAlignmentState() {
        return llOnlyState.toString();
    }

    /**
     * Check if limelight-only alignment is locked.
     */
    public boolean isLimelightOnlyLocked() {
        return llOnlyState == LimelightOnlyAlignmentState.LOCKED;
    }

    /**
     * Check if limelight-only mode is currently searching (tag not visible).
     */
    public boolean isLimelightOnlySearching() {
        return llOnlyState == LimelightOnlyAlignmentState.SEARCHING;
    }

    // =====================================================================
    // POSITION-ONLY TURRET ALIGNMENT (NO LIMELIGHT)
    // =====================================================================
    
    /**
     * POSITION-ONLY TURRET ALIGNMENT
     * 
     * This method aligns the turret using ONLY odometry data - no limelight required.
     * Designed for maximum accuracy and consistency when shooting artifacts.
     * 
     * Features:
     * - 4-state machine: COARSE → FINE → ULTRA_FINE → LOCKED
     * - Low-pass filtering to reduce noise and oscillation
     * - Turret pivot offset compensation for improved accuracy
     * - Very tight tolerances for precision alignment
     * - Hysteresis to prevent oscillation at state boundaries
     * - Variable velocity based on error magnitude
     * 
     * Call this method repeatedly in your control loop when you want position-only alignment.
     */
    public void AlignTurretToGoalPositionOnly() {
        // Throttle updates for stability
        if (turretAlignmentThrottleTimer.milliseconds() < TURRET_ALIGNMENT_THROTTLE_MILLIS) {
            return;
        }
        turretAlignmentThrottleTimer.reset();

        // Calculate turret conversion constants
        double turretAnglePerPulleyRotation = 360.0 / (123.0 / 24.0);
        double degreesPerTick = turretAnglePerPulleyRotation / TURRET_PULSES_PER_REV;
        int maxTicksBeforeClamp = (int) (90.0 / degreesPerTick);

        // Get current robot pose from odometry
        Pose2d currentPose = CrossOpModeStorage.currentPose;
        
        // Get goal coordinates based on alliance
        double goalX = (allianceColor == AllianceColors.RED) ? GOAL_RED_X : GOAL_BLUE_X;
        double goalY = (allianceColor == AllianceColors.RED) ? GOAL_RED_Y : GOAL_BLUE_Y;

        // ============ CHECK IF ROBOT HAS MOVED (to unlock turret if needed) ============
        boolean robotHasMoved = hasRobotMovedForPositionOnly(currentPose);
        
        if (robotHasMoved && posOnlyState == PositionOnlyAlignmentState.LOCKED) {
            Log.i("== LAUNCH SYSTEM ==", "PosOnlyAlign: Robot moved - unlocking");
            posOnlyState = PositionOnlyAlignmentState.COARSE;
            posOnlyStableCounter = 0;
            posOnlyErrorIntegral = 0;  // Reset integral when unlocking
        }

        // ============ CALCULATE TARGET ANGLE ============
        int currentTurretPosition = robotHardware.getLaunchTurretPosition();
        double currentTurretRadians = Math.toRadians(currentTurretPosition * degreesPerTick);
        
        // Calculate desired turret angle using enhanced method with pivot offset
        double rawErrorDegrees = computeTurretDegreesToFaceGoalWithPivot(
            currentPose, goalX, goalY, currentTurretRadians
        );
        rawErrorDegrees = normalizeToTurretRange(rawErrorDegrees, degreesPerTick, maxTicksBeforeClamp);

        // ============ APPLY LOW-PASS FILTER ============
        // Filter the error to reduce noise and prevent jitter
        posOnlyFilteredError = POS_ONLY_ERROR_FILTER_ALPHA * rawErrorDegrees 
                             + (1.0 - POS_ONLY_ERROR_FILTER_ALPHA) * posOnlyFilteredError;
        
        double effectiveError = posOnlyFilteredError;

        // ============ STATE MACHINE ============
        int turretVelocity;
        double fractionToCover;
        
        switch (posOnlyState) {
            case LOCKED:
                // Check if we need to unlock
                if (Math.abs(effectiveError) > POS_ONLY_UNLOCK_THRESHOLD_DEGREES) {
                    Log.i("== LAUNCH SYSTEM ==", "PosOnlyAlign: Error increased - unlocking");
                    posOnlyState = PositionOnlyAlignmentState.FINE;
                    posOnlyStableCounter = 0;
                } else {
                    // Stay locked - maintain position
                    robotHardware.setAlignmentLightColor(ROBOT_ALIGNED_TO_SHOOT_LIGHT);
                    robotHardware.setLaunchTurretPositionAndVelocity(
                        posOnlyLastLockedTurretPosition, 
                        POS_ONLY_TURRET_VELOCITY_ULTRA_FINE
                    );
                    return;
                }
                // Fall through if we unlocked
                
            case ULTRA_FINE:
                turretVelocity = POS_ONLY_TURRET_VELOCITY_ULTRA_FINE;
                fractionToCover = POS_ONLY_FRACTION_ULTRA_FINE;
                
                // Check for lock
                if (Math.abs(effectiveError) <= POS_ONLY_LOCK_THRESHOLD_DEGREES) {
                    posOnlyStableCounter++;
                    if (posOnlyStableCounter >= POS_ONLY_STABLE_CYCLES_REQUIRED) {
                        posOnlyState = PositionOnlyAlignmentState.LOCKED;
                        posOnlyLastLockedPose = currentPose;
                        posOnlyLastLockedTurretPosition = currentTurretPosition;
                        posOnlyErrorIntegral = 0;
                        robotHardware.setAlignmentLightColor(ROBOT_ALIGNED_TO_SHOOT_LIGHT);
                        Log.i("== LAUNCH SYSTEM ==", "PosOnlyAlign: LOCKED at " + posOnlyLastLockedTurretPosition);
                        return;
                    }
                } else if (Math.abs(effectiveError) > POS_ONLY_UNLOCK_THRESHOLD_DEGREES) {
                    posOnlyState = PositionOnlyAlignmentState.FINE;
                    posOnlyStableCounter = 0;
                } else {
                    posOnlyStableCounter = 0;
                }
                break;
                
            case FINE:
                turretVelocity = POS_ONLY_TURRET_VELOCITY_FINE;
                fractionToCover = POS_ONLY_FRACTION_FINE;
                
                // Check for transition to ultra-fine
                if (Math.abs(effectiveError) <= POS_ONLY_LOCK_THRESHOLD_DEGREES * 2) {
                    posOnlyState = PositionOnlyAlignmentState.ULTRA_FINE;
                    posOnlyStableCounter = 0;
                }
                // Check if we need to go back to coarse
                else if (Math.abs(effectiveError) > POS_ONLY_UNLOCK_THRESHOLD_DEGREES * 2) {
                    posOnlyState = PositionOnlyAlignmentState.COARSE;
                    posOnlyStableCounter = 0;
                }
                break;
                
            case COARSE:
            default:
                turretVelocity = POS_ONLY_TURRET_VELOCITY_COARSE;
                fractionToCover = POS_ONLY_FRACTION_COARSE;
                
                // Check for transition to fine
                if (Math.abs(effectiveError) < POS_ONLY_UNLOCK_THRESHOLD_DEGREES * 2) {
                    posOnlyState = PositionOnlyAlignmentState.FINE;
                    posOnlyStableCounter = 0;
                }
                break;
        }

        // ============ APPLY DEADBAND ============
        if (Math.abs(effectiveError) < POS_ONLY_DEADBAND_DEGREES) {
            robotHardware.setAlignmentLightColor(ROBOT_ALIGNED_TO_SHOOT_LIGHT);
            // In deadband but not yet stable enough to lock
            return;
        }

        // ============ CALCULATE MOVEMENT ============
        robotHardware.setAlignmentLightColor(ROBOT_NOT_ALIGNED_TO_SHOOT_LIGHT);
        
        int ticksToMove = (int)((effectiveError * fractionToCover) / degreesPerTick);
        int targetTurretPosition = currentTurretPosition + ticksToMove;

        // Apply soft limits
        targetTurretPosition = Math.max(-maxTicksBeforeClamp, 
                                Math.min(maxTicksBeforeClamp, targetTurretPosition));

        // Check if we're at the limit
        if (TURRET_POWER_SOFT_LIMITS_ENABLED) {
            if ((ticksToMove > 0 && currentTurretPosition >= maxTicksBeforeClamp) ||
                (ticksToMove < 0 && currentTurretPosition <= -maxTicksBeforeClamp)) {
                Log.i("== LAUNCH SYSTEM ==", "PosOnlyAlign: At soft limit");
                robotHardware.setLaunchTurretPower(0);
                return;
            }
        }

        // Command turret movement
        robotHardware.setLaunchTurretPositionAndVelocity(targetTurretPosition, turretVelocity);
    }

    /**
     * Enhanced turret angle calculation that accounts for turret pivot offset.
     * This provides more accurate targeting by calculating from the actual turret
     * position rather than robot center.
     */
    private double computeTurretDegreesToFaceGoalWithPivot(Pose2d robotPose, double goalX, double goalY, double currentTurretRadians) {
        double heading = robotPose.heading.toDouble();
        
        double turretWorldX, turretWorldY;
        
        if (POS_ONLY_USE_PIVOT_OFFSET) {
            // Transform turret pivot offset from robot frame to world frame
            double cos = Math.cos(heading);
            double sin = Math.sin(heading);
            
            double offsetWorldX = TURRET_PIVOT_OFFSET_X * cos - TURRET_PIVOT_OFFSET_Y * sin;
            double offsetWorldY = TURRET_PIVOT_OFFSET_X * sin + TURRET_PIVOT_OFFSET_Y * cos;
            
            turretWorldX = robotPose.position.x + offsetWorldX;
            turretWorldY = robotPose.position.y + offsetWorldY;
        } else {
            turretWorldX = robotPose.position.x;
            turretWorldY = robotPose.position.y;
        }

        double deltaX = goalX - turretWorldX;
        double deltaY = goalY - turretWorldY;

        double targetRadiansRelativeToRobotPosition = Math.atan2(deltaY, deltaX);
        double turretHeadingInFieldSpace = heading - currentTurretRadians;
        double degreesTurretHasToMove = Math.toDegrees(turretHeadingInFieldSpace - targetRadiansRelativeToRobotPosition);

        return degreesTurretHasToMove;
    }

    /**
     * Check if robot moved significantly (for position-only mode with tighter thresholds)
     */
    private boolean hasRobotMovedForPositionOnly(Pose2d currentPose) {
        if (posOnlyLastLockedPose == null) {
            return true;
        }

        double dx = currentPose.position.x - posOnlyLastLockedPose.position.x;
        double dy = currentPose.position.y - posOnlyLastLockedPose.position.y;
        double positionChange = Math.sqrt(dx * dx + dy * dy);
        
        if (positionChange > POS_ONLY_MOVEMENT_THRESHOLD_INCHES) {
            return true;
        }

        double currentHeadingDeg = Math.toDegrees(currentPose.heading.toDouble());
        double lastHeadingDeg = Math.toDegrees(posOnlyLastLockedPose.heading.toDouble());
        double headingChange = Math.abs(normalizeAngle(currentHeadingDeg - lastHeadingDeg));
        
        return headingChange > POS_ONLY_HEADING_THRESHOLD_DEGREES;
    }

    /**
     * Reset the position-only alignment state machine.
     */
    public void resetPositionOnlyAlignment() {
        posOnlyState = PositionOnlyAlignmentState.COARSE;
        posOnlyLastLockedPose = null;
        posOnlyLastLockedTurretPosition = 0;
        posOnlyStableCounter = 0;
        posOnlyFilteredError = 0;
        posOnlyLastError = 0;
        posOnlyErrorIntegral = 0;
        Log.i("== LAUNCH SYSTEM ==", "PosOnlyAlign: Reset");
    }

    /**
     * Initialize position-only alignment from stored pose.
     */
    public void initializePositionOnlyFromStorage() {
        posOnlyLastLockedPose = CrossOpModeStorage.currentPose;
        posOnlyLastLockedTurretPosition = robotHardware.getLaunchTurretPosition();
        posOnlyState = PositionOnlyAlignmentState.FINE;
        posOnlyStableCounter = 0;
        posOnlyFilteredError = 0;
        posOnlyErrorIntegral = 0;
        Log.i("== LAUNCH SYSTEM ==", "PosOnlyAlign: Initialized from storage");
    }

    /**
     * Get the position-only alignment state for telemetry.
     */
    public String getPositionOnlyAlignmentState() {
        return posOnlyState.toString();
    }

    /**
     * Check if position-only alignment is locked.
     */
    public boolean isPositionOnlyLocked() {
        return posOnlyState == PositionOnlyAlignmentState.LOCKED;
    }

    /**
     * Get the distance to goal using odometry only (no limelight).
     */
    public double getDistanceToGoalFromOdometry() {
        Pose2d currentPose = CrossOpModeStorage.currentPose;
        double goalX = (allianceColor == AllianceColors.RED) ? GOAL_RED_X : GOAL_BLUE_X;
        double goalY = (allianceColor == AllianceColors.RED) ? GOAL_RED_Y : GOAL_BLUE_Y;
        
        double dx = goalX - currentPose.position.x;
        double dy = goalY - currentPose.position.y;
        
        return Math.sqrt(dx * dx + dy * dy);
    }

    //0 means no idea. 1 means yes, -1 means no.
    private int isRobotToLeftOfCenterLine() {
        //center line coordinates
        int x1 = 72;
        int y1 = -72;
        int x2 = -72;
        int y2 = 72;

        double currX = 0;
        double currY = 0;

//        Log.i("== LAUNCH SYSTEM ==", "Alliance color: " + allianceColor);

        if (allianceColor == AllianceColors.BLUE) {
            y1 = 72;
            y2 = -72;
        }

        //get the robot pose from limelight
        Pose3D limelightBasedPosition = limelightAprilTagHelper.getRobotPoseFromAprilTags();

        if (limelightBasedPosition == null)
            return 0;

        currX = limelightBasedPosition.getPosition().x * 39.37;
        currY = limelightBasedPosition.getPosition().y * 39.37;
//        Log.i("== LAUNCH SYSTEM ==", "Limelight position: X: " + currX + " Y: " + currY + " Yaw: " + limelightBasedPosition.getOrientation().getYaw(AngleUnit.DEGREES));

        double d = (x2 - x1) * (currY - y1) - (y2 - y1) * (currX - x1);

        //if d > 0, point is to the left of the line.
        return (d > 0)? 1 : -1;
    }
}
