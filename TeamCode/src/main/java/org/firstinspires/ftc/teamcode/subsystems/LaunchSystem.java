package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Actions.LaunchFlywheelAction.FLYWHEEL_FULL_TICKS_PER_SEC;

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
import org.firstinspires.ftc.teamcode.common.LaunchParametersLookup;
import org.firstinspires.ftc.teamcode.common.LimelightYDT;
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
    private ElapsedTime turretAlignmentThrottleTimer;
    private ElapsedTime turretTagNotFoundTimer;
    public static int TURRET_VELOCITY_COARSE = 4000;
    public static int TURRET_VELOCITY_FINE = 1500;  // Slower velocity for fine adjustments

    public static int TURRET_CENTERED_POSITION = 0;
    public static double TURRET_ALIGNMENT_THROTTLE_MILLIS = 20;
    public static double TURRET_ALIGNMENT_TOLERANCE_DEGREES_NEAR = 1.5;
    public static double TURRET_ALIGNMENT_TOLERANCE_DEGREES_FAR = 3;
    public static double TURRET_COARSE_TOLERANCE_DEGREES = 5;
    public static double TURRET_FRACTION_OF_DIFFERENCE_TO_COVER = 0.7;
    public static double TURRET_TAG_NOT_FOUND_THRESHOLD_MILLIS = 2000;
    public static boolean TURRET_BLENDED_ALIGNMENT = true;
    public static boolean TURRET_COARSE_WITH_POSITION_NOT_POWER = true;

    public static double TURRET_POWER_KP = 0.05;
    public static double TURRET_POWER_MIN = 0.12;
    public static double TURRET_POWER_MAX = 0.9;
    public static boolean TURRET_POWER_SOFT_LIMITS_ENABLED = true;

    public static double GOAL_BLUE_X = -64;
    public static double GOAL_BLUE_Y = -64;
    public static double GOAL_RED_X = -64;
    public static double GOAL_RED_Y = 64;
    public static double GOAL_OBELISK_X = -72;
    public static double GOAL_OBELISK_Y = 0;

    public static double TURRET_PULSES_PER_REV = 751.8; //PPR at the output shaft depending on the motor that is used.
    public static double TURRET_ANGLE_PER_PULLEY_ROTATION = 360.0 / (123.0 / 24.0);
    public static double TURRET_DEGREES_PER_TICK = TURRET_ANGLE_PER_PULLEY_ROTATION / TURRET_PULSES_PER_REV;
    public static int TURRET_CLAMP_DEGREES_IN_EACH_DIRECTION = 90;
    public static int TURRET_CLAMP_DEGREES_IN_EACH_DIRECTION_NARROW = 45;
    public static int TURRET_MAX_TICKS_BEFORE_CLAMP = (int) (TURRET_CLAMP_DEGREES_IN_EACH_DIRECTION / TURRET_DEGREES_PER_TICK);
    public static int TURRET_MAX_TICKS_BEFORE_CLAMP_NARROW = (int) (TURRET_CLAMP_DEGREES_IN_EACH_DIRECTION_NARROW / TURRET_DEGREES_PER_TICK);

    private enum TurretBlendedAlignmentState {
        BLENDED_COARSE,
        BLENDED_FINE,
        BLENDED_ALIGNED
    }

    private TurretBlendedAlignmentState blendedAlignmentState = TurretBlendedAlignmentState.BLENDED_COARSE;
    private Pose2d lastPoseWeTransitionedToFineAlignment = null;

    // ========== NEW ROBUST ALIGNMENT SYSTEM PARAMETERS ==========
    // Thresholds for alignment state machine
    public static double TURRET_LOCK_THRESHOLD_DEGREES = 1.5;      // Below this = locked (aligned)
    public static double TURRET_UNLOCK_THRESHOLD_DEGREES = 3.0;    // Above this = needs realignment (hysteresis)
    
    // Robot movement thresholds to trigger turret realignment
    public static double ROBOT_POSITION_CHANGE_THRESHOLD_INCHES = 2.0;   // Robot position change to unlock
    public static double ROBOT_HEADING_CHANGE_THRESHOLD_DEGREES = 3.0;   // Robot heading change to unlock
    
    // Limelight fine adjustment parameters
    public static double LIMELIGHT_FINE_ADJUST_MAX_DEGREES = 8.0;    // Only use limelight below this error
    public static double TURRET_FINE_TOLERANCE_DEGREES = 2; // Ignore limelight below this (noise)
    public static double LIMELIGHT_WEIGHT = 1.0;  // How much to trust limelight vs odometry (0-1)
    
    // Stability parameters
    public static int ALIGNMENT_STABLE_CYCLES_REQUIRED = 3;  // Cycles aligned before locking
    public static double TURRET_DEADBAND_DEGREES = 0.5;  // Don't move if error is below this

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
    public static double ROBOT_ALIGNMENT_NOT_POSSIBLE_LIGHT = 0;


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
                    FLYWHEEL_FULL_TICKS_PER_SEC * FLYWHEEL_POWER_COEFFICIENT_CLOSE,
                    VISOR_POSITION_CLOSE_1,
                    VISOR_POSITION_CLOSE_2,
                    VISOR_POSITION_CLOSE_3),
            FLYWHEEL_POWER_BUCKET_THRESHOLD_MID, new BallLaunchParameters(
                    FLYWHEEL_POWER_BUCKET_THRESHOLD_MID,
                    FLYWHEEL_FULL_TICKS_PER_SEC * FLYWHEEL_POWER_COEFFICIENT_MID,
                    VISOR_POSITION_MID_1,
                    VISOR_POSITION_MID_2,
                    VISOR_POSITION_MID_3),
            FLYWHEEL_POWER_BUCKET_THRESHOLD_FAR, new BallLaunchParameters(
                    FLYWHEEL_POWER_BUCKET_THRESHOLD_FAR,
                    FLYWHEEL_FULL_TICKS_PER_SEC * FLYWHEEL_POWER_COEFFICIENT_FAR,
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
        Log.i("== LAUNCH SYSTEM ==", "getLaunchNextBallAction: DISTANCE: " + ballLaunchParameters.distance);
        Log.i("== LAUNCH SYSTEM ==", "getLaunchNextBallAction: FLYWHEEL: " + ballLaunchParameters.flywheelVelocity);
        Log.i("== LAUNCH SYSTEM ==", "getLaunchNextBallAction: VISOR 1: " + ballLaunchParameters.visorPositions.get(0) + " VISOR 2: " + ballLaunchParameters.visorPositions.get(1) + " VISOR 3: " + ballLaunchParameters.visorPositions.get(2));

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

        Log.i("== LAUNCH SYSTEM ==", "getLaunchAllBallsInSequenceAction: DISTANCE: " + ballLaunchParameters.distance);
        Log.i("== LAUNCH SYSTEM ==", "getLaunchAllBallsInSequenceAction: FLYWHEEL: " + ballLaunchParameters.flywheelVelocity);
        Log.i("== LAUNCH SYSTEM ==", "getLaunchAllBallsInSequenceAction: VISOR 1: " + ballLaunchParameters.visorPositions.get(0) + " VISOR 2: " + ballLaunchParameters.visorPositions.get(1) + " VISOR 3: " + ballLaunchParameters.visorPositions.get(2));

        //we want to wait for flywheel to ramp up only when launching from far
        boolean waitForFirstFlywheelAndSlideToleranceWindow = false;
//        boolean waitForFirstFlywheelAndSlideToleranceWindow = (ballLaunchParameters.distance > FLYWHEEL_POWER_BUCKET_THRESHOLD_FAR);

        if (waitForFirstFlywheelAndSlideToleranceWindow)
            Log.i("== LAUNCH SYSTEM ==", "getLaunchAllBallsInSequenceAction. Will wait for first flywheel and slide target window");

        List<Action> actionsToRun = new ArrayList<>();

        //flywheel and spindex for first one can be kicked off without waiting for the first time
        actionsToRun.add(
                new ParallelAction(
                        waitForFirstFlywheelAndSlideToleranceWindow ? new LaunchFlywheelAction(robotHardware, ballLaunchParameters.flywheelVelocity, true, true) : new NullAction(),
                        new SpindexAction(robotHardware, spindex.storedColors.get(sequence.get(0)).launchPosition)
                )
        );

        for (int index = 0; index < sequence.size(); index++) {
            BallEntry entry = spindex.storedColors.get(sequence.get(index));
            Log.i("== LAUNCH SYSTEM ==", "getLaunchAllBallsInSequenceAction. entry: " + index + " Color: " + entry.ballColor);

            actionsToRun.add(new SequentialAction(
                    new ParallelAction(
                            new LaunchVisorAction(robotHardware, ballLaunchParameters.visorPositions.get(index)),
                            new SpindexAction(robotHardware, entry.launchPosition)
//                            waitForFirstFlywheelAndSlideToleranceWindow ? new LaunchFlywheelAction(robotHardware, ballLaunchParameters.flywheelVelocity) : new NullAction()
                    ),
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
        LimelightYDT ydt = limelightAprilTagHelper.getGoalYawDistanceToleranceFromCurrentPosition();

        BallLaunchParameters launchParameters = oldLaunchParameters;

        if (ydt != null) {
            launchParameters = LaunchParametersLookup.getBallLaunchParameters(ydt.distance);
            Log.i("== LAUNCH SYSTEM ==", "getRobotLaunchParametersBasedOnDistance: DISTANCE: " + launchParameters.distance);
            Log.i("== LAUNCH SYSTEM ==", "getRobotLaunchParametersBasedOnDistance: FLYWHEEL: " + launchParameters.flywheelVelocity);
            Log.i("== LAUNCH SYSTEM ==", "getRobotLaunchParametersBasedOnDistance: VISOR 1: " + launchParameters.visorPositions.get(0) + " VISOR 2: " + launchParameters.visorPositions.get(1) + " VISOR 3: " + launchParameters.visorPositions.get(2));


//            if (ydt.distance < FLYWHEEL_POWER_BUCKET_THRESHOLD_MID) {
//                launchParameters = new BallLaunchParameters(
//                        FLYWHEEL_POWER_BUCKET_THRESHOLD_CLOSE,
//                        LaunchFlywheelAction.FLYWHEEL_FULL_TICKS_PER_SEC * FLYWHEEL_POWER_COEFFICIENT_CLOSE,
//                        VISOR_POSITION_CLOSE_1,
//                        VISOR_POSITION_CLOSE_2,
//                        VISOR_POSITION_CLOSE_3);
//
//            } else if (ydt.distance < FLYWHEEL_POWER_BUCKET_THRESHOLD_FAR) {
//
//                launchParameters = new BallLaunchParameters(
//                        FLYWHEEL_POWER_BUCKET_THRESHOLD_MID,
//                        LaunchFlywheelAction.FLYWHEEL_FULL_TICKS_PER_SEC * FLYWHEEL_POWER_COEFFICIENT_MID,
//                        VISOR_POSITION_MID_1,
//                        VISOR_POSITION_MID_2,
//                        VISOR_POSITION_MID_3);
//
//            } else {
//
//                launchParameters = new BallLaunchParameters(
//                        FLYWHEEL_POWER_BUCKET_THRESHOLD_FAR,
//                        LaunchFlywheelAction.FLYWHEEL_FULL_TICKS_PER_SEC * FLYWHEEL_POWER_COEFFICIENT_FAR,
//                        VISOR_POSITION_FAR_1,
//                        VISOR_POSITION_FAR_2,
//                        VISOR_POSITION_FAR_3);
//            }
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

        BallLaunchParameters ballLaunchParameters = getRobotLaunchParametersBasedOnDistance();
        double targetVelocity = ballLaunchParameters.flywheelVelocity;

        Log.i("== LAUNCH SYSTEM ==", "WARMING UP FLYWHEEL: CURRENT VELOCITY: " + currentVelocity + " TARGET VELOCITY: " + targetVelocity);
//
//        if (Math.abs(targetVelocity - currentVelocity) > LaunchFlywheelAction.FLYWHEEL_TARGET_VELOCITY_TOLERANCE_TPS) {

        robotHardware.setFlywheelVelocityInTPS(targetVelocity);
//        }
    }

    public void AlignTurretToGoalBlended() {

        if (turretAlignmentThrottleTimer.milliseconds() < TURRET_ALIGNMENT_THROTTLE_MILLIS) {
            return;
        }
        turretAlignmentThrottleTimer.reset();

        double goalX = 0;
        double goalY = 0;

        switch (CrossOpModeStorage.allianceColor) {
            case RED:
                goalX = GOAL_RED_X;
                goalY = GOAL_RED_Y;
                break;
            case BLUE:
                goalX = GOAL_BLUE_X;
                goalY = GOAL_BLUE_Y;
                break;
            case OBELISK:
                goalX = GOAL_OBELISK_X;
                goalY = GOAL_OBELISK_Y;
        }

        if (TURRET_POWER_SOFT_LIMITS_ENABLED) {
            int currentPos = robotHardware.getLaunchTurretPosition();
            double currPower = robotHardware.getLaunchTurretPower();
            if ((currPower > 0 && currentPos >= TURRET_MAX_TICKS_BEFORE_CLAMP) || (currPower < 0 && currentPos <= -TURRET_MAX_TICKS_BEFORE_CLAMP)) {
                robotHardware.setLaunchTurretPower(0);
            }
        }

        Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoalBlended : Goal X:" + goalX + " Goal Y: " + goalY);

        double currentTurretRadians = Math.toRadians(robotHardware.getLaunchTurretPosition() * TURRET_DEGREES_PER_TICK);
        double degreesToMoveToFaceGoal = computeTurretDegreesToFaceGoal(CrossOpModeStorage.currentPose, goalX, goalY, currentTurretRadians);

        Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoalBlended : currentTurretDeg in robot space:" + Math.toDegrees(currentTurretRadians) + " degrees to move: " + degreesToMoveToFaceGoal);

        Double normalizedDegreesToMoveToFaceGoal = normalizeToTurretRange(degreesToMoveToFaceGoal, TURRET_DEGREES_PER_TICK, TURRET_MAX_TICKS_BEFORE_CLAMP);

        LimelightYDT pointOfInterestYDT = limelightAprilTagHelper.getPointOfInterestYawDistanceToleranceFromTag();


        switch (blendedAlignmentState) {
            case BLENDED_COARSE:
                Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoalBlended: current state: " + blendedAlignmentState );

                if (normalizedDegreesToMoveToFaceGoal == null) {
                    Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoalBlended : turret alignment not possible - stopping turret");
                    robotHardware.setLaunchTurretPower(0);
                    robotHardware.setAlignmentLightColor(ROBOT_ALIGNMENT_NOT_POSSIBLE_LIGHT);    //turn off the alignment light
                    return;
                }

                Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoalBlended: current state: " + blendedAlignmentState + " normalizedDegreesToMoveToFaceGoal: " + normalizedDegreesToMoveToFaceGoal);

                robotHardware.setAlignmentLightColor(ROBOT_NOT_ALIGNED_TO_SHOOT_LIGHT);    //turn off the alignment light

                if (Math.abs(normalizedDegreesToMoveToFaceGoal) > TURRET_COARSE_TOLERANCE_DEGREES) {

                    if (TURRET_COARSE_WITH_POSITION_NOT_POWER) {
                        int currentTurretPosition = robotHardware.getLaunchTurretPosition();

                        int differenceToCoverInTicks = (int) ((normalizedDegreesToMoveToFaceGoal) / TURRET_DEGREES_PER_TICK);
                        int newMotorPosition = currentTurretPosition + differenceToCoverInTicks;

                        if (newMotorPosition < 0)
                            newMotorPosition = Math.max(-1 * TURRET_MAX_TICKS_BEFORE_CLAMP, newMotorPosition);
                        else
                            newMotorPosition = Math.min(TURRET_MAX_TICKS_BEFORE_CLAMP, newMotorPosition);

                        Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoalBlended: Coarse yaw error: current position: " + currentTurretPosition + " new turret position: " + newMotorPosition);

                        robotHardware.setLaunchTurretPosition(newMotorPosition);

                    } else {

                        double power = Math.max(-TURRET_POWER_MAX, Math.min(TURRET_POWER_MAX, TURRET_POWER_KP * normalizedDegreesToMoveToFaceGoal));
                        if (Math.abs(power) < TURRET_POWER_MIN) {
                            power = Math.copySign(TURRET_POWER_MIN, power);
                        }
                        robotHardware.setLaunchTurretPower(power);
                        Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoalBlended :Coarse yaw error: setLaunchTurretPower:" + power);
                    }
                    break;
                }

                robotHardware.setLaunchTurretPower(0);
                lastPoseWeTransitionedToFineAlignment = CrossOpModeStorage.currentPose;
                //error less than coarse threshold - update state and fall through
                blendedAlignmentState = TurretBlendedAlignmentState.BLENDED_FINE;

            case BLENDED_FINE:
                Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoalBlended: current state: " + blendedAlignmentState );

                if (TURRET_BLENDED_ALIGNMENT) {

                    //now check if we need to adjust based on limelight

                    if (pointOfInterestYDT == null) {
                        Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoalBlended : No POI yaw found FINE alignment.Moving to COARSE");
                        blendedAlignmentState = TurretBlendedAlignmentState.BLENDED_COARSE;
                        return;
                    }

                    double errorDeg = pointOfInterestYDT.yaw;
                    Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoalBlended : POI yaw: " + errorDeg);

                    if (Math.abs(errorDeg) < TURRET_FINE_TOLERANCE_DEGREES) {
                        Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoalBlended : POI yaw within tolerance of: " + TURRET_FINE_TOLERANCE_DEGREES);

                        //robot is now aligned - copy this into robot pose.
                        Pose3D llPose = limelightAprilTagHelper.getRobotPoseFromAprilTags();
                        if (llPose != null) {
                            double currTurretDegrees = robotHardware.getLaunchTurretPosition() * TURRET_DEGREES_PER_TICK;
                            double robotHeading = llPose.getOrientation().getYaw() + currTurretDegrees;

                            robotHardware.correctedRobotPoseViaLimelight = new Pose2d(Math.floor(llPose.getPosition().x), Math.floor(llPose.getPosition().y), Math.toRadians(Math.floor(robotHeading)));

                            Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoalBlended : correctedRobotPoseViaLimelight x: " + robotHardware.correctedRobotPoseViaLimelight.position.x + " y: " + robotHardware.correctedRobotPoseViaLimelight.position.y + " yaw: " + Math.toDegrees(robotHardware.correctedRobotPoseViaLimelight.heading.toDouble()));
                        }

                        blendedAlignmentState = TurretBlendedAlignmentState.BLENDED_ALIGNED;
                        return;
                    }

                    if (hasRobotMovedSignificantlyForBlended(CrossOpModeStorage.currentPose)) {
                        Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoalBlended: current state: " + blendedAlignmentState + " Robot moved significantly ");
                        Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoalBlended: current state: " + blendedAlignmentState + " new state: " + TurretBlendedAlignmentState.BLENDED_COARSE);
                        blendedAlignmentState = TurretBlendedAlignmentState.BLENDED_COARSE;
                        return;
                    } else {
                        Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoalBlended : POI yaw error: " + errorDeg + " - correcting it.");

                        robotHardware.setAlignmentLightColor(ROBOT_NOT_ALIGNED_TO_SHOOT_LIGHT);    //turn off the alignment light

                        int currentTurretPosition = robotHardware.getLaunchTurretPosition();

//                double turretTolerance = (pointOfInterestYDT.distance > FLYWHEEL_POWER_BUCKET_THRESHOLD_FAR)
//                        ? TURRET_ALIGNMENT_TOLERANCE_DEGREES_FAR
//                        : TURRET_ALIGNMENT_TOLERANCE_DEGREES_NEAR;

                        int differenceToCoverInTicks = (int) ((errorDeg * TURRET_FRACTION_OF_DIFFERENCE_TO_COVER) / TURRET_DEGREES_PER_TICK);
                        int newMotorPosition = currentTurretPosition + differenceToCoverInTicks;

                        if (newMotorPosition < 0)
                            newMotorPosition = Math.max(-1 * TURRET_MAX_TICKS_BEFORE_CLAMP, newMotorPosition);
                        else
                            newMotorPosition = Math.min(TURRET_MAX_TICKS_BEFORE_CLAMP, newMotorPosition);

                        Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoalBlended: POI yaw error: current position: " + currentTurretPosition + " new turret position: " + newMotorPosition);

                        robotHardware.setLaunchTurretPositionAndVelocity(newMotorPosition, TURRET_VELOCITY_FINE);

                        break;
                    }
                }

                //fall through to aligned state
                blendedAlignmentState = TurretBlendedAlignmentState.BLENDED_ALIGNED;

            case BLENDED_ALIGNED:
                Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoalBlended: current state: " + blendedAlignmentState + ". Turret is aligned");

                if (normalizedDegreesToMoveToFaceGoal != null && Math.abs(normalizedDegreesToMoveToFaceGoal) > (2 * TURRET_COARSE_TOLERANCE_DEGREES)) {
                    Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoalBlended: current state: " + blendedAlignmentState + " new state: " + TurretBlendedAlignmentState.BLENDED_COARSE);
                    blendedAlignmentState = TurretBlendedAlignmentState.BLENDED_COARSE;
                }

                if (pointOfInterestYDT != null && Math.abs(pointOfInterestYDT.yaw) > (2 * TURRET_FINE_TOLERANCE_DEGREES) && TURRET_BLENDED_ALIGNMENT) {
                    Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoalBlended: current state: " + blendedAlignmentState + " new state: " + TurretBlendedAlignmentState.BLENDED_FINE);
                    blendedAlignmentState = TurretBlendedAlignmentState.BLENDED_FINE;
                }

                robotHardware.setLaunchTurretPower(0);
                robotHardware.setAlignmentLightColor(ROBOT_ALIGNED_TO_SHOOT_LIGHT);    //turn om the alignment light

                break;
        }
    }

    private boolean hasRobotMovedSignificantlyForBlended(Pose2d currentPose) {
        if (lastPoseWeTransitionedToFineAlignment == null) {
            return true;  // Never locked, consider as "moved"
        }

        // Check position change
        double dx = currentPose.position.x - lastPoseWeTransitionedToFineAlignment.position.x;
        double dy = currentPose.position.y - lastPoseWeTransitionedToFineAlignment.position.y;
        double positionChange = Math.sqrt(dx * dx + dy * dy);

        if (positionChange > ROBOT_POSITION_CHANGE_THRESHOLD_INCHES) {
            return true;
        }

        // Check heading change
        double currentHeadingDeg = Math.toDegrees(currentPose.heading.toDouble());
        double lastHeadingDeg = Math.toDegrees(lastPoseWeTransitionedToFineAlignment.heading.toDouble());
        double headingChange = Math.abs(normalizeAngle(currentHeadingDeg - lastHeadingDeg));

        if (headingChange > ROBOT_HEADING_CHANGE_THRESHOLD_DEGREES) {
            return true;
        }

        return false;
    }

    public void AlignTurretToGoalLimelightOnlyNarrowBand() {
        if (turretAlignmentThrottleTimer.milliseconds() < TURRET_ALIGNMENT_THROTTLE_MILLIS) {
            return;
        }
        turretAlignmentThrottleTimer.reset();

        LimelightYDT ydt = limelightAprilTagHelper.getPointOfInterestYawDistanceToleranceFromTag();

        if (ydt == null) {
            Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoalLimelightOnlyNarrowBand : NO Tag found");

            if (turretTagNotFoundTimer.milliseconds() > TURRET_TAG_NOT_FOUND_THRESHOLD_MILLIS) {
                robotHardware.setLaunchTurretPosition(TURRET_CENTERED_POSITION);
                robotHardware.setAlignmentLightColor(ROBOT_ALIGNMENT_NOT_POSSIBLE_LIGHT);
            }
            else
                robotHardware.setAlignmentLightColor(ROBOT_NOT_ALIGNED_TO_SHOOT_LIGHT);

            return;
        }

        //tag found - reset not found timer
        turretTagNotFoundTimer.reset();

        int currentTurretPosition = robotHardware.getLaunchTurretPosition();
        Pose2d currentPose = CrossOpModeStorage.currentPose;

        Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoalLimelightOnlyNarrowBand : Tag found");

        double turretTolerance = (ydt.distance > FLYWHEEL_POWER_BUCKET_THRESHOLD_FAR)
                ? TURRET_ALIGNMENT_TOLERANCE_DEGREES_FAR
                : TURRET_ALIGNMENT_TOLERANCE_DEGREES_NEAR;

//            Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoalLimelightOnlyNarrowBand : Tag found. Tolerance: " + turretTolerance);

//            int isRobotToLeftOfCenterLine = isRobotToLeftOfCenterLine();
//            if (isRobotToLeftOfCenterLine == 1) {
////                Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoalLimelightOnlyNarrowBand : Tag found - adding left of line bias");
//                ydt.yaw = ydt.yaw - turretTolerance;
//            }
//            if (isRobotToLeftOfCenterLine == -1) {
////                Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoalLimelightOnlyNarrowBand : Tag found - adding right of line bias");
//                ydt.yaw = ydt.yaw + turretTolerance;
//            }

        double error = Math.abs(ydt.yaw) - turretTolerance;

        Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoalLimelightOnlyNarrowBand : Tag found - yaw error: " + error);

        if (error <= 0) {   //we are within tolerance
            Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoalLimelightOnlyNarrowBand : Tag found - yaw error within tolerance - aligned");
            robotHardware.setAlignmentLightColor(ROBOT_ALIGNED_TO_SHOOT_LIGHT);
            robotHardware.setLaunchTurretPower(0);
            return;
        }

        if (currentAlignmentState == TurretAlignmentState.LOCKED) {
            if (error > TURRET_UNLOCK_THRESHOLD_DEGREES || hasRobotMovedSignificantly(currentPose)) {
                // Limelight shows we're misaligned - unlock
                Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoalLimelightOnlyNarrowBand: Limelight shows error " + error + " - unlocking");
                currentAlignmentState = TurretAlignmentState.FINE_ALIGNING;
                alignmentStableCounter = 0;
            } else {
                // Stay locked - don't move turret at all
                robotHardware.setAlignmentLightColor(ROBOT_ALIGNED_TO_SHOOT_LIGHT);
                // Maintain position by setting to last locked position
                robotHardware.setLaunchTurretPositionAndVelocity(lastLockedTurretPosition, TURRET_VELOCITY_FINE);
                return;
            }
        }

        if (error <= TURRET_LOCK_THRESHOLD_DEGREES) {
            alignmentStableCounter++;
            if (alignmentStableCounter >= ALIGNMENT_STABLE_CYCLES_REQUIRED) {
                // Stable for enough cycles - lock the turret
                currentAlignmentState = TurretAlignmentState.LOCKED;
                lastLockedPose = currentPose;
                lastLockedTurretPosition = currentTurretPosition;
                robotHardware.setAlignmentLightColor(ROBOT_ALIGNED_TO_SHOOT_LIGHT);
                Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoalLimelightOnlyNarrowBand: LOCKED at position " + lastLockedTurretPosition);
                return;
            }
        } else {
            alignmentStableCounter = 0;  // Reset counter if error increased
        }

        robotHardware.setAlignmentLightColor(ROBOT_NOT_ALIGNED_TO_SHOOT_LIGHT);

        int differenceToCoverInTicks = (int)((error * TURRET_FRACTION_OF_DIFFERENCE_TO_COVER) / TURRET_DEGREES_PER_TICK);

        Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoalLimelightOnlyNarrowBand: Tag found: old turret position: " + currentTurretPosition);

        if (ydt.yaw < 0)
            differenceToCoverInTicks = -1 * differenceToCoverInTicks;

        int newMotorPosition = currentTurretPosition + differenceToCoverInTicks;

        Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoalLimelightOnlyNarrowBand: Tag found: new turret position: " + newMotorPosition);

        if (newMotorPosition < 0)
            newMotorPosition = Math.max( -1 * TURRET_MAX_TICKS_BEFORE_CLAMP_NARROW, newMotorPosition);
        else
            newMotorPosition = Math.min(TURRET_MAX_TICKS_BEFORE_CLAMP_NARROW, newMotorPosition);

        Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoalLimelightOnlyNarrowBand: Tag found: clamped motor position: " + newMotorPosition);

        robotHardware.setLaunchTurretPosition(newMotorPosition);

        if (TURRET_POWER_SOFT_LIMITS_ENABLED) {
            int currentPos = robotHardware.getLaunchTurretPosition();
            double power = robotHardware.getLaunchTurretPower();
            if ((power > 0 && currentPos >= TURRET_MAX_TICKS_BEFORE_CLAMP_NARROW) || (power < 0 && currentPos <= -TURRET_MAX_TICKS_BEFORE_CLAMP_NARROW)) {
//                Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoalLimelightOnlyNarrowBand: stopping motor since it reached clamp");

                robotHardware.setLaunchTurretPower(0);
            }
        }
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
//        robotHardware.updateLimelightYawDegrees(Math.toDegrees(turretHeadingInFieldSpace));

        Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal : turret heading in field space: " + Math.toDegrees(turretHeadingInFieldSpace));

        double degreesTurretHasToMove = Math.toDegrees(turretHeadingInFieldSpace - targetRadiansRelativeToRobotPosition);
//        Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal : degrees turret has to  move: " + degreesTurretHasToMove);

        return degreesTurretHasToMove;
    }

    private Double normalizeToTurretRange(double degreesTurretHasToMove, double degreesPerTick, double maxTicksBeforeClamp) {
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
        return null; // dont move at all
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

        Log.i("== LAUNCH SYSTEM ==", "AlignTurretRobust: entered function");

        // Get current robot pose from odometry (primary source of truth)
        Pose2d currentPose = CrossOpModeStorage.currentPose;
        
        // Get goal coordinates based on alliance
        double goalX = 0;
        double goalY = 0;

        switch (CrossOpModeStorage.allianceColor) {
            case RED:
                goalX = GOAL_RED_X;
                goalY = GOAL_RED_Y;
                break;
            case BLUE:
                goalX = GOAL_BLUE_X;
                goalY = GOAL_BLUE_Y;
                break;
            case OBELISK:
                goalX = GOAL_OBELISK_X;
                goalY = GOAL_OBELISK_Y;
        }

        // ============ CHECK IF ROBOT HAS MOVED (to unlock turret if needed) ============
        boolean robotHasMoved = hasRobotMovedSignificantly(currentPose);
        
        if (robotHasMoved && currentAlignmentState == TurretAlignmentState.LOCKED) {
            // Robot moved - unlock turret and go back to coarse alignment
            Log.i("== LAUNCH SYSTEM ==", "AlignTurretRobust: Robot moved - moving to coarse alignment");
            currentAlignmentState = TurretAlignmentState.COARSE_ALIGNING;
            alignmentStableCounter = 0;
        }

        // ============ CALCULATE ODOMETRY-BASED TARGET ============
        int currentTurretPosition = robotHardware.getLaunchTurretPosition();
        double currentTurretRadians = Math.toRadians(currentTurretPosition * TURRET_DEGREES_PER_TICK);
        
        // Calculate desired turret angle using odometry
        double odometryTargetDegrees = computeTurretDegreesToFaceGoal(currentPose, goalX, goalY, currentTurretRadians);
        Double normalizedDeg = normalizeToTurretRange(odometryTargetDegrees, TURRET_DEGREES_PER_TICK, TURRET_MAX_TICKS_BEFORE_CLAMP);

        if (normalizedDeg == null) {
            Log.i("== LAUNCH SYSTEM ==", "AlignTurretRobust: CurrentState: " + currentAlignmentState + ": Turret unreachable");
            robotHardware.setAlignmentLightColor(ROBOT_ALIGNMENT_NOT_POSSIBLE_LIGHT);
            return;
        }

        odometryTargetDegrees = normalizedDeg;
        lastOdometryTargetDegrees = odometryTargetDegrees;

        // ============ GET LIMELIGHT DATA (if available) ============
        LimelightYDT limelightData = limelightAprilTagHelper.getGoalYawDistanceToleranceFromCurrentPosition();
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
        int turretVelocity = TURRET_VELOCITY_COARSE;

        switch (currentAlignmentState) {
            case LOCKED:
                // In LOCKED state, only move if:
                // 1. Robot has moved (already handled above - would transition to COARSE)
                // 2. Limelight shows significant error
                
                if (limelightAvailable && Math.abs(limelightAdjustmentDegrees) > TURRET_UNLOCK_THRESHOLD_DEGREES) {
                    // Limelight shows we're misaligned - unlock
                    Log.i("== LAUNCH SYSTEM ==", "AlignTurretRobust: CurrentState: " + currentAlignmentState + ": Limelight shows error " + limelightAdjustmentDegrees + " - moving to fine alignment");
                    currentAlignmentState = TurretAlignmentState.FINE_ALIGNING;
                    alignmentStableCounter = 0;
                } else {
                    // Stay locked - don't move turret at all
                    robotHardware.setAlignmentLightColor(ROBOT_ALIGNED_TO_SHOOT_LIGHT);
                    Log.i("== LAUNCH SYSTEM ==", "AlignTurretRobust: CurrentState: " + currentAlignmentState + ": Limelight error " + limelightAdjustmentDegrees + " within tolerance (" + TURRET_UNLOCK_THRESHOLD_DEGREES + ") No action");

                    // Maintain position by setting to last locked position
//                    robotHardware.setLaunchTurretPositionAndVelocity(lastLockedTurretPosition, TURRET_VELOCITY_FINE);
                    return;
                }
                // Fall through to FINE_ALIGNING if we unlocked
                
            case FINE_ALIGNING:
                // Use combination of odometry and limelight
                if (limelightAvailable && Math.abs(limelightAdjustmentDegrees) < LIMELIGHT_FINE_ADJUST_MAX_DEGREES) {
                    // Weighted combination: prefer limelight for small adjustments
                    if (Math.abs(limelightAdjustmentDegrees) > TURRET_FINE_TOLERANCE_DEGREES) {
                        effectiveError = limelightAdjustmentDegrees * LIMELIGHT_WEIGHT 
                                       + odometryTargetDegrees * (1.0 - LIMELIGHT_WEIGHT);
                    } else {
                        // Limelight error is very small - use it directly
                        effectiveError = limelightAdjustmentDegrees;
                    }
                    Log.i("== LAUNCH SYSTEM ==", "AlignTurretRobust: CurrentState: " + currentAlignmentState + ": Limelight error " + limelightAdjustmentDegrees + " within LL adjustment band (" + LIMELIGHT_FINE_ADJUST_MAX_DEGREES + ") Adjusting using Limelight error");

                } else {
                    // No limelight or error too large - use odometry only
                    effectiveError = odometryTargetDegrees;
                    Log.i("== LAUNCH SYSTEM ==", "AlignTurretRobust: CurrentState: " + currentAlignmentState + ": No limelight or error too large - use odometry only");
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
                        Log.i("== LAUNCH SYSTEM ==", "AlignTurretRobust: CurrentState: " + currentAlignmentState + ": LOCKED at position " + lastLockedTurretPosition);
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
                turretVelocity = TURRET_VELOCITY_COARSE;

                Log.i("== LAUNCH SYSTEM ==", "AlignTurretRobust: CurrentState: " + currentAlignmentState + ": effectiveError: " + effectiveError);

                // Check if we're close enough to switch to fine alignment
                if (Math.abs(effectiveError) < LIMELIGHT_FINE_ADJUST_MAX_DEGREES) {
                    currentAlignmentState = TurretAlignmentState.FINE_ALIGNING;
                    alignmentStableCounter = 0;
                    Log.i("== LAUNCH SYSTEM ==", "AlignTurretRobust: CurrentState: " + currentAlignmentState + ": effectiveError: " + effectiveError + " Moving to Fine Aligning");
                }
                break;
        }

        // ============ APPLY DEADBAND ============
        if (Math.abs(effectiveError) < TURRET_DEADBAND_DEGREES) {
            Log.i("== LAUNCH SYSTEM ==", "AlignTurretRobust: CurrentState: " + currentAlignmentState + ": effectiveError: " + effectiveError + " Within deadband (" + TURRET_DEADBAND_DEGREES + ") Turning green and no action");

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
        
        int ticksToMove = (int)((effectiveError * fractionToCover) / TURRET_DEGREES_PER_TICK);
        targetTurretPosition = currentTurretPosition + ticksToMove;

        // Apply soft limits
        if (targetTurretPosition < -TURRET_MAX_TICKS_BEFORE_CLAMP) {
            targetTurretPosition = -TURRET_MAX_TICKS_BEFORE_CLAMP;
        } else if (targetTurretPosition > TURRET_MAX_TICKS_BEFORE_CLAMP) {
            targetTurretPosition = TURRET_MAX_TICKS_BEFORE_CLAMP;
        }

        // Additional soft limit check - don't command movement that would exceed limits
        if (TURRET_POWER_SOFT_LIMITS_ENABLED) {
            if ((ticksToMove > 0 && currentTurretPosition >= TURRET_MAX_TICKS_BEFORE_CLAMP) ||
                (ticksToMove < 0 && currentTurretPosition <= -TURRET_MAX_TICKS_BEFORE_CLAMP)) {
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

        if (CrossOpModeStorage.allianceColor == AllianceColors.BLUE) {
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
