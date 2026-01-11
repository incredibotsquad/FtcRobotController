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
    private ElapsedTime flywheelWarmerThrottleTimer;

    public static int TURRET_VELOCITY = 4000;

    public static int TURRET_SERVO_CENTERED = 0;
    public static double TURRET_ALIGNMENT_THROTTLE_MILLIS = 20;
    public static double TURRET_ALIGNMENT_TOLERANCE_DEGREES_NEAR = 2;
    public static double TURRET_ALIGNMENT_TOLERANCE_DEGREES_FAR = 2;
    public static double TURRET_COARSE_TOLERANCE_DEGREES = 12;
    public static double TURRET_FRACTION_OF_DIFFERENCE_TO_COVER = 0.9;
    public static double TURRET_TAG_NOT_FOUND_TIMER_MILLIS = 2000;

    public static double TURRET_POWER_KP = 0.05;
    public static double TURRET_POWER_MIN = 0.12;
    public static double TURRET_POWER_MAX = 0.9;
    public static boolean TURRET_POWER_SOFT_LIMITS_ENABLED = true;


    public static double GOAL_BLUE_X = -56;
    public static double GOAL_BLUE_Y = -56;
    public static double GOAL_RED_X = -56;
    public static double GOAL_RED_Y = 56;

    public static double TURRET_PULSES_PER_REV = 751.8; //PPR at the output shaft depending on the motor that is used.
    public static double TURRET_SCAN_POWER = 0.5;
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

    public static double VISOR_POSITION_FAR_1 = 0.7;
    public static double VISOR_POSITION_FAR_2 = 0.7;
    public static double VISOR_POSITION_FAR_3 = 0.7;



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
        this.flywheelWarmerThrottleTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
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
//            Log.i("== LAUNCH SYSTEM ==", "getRobotLaunchParametersBasedOnDistance: DISTANCE: " + ydt.distance);

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

    public void AlignTurretToGoal() {
        if (turretAlignmentThrottleTimer.milliseconds() < TURRET_ALIGNMENT_THROTTLE_MILLIS) {
            return;
        }
        turretAlignmentThrottleTimer.reset();

        double turretAnglePerPulleyRotation = 360.0 / (123.0 / 24.0);
        double degreesPerTick = turretAnglePerPulleyRotation / TURRET_PULSES_PER_REV;
        int maxTicksBeforeClamp = (int) (90.0 / degreesPerTick);

        LimelightLaunchParameters ydt = limelightAprilTagHelper.getGoalYawDistanceToleranceFromCurrentPosition();

        //tag not found
        if (ydt == null) {

            if (turretTagNotFoundTimer.milliseconds() > TURRET_TAG_NOT_FOUND_TIMER_MILLIS) {

//                Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal : Tag NOT found");

                double goalX = (allianceColor == AllianceColors.RED) ? GOAL_RED_X : GOAL_BLUE_X;
                double goalY = (allianceColor == AllianceColors.RED) ? GOAL_RED_Y : GOAL_BLUE_Y;

//                Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal : Goal X:" + goalX + " Goal Y: " + goalY);

                robotHardware.setAlignmentLightColor(0);    //turn off the alignment light

                double currentTurretRadians = Math.toRadians(robotHardware.getLaunchTurretPosition() * degreesPerTick);

                double desiredTurretDeg = computeTurretDegreesToFaceGoal(CrossOpModeStorage.currentPose, goalX, goalY, currentTurretRadians);

//                Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal : currentTurretDeg:" + Math.toDegrees(currentTurretRadians) + " desiredTurretDeg: " + desiredTurretDeg);

                double errorDeg = normalizeToTurretRange(desiredTurretDeg, degreesPerTick, maxTicksBeforeClamp);

                if (Math.abs(errorDeg) <= TURRET_COARSE_TOLERANCE_DEGREES) {
//                    Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal : tag not found error in tolerance range - stopping turret");
                    robotHardware.setLaunchTurretPower(0);
                    return;
                }

                double power = Math.max(-TURRET_POWER_MAX, Math.min(TURRET_POWER_MAX, TURRET_POWER_KP * errorDeg));
                if (Math.abs(power) < TURRET_POWER_MIN) {
                    power = Math.copySign(TURRET_POWER_MIN, power);
                }

                if (TURRET_POWER_SOFT_LIMITS_ENABLED) {
                    int currentPos = robotHardware.getLaunchTurretPosition();
                    if ((power > 0 && currentPos >= maxTicksBeforeClamp) || (power < 0 && currentPos <= -maxTicksBeforeClamp)) {
                        power = 0;
                    }
                }

//                Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal : setLaunchTurretPower:" + power);

                robotHardware.setLaunchTurretPower(power);
            }
        }
        else {

            turretTagNotFoundTimer.reset();

//            Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal : Tag found");

            double turretTolerance = (ydt.distance > FLYWHEEL_POWER_BUCKET_THRESHOLD_FAR)
                    ? TURRET_ALIGNMENT_TOLERANCE_DEGREES_FAR
                    : TURRET_ALIGNMENT_TOLERANCE_DEGREES_NEAR;

//            Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal : Tag found. Tolerance: " + turretTolerance);

//            int isRobotToLeftOfCenterLine = isRobotToLeftOfCenterLine();
//            if (isRobotToLeftOfCenterLine == 1) {
//                Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal : Tag found - adding left of line bias");
//                ydt.yaw = ydt.yaw - turretTolerance;
//            }
//            if (isRobotToLeftOfCenterLine == -1) {
//                Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal : Tag found - adding right of line bias");
//                ydt.yaw = ydt.yaw + turretTolerance;
//            }

            double difference = Math.abs(ydt.yaw) - turretTolerance;

//            Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal : Tag found - yaw difference: " + difference);

            if (difference <= 0) {
//                Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal : Tag found - yaw difference within tolerance - aligned");
                robotHardware.setAlignmentLightColor(ROBOT_ALIGNED_TO_SHOOT_LIGHT);
                robotHardware.setLaunchTurretPower(0);
                return;
            }

            robotHardware.setAlignmentLightColor(ROBOT_NOT_ALIGNED_TO_SHOOT_LIGHT);

            int differenceToCoverInTicks = (int)((difference * TURRET_FRACTION_OF_DIFFERENCE_TO_COVER) / degreesPerTick);

            int previousPos = robotHardware.getLaunchTurretPosition();

//            Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal: Tag found: old turret position: " + previousPos);

            if (ydt.yaw < 0)
                differenceToCoverInTicks = -1 * differenceToCoverInTicks;

            int newMotorPosition = previousPos + differenceToCoverInTicks;

//            Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal: Tag found: new turret position: " + newMotorPosition);

            if (newMotorPosition < 0)
                newMotorPosition = Math.max( -1 * maxTicksBeforeClamp, newMotorPosition);
            else
                newMotorPosition = Math.min(maxTicksBeforeClamp, newMotorPosition);

//            Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal: Tag found: clamped motor position: " + newMotorPosition);

            robotHardware.setLaunchTurretPosition(newMotorPosition);
        }

        if (TURRET_POWER_SOFT_LIMITS_ENABLED) {
            int currentPos = robotHardware.getLaunchTurretPosition();
            double power = robotHardware.getLaunchTurretPower();
            if ((power > 0 && currentPos >= maxTicksBeforeClamp) || (power < 0 && currentPos <= -maxTicksBeforeClamp)) {
//                Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal: stopping motor since it reached clamp");

                robotHardware.setLaunchTurretPower(0);
            }
        }
    }

    public double computeTurretDegreesToFaceGoal(Pose2d robotPose, double goalX, double goalY, double currentTurretRadians) {
        double heading = robotPose.heading.toDouble();

        double deltaX = goalX - robotPose.position.x;
        double deltaY = goalY - robotPose.position.y;

//        Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal : robot pose: X: " + robotPose.position.x + " Y: " + robotPose.position.y + " Heading:" + Math.toDegrees(heading));
//        Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal : current Turret Degrees: " + Math.toDegrees(currentTurretRadians));

        double targetRadiansRelativeToRobotPosition = Math.atan2(deltaY, deltaX);

//        Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal : target Angle Relative To Robot X Y only: " + Math.toDegrees(targetRadiansRelativeToRobotPosition));

        double turretHeadingInFieldSpace = heading - currentTurretRadians;

//        Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal : turret heading in field space: " + Math.toDegrees(turretHeadingInFieldSpace));

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

    private static double normalizeDeg180(double deg) {
        double out = deg % 360.0;
        if (out > 180.0) out -= 360.0;
        if (out < -180.0) out += 360.0;
        return out;
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
