package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
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

    public static double TURRET_SERVO_MIN_POS = 0.2;
    public static int TURRET_SERVO_CENTERED = 0;
    public static double TURRET_SERVO_MAX_POS = 0.8;
    public static double TURRET_SERVO_ADJUSTMENT_DELTA_NEAR = 0.002;
    public static double TURRET_SERVO_ADJUSTMENT_DELTA_FAR = 0.001;
    public static double TURRET_ALIGNMENT_THROTTLE_MILLIS = 20;
    public static double TURRET_SERVO_ADJUSTMENT_SCALE = 0.25;
    public static double TURRET_ALIGNMENT_TOLERANCE_DEGREES_NEAR = 3;
    public static double TURRET_ALIGNMENT_TOLERANCE_DEGREES_FAR = 2;
    public static double TURRET_TAG_NOT_FOUND_TIMER_MILLIS = 4000;


    public static double TURRET_FRACTION_OF_DIFFERENCE_TO_COVER = 0.9;
    public static double TURRET_PULSES_PER_REV = 751.8;

    public static double ROBOT_NOT_ALIGNED_TO_SHOOT_LIGHT = 0.3;    //RED
    public static double ROBOT_ALIGNED_TO_SHOOT_LIGHT = 0.5;    //GREEN


    public static double FLYWHEEL_WARM_THROTTLE_MILLIS = 50;
    public static double FLYWHEEL_POWER_BUCKET_THRESHOLD_LOW = 0;
    public static double FLYWHEEL_POWER_BUCKET_THRESHOLD_MID = 60;
    public static double FLYWHEEL_POWER_BUCKET_THRESHOLD_FAR = 100;

    public static double FLYWHEEL_POWER_COEFFICIENT_CLOSE = 0.43;
    public static double FLYWHEEL_POWER_COEFFICIENT_MID = 0.45;
    public static double FLYWHEEL_POWER_COEFFICIENT_FAR = 0.58;

    public static double DEFAULT_FLYWHEEL_POWER_COEFFICIENT = FLYWHEEL_POWER_COEFFICIENT_MID;

    public static double VISOR_POSITION_CLOSE_1 = 0.15;
    public static double VISOR_POSITION_CLOSE_2 = 0.15;
    public static double VISOR_POSITION_CLOSE_3 = 0.15;

    public static double VISOR_POSITION_MID_1 = 0.24;
    public static double VISOR_POSITION_MID_2 = 0.24;
    public static double VISOR_POSITION_MID_3 = 0.24;

    public static double VISOR_POSITION_FAR_1 = 0.69;
    public static double VISOR_POSITION_FAR_2 = 0.65;
    public static double VISOR_POSITION_FAR_3 = 0.61;
    public static double DEFAULT_VISOR_POSITION = VISOR_POSITION_MID_1;

    BallLaunchParameters oldLaunchParameters;
    Map<Double, BallLaunchParameters> distancePowerVisorMap = Map.of(
            FLYWHEEL_POWER_BUCKET_THRESHOLD_LOW, new BallLaunchParameters(
                    LaunchFlywheelAction.FLYWHEEL_FULL_TICKS_PER_SEC * FLYWHEEL_POWER_COEFFICIENT_CLOSE,
                    VISOR_POSITION_CLOSE_1,
                    VISOR_POSITION_CLOSE_2,
                    VISOR_POSITION_CLOSE_3),
            FLYWHEEL_POWER_BUCKET_THRESHOLD_MID, new BallLaunchParameters(
                    LaunchFlywheelAction.FLYWHEEL_FULL_TICKS_PER_SEC * FLYWHEEL_POWER_COEFFICIENT_MID,
                    VISOR_POSITION_MID_1,
                    VISOR_POSITION_MID_2,
                    VISOR_POSITION_MID_3),
            FLYWHEEL_POWER_BUCKET_THRESHOLD_FAR, new BallLaunchParameters(
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
        return getLaunchNextBallAction(distancePowerVisorMap.get(FLYWHEEL_POWER_BUCKET_THRESHOLD_LOW));
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
        List<Action> actionsToRun = new ArrayList<>();

        //flywheel and spindex for first one can happen in parallel
        actionsToRun.add(
                new ParallelAction(
                        new LaunchFlywheelAction(robotHardware, ballLaunchParameters.flywheelVelocity),
                        new SpindexAction(robotHardware, spindex.storedColors.get(sequence.get(0)).launchPosition)
                )
        );

        for (int index = 0; index < sequence.size(); index++) {
            BallEntry entry = spindex.storedColors.get(sequence.get(index));
            Log.i("== LAUNCH SYSTEM ==", "getLaunchAllBallsInSequenceAction. entry: " + index + " Color: " + entry.ballColor);

            actionsToRun.add(new SequentialAction(
                    new ParallelAction(
                            new LaunchVisorAction(robotHardware, ballLaunchParameters.visorPositions.get(index)),
                            new SpindexAction(robotHardware, entry.launchPosition)),
//                        new InstantAction(() -> Log.i("== LAUNCH SYSTEM ==", "RPM Before kick:" + robotHardware.getFlywheelVelocityInTPS())),
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
                        LaunchFlywheelAction.FLYWHEEL_FULL_TICKS_PER_SEC * FLYWHEEL_POWER_COEFFICIENT_CLOSE,
                        VISOR_POSITION_CLOSE_1,
                        VISOR_POSITION_CLOSE_2,
                        VISOR_POSITION_CLOSE_3);

            } else if (ydt.distance < FLYWHEEL_POWER_BUCKET_THRESHOLD_FAR) {

                launchParameters = new BallLaunchParameters(
                        LaunchFlywheelAction.FLYWHEEL_FULL_TICKS_PER_SEC * FLYWHEEL_POWER_COEFFICIENT_MID,
                        VISOR_POSITION_MID_1,
                        VISOR_POSITION_MID_2,
                        VISOR_POSITION_MID_3);

            } else {

                launchParameters = new BallLaunchParameters(
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
        if (flywheelWarmerThrottleTimer.milliseconds() < FLYWHEEL_WARM_THROTTLE_MILLIS)
            return;

        flywheelWarmerThrottleTimer.reset();

        //call to set if the velocity is more than 10% off
        double currentVelocity = robotHardware.getFlywheelVelocityInTPS();
        double targetVelocity = getRobotLaunchParametersBasedOnDistance().flywheelVelocity;

        if (Math.abs(targetVelocity - currentVelocity) > LaunchFlywheelAction.FLYWHEEL_TARGET_VELOCITY_TOLERANCE_TPS) {

//            Log.i("LAUNCH SYSTEM", "WARMING UP FLYWHEEL: CURRENT VELOCITY: " + currentVelocity + " TARGET VELOCITY: " + targetVelocity);
            robotHardware.setFlywheelVelocityInTPS(targetVelocity);
        }
    }

    public void AlignTurretToGoalNew() {
        /*
         * Turret pulley is 123 teeth. The Servo pulley is 24 tooth.
         * it takes 123/24 = 5.125 turns of the driver unit for a full 360 of the turret
         * So in 1 turn of the driving pulley, the turret would turn 360 / 5.125 = 70.24390244 degrees
         * A 435 motor has 384.5 Ticks Per Rev
         * so in 384.5 ticks, the turret would turn 70.24390244 degrees.
         * so in 1 tick, the turret turns 70.24390244 / 384.5 = 0.18268895 degrees
         * We would cover 350 degrees (wire management) in 350 / 0.18268895 = 1916 ticks
         * half a rotation would be 1916 / 2 = 958 ticks positive or negative to constrain to a 180 degree plane
         * */

        double turretAnglePerPulleyRotation = 360.0 / (123.0 / 24.0);
        double degreesPerTick = turretAnglePerPulleyRotation / TURRET_PULSES_PER_REV;
        int maxTicksBeforeClamp = (int)(180.0 / degreesPerTick);

        LimelightLaunchParameters ydt = limelightAprilTagHelper.getGoalYawDistanceToleranceFromCurrentPosition();
        if (ydt != null) {

            double turretTolerance = TURRET_ALIGNMENT_TOLERANCE_DEGREES_NEAR;

            if (ydt.distance > FLYWHEEL_POWER_BUCKET_THRESHOLD_FAR) {
                turretTolerance = TURRET_ALIGNMENT_TOLERANCE_DEGREES_FAR;
            }

            int isRobotToLeftOfCenterLine = isRobotToLeftOfCenterLine();

            //robot is to left of line - extra bias to stay to the left
            if (isRobotToLeftOfCenterLine == 1) {
                //            Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal: POINT TO LEFT OF LINE: ");
                ydt.yaw = ydt.yaw - turretTolerance;
            }

            //robot is to the right of the line - extra bias to the right
            if (isRobotToLeftOfCenterLine == -1) {
                //            Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal: POINT TO RIGHT OF LINE: ");
                ydt.yaw = ydt.yaw + turretTolerance;
            }

            double difference = Math.abs(ydt.yaw) - turretTolerance;

            if (difference > 0) {
                robotHardware.setAlignmentLightColor(ROBOT_NOT_ALIGNED_TO_SHOOT_LIGHT);

                int differenceToCoverInTicks = (int)((difference * TURRET_FRACTION_OF_DIFFERENCE_TO_COVER) / degreesPerTick);

                int previousPos = robotHardware.getLaunchTurretPosition();

//                Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal: old motor position: " + previousPos);

                if (ydt.yaw < 0)
                    differenceToCoverInTicks = -1 * differenceToCoverInTicks;

                int newMotorPosition = previousPos + differenceToCoverInTicks;

//                Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal: new motor position: " + newMotorPosition);

                if (newMotorPosition < 0)
                    newMotorPosition = Math.max( -1* maxTicksBeforeClamp, newMotorPosition);
                else
                    newMotorPosition = Math.min(maxTicksBeforeClamp, newMotorPosition);

//                Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal: clamped motor position: " + newMotorPosition);

                robotHardware.setLaunchTurretPosition(newMotorPosition);

            }
            else {
                robotHardware.setAlignmentLightColor(ROBOT_ALIGNED_TO_SHOOT_LIGHT);
            }
        }
        else {
            robotHardware.setAlignmentLightColor(ROBOT_NOT_ALIGNED_TO_SHOOT_LIGHT);
        }
    }

    public void AlignTurretToGoal() {

        /*
         * Turret pulley is 123 teeth. The Servo pulley is 24 tooth. it takes 123/24 = 5.125 turns of the servo for a full 360 of the turret
         * The servo is 5 turns so in 5 turns, it will turn the turret 351 degrees.
         * if we divide the servo movement into 1000 steps, each .001 increment of the position will turn the turret .351 degrees
         * We scale the servo adjustment with the yaw difference in order to keep up with the change
         * */

        if (turretAlignmentThrottleTimer.milliseconds() < TURRET_ALIGNMENT_THROTTLE_MILLIS)
            return;

        turretAlignmentThrottleTimer.reset();

    //        Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal: PASSED TIME THRESHOLD");

        //get the yaw from the april tag helper
        LimelightLaunchParameters ydt = limelightAprilTagHelper.getGoalYawDistanceToleranceFromCurrentPosition();

        if (ydt != null) {
    //            Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal: YDT FOUND");
    //        Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal: YAW: " + ydt.yaw);

            turretTagNotFoundTimer.reset();
            double turretTolerance = TURRET_ALIGNMENT_TOLERANCE_DEGREES_NEAR;
            double turretDelta = TURRET_SERVO_ADJUSTMENT_DELTA_NEAR;

            if (ydt.distance > FLYWHEEL_POWER_BUCKET_THRESHOLD_FAR) {
                turretTolerance = TURRET_ALIGNMENT_TOLERANCE_DEGREES_FAR;
                turretDelta = TURRET_SERVO_ADJUSTMENT_DELTA_FAR;
            }

            int isRobotToLeftOfCenterLine = isRobotToLeftOfCenterLine();

            //robot is to left of line - extra bias to stay to the left
            if (isRobotToLeftOfCenterLine == 1) {
    //            Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal: POINT TO LEFT OF LINE: ");

                ydt.yaw = ydt.yaw - turretTolerance;
            }

            //robot is to the right of the line - extra bias to the right
            if (isRobotToLeftOfCenterLine == -1) {
    //            Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal: POINT TO RIGHT OF LINE: ");

                ydt.yaw = ydt.yaw + turretTolerance;
            }

            double difference = Math.abs(ydt.yaw) - turretTolerance;

            // move the servo to account for the yaw.
            // Move the servo if the error is outside the tolerance
            if (difference > 0) {

                //clear out the alignment light
                robotHardware.setAlignmentLightColor(ROBOT_NOT_ALIGNED_TO_SHOOT_LIGHT);

                //scale the delta to 25% of the difference
                double servoDelta = TURRET_SERVO_ADJUSTMENT_SCALE * difference * turretDelta;

                //servoDelta = Math.max(servoDelta, turretDelta); //move at least by turret delta - scaling to a fraction can reduce it.
                servoDelta = Math.min(servoDelta, 5 * turretDelta); //dont move more than 5 times the turret delta

                servoDelta = ydt.yaw > 0 ? servoDelta : -1 * servoDelta;

    //                Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal: servoDelta: " + servoDelta);

                // Calculate the new potential servo position and constrain it
                double newServoPosition = robotHardware.getLaunchTurretPosition() + servoDelta;

    //            Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal: calling to change servo position to: " + newServoPosition);

                newServoPosition = Math.max(TURRET_SERVO_MIN_POS, Math.min(TURRET_SERVO_MAX_POS, newServoPosition));

    //                Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal: Adjusting... New Position: " + newServoPosition);

                //the cycle might be so fast that the servo is still turning
                //since we are not using an encoder on the turret, the get position will return
                //the last position that the servo was told to go to. So if the calculation is the same,
                //no need to call the command on the servo again - this should prevent jitter.
                if (robotHardware.getLaunchTurretPosition() == newServoPosition)
                    return;

    //            Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal: clamped servo position being changed to: " + newServoPosition);
//                robotHardware.setLaunchTurretPosition(newServoPosition);

            } else {
                robotHardware.setAlignmentLightColor(ROBOT_ALIGNED_TO_SHOOT_LIGHT);
    //                Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal: Turret Aligned At: " + robotHardware.getLaunchTurretPosition());
            }

        } else {
            //No tag found - center the turret
    //            robotHardware.setLaunchTurretPosition(TURRET_SERVO_CENTERED);
            robotHardware.setAlignmentLightColor(ROBOT_NOT_ALIGNED_TO_SHOOT_LIGHT);

            if (turretTagNotFoundTimer.milliseconds() > TURRET_TAG_NOT_FOUND_TIMER_MILLIS) {
                robotHardware.setLaunchTurretPosition(TURRET_SERVO_CENTERED);
            }
    //            Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal: no tag found");
        }
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
