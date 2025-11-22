package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Actions.LaunchVisorAction.LAUNCH_VISOR_MID;
import static org.firstinspires.ftc.teamcode.Actions.LaunchVisorAction.LAUNCH_VISOR_RESTING;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Actions.LaunchFlywheelAction;
import org.firstinspires.ftc.teamcode.Actions.LaunchKickAction;
import org.firstinspires.ftc.teamcode.Actions.LaunchVisorAction;
import org.firstinspires.ftc.teamcode.Actions.SpindexAction;
import org.firstinspires.ftc.teamcode.common.AllianceColors;
import org.firstinspires.ftc.teamcode.common.BallEntry;
import org.firstinspires.ftc.teamcode.common.GameColors;
import org.firstinspires.ftc.teamcode.common.GamePattern;
import org.firstinspires.ftc.teamcode.common.LimelightLaunchParameters;
import org.firstinspires.ftc.teamcode.common.LimelightAprilTagHelper;
import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.common.RobotLaunchParameters;

import java.util.List;
import java.util.stream.Collectors;

@Config
public class LaunchSystem {

    private RobotHardware robotHardware;
    private  Spindex spindex;

    private LimelightAprilTagHelper limelightAprilTagHelper;
    private AllianceColors allianceColor;
    private ElapsedTime turretAlignmentThrottleTimer;
    private ElapsedTime turretTagNotFoundTimer;

    public static double TURRET_SERVO_MIN_POS = 0.25;
    public static double TURRET_SERVO_CENTERED = 0.5;
    public static double TURRET_SERVO_MAX_POS = 0.75;
    public static double TURRET_SERVO_ADJUSTMENT_DELTA = 0.004;
    public static double TURRET_ALIGNMENT_THROTTLE_MILLIS = 50;
    public static double TURRET_ALIGNMENT_TOLERANCE_DEGREES = 3;
    public static double TURRET_TAG_NOT_FOUND_TIMER_MILLIS = 2000;
    public static double ROBOT_NOT_ALIGNED_TO_SHOOT_LIGHT = 0.277;    //RED
    public static double ROBOT_ALIGNED_TO_SHOOT_LIGHT = 0.5;    //GREEN
    public static double FLYWHEEL_POWER_COEFFICIENT_WARM_UP = 0.5;

    public static double ROBOT_DISTANCE_CLOSE = 60;
    public static double ROBOT_DISTANCE_FAR = 100;

    public static double FLYWHEEL_POWER_COEFFICIENT_CLOSE = 0.4;
    public static double FLYWHEEL_POWER_COEFFICIENT_MID = 0.5;
    public static double FLYWHEEL_POWER_COEFFICIENT_FAR = 0.6;
    public static double FLYWHEEL_POWER_BUCKET_THRESHOLD = 90;
    public static double DEFAULT_FLYWHEEL_POWER_COEFFICIENT = FLYWHEEL_POWER_COEFFICIENT_MID;
    public static double DEFAULT_VISOR_POSITION = LAUNCH_VISOR_MID;


    public LaunchSystem(RobotHardware robotHardware, Spindex spindex) {
        this.robotHardware = robotHardware;
        this.spindex = spindex;
        this.limelightAprilTagHelper = new LimelightAprilTagHelper(robotHardware);
        this.turretAlignmentThrottleTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        this.turretTagNotFoundTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    //make default constructor private
    private LaunchSystem() {}

    public void setAllianceColor(AllianceColors color) {
        this.allianceColor = color;
        limelightAprilTagHelper.setAllianceColor(color);
    }

    public Action getKeepWarmAction() {
        Log.i("== LAUNCH SYSTEM ==", "Keep warm");
        return new LaunchFlywheelAction(robotHardware, LaunchFlywheelAction.FLYWHEEL_FULL_TICKS_PER_SEC * FLYWHEEL_POWER_COEFFICIENT_WARM_UP, false);
    }

    public Action getTurnOffAction() {
        Log.i("== LAUNCH SYSTEM ==", "Turned Off");
        return new LaunchFlywheelAction(robotHardware, 0);
    }

    private Action getLaunchBallAction(RobotLaunchParameters robotLaunchParameters) {
        Log.i("== LAUNCH SYSTEM ==", "getLaunchBallAction");
        return new SequentialAction(
                new LaunchFlywheelAction(robotHardware, robotLaunchParameters.flywheelVelocity),
                new LaunchVisorAction(robotHardware, robotLaunchParameters.visorPosition),
                new LaunchKickAction(robotHardware),
                new LaunchVisorAction(robotHardware, LAUNCH_VISOR_RESTING)
        );
    }

    public Action getLaunchNextBallAction() {
        Log.i("== LAUNCH SYSTEM ==", "Launch Next Ball Action");
        return getLaunchNextBallAction(getRobotLaunchParametersBasedOnDistance());
    }

    public Action getLaunchNextBallAction(RobotLaunchParameters robotLaunchParameters) {
        Log.i("== LAUNCH SYSTEM ==", "Launch Next Ball Action");

        return spindex.isEmpty() ? new NullAction() :
                new SequentialAction(
                spindex.moveToNextFullSlotAction(),
                getLaunchBallAction(robotLaunchParameters),
                new InstantAction(() -> spindex.clearCurrentBall())
        );
    }

    public Action getLaunchNextBallCloseAction() {
        Log.i("== LAUNCH SYSTEM ==", "Launch Next Ball Close Action");
        return getLaunchNextBallAction(new RobotLaunchParameters(LaunchFlywheelAction.FLYWHEEL_FULL_TICKS_PER_SEC * FLYWHEEL_POWER_COEFFICIENT_CLOSE, LaunchVisorAction.LAUNCH_VISOR_RESTING));
    }

    public Action getLaunchNextBallMidAction() {
        Log.i("== LAUNCH SYSTEM ==", "Launch Next Ball Mid Action");
        return getLaunchNextBallAction(new RobotLaunchParameters(LaunchFlywheelAction.FLYWHEEL_FULL_TICKS_PER_SEC * FLYWHEEL_POWER_COEFFICIENT_MID, LAUNCH_VISOR_MID));
    }

    public Action getLaunchNextBallFarAction() {
        Log.i("== LAUNCH SYSTEM ==", "Launch Next Ball Far Action");
        return getLaunchNextBallAction(new RobotLaunchParameters(LaunchFlywheelAction.FLYWHEEL_FULL_TICKS_PER_SEC * FLYWHEEL_POWER_COEFFICIENT_FAR, LaunchVisorAction.LAUNCH_VISOR_MAX));
    }

    public Action getLaunchGreenBallAction() {
        Log.i("== LAUNCH SYSTEM ==", "Launch Green Ball Action");
        return getLaunchGreenBallAction(getRobotLaunchParametersBasedOnDistance());
    }

    public Action getLaunchGreenBallAction(RobotLaunchParameters robotLaunchParameters) {
        Log.i("== LAUNCH SYSTEM ==", "Launch Green Ball Action");
        return  new SequentialAction(
                spindex.moveToNextGreenSlotAction(),
                getLaunchBallAction(robotLaunchParameters),
                new InstantAction(() -> spindex.clearCurrentBall())
        );
    }

    public Action getLaunchPurpleBallAction() {
        Log.i("== LAUNCH SYSTEM ==", "Launch Purple Ball Action");
        return getLaunchPurpleBallAction(getRobotLaunchParametersBasedOnDistance());
    }

    public Action getLaunchPurpleBallAction(RobotLaunchParameters robotLaunchParameters) {
        Log.i("== LAUNCH SYSTEM ==", "Launch Purple Ball Action");
        return new SequentialAction(
                spindex.moveToNextPurpleSlotAction(),
                getLaunchBallAction(robotLaunchParameters),
                new InstantAction(() -> spindex.clearCurrentBall())
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

        if (pattern == null) return getLaunchAllBallsAction();

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

        return getLaunchAllBallsAction();
    }

    public Action getLaunchGPPAction() {

        RobotLaunchParameters robotLaunchParameters = getRobotLaunchParametersBasedOnDistance();
        Action ball1 = new NullAction();
        Action ball2 = new NullAction();
        Action ball3 = new NullAction();

        int greenSlot = spindex.getNextGreenSlotIndex();
        List<BallEntry> purpleSlots = spindex.storedColors.stream().filter(c -> c.index != greenSlot).collect(Collectors.toList());

        if ((greenSlot < 0) || purpleSlots.size() < 2)
            return getLaunchAllBallsAction();

        ball1 = new SequentialAction(
                new SpindexAction(robotHardware, spindex.storedColors.get(greenSlot).launchPosition),
                getLaunchBallAction(robotLaunchParameters),
                new InstantAction(() -> spindex.storedColors.get(greenSlot).ballColor = GameColors.NONE)
        );


        ball2 = new SequentialAction(
                new SpindexAction(robotHardware, spindex.storedColors.get(purpleSlots.get(0).index).launchPosition),
                getLaunchBallAction(robotLaunchParameters),
                new InstantAction(() -> spindex.storedColors.get(purpleSlots.get(0).index).ballColor = GameColors.NONE));

        if (purpleSlots.size() == 2) {
            ball3 = new SequentialAction(
                    new SpindexAction(robotHardware, spindex.storedColors.get(purpleSlots.get(1).index).launchPosition),
                    getLaunchBallAction(robotLaunchParameters),
                    new InstantAction(() -> spindex.storedColors.get(purpleSlots.get(1).index).ballColor = GameColors.NONE)
            );
        }

        return new SequentialAction(
                ball1,
                ball2,
                ball3
        );
    }

    public Action getLaunchPGPAction() {

        RobotLaunchParameters robotLaunchParameters = getRobotLaunchParametersBasedOnDistance();
        Action ball1 = new NullAction();
        Action ball2 = new NullAction();
        Action ball3 = new NullAction();

        int greenSlot = spindex.getNextGreenSlotIndex();
        List<BallEntry> purpleSlots = spindex.storedColors.stream().filter(c -> c.index != greenSlot).collect(Collectors.toList());

        if ((greenSlot < 0) || purpleSlots.size() < 2)
            return getLaunchAllBallsAction();

        ball1 = new SequentialAction(
                new SpindexAction(robotHardware, spindex.storedColors.get(greenSlot).launchPosition),
                getLaunchBallAction(robotLaunchParameters),
                new InstantAction(() -> spindex.storedColors.get(greenSlot).ballColor = GameColors.NONE)
        );

        ball2 = new SequentialAction(
                new SpindexAction(robotHardware, spindex.storedColors.get(purpleSlots.get(0).index).launchPosition),
                getLaunchBallAction(robotLaunchParameters),
                new InstantAction(() -> spindex.storedColors.get(purpleSlots.get(0).index).ballColor = GameColors.NONE));

        if (purpleSlots.size() == 2) {
            ball3 = new SequentialAction(
                    new SpindexAction(robotHardware, spindex.storedColors.get(purpleSlots.get(1).index).launchPosition),
                    getLaunchBallAction(robotLaunchParameters),
                    new InstantAction(() -> spindex.storedColors.get(purpleSlots.get(1).index).ballColor = GameColors.NONE)
            );
        }

        return new SequentialAction(
                ball2,
                ball1,
                ball3
        );
    }

    public Action getLaunchPPGAction() {

        RobotLaunchParameters robotLaunchParameters = getRobotLaunchParametersBasedOnDistance();
        Action ball1 = new NullAction();
        Action ball2 = new NullAction();
        Action ball3 = new NullAction();

        int greenSlot = spindex.getNextGreenSlotIndex();
        List<BallEntry> purpleSlots = spindex.storedColors.stream().filter(c -> c.index != greenSlot).collect(Collectors.toList());

        if ((greenSlot < 0) || purpleSlots.size() < 2)
            return getLaunchAllBallsAction();

        ball1 = new SequentialAction(
                new SpindexAction(robotHardware, spindex.storedColors.get(greenSlot).launchPosition),
                getLaunchBallAction(robotLaunchParameters),
                new InstantAction(() -> spindex.storedColors.get(greenSlot).ballColor = GameColors.NONE)
        );

        ball2 = new SequentialAction(
                new SpindexAction(robotHardware, spindex.storedColors.get(purpleSlots.get(0).index).launchPosition),
                getLaunchBallAction(robotLaunchParameters),
                new InstantAction(() -> spindex.storedColors.get(purpleSlots.get(0).index).ballColor = GameColors.NONE));

        if (purpleSlots.size() == 2) {
            ball3 = new SequentialAction(
                    new SpindexAction(robotHardware, spindex.storedColors.get(purpleSlots.get(1).index).launchPosition),
                    getLaunchBallAction(robotLaunchParameters),
                    new InstantAction(() -> spindex.storedColors.get(purpleSlots.get(1).index).ballColor = GameColors.NONE)
            );
        }

        return new SequentialAction(
                ball2,
                ball3,
                ball1
        );
    }


    public Action getLaunchAllBallsAction() {
        Log.i("== LAUNCH SYSTEM ==", "Launch all Balls Action");

        return getLaunchAllBallsAction(getRobotLaunchParametersBasedOnDistance());
    }

    public Action getLaunchAllBallsAction(RobotLaunchParameters robotLaunchParameters) {
        Log.i("== LAUNCH SYSTEM ==", "Launching " + spindex.fullSlotCount() + " Balls Action: ");

        Action ball1 = new NullAction();
        Action ball2 = new NullAction();
        Action ball3 = new NullAction();

        if (spindex.storedColors.get(2).ballColor != GameColors.NONE){
            Log.i("== LAUNCH SYSTEM ==", "Launch all Balls. Third");
            ball3 = new SequentialAction(
                    new SpindexAction(robotHardware, spindex.storedColors.get(2).launchPosition),
                    getLaunchBallAction(robotLaunchParameters),
                    new InstantAction(() -> spindex.storedColors.get(2).ballColor = GameColors.NONE)
            );
        }

        if (spindex.storedColors.get(1).ballColor != GameColors.NONE){
            Log.i("== LAUNCH SYSTEM ==", "Launch all Balls. Second");
            ball2 = new SequentialAction(
                    new SpindexAction(robotHardware, spindex.storedColors.get(1).launchPosition),
                    getLaunchBallAction(robotLaunchParameters),
                    new InstantAction(() -> spindex.storedColors.get(1).ballColor = GameColors.NONE)
            );
        }

        if (spindex.storedColors.get(0).ballColor != GameColors.NONE){
            Log.i("== LAUNCH SYSTEM ==", "Launch all Balls. First");

            ball1 = new SequentialAction(
                    new SpindexAction(robotHardware, spindex.storedColors.get(0).launchPosition),
                    getLaunchBallAction(robotLaunchParameters),
                    new InstantAction(() -> spindex.storedColors.get(0).ballColor = GameColors.NONE)
            );
        }

        return new SequentialAction(
                ball3,
                ball2,
                ball1
        );
    }

    private RobotLaunchParameters getRobotLaunchParametersBasedOnDistance() {
        LimelightLaunchParameters ydt = limelightAprilTagHelper.getGoalYawDistanceToleranceFromCurrentPosition();

        double flywheelPowerCoefficient = DEFAULT_FLYWHEEL_POWER_COEFFICIENT;
        double visorPosition = DEFAULT_VISOR_POSITION;

        if (ydt != null) {
//            flywheelPowerCoefficient = Math.max(FLYWHEEL_POWER_COEFFICIENT_WARM_UP,  (0.0019985 * ydt.distance + 0.29006));
//            flywheelPowerCoefficient = Math.floor(flywheelPowerCoefficient * 100) / 100;
//
//            flywheelPowerCoefficient = Math.min(flywheelPowerCoefficient, FLYWHEEL_POWER_COEFFICIENT_FAR);
            if (ydt.distance > FLYWHEEL_POWER_BUCKET_THRESHOLD) {
                flywheelPowerCoefficient = FLYWHEEL_POWER_COEFFICIENT_FAR;
            }

            Log.i("== LAUNCH SYSTEM ==", "getFlywheelVelocityBasedOnDistance. D: " + ydt.distance + " C: " + flywheelPowerCoefficient);
        }
        else {
            Log.i("== LAUNCH SYSTEM ==", "getFlywheelVelocityBasedOnDistance. No limeight results. C: " + flywheelPowerCoefficient);
        }

        //TODO: add code for visor position

        return new RobotLaunchParameters(LaunchFlywheelAction.FLYWHEEL_FULL_TICKS_PER_SEC * flywheelPowerCoefficient, visorPosition);
    }

    public void AlignTurretToGoalAndKeepLauncherWarm() {

        if (turretAlignmentThrottleTimer.milliseconds() < TURRET_ALIGNMENT_THROTTLE_MILLIS)
            return;

        turretAlignmentThrottleTimer.reset();

        //get the yaw from the april tag helper
        LimelightLaunchParameters ydt = limelightAprilTagHelper.getGoalYawDistanceToleranceFromCurrentPosition();

        if (ydt != null) {

            turretTagNotFoundTimer.reset();

            // move the servo to account for the yaw.
            // Move the servo if the error is outside the tolerance
            if (Math.abs(ydt.yaw) > TURRET_ALIGNMENT_TOLERANCE_DEGREES) {

                //clear out the alignment light
                robotHardware.setAlignmentLightColor(ROBOT_NOT_ALIGNED_TO_SHOOT_LIGHT);

    //            double servoDelta = ydt.yaw * TURRET_SERVO_ADJUSTMENT_DELTA;

                double servoDelta = ydt.yaw > 0 ? TURRET_SERVO_ADJUSTMENT_DELTA : -1 * TURRET_SERVO_ADJUSTMENT_DELTA;

//                Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoalAndKeepLauncherWarm: servoDelta: " + servoDelta);


                // Calculate the new potential servo position and constrain it
    //            double newServoPosition = TURRET_SERVO_CENTERED + servoDelta;

                double newServoPosition = robotHardware.getLaunchTurretPosition() + servoDelta;

                newServoPosition = Math.max(TURRET_SERVO_MIN_POS, Math.min(TURRET_SERVO_MAX_POS, newServoPosition));

//                Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoalAndKeepLauncherWarm: Adjusting... New Position: " + newServoPosition);

                //the cycle might be so fast that the servo is still turning
                //since we are not using an encoder on the turret, the get position will return
                //the last position that the servo was told to go to. So if the calculation is the same,
                //no need to call the command on the servo again - this should prevent jitter.
                if (robotHardware.getLaunchTurretPosition() == newServoPosition)
                    return;

                robotHardware.setLaunchTurretPosition(newServoPosition);

            } else {

                robotHardware.setAlignmentLightColor(ROBOT_ALIGNED_TO_SHOOT_LIGHT);
//                Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoalAndKeepLauncherWarm: Turret Aligned!");

            }

            if (ydt.distance < FLYWHEEL_POWER_BUCKET_THRESHOLD) {
                robotHardware.setFlywheelMotorVelocityInTPS(LaunchFlywheelAction.FLYWHEEL_FULL_TICKS_PER_SEC * FLYWHEEL_POWER_COEFFICIENT_MID);
            } else {
                robotHardware.setFlywheelMotorVelocityInTPS(LaunchFlywheelAction.FLYWHEEL_FULL_TICKS_PER_SEC * FLYWHEEL_POWER_COEFFICIENT_FAR);
            }

        } else {

            //No tag found - center the turret
//            robotHardware.setLaunchTurretPosition(TURRET_SERVO_CENTERED);
            robotHardware.setAlignmentLightColor(ROBOT_NOT_ALIGNED_TO_SHOOT_LIGHT);

            if (turretTagNotFoundTimer.milliseconds() > TURRET_TAG_NOT_FOUND_TIMER_MILLIS) {
                robotHardware.setLaunchTurretPosition(TURRET_SERVO_CENTERED);

                robotHardware.setFlywheelMotorVelocityInTPS(LaunchFlywheelAction.FLYWHEEL_FULL_TICKS_PER_SEC * FLYWHEEL_POWER_COEFFICIENT_MID);
            }

//            Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal: no tag found");
        }
    }
}
