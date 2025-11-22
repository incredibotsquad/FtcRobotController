package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Actions.LaunchFlywheelAction;
import org.firstinspires.ftc.teamcode.Actions.LaunchKickAction;
import org.firstinspires.ftc.teamcode.Actions.SpindexAction;
import org.firstinspires.ftc.teamcode.common.AllianceColors;
import org.firstinspires.ftc.teamcode.common.BallEntry;
import org.firstinspires.ftc.teamcode.common.GameColors;
import org.firstinspires.ftc.teamcode.common.GamePattern;
import org.firstinspires.ftc.teamcode.common.LaunchYawDistanceTolerance;
import org.firstinspires.ftc.teamcode.common.LimelightAprilTagHelper;
import org.firstinspires.ftc.teamcode.common.RobotHardware;

import java.util.List;
import java.util.stream.Collectors;

@Config
public class LaunchSystem {

    private RobotHardware robotHardware;
    private  Spindex spindex;

    private LimelightAprilTagHelper limelightAprilTagHelper;
    private AllianceColors allianceColor;
    private ElapsedTime turretAlignmentThrottleTimer;
    public static double FLYWHEEL_POWER_COEFFICIENT_WARM_UP = 1;
    public static double FLYWHEEL_POWER_COEFFICIENT_CLOSE = 0.37;
    public static double FLYWHEEL_POWER_COEFFICIENT_MID = 0.42;
    public static double FLYWHEEL_POWER_COEFFICIENT_FAR = 0.55;
    public static double TURRET_SERVO_MIN_POS = 0;
    public static double TURRET_SERVO_CENTERED = 0.5;
    public static double TURRET_SERVO_MAX_POS = 1;
    public static double TURRET_SERVO_ADJUSTMENT_DELTA = 0.008;
//    public static double TURRET_SERVO_ADJUSTMENT_DELTA = (double)1/300;
    public static double TURRET_ALIGNMENT_THROTTLE_MILLIS = 50;
    public static double TURRET_ALIGNMENT_TOLERANCE_DEGREES = 5;
    public static double ROBOT_NOT_ALIGNED_TO_SHOOT_LIGHT = 0.277;    //RED
    public static double ROBOT_ALIGNED_TO_SHOOT_LIGHT = 0.5;    //GREEN

    public LaunchSystem(RobotHardware robotHardware, Spindex spindex) {
        this.robotHardware = robotHardware;
        this.spindex = spindex;
        this.limelightAprilTagHelper = new LimelightAprilTagHelper(robotHardware);
        this.turretAlignmentThrottleTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        //TODO: remove this - cant move on init
        robotHardware.setLaunchTurretPosition(TURRET_SERVO_CENTERED);
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

    private Action getLaunchBallAction(double flywheelVelocity) {
        Log.i("== LAUNCH SYSTEM ==", "getLaunchBallAction");
        return new SequentialAction(
                new LaunchFlywheelAction(robotHardware, flywheelVelocity),
                new LaunchKickAction(robotHardware)
        );
    }

    public Action getLaunchNextBallAction() {
        Log.i("== LAUNCH SYSTEM ==", "Launch Next Ball Action");
        return getLaunchNextBallAction(getFlywheelVelocityBasedOnDistance());
    }

    public Action getLaunchNextBallAction(double flywheelVelocity) {
        Log.i("== LAUNCH SYSTEM ==", "Launch Next Ball Action");
        return new SequentialAction(
                spindex.moveToNextFullSlotAction(),
                getLaunchBallAction(flywheelVelocity),
                new InstantAction(() -> spindex.clearCurrentBall())
        );
    }

    public Action getLaunchNextBallCloseAction() {
        Log.i("== LAUNCH SYSTEM ==", "Launch Next Ball Close Action");
        return getLaunchNextBallAction(LaunchFlywheelAction.FLYWHEEL_FULL_TICKS_PER_SEC * FLYWHEEL_POWER_COEFFICIENT_CLOSE);
    }

    public Action getLaunchNextBallMidAction() {
        Log.i("== LAUNCH SYSTEM ==", "Launch Next Ball Mid Action");
        return getLaunchNextBallAction(LaunchFlywheelAction.FLYWHEEL_FULL_TICKS_PER_SEC * FLYWHEEL_POWER_COEFFICIENT_MID);
    }

    public Action getLaunchNextBallFarAction() {
        Log.i("== LAUNCH SYSTEM ==", "Launch Next Ball Far Action");
        return getLaunchNextBallAction(LaunchFlywheelAction.FLYWHEEL_FULL_TICKS_PER_SEC * FLYWHEEL_POWER_COEFFICIENT_FAR);
    }

    public Action getLaunchGreenBallAction() {
        Log.i("== LAUNCH SYSTEM ==", "Launch Green Ball Action");
        return getLaunchGreenBallAction(getFlywheelVelocityBasedOnDistance());
    }

    public Action getLaunchGreenBallAction(double flywheelVelocity) {
        Log.i("== LAUNCH SYSTEM ==", "Launch Green Ball Action");
        return  new SequentialAction(
                spindex.moveToNextGreenSlotAction(),
                getLaunchBallAction(flywheelVelocity),
                new InstantAction(() -> spindex.clearCurrentBall())
        );
    }

    public Action getLaunchPurpleBallAction() {
        Log.i("== LAUNCH SYSTEM ==", "Launch Purple Ball Action");
        return getLaunchPurpleBallAction(getFlywheelVelocityBasedOnDistance());
    }

    public Action getLaunchPurpleBallAction(double flywheelVelocity) {
        Log.i("== LAUNCH SYSTEM ==", "Launch Purple Ball Action");
        return new SequentialAction(
                spindex.moveToNextPurpleSlotAction(),
                getLaunchBallAction(flywheelVelocity),
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

        double flywheelVelocity = getFlywheelVelocityBasedOnDistance();
        Action ball1 = new NullAction();
        Action ball2 = new NullAction();
        Action ball3 = new NullAction();

        int greenSlot = spindex.getNextGreenSlotIndex();
        List<BallEntry> purpleSlots = spindex.storedColors.stream().filter(c -> c.index != greenSlot).collect(Collectors.toList());

        if ((greenSlot < 0) || purpleSlots.size() < 2)
            return getLaunchAllBallsAction();

        ball1 = new SequentialAction(
                new SpindexAction(robotHardware, spindex.storedColors.get(greenSlot).launchPosition),
                getLaunchBallAction(flywheelVelocity),
                new InstantAction(() -> spindex.storedColors.get(greenSlot).ballColor = GameColors.NONE)
        );


        ball2 = new SequentialAction(
                new SpindexAction(robotHardware, spindex.storedColors.get(purpleSlots.get(0).index).launchPosition),
                getLaunchBallAction(flywheelVelocity),
                new InstantAction(() -> spindex.storedColors.get(purpleSlots.get(0).index).ballColor = GameColors.NONE));

        if (purpleSlots.size() == 2) {
            ball3 = new SequentialAction(
                    new SpindexAction(robotHardware, spindex.storedColors.get(purpleSlots.get(1).index).launchPosition),
                    getLaunchBallAction(flywheelVelocity),
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

        double flywheelVelocity = getFlywheelVelocityBasedOnDistance();
        Action ball1 = new NullAction();
        Action ball2 = new NullAction();
        Action ball3 = new NullAction();

        int greenSlot = spindex.getNextGreenSlotIndex();
        List<BallEntry> purpleSlots = spindex.storedColors.stream().filter(c -> c.index != greenSlot).collect(Collectors.toList());

        if ((greenSlot < 0) || purpleSlots.size() < 2)
            return getLaunchAllBallsAction();

        ball1 = new SequentialAction(
                new SpindexAction(robotHardware, spindex.storedColors.get(greenSlot).launchPosition),
                getLaunchBallAction(flywheelVelocity),
                new InstantAction(() -> spindex.storedColors.get(greenSlot).ballColor = GameColors.NONE)
        );

        ball2 = new SequentialAction(
                new SpindexAction(robotHardware, spindex.storedColors.get(purpleSlots.get(0).index).launchPosition),
                getLaunchBallAction(flywheelVelocity),
                new InstantAction(() -> spindex.storedColors.get(purpleSlots.get(0).index).ballColor = GameColors.NONE));

        if (purpleSlots.size() == 2) {
            ball3 = new SequentialAction(
                    new SpindexAction(robotHardware, spindex.storedColors.get(purpleSlots.get(1).index).launchPosition),
                    getLaunchBallAction(flywheelVelocity),
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

        double flywheelVelocity = getFlywheelVelocityBasedOnDistance();
        Action ball1 = new NullAction();
        Action ball2 = new NullAction();
        Action ball3 = new NullAction();

        int greenSlot = spindex.getNextGreenSlotIndex();
        List<BallEntry> purpleSlots = spindex.storedColors.stream().filter(c -> c.index != greenSlot).collect(Collectors.toList());

        if ((greenSlot < 0) || purpleSlots.size() < 2)
            return getLaunchAllBallsAction();

        ball1 = new SequentialAction(
                new SpindexAction(robotHardware, spindex.storedColors.get(greenSlot).launchPosition),
                getLaunchBallAction(flywheelVelocity),
                new InstantAction(() -> spindex.storedColors.get(greenSlot).ballColor = GameColors.NONE)
        );

        ball2 = new SequentialAction(
                new SpindexAction(robotHardware, spindex.storedColors.get(purpleSlots.get(0).index).launchPosition),
                getLaunchBallAction(flywheelVelocity),
                new InstantAction(() -> spindex.storedColors.get(purpleSlots.get(0).index).ballColor = GameColors.NONE));

        if (purpleSlots.size() == 2) {
            ball3 = new SequentialAction(
                    new SpindexAction(robotHardware, spindex.storedColors.get(purpleSlots.get(1).index).launchPosition),
                    getLaunchBallAction(flywheelVelocity),
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

        return getLaunchAllBallsAction(getFlywheelVelocityBasedOnDistance());
    }

    public Action getLaunchAllBallsAction(double flywheelVelocity) {
        Log.i("== LAUNCH SYSTEM ==", "Launching " + spindex.fullSlotCount() + " Balls Action: ");

        Action ball1 = new NullAction();
        Action ball2 = new NullAction();
        Action ball3 = new NullAction();

        if (spindex.storedColors.get(2).ballColor != GameColors.NONE){
            Log.i("== LAUNCH SYSTEM ==", "Launch all Balls. Third");
            ball3 = new SequentialAction(
                    new SpindexAction(robotHardware, spindex.storedColors.get(2).launchPosition),
                    getLaunchBallAction(flywheelVelocity),
                    new InstantAction(() -> spindex.storedColors.get(2).ballColor = GameColors.NONE)
            );
        }

        if (spindex.storedColors.get(1).ballColor != GameColors.NONE){
            Log.i("== LAUNCH SYSTEM ==", "Launch all Balls. Second");
            ball2 = new SequentialAction(
                    new SpindexAction(robotHardware, spindex.storedColors.get(1).launchPosition),
                    getLaunchBallAction(flywheelVelocity),
                    new InstantAction(() -> spindex.storedColors.get(1).ballColor = GameColors.NONE)
            );
        }

        if (spindex.storedColors.get(0).ballColor != GameColors.NONE){
            Log.i("== LAUNCH SYSTEM ==", "Launch all Balls. First");

            ball1 = new SequentialAction(
                    new SpindexAction(robotHardware, spindex.storedColors.get(0).launchPosition),
                    getLaunchBallAction(flywheelVelocity),
                    new InstantAction(() -> spindex.storedColors.get(0).ballColor = GameColors.NONE)
            );
        }

        return new SequentialAction(
                ball3,
                ball2,
                ball1
        );
    }

    private double getFlywheelVelocityBasedOnDistance() {
        LaunchYawDistanceTolerance ydt = limelightAprilTagHelper.getGoalYawDistanceToleranceFromCurrentPosition();

        double coefficient = FLYWHEEL_POWER_COEFFICIENT_FAR;

        if (ydt != null) {
            coefficient = Math.max(FLYWHEEL_POWER_COEFFICIENT_WARM_UP,  (0.0019985 * ydt.distance + 0.29006));
            coefficient = Math.floor(coefficient * 100) / 100;

            coefficient = Math.min(coefficient, FLYWHEEL_POWER_COEFFICIENT_FAR);

            Log.i("== LAUNCH SYSTEM ==", "getFlywheelVelocityBasedOnDistance. D: " + ydt.distance + " C: " + coefficient);
        }
        else {
            Log.i("== LAUNCH SYSTEM ==", "getFlywheelVelocityBasedOnDistance. No limeight results. C: " + coefficient);
        }

        return LaunchFlywheelAction.FLYWHEEL_FULL_TICKS_PER_SEC * coefficient;
    }

    public void AlignTurretToGoal() {

        if (turretAlignmentThrottleTimer.milliseconds() < TURRET_ALIGNMENT_THROTTLE_MILLIS)
            return;

        turretAlignmentThrottleTimer.reset();

        //get the yaw from the april tag helper
        LaunchYawDistanceTolerance ydt = limelightAprilTagHelper.getGoalYawDistanceToleranceFromCurrentPosition();

        if (ydt != null) {

        Log.i("== LAUNCH SYSTEM ==", "YAW: " + ydt.yaw);
        Log.i("== LAUNCH SYSTEM ==", "DISTANCE: " + ydt.distance);
        Log.i("== LAUNCH SYSTEM ==", "TOLERANCE: " + ydt.tolerance);

        // A positive yaw means the tag is to the right of the camera's center.
        if (ydt.yaw < 0)
            Log.i("== LAUNCH SYSTEM ==", "YAW LESS THAN ZERO: " + ydt.yaw);
        else
            Log.i("== LAUNCH SYSTEM ==", "YAW MORE THAN ZERO: " + ydt.yaw);


        // move the servo to account for the yaw.
        // Move the servo if the error is outside the tolerance
        if (Math.abs(ydt.yaw) > TURRET_ALIGNMENT_TOLERANCE_DEGREES) {

            //clear out the alignment light
            robotHardware.setAlignmentLightColor(ROBOT_NOT_ALIGNED_TO_SHOOT_LIGHT);

//            double servoDelta = ydt.yaw * TURRET_SERVO_ADJUSTMENT_DELTA;

            double servoDelta = ydt.yaw > 0 ? TURRET_SERVO_ADJUSTMENT_DELTA : -1 * TURRET_SERVO_ADJUSTMENT_DELTA;

            Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal: servoDelta: " + servoDelta);


            // Calculate the new potential servo position and constrain it
//            double newServoPosition = TURRET_SERVO_CENTERED + servoDelta;

            double newServoPosition = robotHardware.getLaunchTurretPosition() + servoDelta;

            newServoPosition = Math.max(TURRET_SERVO_MIN_POS, Math.min(TURRET_SERVO_MAX_POS, newServoPosition));

            Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal: Adjusting... New Position: " + newServoPosition);

            //the cycle might be so fast that the servo is still turning
            //since we are not using an encoder on the turret, the get position will return
            //the last position that the servo was told to go to. So if the calculation is the same,
            //no need to call the command on the servo again - this should prevent jitter.
            if (robotHardware.getLaunchTurretPosition() == newServoPosition)
                return;

            robotHardware.setLaunchTurretPosition(newServoPosition);

        } else {

            robotHardware.setAlignmentLightColor(ROBOT_ALIGNED_TO_SHOOT_LIGHT);
            Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal: Turret Aligned!");

        }
    } else {

            //No tag found - center the turret
//            robotHardware.setLaunchTurretPosition(TURRET_SERVO_CENTERED);
//            robotHardware.setAlignmentLightColor(ROBOT_NOT_ALIGNED_TO_SHOOT_LIGHT);

//            Log.i("== LAUNCH SYSTEM ==", "AlignTurretToGoal: no tag found");
        }
    }
}
