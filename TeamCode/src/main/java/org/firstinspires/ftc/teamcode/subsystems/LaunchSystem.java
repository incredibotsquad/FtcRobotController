package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;

import org.firstinspires.ftc.teamcode.Actions.LaunchFlywheelAction;
import org.firstinspires.ftc.teamcode.Actions.LaunchGateAction;
import org.firstinspires.ftc.teamcode.Actions.LaunchKickAction;
import org.firstinspires.ftc.teamcode.Actions.SpindexAction;
import org.firstinspires.ftc.teamcode.AllianceColors;
import org.firstinspires.ftc.teamcode.BallEntry;
import org.firstinspires.ftc.teamcode.GameColors;
import org.firstinspires.ftc.teamcode.GamePattern;
import org.firstinspires.ftc.teamcode.LaunchYawDistanceTolerance;
import org.firstinspires.ftc.teamcode.RobotHardware;

import java.security.PublicKey;
import java.util.List;
import java.util.stream.Collectors;

@Config
public class LaunchSystem {

    private RobotHardware robotHardware;
    private  Spindex spindex;

    private LightSystem lightSystem;
    private LimelightAprilTagHelper limelightAprilTagHelper;
    private AllianceColors allianceColor;

    public static double FLYWHEEL_POWER_COEFFICIENT_WARM_UP = 0.35;
    public static double FLYWHEEL_POWER_COEFFICIENT_CLOSE = 0.37;
    public static double FLYWHEEL_POWER_COEFFICIENT_MID = 0.42;
    public static double FLYWHEEL_POWER_COEFFICIENT_FAR = 0.55;

    public LaunchSystem(RobotHardware robotHardware, Spindex spindex, LightSystem lightSystem) {
        this.robotHardware = robotHardware;
        this.spindex = spindex;
        this.lightSystem = lightSystem;
        this.limelightAprilTagHelper = new LimelightAprilTagHelper(robotHardware);
    }

    //make default constructor private
    private LaunchSystem() {}

    public void setAllianceColor(AllianceColors color) {
        this.allianceColor = color;
        limelightAprilTagHelper.setAllianceColor(color);
    }

    public Action getKeepWarmAction() {
        Log.i("== LAUNCH SYSTEM ==", "Keep warm");
        return new LaunchFlywheelAction(robotHardware, LaunchFlywheelAction.FLYWHEEL_FULL_TICKS_PER_SEC * FLYWHEEL_POWER_COEFFICIENT_WARM_UP);
    }

    public Action getTurnOffAction() {
        Log.i("== LAUNCH SYSTEM ==", "Turned Off");
        return new LaunchFlywheelAction(robotHardware, 0);
    }

    private Action getLaunchBallAction(double flywheelVelocity) {
        Log.i("== LAUNCH SYSTEM ==", "getLaunchBallAction");
        return new SequentialAction(
                new LaunchFlywheelAction(robotHardware, flywheelVelocity),
                new LaunchGateAction(robotHardware, true),
                new LaunchKickAction(robotHardware, true),
                new ParallelAction(
                    new LaunchKickAction(robotHardware, false),
                    new LaunchGateAction(robotHardware, false)
                )
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

    public Action getLockLauncherForIntakeAction() {
        Log.i("== LAUNCH SYSTEM ==", "Lock Launcher For Intake Action");
        return new ParallelAction(
                new LaunchGateAction(robotHardware, false),
                new LaunchKickAction(robotHardware, false)
        );
    }

    private double getFlywheelVelocityBasedOnDistance() {
        LaunchYawDistanceTolerance ydt = limelightAprilTagHelper.getGoalYawDistanceToleranceFromCurrentPosition();

        double coefficient = FLYWHEEL_POWER_COEFFICIENT_CLOSE;

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
}
