package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;

import org.firstinspires.ftc.teamcode.Actions.LaunchFlywheelAction;
import org.firstinspires.ftc.teamcode.Actions.LaunchGateAction;
import org.firstinspires.ftc.teamcode.Actions.LaunchKickAction;
import org.firstinspires.ftc.teamcode.AllianceColors;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Config
public class LaunchSystem {

    private RobotHardware robotHardware;
    private  Spindex spindex;
    private LimelightAprilTagHelper limelightAprilTagHelper;
    private AllianceColors allianceColor;

    public static double FLYWHEEL_POWER_COEFFICIENT_CLOSE = 0.4625;
    public static double FLYWHEEL_POWER_COEFFICIENT_MID = 0.55;
    public static double FLYWHEEL_POWER_COEFFICIENT_FAR = 0.825;

    public LaunchSystem(RobotHardware robotHardware, Spindex spindex) {
        this.robotHardware = robotHardware;
        this.spindex = spindex;
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
        return new LaunchFlywheelAction(robotHardware, LaunchFlywheelAction.FLYWHEEL_FULL_TICKS_PER_SEC * 0.35);
    }

    public Action getTurnOffAction() {
        Log.i("== LAUNCH SYSTEM ==", "Turned Off");
        return new LaunchFlywheelAction(robotHardware, 0);
    }

    private Action getLaunchBallAction(double flywheelVelocity) {
        Log.i("== LAUNCH SYSTEM ==", "getLaunchBallAction");
        //TODO: determine the flywheel velocity using Limelight here
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
        limelightAprilTagHelper.getGoalYawDistanceToleranceFromCurrentPosition();
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

    public Action getLaunchAllBallsAction() {
        Log.i("== LAUNCH SYSTEM ==", "Launch all Balls Action");

        return getLaunchAllBallsAction(getFlywheelVelocityBasedOnDistance());
    }

    public Action getLaunchAllBallsAction(double flywheelVelocity) {
        Log.i("== LAUNCH SYSTEM ==", "Launch all Balls Action");
        return new SequentialAction(
                getLaunchNextBallAction(flywheelVelocity),
                spindex.fullSlotCount() > 1? getLaunchNextBallAction(flywheelVelocity) : new NullAction(),
                spindex.fullSlotCount() > 2? getLaunchNextBallAction(flywheelVelocity) : new NullAction()
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
        Log.i("== LAUNCH SYSTEM ==", "getFlywheelVelocityBasedOnDistance");

//        double distanceFromGoal = limelightAprilTagHelper.getGoalYawDistanceToleranceFromCurrentPosition();
        return LaunchFlywheelAction.FLYWHEEL_FULL_TICKS_PER_SEC;
    }
}
