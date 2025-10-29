package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;

import org.firstinspires.ftc.teamcode.Actions.LaunchFlywheelAction;
import org.firstinspires.ftc.teamcode.Actions.LaunchGateAction;
import org.firstinspires.ftc.teamcode.Actions.LaunchKickAction;
import org.firstinspires.ftc.teamcode.RobotHardware;

public class LaunchSystem {

    private RobotHardware robotHardware;
    private  Spindex spindex;
    public LaunchSystem(RobotHardware robotHardware, Spindex spindex) {
        this.robotHardware = robotHardware;
        this.spindex = spindex;
    }

    //make default constructor private
    private LaunchSystem() {}

    public Action getTurnOnAction() {
        Log.i("== LAUNCH SYSTEM ==", "Turned Off");
        return new LaunchFlywheelAction(robotHardware, LaunchFlywheelAction.FLYWHEEL_FULL_TICKS_PER_SEC);
    }

    public Action getTurnOffAction() {
        Log.i("== LAUNCH SYSTEM ==", "Turned Off");
        return new LaunchFlywheelAction(robotHardware, 0);
    }

    private Action getLaunchBallAction() {
        //TODO: determine the flywheel velocity using Limelight here
        return new SequentialAction(
                new LaunchFlywheelAction(robotHardware, LaunchFlywheelAction.FLYWHEEL_FULL_TICKS_PER_SEC*0.5),
                new LaunchGateAction(robotHardware, true),
                new LaunchKickAction(robotHardware, true),
                new LaunchKickAction(robotHardware, false)
        );
    }

    public Action getLaunchNextBallAction() {
        Log.i("== LAUNCH SYSTEM ==", "Launch Next Ball Action");
        return new SequentialAction(
                spindex.moveToNextFullSlotAction(),
                getLaunchBallAction(),
                new InstantAction(() -> spindex.clearCurrentBall())
        );
    }

    public Action getLaunchGreenBallAction() {
        Log.i("== LAUNCH SYSTEM ==", "Launch Green Ball Action");
        return  new SequentialAction(
                spindex.moveToNextGreenSlotAction(),
                getLaunchBallAction(),
                new InstantAction(() -> spindex.clearCurrentBall())
        );
    }

    public Action getLaunchPurpleBallAction() {
        Log.i("== LAUNCH SYSTEM ==", "Launch Purple Ball Action");
        return new SequentialAction(
                spindex.moveToNextPurpleSlotAction(),
                getLaunchBallAction(),
                new InstantAction(() -> spindex.clearCurrentBall())
        );
    }

    public Action getLaunchAllBallsAction() {
        Log.i("== LAUNCH SYSTEM ==", "Launch all Balls Action");
        return new SequentialAction(
                getLaunchNextBallAction(),
                spindex.fullSlotCount() > 1? getLaunchNextBallAction() : new NullAction(),
                spindex.fullSlotCount() > 2? getLaunchNextBallAction() : new NullAction()
        );
    }

    public Action getLockLauncherForIntakeAction() {
        Log.i("== LAUNCH SYSTEM ==", "Lock Launcher For Intake Action");
        return new ParallelAction(
                new LaunchGateAction(robotHardware, false),
                new LaunchKickAction(robotHardware, false)
        );
    }
}
