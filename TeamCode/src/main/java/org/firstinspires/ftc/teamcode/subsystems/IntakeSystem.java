package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Actions.IntakeWheelsAction;
import org.firstinspires.ftc.teamcode.Actions.SpindexAction;
import org.firstinspires.ftc.teamcode.common.BallEntry;
import org.firstinspires.ftc.teamcode.common.GameColors;
import org.firstinspires.ftc.teamcode.common.RobotHardware;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

@Config
public class IntakeSystem {
    public static double INTAKE_THROTTLE_TIME_MS = 900;
    public static double INTAKE_DELAY_TO_LET_BALL_SETTLE_SECS = 0;
    private RobotHardware robotHardware;
    private Spindex spindex;
    public boolean isOn;
    private ElapsedTime timeSinceLastIntake;

    public IntakeSystem(RobotHardware robotHardware, Spindex spindex) {
        this.robotHardware = robotHardware;
        this.spindex = spindex;
        this.isOn = false;
        timeSinceLastIntake = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    //make the default constructor private
    private IntakeSystem() {}

    public Action getTurnOnAction() {
        isOn = true;
        return new ParallelAction(
                new IntakeWheelsAction(robotHardware, true),
                spindex.moveToNextEmptySlotAction()
        );
    }

    public Action getTurnOffAction() {
        isOn = false;
        return new IntakeWheelsAction(robotHardware, false);
    }

    public Action getReverseIntakeAction() {

        return new SequentialAction(
                getTurnOffAction(),
                new IntakeWheelsAction(robotHardware, true, -1),
                new SleepAction(0.5),
                getTurnOffAction(),
                isOn ? getTurnOnAction() : new NullAction()
        );
    }

    public Action checkForBallIntakeAndGetAction() {
        //we dont want this to be every loop
        if (timeSinceLastIntake.milliseconds() < INTAKE_THROTTLE_TIME_MS)
            return new NullAction();

        timeSinceLastIntake.reset();
//
//        if (robotHardware.isSpindexMoving) {
//            Log.i("INTAKE SYSTEM: ", "checkForBallIntakeAndGetAction: SKIPPED SINCE SPINDEX MOVING ");
//            return new NullAction();
//        }

        //check color sensors and if there is a ball there, there are things to do
        //else null action

        boolean ballDetected = robotHardware.didBallDetectionBeamBreak();

        if (isOn && ballDetected) {
            spindex.storeCurrentBall(GameColors.UNKNOWN);   //default to unknown - we will update color later
            Log.i("INTAKE SYSTEM", "checkForBallIntakeAndGetAction: Ball Detected: indexed as UNKNOWN ");
            return new SequentialAction(
                    spindex.moveToNextEmptySlotAction(),
                    new SleepAction(INTAKE_THROTTLE_TIME_MS/1000),   //wait for the ball to stabilize
                    spindex.updateBallColorForPreviousIndex()
            );
        }

        return new NullAction();
    }

    public Action indexAnyUnknowns() {
        List<Action> actionsToRun = new ArrayList<>();

        //find all the unknown balls
        List<BallEntry> unknownList = spindex.storedColors.stream()
                .filter(entry -> entry.ballColor == GameColors.UNKNOWN)
                .collect(Collectors.toList());

        Log.i("INTAKE SYSTEM", "INDEXING ANY UNKNOWNS. FOUND: " + unknownList.size());

        for (BallEntry entry: unknownList) {

            Log.i("INTAKE SYSTEM", "UNKNOWN FOUND AT: " + entry.index);

            Action ballAction = new SequentialAction(
                    new SpindexAction(robotHardware, entry.colorDetectionPosition),
                    new SleepAction(INTAKE_THROTTLE_TIME_MS/1000),   //wait for the ball to stabilize
                    new InstantAction(() -> spindex.storedColors.get(entry.index).ballColor = robotHardware.getDetectedBallColor())
            );

            actionsToRun.add(ballAction);
        }

        return new SequentialAction(actionsToRun);
    }

    public Action reIndexBalls() {
        Log.i("INTAKE SYSTEM", "REINDEXING");

        Action ball1 = new SequentialAction(
                new SpindexAction(robotHardware, spindex.storedColors.get(0).intakePosition),
                new SleepAction(INTAKE_THROTTLE_TIME_MS/900),   //wait for the ball to stabilize
                new InstantAction(() -> spindex.storedColors.get(2).ballColor = robotHardware.getDetectedBallColor())
        );

        Action ball2 = new SequentialAction(
                new SpindexAction(robotHardware, spindex.storedColors.get(1).intakePosition),
                new SleepAction(INTAKE_THROTTLE_TIME_MS/900),   //wait for the ball to stabilize
                new InstantAction(() -> spindex.storedColors.get(0).ballColor = robotHardware.getDetectedBallColor())
        );

        Action ball3 = new SequentialAction(
                new SpindexAction(robotHardware, spindex.storedColors.get(2).intakePosition),
                new SleepAction(INTAKE_THROTTLE_TIME_MS/900),   //wait for the ball to stabilize
                new InstantAction(() -> spindex.storedColors.get(1).ballColor = robotHardware.getDetectedBallColor())
        );

        return new SequentialAction(
                ball1,
                ball2,
                ball3
        );
    }
}
