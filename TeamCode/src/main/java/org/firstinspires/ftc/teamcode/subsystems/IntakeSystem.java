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
    private RobotHardware robotHardware;
    private Spindex spindex;
    public boolean isOn;
    private ElapsedTime timeSinceLastIntake;

    private boolean isUnknownIndexingOngoing;

    public IntakeSystem(RobotHardware robotHardware, Spindex spindex) {
        this.robotHardware = robotHardware;
        this.spindex = spindex;
        this.isOn = false;
        timeSinceLastIntake = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        this.isUnknownIndexingOngoing = false;
    }

    //make the default constructor private
    private IntakeSystem() {}

    public Action getTurnOnAction(boolean moveSpindexToEmptySlot) {
        isOn = true;
        return new ParallelAction(
                new IntakeWheelsAction(robotHardware, true),
                moveSpindexToEmptySlot? spindex.moveToNextEmptySlotAction() : new NullAction()
        );
    }

    public Action getTurnOnAction() {
        return getTurnOnAction(true);
    }

    public Action getTurnOffAction() {
        isOn = false;
        return new IntakeWheelsAction(robotHardware, false);
    }

    public Action getReverseIntakeAction(boolean moveSpindexToEmptySlot) {
        return new SequentialAction(
                getTurnOffAction(),
                new IntakeWheelsAction(robotHardware, true, -1),
                new SleepAction(2),
                getTurnOffAction(),
                getTurnOnAction(moveSpindexToEmptySlot)
        );
    }

    public Action getReverseIntakeAction() {
        return getReverseIntakeAction(true);
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
            //make sure the spindex is not stalled - the current position should be the intake position for the index at hand
            if (Math.abs(robotHardware.getSpindexPositionFromEncoder() - spindex.storedColors.get(spindex.currentIndex).intakePosition) > SpindexAction.SPINDEX_POSITION_TOLERANCE)
                return new NullAction();


            spindex.storeCurrentBall(GameColors.UNKNOWN);   //default to unknown - we will update color later
            Log.i("INTAKE SYSTEM", "checkForBallIntakeAndGetAction: Ball Detected: indexed as UNKNOWN ");
            Log.i("INTAKE SYSTEM", "checkForBallIntakeAndGetAction: current index: " + spindex.currentIndex);

            return new SequentialAction(
                    spindex.moveToNextEmptySlotAction(),
//                    new InstantAction(()->Log.i("INTAKE SYSTEM", "checkForBallIntakeAndGetAction. Spindex after rotating: " + robotHardware.getSpindexPosition())),
//                    new SleepAction(INTAKE_THROTTLE_TIME_MS/1000),   //wait for the ball to stabilize
                    new InstantAction(()->Log.i("INTAKE SYSTEM", "checkForBallIntakeAndGetAction. Spindex after rotating: " + robotHardware.getSpindexPositionFromEncoder())),
                    spindex.updateBallColorForPreviousIndex(),
                    spindex.isFull() ?
                            new SequentialAction(
                                    getTurnOffAction(),
                                    indexAnyUnknowns(),
                                    spindex.moveToNextFullSlotAction()) : new NullAction()
            );
        }

        return new NullAction();
    }

    public Action indexAnyUnknowns() {
        if (isUnknownIndexingOngoing) {
            return new NullAction();
        }

        List<Action> actionsToRun = new ArrayList<>();

        //find all the unknown balls
        List<BallEntry> unknownList = spindex.storedColors.stream()
                .filter(entry -> entry.ballColor == GameColors.UNKNOWN)
                .collect(Collectors.toList());

        Log.i("INTAKE SYSTEM", "INDEXING ANY UNKNOWNS. FOUND: " + unknownList.size());

        Action updateRunState = new InstantAction(() ->
        {
            Log.i("INTAKE SYSTEM", "SETTING LOCAL VARIABLE TO REPRESENT ON GOING INDEXING");
            isUnknownIndexingOngoing = true;
        });
        actionsToRun.add(updateRunState);

        for (BallEntry entry: unknownList) {

            Log.i("INTAKE SYSTEM", "UNKNOWN FOUND AT: " + entry.index);

            Action ballAction = new SequentialAction(
                    new SpindexAction(robotHardware, entry.colorDetectionPosition),
                    new SleepAction(0.25),   //wait for the ball to stabilize
                    new InstantAction(() -> {
                        final int entryIndex = entry.index;
                        spindex.storedColors.get(entryIndex).ballColor = robotHardware.getDetectedBallColor();
                    })
            );

            actionsToRun.add(ballAction);
        }

        Action clearRunState = new InstantAction(() -> {
            Log.i("INTAKE SYSTEM", "SETTING LOCAL VARIABLE TO REPRESENT COMPLETED INDEXING");
            isUnknownIndexingOngoing = false;
        });
        actionsToRun.add(clearRunState);

        return new SequentialAction(actionsToRun);
    }

    public Action ReIndexBalls() {
        Log.i("INTAKE SYSTEM", "RE INDEXING");


        Action ball1Action = new SequentialAction(
                new SpindexAction(robotHardware, spindex.storedColors.get(0).intakePosition),
                new SleepAction(0.25),
                new InstantAction(() -> {
                    if (robotHardware.didBallDetectionBeamBreak()) {
                        spindex.storedColors.get(0).ballColor = GameColors.UNKNOWN;
                        Log.i("INTAKE SYSTEM", "RE INDEXED INDEX 0 AS UNKNOWN");
                    } else {
                        spindex.storedColors.get(0).ballColor = GameColors.NONE;
                        Log.i("INTAKE SYSTEM", "RE INDEXED INDEX 0 AS NONE");
                    }
                })
        );

        Action ball2Action = new SequentialAction(
                new SpindexAction(robotHardware, spindex.storedColors.get(1).intakePosition),
                new SleepAction(0.25),
                new InstantAction(() -> {
                    if (robotHardware.didBallDetectionBeamBreak()) {
                        spindex.storedColors.get(1).ballColor = GameColors.UNKNOWN;
                        Log.i("INTAKE SYSTEM", "RE INDEXED INDEX 1 AS UNKNOWN");
                    } else {
                        spindex.storedColors.get(1).ballColor = GameColors.NONE;
                        Log.i("INTAKE SYSTEM", "RE INDEXED INDEX 1 AS NONE");
                    }
                }),
                new InstantAction(() -> {
                    if (spindex.storedColors.get(0).ballColor == GameColors.UNKNOWN) {
                        GameColors color = robotHardware.getDetectedBallColor();
                        spindex.storedColors.get(0).ballColor = color;
                        Log.i("INTAKE SYSTEM", "RE INDEXED INDEX 0 COLOR AS: " + color);
                    }
                })
        );

        Action ball3Action = new SequentialAction(
                new SpindexAction(robotHardware, spindex.storedColors.get(2).intakePosition),
                new SleepAction(0.25),
                new InstantAction(() -> {
                    if (robotHardware.didBallDetectionBeamBreak()) {
                        Log.i("INTAKE SYSTEM", "RE INDEXED INDEX 2 AS UNKNOWN");
                        spindex.storedColors.get(2).ballColor = GameColors.UNKNOWN;
                    } else {
                        Log.i("INTAKE SYSTEM", "RE INDEXED INDEX 2 AS NONE");
                        spindex.storedColors.get(2).ballColor = GameColors.NONE;
                    }
                }),
                new InstantAction(() -> {
                    if (spindex.storedColors.get(1).ballColor == GameColors.UNKNOWN) {
                        GameColors color = robotHardware.getDetectedBallColor();
                        spindex.storedColors.get(1).ballColor = color;
                        Log.i("INTAKE SYSTEM", "RE INDEXED INDEX 1 COLOR AS: " + color);
                    }
                })
        );

        Action colorForBall3Action = new SequentialAction(
                new SpindexAction(robotHardware, spindex.storedColors.get(0).intakePosition),
                new SleepAction(0.25),
                new InstantAction(() -> {
                    if (spindex.storedColors.get(2).ballColor == GameColors.UNKNOWN) {
                        GameColors color = robotHardware.getDetectedBallColor();
                        spindex.storedColors.get(2).ballColor = color;
                        Log.i("INTAKE SYSTEM", "RE INDEXED INDEX 2 COLOR AS: " + color);
                    }
                })
        );

        return new SequentialAction(
                ball1Action,
                ball2Action,
                ball3Action,
                colorForBall3Action);
    }

}
