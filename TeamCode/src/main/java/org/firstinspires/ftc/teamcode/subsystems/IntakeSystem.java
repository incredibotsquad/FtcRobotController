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
import org.firstinspires.ftc.teamcode.common.GameColors;
import org.firstinspires.ftc.teamcode.common.RobotHardware;

@Config
public class IntakeSystem {
    public static double INTAKE_THROTTLE_TIME_MS = 90;
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


        Action returnAction = new NullAction();
        boolean ballDetected = robotHardware.didBallDetectionBeamBreak();

        if (isOn && ballDetected && spindex.isReadyForIntake()) {

            spindex.storeCurrentBall(GameColors.UNKNOWN);   //default to unknown - we will update color later
            Log.i("INTAKE SYSTEM", "checkForBallIntakeAndGetAction: Ball Detected: indexed as UNKNOWN ");
            Log.i("INTAKE SYSTEM", "checkForBallIntakeAndGetAction: current index: " + spindex.currentIndex);

            if (spindex.isFull()) {
                returnAction = new SequentialAction(
                        spindex.moveToSlotZeroLaunchPosition(),
                        new ParallelAction(
                                getReverseIntakeAction(false),
                                updateBallColors()
                        )
                );
            } else {
                returnAction = spindex.moveToNextEmptySlotAction();
            }
        }

        return returnAction;
    }

    public Action updateBallColors() {
        return new ParallelAction(
                //slot 0 is back color sensor
                new InstantAction(() -> {
                    GameColors slot0Color = robotHardware.getDetectedBallColorFromBackSensor();
                    spindex.storeCurrentBall(slot0Color);
                }),

                //slot 1 is left color sensor
                new InstantAction(() -> {
                    GameColors slot1Color = robotHardware.getDetectedBallColorFromLeftSensor();
                    spindex.storeCurrentBall(slot1Color);
                }),

                //slot 2 is right color sensor
                new InstantAction(() -> {
                    GameColors slot2Color = robotHardware.getDetectedBallColorFromRightSensor();
                    spindex.storeCurrentBall(slot2Color);
                })
        );
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
                        GameColors color = robotHardware.getDetectedBallColorFromLeftSensor();
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
                        GameColors color = robotHardware.getDetectedBallColorFromLeftSensor();
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
                        GameColors color = robotHardware.getDetectedBallColorFromLeftSensor();
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
