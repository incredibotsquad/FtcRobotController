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
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.Actions.IntakeWheelsAction;
import org.firstinspires.ftc.teamcode.Actions.SpindexAction;
import org.firstinspires.ftc.teamcode.common.GameColors;
import org.firstinspires.ftc.teamcode.common.RobotHardware;

@Config
public class IntakeSystem {
    public static double INTAKE_THROTTLE_TIME_MS = 25;
    private RobotHardware robotHardware;
    private Spindex spindex;
    public boolean isOn;
    private ElapsedTime timeSinceLastIntake;

    public static double ZERO_BALL_COLOR = 0;
    public static double ONE_BALL_COLOR = 0.29; //RED
    public static double TWO_BALL_COLOR = 0.388; //YELLOW
    public static double THREE_BALL_COLOR = 0.5; //GREEN

    public IntakeSystem(RobotHardware robotHardware, Spindex spindex) {
        this.robotHardware = robotHardware;
        this.spindex = spindex;
        this.isOn = false;
        timeSinceLastIntake = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
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
                new SleepAction(1.5),
                getTurnOffAction(),
                getTurnOnAction(moveSpindexToEmptySlot)
        );
    }

    public Action getReverseIntakeAction() {
        return getReverseIntakeAction(true);
    }

    public Action checkForBallIntakeAndGetActionTeleop() {
        //we dont want this to be every loop
        if (timeSinceLastIntake.milliseconds() < INTAKE_THROTTLE_TIME_MS)
            return new NullAction();

        timeSinceLastIntake.reset();

        return checkForBallIntakeAndGetAction();
    }

    private Action checkForBallIntakeAndGetAction() {
        Action returnAction = new NullAction();

        if (spindex.isReadyForIntake() && isOn) {
            if(robotHardware.didBallDetectionBeamBreak()) {

                spindex.updateBallColorAtCurrentIndex(GameColors.UNKNOWN);   //default to unknown - we will update color later
                Log.i("INTAKE SYSTEM", "checkForBallIntakeAndGetAction: Ball Detected: indexed as UNKNOWN ");
                Log.i("INTAKE SYSTEM", "checkForBallIntakeAndGetAction: current index: " + spindex.currentIndex);

                if (spindex.isFull()) {
                    returnAction = new SequentialAction(
                            spindex.moveToSlotZeroLaunchPosition(),
                            new ParallelAction(
                                    getReverseIntakeAction(false),
                                    updateBallColorsAction()
                            ),
                            getTurnOffAction()
                    );
                } else {
                    returnAction = spindex.moveToNextEmptySlotAction();
                }
            }
        }

        return returnAction;
    }


    public Action updateBallColorsAction() {
        return new ParallelAction(
                //slot 0 is back color sensor
                new InstantAction(() -> {
                    GameColors slot0Color = robotHardware.getDetectedBallColorFromBackSensor();
                    if (spindex.storedColors.get(0).ballColor == GameColors.UNKNOWN) {
                        Log.i("Intake System: ", "updateBallColors: Slot 0 color: " + slot0Color);
                        spindex.updateBallColorAtIndex(0, slot0Color);
                    }
                }),

                //slot 2 is left color sensor
                new InstantAction(() -> {
                    GameColors slot2Color = robotHardware.getDetectedBallColorFromLeftSensor();
                    if (spindex.storedColors.get(2).ballColor == GameColors.UNKNOWN) {
                        Log.i("Intake System: ", "updateBallColors: Slot 2 color: " + slot2Color);
                        spindex.updateBallColorAtIndex(2, slot2Color);
                    }
                }),

                //slot 1 is right color sensor
                new InstantAction(() -> {
                    GameColors slot1Color = robotHardware.getDetectedBallColorFromRightSensor();
                    if (spindex.storedColors.get(1).ballColor == GameColors.UNKNOWN) {
                        Log.i("Intake System: ", "updateBallColors: Slot 1 color: " + slot1Color);
                        spindex.updateBallColorAtIndex(1, slot1Color);
                    }
                })
        );
    }

    public Action getResetAction() {
        return ReIndexBalls();
    }

    public void updateStatusLight() {
//        Log.i("Intake System: ", "updateStatusLight: Full slots: " + spindex.fullSlotCount());

        double color =  ZERO_BALL_COLOR;

        switch (spindex.fullSlotCount()) {
            case 1:
                color = ONE_BALL_COLOR;
                break;
            case 2:
                color = TWO_BALL_COLOR;
                break;
            case 3:
                color = THREE_BALL_COLOR;
                break;
        }

//        Log.i("Intake System: ", "updateStatusLight: Color: " + color);

        robotHardware.setspindexStatusLightColor(color);
    }
    private Action ReIndexBalls() {
        Log.i("INTAKE SYSTEM", "RE INDEXING");

        //spindex will be in a position where 0 is left, 1 is center, and 2 is right.

        //move to 1 and see if there is a ball there.
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

        //move to 2 and see if there is a ball there.
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
                })
        );

        //move to 3 and see if there is a ball there.
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
                })
        );

        Action colorForAllBallsAction = new SequentialAction(
            new SpindexAction(robotHardware, spindex.storedColors.get(1).launchPosition),
            new SleepAction(0.25),
            updateBallColorsAction()
        );

        return new SequentialAction(
                ball1Action,
                ball2Action,
                ball3Action,
                colorForAllBallsAction);
    }

}
