package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.SequentialAction;

import org.firstinspires.ftc.teamcode.Actions.SpindexAction;
import org.firstinspires.ftc.teamcode.common.BallEntry;
import org.firstinspires.ftc.teamcode.common.GameColors;
import org.firstinspires.ftc.teamcode.common.RobotHardware;

import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

public class Spindex {
    public static double INTAKE_POS_1 = 0.02;
    public static double INTAKE_POS_2 = 0.39;
    public static double INTAKE_POS_3 = 0.76;

    public static double COLOR_POS_1 = INTAKE_POS_2;
    public static double COLOR_POS_2 = INTAKE_POS_3;
    public static double COLOR_POS_3 = INTAKE_POS_1;

    public static double LAUNCH_POS_1 = 0.58;
    public static double LAUNCH_POS_2 = 0.95;
    public static double LAUNCH_POS_3 = 0.205;
    public List<BallEntry> storedColors = List.of(
            new BallEntry(0, INTAKE_POS_1, COLOR_POS_1, LAUNCH_POS_1, GameColors.NONE),
            new BallEntry(1, INTAKE_POS_2, COLOR_POS_2, LAUNCH_POS_2, GameColors.NONE),
            new BallEntry(2, INTAKE_POS_3, COLOR_POS_3, LAUNCH_POS_3, GameColors.NONE));

    private int previousIndex;
    public int currentIndex;
    private RobotHardware robotHardware;
    public Spindex(RobotHardware robotHardware){
        this.previousIndex = -1;
        this.currentIndex = -1; //to ensure the first move happens
        this.robotHardware = robotHardware;
    }

    private Spindex() {}

    public void initializeWithPPG() {
        storedColors.get(0).ballColor = GameColors.PURPLE;
        storedColors.get(1).ballColor = GameColors.PURPLE;
        storedColors.get(2).ballColor = GameColors.GREEN;
    }

    private int getNextEmptySlotIndex() {
        List<BallEntry> list = storedColors.stream()
                .filter(entry -> entry.ballColor == GameColors.NONE)
                .collect(Collectors.toList());

        int nextEmptySlotIndex = -1;

        if (!list.isEmpty())
            nextEmptySlotIndex = list.get(0).index;

//        Log.i("SPINDEXER", "NEXT EMPTY SLOT INDEX: " + nextEmptySlotIndex);
        return nextEmptySlotIndex;
    }

    private int getNextFullSlotIndex() {
        List<BallEntry> list = storedColors.stream()
                .filter(entry -> entry.ballColor != GameColors.NONE)
                .collect(Collectors.toList());

        int nextFullSlotIndex = -1;

        if (!list.isEmpty()) {
//            double distance = 1;
//            double currPos = robotHardware.getSpindexPosition();
//
//            //this gets the closest full slot to current position
//            for (BallEntry entry: list) {
//                if (Math.abs(entry.launchPosition - currPos) < distance) {
//                    distance = Math.abs(entry.launchPosition - currPos);
//                    nextFullSlotIndex = entry.index;
//                }
//            }

            nextFullSlotIndex = list.get(list.size() - 1).index;
        }

//        Log.i("SPINDEXER", "NEXT FULL SLOT INDEX: " + nextFullSlotIndex);
        return nextFullSlotIndex;
    }

    public int getNextGreenSlotIndex() {
        List<BallEntry> list = storedColors.stream()
                .filter(entry -> entry.ballColor == GameColors.GREEN)
                .collect(Collectors.toList());

        int nextGreenSlotIndex = -1;

        if (!list.isEmpty())
            nextGreenSlotIndex = list.get(0).index;

//        Log.i("SPINDEXER", "NEXT GREEN SLOT INDEX: " + nextGreenSlotIndex);
        return nextGreenSlotIndex;
    }

    public int getNextPurpleSlotIndex() {
        List<BallEntry> list = storedColors.stream()
                .filter(entry -> entry.ballColor == GameColors.PURPLE)
                .collect(Collectors.toList());

        int nextPurpleSlotIndex = -1;

        if (!list.isEmpty())
            nextPurpleSlotIndex = list.get(0).index;

//        Log.i("SPINDEXER", "NEXT GREEN SLOT INDEX: " + nextPurpleSlotIndex);
        return nextPurpleSlotIndex;
    }

    public int getSlotIndexClosestToColorSensor() {
        double pos = robotHardware.getSpindexPositionFromEncoder();

        Log.i("SPINDEXER", "getSlotIndexClosestToColorSensor. Spindex pos: " + pos);


        for (BallEntry entry: storedColors) {
            if (Math.abs(pos - entry.colorDetectionPosition) < SpindexAction.SPINDEX_POSITION_TOLERANCE) {
                return entry.index;
            }
        }

        return -1;
    }

    public Action moveToNextEmptySlotAction() {
        int nextIndex = getNextEmptySlotIndex();
        if (nextIndex < 0) return new NullAction();

        previousIndex = currentIndex;
        currentIndex = nextIndex;

        Log.i("SPINDEXER", "moveToNextEmptySlotAction: current index: " + currentIndex);


        return new SpindexAction(robotHardware, storedColors.get(currentIndex).intakePosition);
    }

    public Action moveToNextFullSlotAction() {
        int nextIndex = getNextFullSlotIndex();
        if (nextIndex < 0 ) return new NullAction();

        previousIndex = currentIndex;
        currentIndex = nextIndex;

        return new SpindexAction(robotHardware, storedColors.get(currentIndex).launchPosition);
    }

    public Action moveToNextGreenSlotAction() {
        int nextIndex = getNextGreenSlotIndex();
        if (nextIndex < 0) return new NullAction();

        previousIndex = currentIndex;
        currentIndex = nextIndex;
        return new SpindexAction(robotHardware, storedColors.get(currentIndex).launchPosition);
    }

    public Action moveToNextPurpleSlotAction() {
        int nextIndex = getNextPurpleSlotIndex();
        if (nextIndex < 0) return new NullAction();

        previousIndex = currentIndex;
        currentIndex = nextIndex;

        return new SpindexAction(robotHardware, storedColors.get(currentIndex).launchPosition);
    }

    public boolean isFull() {
        return storedColors.stream().noneMatch(ballEntry -> ballEntry.ballColor == GameColors.NONE);
    }

    public int fullSlotCount() {
        return (int) storedColors.stream().filter(ballEntry -> ballEntry.ballColor != GameColors.NONE).count();
    }

    public boolean isEmpty() {
        return storedColors.stream().noneMatch(ballEntry -> ballEntry.ballColor != GameColors.NONE);
    }

    public void storeCurrentBall(GameColors color) {
//        Log.i("SPINDEXER", "STORE CURRENT BALL: " + color);
        storedColors.get(currentIndex).ballColor = color;
    }

    public void clearCurrentBall() {

        Log.i("SPINDEXER", "CLEAR CURRENT BALL");
        storedColors.get(currentIndex).ballColor = GameColors.NONE;
    }

    public Action updateBallColorForPreviousIndex() {
        //get the ball with color position as the current spindexer position

        AtomicInteger indexToUpdate = new AtomicInteger(-1);
        AtomicReference<GameColors> color = new AtomicReference<>(GameColors.UNKNOWN);

        Action returnAction;

        returnAction = new SequentialAction(
//                new InstantAction(() -> Log.i("SPINDEXER", "updateBallColorForPreviousIndex: indexToUpdate before: " + indexToUpdate.get())),
                new InstantAction(() -> indexToUpdate.set(getSlotIndexClosestToColorSensor())),
//                new InstantAction(() -> Log.i("SPINDEXER", "updateBallColorForPreviousIndex: indexToUpdate after: " + indexToUpdate.get())),
//                new InstantAction(() -> Log.i("SPINDEXER", "updateBallColorForPreviousIndex: Existing Ball Color: " + storedColors.get(indexToUpdate.get()).ballColor)),
                new InstantAction(() -> color.set(robotHardware.getDetectedBallColor())),
//                new InstantAction(() -> Log.i("SPINDEXER", "updateBallColorForPreviousIndex: color: " + color.get())),
                new InstantAction(() -> {
//                    Log.i("SPINDEXER", "updateBallColorForPreviousIndex: indexToUpdate after: " + indexToUpdate.get());
                    if(indexToUpdate.get() >= 0 && storedColors.get(indexToUpdate.get()).ballColor == GameColors.UNKNOWN) {
                        storedColors.get(indexToUpdate.get()).ballColor = color.get();
                        Log.i("SPINDEXER", "updateBallColorForPreviousIndex: Updated Ball at index: " + indexToUpdate.get() + " to: " + storedColors.get(indexToUpdate.get()).ballColor);
                    }
                })
        );

        return returnAction;

    }
}
