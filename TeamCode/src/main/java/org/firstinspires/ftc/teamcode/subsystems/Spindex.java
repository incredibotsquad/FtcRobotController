package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.NullAction;

import org.firstinspires.ftc.teamcode.Actions.SpindexAction;
import org.firstinspires.ftc.teamcode.common.BallEntry;
import org.firstinspires.ftc.teamcode.common.GameColors;
import org.firstinspires.ftc.teamcode.common.RobotHardware;

import java.util.List;
import java.util.stream.Collectors;

public class Spindex {

    public static int DELTA_BETWEEN_POSITIONS = 465;
    public static int SPINDEX_VELOCITY = 1350;

    public static int INTAKE_POS_1 = DELTA_BETWEEN_POSITIONS / 2;
    public static int INTAKE_POS_2 = INTAKE_POS_1 + DELTA_BETWEEN_POSITIONS;
    public static int INTAKE_POS_3 = INTAKE_POS_2 + DELTA_BETWEEN_POSITIONS;

    public static int LAUNCH_POS_1 = INTAKE_POS_3 - (DELTA_BETWEEN_POSITIONS / 2);
    public static int LAUNCH_POS_2 = LAUNCH_POS_1 - DELTA_BETWEEN_POSITIONS;
    public static int LAUNCH_POS_3 = LAUNCH_POS_2 - DELTA_BETWEEN_POSITIONS;

//    public static int INTAKE_POS_1 = 698;
//    public static int INTAKE_POS_3 = INTAKE_POS_1 - DELTA_BETWEEN_POSITIONS;
//    public static int INTAKE_POS_2 = INTAKE_POS_3 - DELTA_BETWEEN_POSITIONS;
//
//    public static int LAUNCH_POS_1 = 0;
//    public static int LAUNCH_POS_2 = LAUNCH_POS_1 + DELTA_BETWEEN_POSITIONS;
//    public static int LAUNCH_POS_3 = LAUNCH_POS_2 + DELTA_BETWEEN_POSITIONS;

    public static int COLOR_DETECTION_POS = LAUNCH_POS_1;

    public List<BallEntry> storedColors = List.of(
            new BallEntry(0, INTAKE_POS_1, LAUNCH_POS_1, GameColors.NONE),
            new BallEntry(1, INTAKE_POS_2, LAUNCH_POS_2, GameColors.NONE),
            new BallEntry(2, INTAKE_POS_3, LAUNCH_POS_3, GameColors.NONE));

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
            double distance = 5000;
            double currPos = robotHardware.getSpindexPosition();

            //this gets the closest full slot to current position
            for (BallEntry entry: list) {
                if (Math.abs(entry.launchPosition - currPos) < distance) {
                    distance = Math.abs(entry.launchPosition - currPos);
                    nextFullSlotIndex = entry.index;
                }
            }
//            nextFullSlotIndex = list.get(0).index;
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


    public Action moveToNextEmptySlotAction() {
        int nextIndex = getNextEmptySlotIndex();
        if (nextIndex < 0) return new NullAction();

        previousIndex = currentIndex;
        currentIndex = nextIndex;

        Log.i("SPINDEXER", "moveToNextEmptySlotAction: current index: " + currentIndex);


        return new SpindexAction(robotHardware, storedColors.get(currentIndex).intakePosition);
    }

    public Action moveToSlotZeroLaunchPosition() {
        previousIndex = currentIndex;
        currentIndex = 0;

        return new SpindexAction(robotHardware, storedColors.get(currentIndex).launchPosition);
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

    public boolean isReadyForIntake() {
        boolean retVal = false;

        int currentPos = robotHardware.getSpindexPosition();
//        Log.i("SPINDEXER", "isReadyForIntake: currentPos: " + currentPos);

        for (BallEntry entry: storedColors) {
//            Log.i("SPINDEXER", "isReadyForIntake: Entry Position: " + entry.intakePosition);

            if (Math.abs(entry.intakePosition - currentPos) < SpindexAction.SPINDEX_POSITION_TOLERANCE && entry.ballColor == GameColors.NONE) {
//                Log.i("SPINDEXER", "isReadyForIntake: we are at an intake position: ");
                retVal = true;
                break;
            }
        }

//        Log.i("SPINDEXER", "isReadyForIntake: " + retVal);

        retVal = retVal && !robotHardware.isSpindexBusy() && !isFull();

//        Log.i("SPINDEXER", "isReadyForIntake: " + retVal);

        //spindex should not be busy moving and should be at an intake position
        return retVal;
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

    public void updateBallColorAtCurrentIndex(GameColors color) {
//        Log.i("SPINDEXER", "STORE CURRENT BALL: " + color);
        storedColors.get(currentIndex).ballColor = color;
    }

    public void updateBallColorAtIndex(int index, GameColors color) {
//        Log.i("SPINDEXER", "STORE CURRENT BALL: " + color);
        storedColors.get(index).ballColor = color;
    }

    public void clearBallAtCurrentIndex() {

//        Log.i("SPINDEXER", "CLEAR CURRENT BALL");
        storedColors.get(currentIndex).ballColor = GameColors.NONE;
    }

    public void clearBallAtIndex(int index) {

//        Log.i("SPINDEXER", "CLEAR BALL AT INDEX");
        storedColors.get(index).ballColor = GameColors.NONE;
    }
}
