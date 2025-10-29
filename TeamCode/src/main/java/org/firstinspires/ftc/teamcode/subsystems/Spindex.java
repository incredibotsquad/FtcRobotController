package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.NullAction;

import org.firstinspires.ftc.teamcode.Actions.SpindexAction;
import org.firstinspires.ftc.teamcode.BallEntry;
import org.firstinspires.ftc.teamcode.GameColors;
import org.firstinspires.ftc.teamcode.RobotHardware;

import java.util.List;
import java.util.stream.Collectors;

public class Spindex {
    public static double INTAKE_POS_1 = 0.2;
    public static double INTAKE_POS_2 = 0.575;
    public static double INTAKE_POS_3 = 0.95;
    public static double LAUNCH_POS_1 = 0;
    public static double LAUNCH_POS_2 = 0.37;
    public static double LAUNCH_POS_3 = 0.74;
    public List<BallEntry> storedColors = List.of(
            new BallEntry(0, INTAKE_POS_1, LAUNCH_POS_1, GameColors.NONE),
            new BallEntry(1, INTAKE_POS_2, LAUNCH_POS_2, GameColors.NONE),
            new BallEntry(2, INTAKE_POS_3, LAUNCH_POS_3, GameColors.NONE));

    private int currentIndex = 0;
    private RobotHardware robotHardware;
    public Spindex(RobotHardware robotHardware){
        this.currentIndex = 0;
        this.robotHardware = robotHardware;
    }

    private Spindex() {}

    private int getNextEmptySlotIndex() {
        List<BallEntry> list = storedColors.stream()
                .filter(entry -> entry.ballColor == GameColors.NONE)
                .collect(Collectors.toList());

        int nextEmptySlotIndex = -1;

        if (!list.isEmpty())
            nextEmptySlotIndex = list.get(0).index;

        Log.i("SPINDEXER", "NEXT EMPTY SLOT INDEX: " + nextEmptySlotIndex);
        return nextEmptySlotIndex;
    }

    private int getNextFullSlotIndex() {
        List<BallEntry> list = storedColors.stream()
                .filter(entry -> entry.ballColor != GameColors.NONE)
                .collect(Collectors.toList());

        int nextFullSlotIndex = -1;

        if (!list.isEmpty())
            nextFullSlotIndex = list.get(0).index;

        Log.i("SPINDEXER", "NEXT FULL SLOT INDEX: " + nextFullSlotIndex);
        return nextFullSlotIndex;
    }

    private int getNextGreenSlotIndex() {
        List<BallEntry> list = storedColors.stream()
                .filter(entry -> entry.ballColor == GameColors.GREEN)
                .collect(Collectors.toList());

        int nextGreenSlotIndex = -1;

        if (!list.isEmpty())
            nextGreenSlotIndex = list.get(0).index;

        Log.i("SPINDEXER", "NEXT GREEN SLOT INDEX: " + nextGreenSlotIndex);
        return nextGreenSlotIndex;
    }

    private int getNextPurpleSlotIndex() {
        List<BallEntry> list = storedColors.stream()
                .filter(entry -> entry.ballColor == GameColors.PURPLE)
                .collect(Collectors.toList());

        int nextPurpleSlotIndex = -1;

        if (!list.isEmpty())
            nextPurpleSlotIndex = list.get(0).index;

        Log.i("SPINDEXER", "NEXT GREEN SLOT INDEX: " + nextPurpleSlotIndex);
        return nextPurpleSlotIndex;
    }

    public Action moveToNextEmptySlotAction() {
        int nextIndex = getNextEmptySlotIndex();
        if (nextIndex < 0) return new NullAction();

        currentIndex = nextIndex;

        return new SpindexAction(robotHardware, storedColors.get(currentIndex).intakePosition);
    }

    public Action moveToNextFullSlotAction() {
        int nextIndex = getNextFullSlotIndex();
        if (nextIndex < 0) return new NullAction();

        currentIndex = nextIndex;

        return new SpindexAction(robotHardware, storedColors.get(currentIndex).launchPosition);
    }

    public Action moveToNextGreenSlotAction() {
        int nextIndex = getNextGreenSlotIndex();
        if (nextIndex < 0) return new NullAction();

        currentIndex = nextIndex;
        return new SpindexAction(robotHardware, storedColors.get(currentIndex).launchPosition);
    }

    public Action moveToNextPurpleSlotAction() {
        int nextIndex = getNextPurpleSlotIndex();
        if (nextIndex < 0) return new NullAction();

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
        Log.i("SPINDEXER", "STORE CURRENT BALL: " + color);
        storedColors.get(currentIndex).ballColor = color;
    }

    public void clearCurrentBall() {
        Log.i("SPINDEXER", "CLEAR CURRENT BALL");
        storedColors.get(currentIndex).ballColor = GameColors.NONE;
    }
}
