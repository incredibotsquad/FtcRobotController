package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Actions.IntakeLightAction;
import org.firstinspires.ftc.teamcode.Actions.SpindexAction;
import org.firstinspires.ftc.teamcode.BallEntry;
import org.firstinspires.ftc.teamcode.GameColors;
import org.firstinspires.ftc.teamcode.RobotHardware;

import java.util.Collections;
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

    public int currentIndex;
    private RobotHardware robotHardware;
    public Spindex(RobotHardware robotHardware){
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

        if (!list.isEmpty())
            nextFullSlotIndex = list.get(list.size() - 1).index;

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

        currentIndex = nextIndex;

        return new SpindexAction(robotHardware, storedColors.get(currentIndex).intakePosition);
    }

    public Action moveToNextFullSlotAction() {
        int nextIndex = getNextFullSlotIndex();
        if (nextIndex < 0 ) return new NullAction();

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

    public Action reIndexBalls() {
        Log.i("SPINDEXER", "REINDEXING");

        Action ball1 = new SequentialAction(
                new SpindexAction(robotHardware, storedColors.get(0).intakePosition),
                new InstantAction(() -> storedColors.get(0).ballColor = tryToGetDetectedColor())
        );

        Action ball2 = new SequentialAction(
                new SpindexAction(robotHardware, storedColors.get(1).intakePosition),
                new InstantAction(() -> storedColors.get(1).ballColor = tryToGetDetectedColor())
        );

        Action ball3 = new SequentialAction(
                new SpindexAction(robotHardware, storedColors.get(2).intakePosition),
                new InstantAction(() -> storedColors.get(2).ballColor = tryToGetDetectedColor())
        );

        return new SequentialAction(
                ball1,
                ball2,
                ball3
        );
    }

    private GameColors tryToGetDetectedColor() {
        GameColors detectedColor = GameColors.NONE;
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        do {
            detectedColor = robotHardware.getDetectedBallColor();

        } while (detectedColor == GameColors.NONE && timer.milliseconds() < 300);

        Log.i("SPINDEXER", "REINDEX COLOR DETECTED:" + detectedColor);

        return detectedColor;
    }
}
