package org.firstinspires.ftc.teamcode.SubSystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.BallEntry;
import org.firstinspires.ftc.teamcode.GameColors;
import org.firstinspires.ftc.teamcode.commands.SetFeedbackServoPosition;

import java.util.List;
import java.util.stream.Collectors;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.FeedbackServoEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

@Config
public class Spindexer implements Subsystem {
    public static double INTAKE_POS_1 = 0.575;
    public static double INTAKE_POS_2 = 0.2;
    public static double INTAKE_POS_3 = 0.95;
    public static double LAUNCH_POS_1 = 0.37;
    public static double LAUNCH_POS_2 = 0;
    public static double LAUNCH_POS_3 = 0.74;
    private int currentIndex = 0;
    public List<BallEntry> storedColors = List.of(
            new BallEntry(0, INTAKE_POS_1, LAUNCH_POS_1, GameColors.NONE),
            new BallEntry(1, INTAKE_POS_2, LAUNCH_POS_2, GameColors.NONE),
            new BallEntry(2, INTAKE_POS_3, LAUNCH_POS_3, GameColors.NONE));


//    private final FeedbackServoEx spindexServo = new FeedbackServoEx("", "SpindexServo");

    private final ServoEx spindexServo = new ServoEx("SpindexServo");

    public static final Spindexer INSTANCE = new Spindexer();
    private Spindexer() {
    }

    @Override
    public void initialize() {
        currentIndex = 0;
    }

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

    public void moveToNextEmptySlot() {
        int nextIndex = getNextEmptySlotIndex();
        if (nextIndex < 0) return;

        currentIndex = nextIndex;
        moveToIntakeForCurrentIndex.invoke();
    }

//    public Command moveToNextEmptySlot = new InstantCommand(() -> currentIndex = getNextEmptySlotIndex())
//            .then(new SetFeedbackServoPosition(spindexServo, storedColors.get(currentIndex).intakePosition))
//            .requires(spindexServo)
//            .named("Spindexer moveToNextEmptySlot: " + currentIndex);

    private Command moveToIntakeForCurrentIndex = new SetFeedbackServoPosition(spindexServo, storedColors.get(currentIndex).intakePosition)
            .requires(spindexServo)
            .named("SPINDEX: moveToIntakeForCurrentIndex");

    public void moveToNextFullSlot() {
        int nextIndex = getNextFullSlotIndex();
        if (nextIndex < 0) return;

        currentIndex = nextIndex;
        moveToLaunchForCurrentIndex.invoke();
    }

    private Command moveToLaunchForCurrentIndex = new SetFeedbackServoPosition(spindexServo, storedColors.get(currentIndex).launchPosition)
                                                        .requires(spindexServo)
                                                        .named("SPINDEX: moveToLaunchForCurrentIndex");

//    public Command moveToNextFullSlot = new InstantCommand(() -> currentIndex = getNextFullSlotIndex())
//            .then(new SetFeedbackServoPosition(spindexServo, storedColors.get(currentIndex).launchPosition))
//            .requires(spindexServo)
//            .named("Spindexer moveToNextFullSlot: " + currentIndex);

    public void  moveToNextGreenSlot() {
        int nextIndex = getNextGreenSlotIndex();
        if (nextIndex < 0) return;

        currentIndex = nextIndex;
        moveToLaunchForCurrentIndex.invoke();
    }

//    public Command moveToNextGreenSlot = new InstantCommand(() -> currentIndex = getNextGreenSlotIndex())
//            .then(new SetFeedbackServoPosition(spindexServo, storedColors.get(currentIndex).launchPosition))
//            .requires(spindexServo)
//            .named("Spindexer moveToNextGreenSlot: " + currentIndex);

    public void  moveToNextPurpleSlot() {
        int nextIndex = getNextPurpleSlotIndex();
        if (nextIndex < 0) return;

        currentIndex = nextIndex;
        moveToLaunchForCurrentIndex.invoke();
    }

//    public Command moveToNextPurpleSlot = new InstantCommand(() -> currentIndex = getNextPurpleSlotIndex())
//            .then(new SetFeedbackServoPosition(spindexServo, storedColors.get(currentIndex).launchPosition))
//            .requires(spindexServo)
//            .named("Spindexer moveToNextPurpleSlot: " + currentIndex);

    public void storeCurrentBall(GameColors color) {
        Log.i("SPINDEXER", "STORE CURRENT BALL: " + color);
        storedColors.get(currentIndex).ballColor = color;
    }

    public void clearCurrentBall() {
        Log.i("SPINDEXER", "CLEAR CURRENT BALL");
        storedColors.get(currentIndex).ballColor = GameColors.NONE;
    }
}
