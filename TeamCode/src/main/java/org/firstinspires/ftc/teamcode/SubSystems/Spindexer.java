package org.firstinspires.ftc.teamcode.SubSystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.BallEntry;
import org.firstinspires.ftc.teamcode.GameColors;
import org.firstinspires.ftc.teamcode.commands.SetFeedbackServoPosition;

import java.util.List;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.FeedbackServoEx;
import dev.nextftc.hardware.impl.ServoEx;

@Config
public class Spindexer implements Subsystem {
    public static double INTAKE_POS_1 = 0.53;
    public static double INTAKE_POS_2 = 0.53;
    public static double INTAKE_POS_3 = 0.53;
    public static double LAUNCH_POS_1 = 0.35;
    public static double LAUNCH_POS_2 = 0.35;
    public static double LAUNCH_POS_3 = 0.35;
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
        spindexServo.setPosition(storedColors.get(0).intakePosition);
        currentIndex = 0;
    }

    private int getNextEmptySlotIndex() {
        return storedColors.stream().filter(entry -> entry.ballColor == GameColors.NONE).findFirst().get().index;
    }

    private int getNextFullSlotIndex() {
        return storedColors.stream().filter(entry -> entry.ballColor != GameColors.NONE).findFirst().get().index;
    }

    private int getNextGreenSlotIndex() {
        return storedColors.stream().filter(entry -> entry.ballColor == GameColors.GREEN).findFirst().get().index;
    }

    private int getNextPurpleSlotIndex() {
        return storedColors.stream().filter(entry -> entry.ballColor == GameColors.PURPLE).findFirst().get().index;
    }

    public Command moveToNextEmptySlot = new InstantCommand(() -> currentIndex = getNextEmptySlotIndex())
            .then(new SetFeedbackServoPosition(spindexServo, storedColors.get(currentIndex).intakePosition))
            .requires(spindexServo)
            .named("Spindexer moveToNextEmptySlot: " + currentIndex);

    public Command moveToNextFullSlot = new InstantCommand(() -> currentIndex = getNextFullSlotIndex())
            .then(new SetFeedbackServoPosition(spindexServo, storedColors.get(currentIndex).launchPosition))
            .requires(spindexServo)
            .named("Spindexer moveToNextFullSlot: " + currentIndex);

    public Command moveToNextGreenSlot = new InstantCommand(() -> currentIndex = getNextGreenSlotIndex())
            .then(new SetFeedbackServoPosition(spindexServo, storedColors.get(currentIndex).launchPosition))
            .requires(spindexServo)
            .named("Spindexer moveToNextGreenSlot: " + currentIndex);

    public Command moveToNextPurpleSlot = new InstantCommand(() -> currentIndex = getNextPurpleSlotIndex())
            .then(new SetFeedbackServoPosition(spindexServo, storedColors.get(currentIndex).launchPosition))
            .requires(spindexServo)
            .named("Spindexer moveToNextPurpleSlot: " + currentIndex);

    public void storeCurrentBall(GameColors color) {
        Log.i("SPINDEXER", "STORE CURRENT BALL: " + color);
        storedColors.get(currentIndex).ballColor = color;
    }

    public void clearCurrentBall() {
        Log.i("SPINDEXER", "CLEAR CURRENT BALL");
        storedColors.get(currentIndex).ballColor = GameColors.NONE;
    }
}
