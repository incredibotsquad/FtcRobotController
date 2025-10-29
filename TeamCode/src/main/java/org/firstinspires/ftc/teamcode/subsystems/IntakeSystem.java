package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Actions.IntakeLightAction;
import org.firstinspires.ftc.teamcode.Actions.IntakeWheelsAction;
import org.firstinspires.ftc.teamcode.GameColors;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Config
public class IntakeSystem {
    public static double INTAKE_LIGHT_SUCCESSFUL_INTAKE = 0.333;    //ORANGE BLINK
    public static double INTAKE_LIGHT_EMPTY = 0.277;    //RED
    public static double INTAKE_LIGHT_ONE_BALL = 0.333; //ORANGE
    public static double INTAKE_LIGHT_TWO_BALLS = 0.388;    //YELLOW
    public static double INTAKE_LIGHT_THREE_BALLS = 0.5;    //GREEN
    public static double INTAKE_THROTTLE_TIME_MS = 200;
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

    public Action getLightActionForSpindexState() {

        switch (spindex.fullSlotCount()) {
            case 0:
                return new IntakeLightAction(robotHardware, INTAKE_LIGHT_EMPTY);
            case 1:
                return new IntakeLightAction(robotHardware, INTAKE_LIGHT_ONE_BALL);
            case 2:
                return new IntakeLightAction(robotHardware, INTAKE_LIGHT_TWO_BALLS);
            default:
                return new IntakeLightAction(robotHardware, INTAKE_LIGHT_THREE_BALLS);
        }
    }

    public Action getLightOffAction() {
        return new IntakeLightAction(robotHardware, 0);
    }

    public Action getSuccesfulBallIntakeLightAction() {
        return new SequentialAction(
                new IntakeLightAction(robotHardware, INTAKE_LIGHT_SUCCESSFUL_INTAKE),
                new SleepAction(0.1),
                getLightOffAction(),
                new IntakeLightAction(robotHardware, INTAKE_LIGHT_SUCCESSFUL_INTAKE),
                new SleepAction(0.1),
                getLightOffAction()
        );
    }

    public Action checkForBallIntakeAndGetAction() {
        //check color sensors and if there is a ball there, there are things to do
        //else null action
        GameColors detectedColor = robotHardware.getDetectedBallColor();

        Log.i("INTAKE SYSTEM: ", "checkForBallIntakeAndGetAction: DETECTED COLOR: " + detectedColor);

        if (isOn && detectedColor != GameColors.NONE && timeSinceLastIntake.milliseconds() > INTAKE_THROTTLE_TIME_MS) {
            spindex.storeCurrentBall(detectedColor);

            Log.i("INTAKE SYSTEM: ", "checkForBallIntakeAndGetAction: BALL INDEXED AND MOVED TO NEXT EMPTY SLOT");
            timeSinceLastIntake.reset();

            return new SequentialAction(
                    spindex.moveToNextEmptySlotAction(),
                    getSuccesfulBallIntakeLightAction(),
                    getLightActionForSpindexState()
            );
         }

        return new NullAction();
    }

}
