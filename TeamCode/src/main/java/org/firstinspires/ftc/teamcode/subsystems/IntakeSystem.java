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
import org.firstinspires.ftc.teamcode.GameColors;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Config
public class IntakeSystem {
    public static double INTAKE_THROTTLE_TIME_MS = 500;
    private RobotHardware robotHardware;
    private Spindex spindex;
    private LightSystem lightSystem;
    public boolean isOn;
    private ElapsedTime timeSinceLastIntake;

    public IntakeSystem(RobotHardware robotHardware, Spindex spindex, LightSystem lightSystem) {
        this.robotHardware = robotHardware;
        this.spindex = spindex;
        this.lightSystem = lightSystem;
        this.isOn = false;
        timeSinceLastIntake = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    //make the default constructor private
    private IntakeSystem() {}

    public Action getTurnOnAction() {
        isOn = true;
        return new ParallelAction(
                new IntakeWheelsAction(robotHardware, true),
                spindex.moveToNextEmptySlotAction(),
                lightSystem.getIntakeOnLightAction()
        );
    }

    public Action getTurnOffAction() {
        isOn = false;
        return new ParallelAction(
                new IntakeWheelsAction(robotHardware, false),
                lightSystem.getIntakeOffLightAction()
        );
    }

    public Action getReverseIntakeAction() {

        return new SequentialAction(
                getTurnOffAction(),
                new IntakeWheelsAction(robotHardware, true, -1),
                new SleepAction(0.5),
                getTurnOffAction(),
                isOn ? getTurnOnAction() : new NullAction()
        );
    }

    public Action checkForBallIntakeAndGetAction() {
        //we dont want this to be every loop
        if (timeSinceLastIntake.milliseconds() < INTAKE_THROTTLE_TIME_MS)
            return new NullAction();

        timeSinceLastIntake.reset();

        //check color sensors and if there is a ball there, there are things to do
        //else null action
        GameColors detectedColor = robotHardware.getDetectedBallColor();

        Log.i("INTAKE SYSTEM: ", "checkForBallIntakeAndGetAction: Detected Color: " + detectedColor);

        if (isOn && detectedColor != GameColors.NONE) {
            spindex.storeCurrentBall(detectedColor);

            Log.i("INTAKE SYSTEM: ", "checkForBallIntakeAndGetAction: BALL INDEXED AND MOVED TO NEXT EMPTY SLOT");

            return spindex.moveToNextEmptySlotAction();
         }

        return new NullAction();
    }

    public Action checkForBallIntakeAndGetAction_Auto() {

        //check color sensors and if there is a ball there, there are things to do
        //else null action
        GameColors detectedColor = robotHardware.getDetectedBallColor();

        Log.i("INTAKE SYSTEM: ", "checkForBallIntakeAndGetAction: Detected Color: " + detectedColor);

        if (isOn && detectedColor != GameColors.NONE) {
            spindex.storeCurrentBall(detectedColor);

            Log.i("INTAKE SYSTEM: ", "checkForBallIntakeAndGetAction: BALL INDEXED AND MOVED TO NEXT EMPTY SLOT");

            return spindex.moveToNextEmptySlotAction();
        }

        return new NullAction();
    }
}
