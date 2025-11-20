package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Actions.IntakeWheelsAction;
import org.firstinspires.ftc.teamcode.common.GameColors;
import org.firstinspires.ftc.teamcode.common.RobotHardware;

@Config
public class IntakeSystem {
    public static double INTAKE_THROTTLE_TIME_MS = 500;
    public static double INTAKE_DELAY_TO_LET_BALL_SETTLE_SECS = 0.5;
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

        boolean ballDetected = robotHardware.didBallDetectionBeamBreak();

        if (isOn && ballDetected) {
            spindex.storeCurrentBall(GameColors.UNKNOWN);   //default to unknown - we will update color later
            Log.i("INTAKE SYSTEM: ", "checkForBallIntakeAndGetAction: Ball Detected: indexed as UNKNOWN ");
            return new SequentialAction(
                    new SleepAction(INTAKE_DELAY_TO_LET_BALL_SETTLE_SECS),   //wait for the ball to stabilize
                    spindex.moveToNextEmptySlotAction(),
                    spindex.updateBallColor()
            );
        }


//        detectedColor = robotHardware.getDetectedBallColor();
//
//        Log.i("INTAKE SYSTEM: ", "checkForBallIntakeAndGetAction: Detected Color: " + detectedColor);
//
//
//        if (isOn && detectedColor != GameColors.NONE) {
//            spindex.storeCurrentBall(detectedColor);
//
//            Log.i("INTAKE SYSTEM: ", "checkForBallIntakeAndGetAction: BALL INDEXED AND MOVED TO NEXT EMPTY SLOT");
//
//         }

        return new NullAction();
    }
}
