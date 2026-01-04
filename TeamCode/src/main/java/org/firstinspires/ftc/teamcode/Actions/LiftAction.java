package org.firstinspires.ftc.teamcode.Actions;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.RobotHardware;

@Config
public class LiftAction implements Action {

    public static double LIFT_ROBOT = 0.4;
    public static double LIFT_RESET = 0;

    private RobotHardware robotHardware;
    private double position;
    private boolean initialized = false;
    private boolean waitForAction = true;
    private ElapsedTime actionDuration;

    public LiftAction(RobotHardware robotHardware, double position) {
        this(robotHardware, position, true);
    }

    public LiftAction(RobotHardware robotHardware, double position, boolean waitForAction) {
        this.robotHardware = robotHardware;
        this.position = position;
        this.initialized = false;
        this.waitForAction = waitForAction;
    }

    @Override
    public boolean run (@NonNull TelemetryPacket packet) {

        if (!initialized) {
            robotHardware.setLiftPosition(position);
            initialized = true;
            actionDuration = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        }

        if (waitForAction) {
            boolean retVal = (actionDuration.milliseconds() < 500);

            if (!retVal)
                Log.i("Lift ACTION", "Total Time Taken: " + actionDuration.milliseconds());

            return retVal;
        }

        return false;
    }
}