package org.firstinspires.ftc.teamcode.Actions;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.RobotHardware;

@Config
public class SpindexAction implements Action {

    private RobotHardware robotHardware;
    private double position;
    private boolean initialized = false;
    private boolean waitForAction = true;
    public static double SPINDEX_POSITION_TOLERANCE = 0.05;
    private ElapsedTime actionDuration;

    public SpindexAction(RobotHardware robotHardware, double position) {
        this(robotHardware, position, true);
    }

    public SpindexAction(RobotHardware robotHardware, double position, boolean waitForAction) {
        this.robotHardware = robotHardware;
        this.position = position;
        this.initialized = false;
        this.waitForAction = waitForAction;
    }

    @Override
    public boolean run (@NonNull TelemetryPacket packet) {

        if (!initialized) {
            robotHardware.setSpindexPosition(position);
            initialized = true;
            actionDuration = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        }

        if (waitForAction) {

            double spindexPos = robotHardware.getSpindexPositionFromEncoder();
            boolean retVal = (Math.abs(spindexPos - position) > SPINDEX_POSITION_TOLERANCE);

//            Log.i("SPINDEX ACTION", "POSITION: " + spindexPos + " Target: " + position);


            if (!retVal)
                Log.i("SPINDEX ACTION", "Total Time Taken: " + actionDuration.milliseconds());

            return retVal;
        }

        return false;
    }
}