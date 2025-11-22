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
    private double originalPosition;
    private double lastRecordedPosition;
    private double position;
    private boolean initialized = false;
    private ElapsedTime pollingTimer;
    private ElapsedTime stallTimer;
    private boolean actionBeingReset;
    private ElapsedTime actionResetTimer;
    public static double SPINDEX_POSITION_TOLERANCE = 0.05;
    public static double SPINDEX_STALL_TIMER_MILLIS = 1000;
    public static double ACTION_RESET_TIME_DURATION_MILLIS = 1000;

    public SpindexAction(RobotHardware robotHardware, double position) {
        this.robotHardware = robotHardware;
        this.position = position;
        this.initialized = false;
        this.actionBeingReset = false;
    }

    @Override
    public boolean run (@NonNull TelemetryPacket packet) {
        if (!initialized) {
            originalPosition = robotHardware.getSpindexPosition();
            Log.i("SPINDEX ACTION", "ORIGINAL POSITION: " + originalPosition);

            pollingTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            stallTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

            lastRecordedPosition = originalPosition;
            robotHardware.setSpindexPosition(position);
            initialized = true;
        }

        //run this loop every 50 ms.
        if (pollingTimer.milliseconds() < 50) return true;
        pollingTimer.reset();

        if (actionBeingReset) {
            Log.i("SPINDEX ACTION", "INSIDE ACTION BEING RESET");
            if (actionResetTimer.milliseconds() > ACTION_RESET_TIME_DURATION_MILLIS) {
                Log.i("SPINDEX ACTION", "INSIDE ACTION BEING RESET - SETTING INTIALIZED TO FALSE");
                initialized = false;
                actionBeingReset = false;
            }
            return true;
        }

        double newPos = robotHardware.getSpindexPosition();

        if (stallTimer.milliseconds() > SPINDEX_STALL_TIMER_MILLIS) {
            if (Math.abs(newPos - lastRecordedPosition) < SPINDEX_POSITION_TOLERANCE) {
                Log.i("SPINDEX ACTION", "STALL DETECTED");
                robotHardware.isSpindexStalled = true;

                Log.i("SPINDEX ACTION", "SETTING SPINDEX TO ORIGINAL POSITION: " + originalPosition);
                robotHardware.setSpindexPosition(originalPosition);
                actionBeingReset = true;
                actionResetTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

                return true;   //action has to be reset now
            }
            else {
                stallTimer.reset();
                lastRecordedPosition = newPos;
            }
        }

        return (Math.abs(newPos - position) > SPINDEX_POSITION_TOLERANCE);
    }
}