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
    private double lastRecordedPositionForStallDetection;
    private double position;
    private boolean initialized = false;
    private ElapsedTime pollingTimer;
    private ElapsedTime stallTimer;
    private boolean actionBeingReset;
    private ElapsedTime actionResetTimer;
    public static double SPINDEX_POSITION_TOLERANCE = 0.06;
    public static double SPINDEX_STALL_TIMER_MILLIS = 1000;
    public static double ACTION_RESET_TIME_DURATION_MILLIS = 1500;
    private boolean originalPositionRecorded = false;

    public SpindexAction(RobotHardware robotHardware, double position) {
        this.robotHardware = robotHardware;
        this.position = position;
        this.initialized = false;
        this.actionBeingReset = false;
        this.originalPositionRecorded = false;
    }

    @Override
    public boolean run (@NonNull TelemetryPacket packet) {
//        if (!originalPositionRecorded) {
//            originalPosition = robotHardware.getSpindexPositionRaw();
//            Log.i("SPINDEX ACTION", "OBJECT ID:" + this.hashCode() + " ORIGINAL RAW POSITION: " + originalPosition);
//            Log.i("SPINDEX ACTION", "OBJECT ID:" + this.hashCode() + " TARGET POSITION: " + position);
//            originalPositionRecorded = true;
//
//            //dont need to do anything if spindexer is already there
//            if (Math.abs(originalPosition - position) < SPINDEX_POSITION_TOLERANCE)
//                return false;
//        }

        if (!initialized) {

            pollingTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
//            stallTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
//
//            lastRecordedPositionForStallDetection = originalPosition;
            robotHardware.setSpindexPosition(position);
            initialized = true;

//            Log.i("SPINDEX ACTION", "OBJECT ID:" + this.hashCode() + " INITIALIZED");
//            Log.i("SPINDEX ACTION", "OBJECT ID:" + this.hashCode() + " INITIALIZED. lastRecordedPositionForStallDetection: " + lastRecordedPositionForStallDetection);

        }

        //run this loop every 50 ms.
        if (pollingTimer.milliseconds() < 50) return true;
        pollingTimer.reset();

//        if (actionBeingReset) {
//            Log.i("SPINDEX ACTION", "OBJECT ID:" + this.hashCode() + " INSIDE ACTION BEING RESET");
//            if (actionResetTimer.milliseconds() > ACTION_RESET_TIME_DURATION_MILLIS) {
//                Log.i("SPINDEX ACTION", "OBJECT ID:" + this.hashCode() + " INSIDE ACTION BEING RESET - SETTING INTIALIZED TO FALSE");
//                initialized = false;
//                actionBeingReset = false;
//            }
//            return true;
//        }
//
        double newPos = robotHardware.getSpindexPositionFromEncoder();
//
//        Log.i("SPINDEX ACTION", "OBJECT ID:" + this.hashCode() + " NEW SERVO POSITION: " + newPos);
//
//        //if spindexer hasnt moved significantly in the last 1 second.
//        if (stallTimer.milliseconds() > SPINDEX_STALL_TIMER_MILLIS) {
//            if (Math.abs(newPos - lastRecordedPositionForStallDetection) < SPINDEX_POSITION_TOLERANCE) {
//                Log.i("SPINDEX ACTION", "OBJECT ID:" + this.hashCode() + " STALL DETECTED");
//                robotHardware.isSpindexStalled = true;
//
//                Log.i("SPINDEX ACTION", "OBJECT ID:" + this.hashCode() + " SETTING SPINDEX TO ORIGINAL POSITION: " + originalPosition);
//                robotHardware.setSpindexPosition(originalPosition);
//                actionBeingReset = true;
//                actionResetTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
//
//                return true;   //action has to be reset now
//            }
//            else {
//                stallTimer.reset();
//                lastRecordedPositionForStallDetection = newPos;
//            }
//        }

        return (Math.abs(newPos - position) > SPINDEX_POSITION_TOLERANCE);
    }
}