package org.firstinspires.ftc.teamcode.Actions;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.common.RobotHardware;

@Config
public class SpindexAction implements Action {

    private RobotHardware robotHardware;
    private int position;
    private boolean initialized = false;
    private boolean waitForAction = false;
    public static int SPINDEX_POSITION_TOLERANCE = 40;

    public SpindexAction(RobotHardware robotHardware, int position) {
        this(robotHardware, position, true);
    }

    public SpindexAction(RobotHardware robotHardware, int position, boolean waitForAction) {
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
        }

        if (waitForAction) {
            return (Math.abs(robotHardware.getSpindexPosition() - position) > SPINDEX_POSITION_TOLERANCE);
        }

        return false;
    }
}