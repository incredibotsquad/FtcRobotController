package org.firstinspires.ftc.teamcode.Actions;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.RobotHardware;

@Config
public class LaunchVisorAction implements Action {
    private RobotHardware robotHardware;
    private double position;
    private boolean initialized = false;
    public static double LAUNCH_VISOR_RESTING = 0.15;
    public static double LAUNCH_VISOR_MAX = 0.8;
    public static double LAUNCH_VISOR_MID = (LAUNCH_VISOR_RESTING + LAUNCH_VISOR_MAX)/2;

    public static double VISOR_POSITION_TOLERANCE = 0.0005;
    private boolean waitForAction;
    private ElapsedTime timer;
    private ElapsedTime actionDuration;

    public LaunchVisorAction(RobotHardware robotHardware, double position) {
        this(robotHardware, position, true);
    }

    public LaunchVisorAction(RobotHardware robotHardware, double position, boolean waitForAction) {
        this.robotHardware = robotHardware;
        this.position = Math.max(LAUNCH_VISOR_RESTING, Math.min(position, LAUNCH_VISOR_MAX));
        this.initialized = false;
        this.waitForAction = waitForAction;
    }

    @Override
    public boolean run (@NonNull TelemetryPacket packet) {
        if (!initialized) {
            timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            robotHardware.setLaunchVisorPosition(position);
            initialized = true;
            actionDuration = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        }

        if (!waitForAction)
            return false;

        //run this loop every 20 ms.
        if (timer.milliseconds() < 20) return true;
        timer.reset();

        double visorPos = robotHardware.getLaunchVisorPositionFromEncoder();
//        Log.i("LAUNCH VISOR ACTION", "POSITION: " + visorPos);

        boolean retVal = (Math.abs(position - visorPos) > VISOR_POSITION_TOLERANCE);


        if (!retVal)
            Log.i("LAUNCH VISOR ACTION", "Total time taken: " + actionDuration.milliseconds());
        else
            Log.i("LAUNCH VISOR ACTION", "Curret: " + visorPos + " Target: " + position);


        return retVal;
    }
}