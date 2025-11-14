package org.firstinspires.ftc.teamcode.Actions;

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
    public static double LAUNCH_VISOR_RESTING = 0;
    public static double LAUNCH_VISOR_ACTION_DELAY_MILLIS = 100;

    private ElapsedTime timer;

    public LaunchVisorAction(RobotHardware robotHardware, double position) {
        this.robotHardware = robotHardware;
        this.position = position;
        this.initialized = false;
    }

    @Override
    public boolean run (@NonNull TelemetryPacket packet) {
        if (!initialized) {

            timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            robotHardware.setLaunchVisorPosition(position);

            initialized = true;
        }

        return (timer.milliseconds() < LAUNCH_VISOR_ACTION_DELAY_MILLIS);
    }
}