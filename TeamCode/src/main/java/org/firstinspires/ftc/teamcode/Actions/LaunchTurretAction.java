package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.RobotHardware;

@Config
public class LaunchTurretAction implements Action {
    private RobotHardware robotHardware;
    private int position;
    private boolean initialized = false;
    public static double LAUNCH_TURRET_ACTION_DELAY_MILLIS = 200;
    private ElapsedTime timer;

    public LaunchTurretAction(RobotHardware robotHardware, int position) {
        this.robotHardware = robotHardware;
        this.position = position;
        this.initialized = false;
    }

    @Override
    public boolean run (@NonNull TelemetryPacket packet) {
        if (!initialized) {

            timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            robotHardware.setLaunchTurretPosition(position);

            initialized = true;
        }

        return (timer.milliseconds() < LAUNCH_TURRET_ACTION_DELAY_MILLIS);
    }
}