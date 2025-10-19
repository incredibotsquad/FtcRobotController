package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.RobotHardware;

@Config
public class LaunchVisor implements Action {
    private RobotHardware robotHardware;
    private boolean raise;
    private boolean initialized = false;
    public static double LAUNCH_VISOR_RESTING = 0;
    public static double LAUNCH_VISOR_RAISED = 1;

    public LaunchVisor(RobotHardware robotHardware, boolean raise) {
        this.robotHardware = robotHardware;
        this.raise = raise;
        this.initialized = false;
    }

    @Override
    public boolean run (@NonNull TelemetryPacket packet) {
        if (!initialized) {

            if (raise) {
                robotHardware.setLaunchVisorPosition(LAUNCH_VISOR_RAISED);
            }
            else {
                robotHardware.setLaunchVisorPosition(LAUNCH_VISOR_RESTING);
            }

            initialized = true;
        }

        return false;
    }
}