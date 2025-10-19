package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.RobotHardware;

@Config
public class LaunchGate implements Action {
    private RobotHardware robotHardware;
    private boolean open;
    private boolean initialized = false;
    public static double LAUNCH_GATE_CLOSED = 0.6;
    public static double LAUNCH_GATE_OPEN = 1;

    public LaunchGate(RobotHardware robotHardware, boolean open) {
        this.robotHardware = robotHardware;
        this.open = open;
        this.initialized = false;
    }

    @Override
    public boolean run (@NonNull TelemetryPacket packet) {
        if (!initialized) {

            if (open) {
                robotHardware.setLaunchGatePosition(LAUNCH_GATE_OPEN);
            }
            else {
                robotHardware.setLaunchGatePosition(LAUNCH_GATE_CLOSED);
            }

            initialized = true;
        }

        return false;
    }
}