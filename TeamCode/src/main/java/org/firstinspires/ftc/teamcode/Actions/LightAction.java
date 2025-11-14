package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.common.RobotHardware;

public class LightAction implements Action {

    private RobotHardware robotHardware;
    private double color;
    private boolean initialized = false;

    public LightAction(RobotHardware robotHardware, double color) {
        this.robotHardware = robotHardware;
        this.color = color;
        this.initialized = false;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!initialized) {
            robotHardware.setLightColor(color);
            initialized = true;
        }

        return false;
    }
}
