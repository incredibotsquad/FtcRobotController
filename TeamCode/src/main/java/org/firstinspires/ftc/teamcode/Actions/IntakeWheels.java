package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.RobotHardware;

@Config
public class IntakeWheels implements Action {
    private RobotHardware robotHardware;
    private boolean start;
    private boolean initialized = false;
    public static double INTAKE_POWER = 1;

    public IntakeWheels(RobotHardware robotHardware, boolean start) {
        this.robotHardware = robotHardware;
        this.start = start;
        this.initialized = false;
    }

    @Override
    public boolean run (@NonNull TelemetryPacket packet) {
        if (!initialized) {

            if (start) {
                robotHardware.setIntakeMotorPower(INTAKE_POWER);
            }
            else {
                robotHardware.setIntakeMotorPower(0);
            }

            initialized = true;
        }

        return false;
    }
}