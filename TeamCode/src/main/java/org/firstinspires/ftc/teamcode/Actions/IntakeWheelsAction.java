package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.common.RobotHardware;

@Config
public class IntakeWheelsAction implements Action {
    private RobotHardware robotHardware;
    private boolean start;
    private boolean initialized = false;
    public static double INTAKE_POWER = 1;
    private double power;

    public IntakeWheelsAction(RobotHardware robotHardware, boolean start) {
        this(robotHardware, start, INTAKE_POWER);
    }

    public IntakeWheelsAction(RobotHardware robotHardware, boolean start, double power) {
        this.robotHardware = robotHardware;
        this.start = start;
        this.initialized = false;
        this.power = power;
    }

    @Override
    public boolean run (@NonNull TelemetryPacket packet) {
        if (!initialized) {

            if (start) {
                robotHardware.setIntakeMotorPower(power);
            }
            else {
                robotHardware.setIntakeMotorPower(0);
            }

            initialized = true;
        }

        return false;
    }
}