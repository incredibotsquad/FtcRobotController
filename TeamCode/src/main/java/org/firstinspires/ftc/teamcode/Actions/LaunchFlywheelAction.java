package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.RobotHardware;

@Config
public class LaunchFlywheelAction implements Action {
    private RobotHardware robotHardware;
    private boolean initialized = false;
    public static double FLYWHEEL_POWER_COEFFICIENT_CLOSE = 0.4625;
    public static double FLYWHEEL_POWER_COEFFICIENT_MID = 0.55;
    public static double FLYWHEEL_POWER_COEFFICIENT_FAR = 0.825;

    public static double FLYWHEEL_FULL_TICKS_PER_SEC = 1900;
    public static double FLYWHEEL_TARGET_VELOCITY_TOLERANCE_PERCENTAGE = 2;
    private double targetVelocity;

    public LaunchFlywheelAction(RobotHardware robotHardware, double flywheelVelocityTPS) {
        this.robotHardware = robotHardware;
        this.initialized = false;
        this.targetVelocity = flywheelVelocityTPS;
    }

    @Override
    public boolean run (@NonNull TelemetryPacket packet) {
        if (!initialized) {

            robotHardware.setFlywheelMotorVelocityInTPS(targetVelocity);

            initialized = true;
        }

        if (targetVelocity > 0) {

            // wait till the difference is more than 2%
            return (Math.abs( targetVelocity - robotHardware.getFlywheelMotorVelocityInTPS() ) > (targetVelocity * FLYWHEEL_TARGET_VELOCITY_TOLERANCE_PERCENTAGE / 100));
        }

        return false;
    }
}