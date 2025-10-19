package org.firstinspires.ftc.teamcode.Actions;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.LaunchPositions;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Config
public class LaunchFlywheel implements Action {
    private RobotHardware robotHardware;
    private boolean initialized = false;
    public static double FLYWHEEL_POWER_COEFFICIENT_CLOSE = 0.4625;
    public static double FLYWHEEL_POWER_COEFFICIENT_MID = 0.55;
    public static double FLYWHEEL_POWER_COEFFICIENT_FAR = 0.825;
    private double targetVelocity;
    private LaunchPositions launchPosition;

    public LaunchFlywheel(RobotHardware robotHardware, LaunchPositions launchPosition) {
        this.robotHardware = robotHardware;
        this.initialized = false;
        this.launchPosition = launchPosition;
    }

    @Override
    public boolean run (@NonNull TelemetryPacket packet) {
        if (!initialized) {
            switch (launchPosition) {
                case NONE:
                    targetVelocity = 0;
                    break;
                case CLOSE:
                    targetVelocity = 1900 * FLYWHEEL_POWER_COEFFICIENT_CLOSE;
                    break;
                case MID:
                    targetVelocity = 1900 * FLYWHEEL_POWER_COEFFICIENT_MID;
                    break;
                case FAR:
                    targetVelocity = 1900 * FLYWHEEL_POWER_COEFFICIENT_FAR;
                    break;
                }

                robotHardware.setFlywheelMotorVelocityInTPS(targetVelocity);

            initialized = true;
        }

        if (targetVelocity > 0) {
            // wait till the difference is more than 5%
            return Math.abs( targetVelocity - robotHardware.getFlywheelMotorVelocityInTPS() ) > targetVelocity * 0.02;
        }

        return false;
    }
}