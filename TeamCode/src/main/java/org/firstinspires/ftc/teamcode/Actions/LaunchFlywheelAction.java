package org.firstinspires.ftc.teamcode.Actions;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;

@Config
public class LaunchFlywheelAction implements Action {
    private RobotHardware robotHardware;
    private boolean initialized = false;

    public static double FLYWHEEL_FULL_TICKS_PER_SEC = 2800; //1900
    public static double FLYWHEEL_TARGET_VELOCITY_TOLERANCE_TPS = 10;
    private double targetVelocity;
    private ElapsedTime timer;

    public LaunchFlywheelAction(RobotHardware robotHardware, double flywheelVelocityTPS) {
        this.robotHardware = robotHardware;
        this.initialized = false;
        this.targetVelocity = flywheelVelocityTPS;
    }

    @Override
    public boolean run (@NonNull TelemetryPacket packet) {
        if (!initialized) {

            robotHardware.setFlywheelMotorVelocityInTPS(targetVelocity);
            timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            initialized = true;
        }

        if (targetVelocity > 0) {
            if (timer.milliseconds() < 100) return true; //check no frequent than 100 ms

            timer.reset();

            Log.i("LaunchFlywheelAction", "Target Velocity: " + targetVelocity);

            double currentVelocity = robotHardware.getFlywheelMotorVelocityInTPS();

            Log.i("LaunchFlywheelAction", "Current Velocity: " + currentVelocity);

            // wait till the difference is more than 2%
            return (Math.abs( targetVelocity -  currentVelocity) > FLYWHEEL_TARGET_VELOCITY_TOLERANCE_TPS);
        }

        return false;
    }
}