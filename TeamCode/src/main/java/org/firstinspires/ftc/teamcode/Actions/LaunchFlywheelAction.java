package org.firstinspires.ftc.teamcode.Actions;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.RobotHardware;

@Config
public class LaunchFlywheelAction implements Action {
    private RobotHardware robotHardware;
    private boolean initialized = false;

    public static double FLYWHEEL_FULL_TICKS_PER_SEC = 2800; //1900
    public static double FLYWHEEL_TARGET_VELOCITY_TOLERANCE_TPS = 30;
    private double targetVelocity;
    private boolean waitForAction;
    private ElapsedTime throttleTimer;
    private ElapsedTime actionDuration;

    public LaunchFlywheelAction(RobotHardware robotHardware, double flywheelVelocityTPS) {
        this(robotHardware, flywheelVelocityTPS, true);
    }

    public LaunchFlywheelAction(RobotHardware robotHardware, double flywheelVelocityTPS, boolean waitForAction) {
        this.robotHardware = robotHardware;
        this.initialized = false;
        this.targetVelocity = flywheelVelocityTPS;
        this.waitForAction = waitForAction;
    }


    @Override
    public boolean run (@NonNull TelemetryPacket packet) {
        if (!initialized) {
            //we are warming up, there might not be a need to set the velocity again
            if (Math.abs( targetVelocity -  robotHardware.getFlywheelVelocityInTPS()) > FLYWHEEL_TARGET_VELOCITY_TOLERANCE_TPS)
                robotHardware.setFlywheelVelocityInTPS(targetVelocity);

            throttleTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            actionDuration = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            initialized = true;
        }

        if (!waitForAction)
            return false;

        if (targetVelocity > 0) {
            if (throttleTimer.milliseconds() < 20) return true; //check no frequent than 20 ms

            throttleTimer.reset();

//            Log.i("LaunchFlywheelAction", "Target Velocity: " + targetVelocity);

            double currentVelocity = robotHardware.getFlywheelVelocityInTPS();

//            Log.i("LaunchFlywheelAction", "Current Velocity: " + currentVelocity);

            boolean retVal = (Math.abs( targetVelocity -  currentVelocity) > FLYWHEEL_TARGET_VELOCITY_TOLERANCE_TPS);

            if (!retVal)
                Log.i("LAUNCH FLYWHEEL ACTION", "Total Time taken: " + actionDuration.milliseconds());

            // wait till the difference is more than 30 TPS
            return retVal;
        }

//        Log.i("LaunchFlywheelAction", "Current Velocity: this would return false");

        return false;
    }
}