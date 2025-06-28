package org.firstinspires.ftc.teamcode.drive.opmode.auto.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class HorizontalShoulderAction implements Action {
    private RobotHardware robotHardware;
    private double position;
    private boolean initialized = false;
    private boolean waitForAction = false;
    private boolean shortWait = false;
    private ElapsedTime timer;

    public HorizontalShoulderAction(RobotHardware robotHardware, double position, boolean waitForAction, boolean shortWait) {
        this.robotHardware = robotHardware;
        this.position = position;
        this.initialized = false;
        this.waitForAction = waitForAction;
        this.shortWait = shortWait;
    }

    @Override
    public boolean run (@NonNull TelemetryPacket packet) {
        if (!initialized) {
            timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            robotHardware.setHorizontalShoulderServoPosition(position);
            initialized = true;
        }

        if (waitForAction) {
            if (shortWait) {
                return (timer.milliseconds() < 300);
            }

            //tell RR we need to keep running if duration has not elapsed
            return (timer.milliseconds() < 1000);
        }

        return false;
    }
}