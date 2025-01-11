package org.firstinspires.ftc.teamcode.drive.opmode.auto.roadrunner;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class WristMotionAsRRAction implements Action {
    private RobotHardware myHardware;
    private double position;
    private boolean initialized = false;
    private boolean waitForAction = false;
    private boolean shortWait = false;
    private ElapsedTime timer;

    public WristMotionAsRRAction(RobotHardware robotHardware, double position, boolean waitForAction, boolean shortWait) {
        this.myHardware = robotHardware;
        this.position = position;
        this.initialized = false;
        this.waitForAction = waitForAction;
        this.shortWait = shortWait;
    }

    @Override
    public boolean run (@NonNull TelemetryPacket packet) {
        if (!initialized) {
            timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            myHardware.operateWristServo(position);
            initialized = true;
        }

        if (waitForAction) {
            if (shortWait) {
                return (timer.milliseconds() < 250);
            }

            //tell RR we need to keep running if duration has not elapsed
            return (myHardware.getWristServoPosition() != position);
        }

        return false;
    }
}
