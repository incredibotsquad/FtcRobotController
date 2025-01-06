package org.firstinspires.ftc.teamcode.drive.opmode.auto.roadrunner;

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
            myHardware.operateWristServo(position);
            timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            initialized = true;
        }

        if (waitForAction) {
            int timeDuration = 750;

            if (shortWait) {
                timeDuration = 200;
            }

            //tell RR we need to keep running if duration has not elapsed
            return (timer.milliseconds() < timeDuration);
        }

        return false;
    }
}
