package org.firstinspires.ftc.teamcode.drive.opmode.auto.roadrunner;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class CustomClawPositionAsRRAction implements Action {
    private RobotHardware myHardware;
    private boolean initialized = false;
    private boolean waitForAction = false;
    private boolean shortWait = false;

    private double position;
    private ElapsedTime timer;

    public CustomClawPositionAsRRAction(RobotHardware robotHardware, double position, boolean waitForAction, boolean shortWait) {
        this.myHardware = robotHardware;
        this.initialized = false;
        this.position = position;
        this.waitForAction = waitForAction;
        this.shortWait = shortWait;
    }

    @Override
    public boolean run (@NonNull TelemetryPacket packet) {
        if (!initialized) {
            myHardware.operateClawServo(position);
            timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            initialized = true;
        }

        if (waitForAction) {
            int timeDuration = 400;
            if (shortWait) {
                timeDuration = 200;
            }
            boolean timerPending = timer.milliseconds() < timeDuration;

            //tell RR we need to keep running if duration has not elapsed
            return (timerPending);
        }

        return false;
    }
}
