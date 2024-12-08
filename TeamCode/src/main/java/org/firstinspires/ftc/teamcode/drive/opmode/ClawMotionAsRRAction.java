package org.firstinspires.ftc.teamcode.drive.opmode;

import android.util.Log;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class ClawMotionAsRRAction implements Action {
    private RobotHardware myHardware;
    private boolean open;
    private boolean initialized = false;
    private boolean waitForAction = false;
    private boolean shortWait = false;
    private ElapsedTime timer;

    public ClawMotionAsRRAction(RobotHardware robotHardware, boolean open, boolean waitForAction, boolean shortWait) {
        this.myHardware = robotHardware;
        this.open = open;
        this.initialized = false;
        this.waitForAction = waitForAction;
        this.shortWait = shortWait;
    }

    @Override
    public boolean run (@NonNull TelemetryPacket packet) {
        if (!initialized) {
            myHardware.operateClawServo(open);
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
