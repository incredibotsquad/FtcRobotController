package org.firstinspires.ftc.teamcode.drive.opmode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class SlideMotionAsRRAction implements Action {
    private RobotHardware myHardware;
    private int slidePosition;
    private boolean initialized = false;
    private boolean waitForAction = false;
    private boolean shortWait = false;
    private ElapsedTime timer;

    public SlideMotionAsRRAction(RobotHardware robotHardware, int slidePosition, boolean waitForAction) {
        this(robotHardware, slidePosition, waitForAction, true);
    }

    public SlideMotionAsRRAction(RobotHardware robotHardware, int slidePosition, boolean waitForAction, boolean shortWait) {
        this.myHardware = robotHardware;
        this.slidePosition = slidePosition;
        this.initialized = false;
        this.waitForAction = waitForAction;
        this.shortWait = shortWait;
    }

    @Override
    public boolean run (@NonNull TelemetryPacket packet) {
        if (!initialized) {
            myHardware.setSlidePositionAndVelocity(slidePosition, IncredibotsArmControl.SLIDE_VELOCITY_EXPANDING);
            timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            initialized = true;
        }

        if (waitForAction) {
            return shortWait? (timer.milliseconds() < 400):myHardware.isSlideMotorBusy();
        }

        return false;
    }
}
