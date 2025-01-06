package org.firstinspires.ftc.teamcode.drive.opmode.auto.roadrunner;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.opmode.IncredibotsArmControl;

public class SlideMotionAsRRAction implements Action {
    private RobotHardware myHardware;
    private int slidePosition;

    private int slideVelocity;
    private boolean initialized = false;
    private boolean waitForAction = false;
    private boolean shortWait = false;
    private ElapsedTime timer;

    public SlideMotionAsRRAction(RobotHardware robotHardware, int slidePosition, boolean waitForAction) {
        this(robotHardware, slidePosition, IncredibotsArmControl.SLIDE_VELOCITY_EXPANDING, waitForAction, true);
    }

    public SlideMotionAsRRAction(RobotHardware robotHardware, int slidePosition, int slideVelocity, boolean waitForAction) {
        this(robotHardware, slidePosition, slideVelocity, waitForAction, true);
    }

    public SlideMotionAsRRAction(RobotHardware robotHardware, int slidePosition, boolean waitForAction, boolean shortWait) {
        this(robotHardware, slidePosition, IncredibotsArmControl.SLIDE_VELOCITY_EXPANDING, waitForAction, shortWait);
    }

    public SlideMotionAsRRAction(RobotHardware robotHardware, int slidePosition, int slideVelocity, boolean waitForAction, boolean shortWait) {
        this.myHardware = robotHardware;
        this.slidePosition = slidePosition;
        this.slideVelocity = slideVelocity;
        this.initialized = false;
        this.waitForAction = waitForAction;
        this.shortWait = shortWait;
    }

    @Override
    public boolean run (@NonNull TelemetryPacket packet) {
        if (!initialized) {
            myHardware.setSlidePositionAndVelocity(slidePosition, slideVelocity);
            timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            initialized = true;
        }

        if (waitForAction) {

            if (shortWait) {
                return (timer.milliseconds() < 400);
            }

            return (Math.abs(myHardware.getSlidePos() - slidePosition) > 50);
        }

        return false;
    }
}
