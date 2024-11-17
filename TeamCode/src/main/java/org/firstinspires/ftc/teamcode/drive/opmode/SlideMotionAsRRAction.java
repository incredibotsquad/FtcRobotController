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
    private ElapsedTime timer;

    public SlideMotionAsRRAction(RobotHardware robotHardware, int slidePosition, boolean waitForAction) {
        this.myHardware = robotHardware;
        this.slidePosition = slidePosition;
        this.initialized = false;
        this.waitForAction = waitForAction;
    }

    @Override
    public boolean run (@NonNull TelemetryPacket packet) {
        if (!initialized) {
            myHardware.setSlidePositionAndVelocity(slidePosition, IncredibotsArmControl.SLIDE_VELOCITY_CONTRACTING - 2000);
            timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            initialized = true;
        }

        if (waitForAction) {
            return timer.milliseconds() < 400;
        }

        return false;
    }
}
