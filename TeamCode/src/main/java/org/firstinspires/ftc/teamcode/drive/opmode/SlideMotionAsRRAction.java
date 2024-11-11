package org.firstinspires.ftc.teamcode.drive.opmode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.teamcode.RobotHardware;

public class SlideMotionAsRRAction implements Action {
    private RobotHardware myHardware;
    private int slidePosition;
    private int slideVelocity;
    private boolean initialized = false;

    public Action SlideMotorAsRRAction(RobotHardware robotHardware, int slidePosition, int slideVelocity) {
        this.myHardware = robotHardware;
        this.slidePosition = slidePosition;
        this.slideVelocity = slideVelocity;
        this.initialized = false;
        return new SlideMotionAsRRAction();
    }

    @Override
    public boolean run (@NonNull TelemetryPacket packet) {
        if (!initialized) {
            myHardware.setSlidePositionAndVelocity(slidePosition, slideVelocity);
            initialized = true;
        }
        return myHardware.isSlideMotorBusy();
    }
}
