package org.firstinspires.ftc.teamcode.drive.opmode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.teamcode.RobotHardware;

public class SlideMotionAsRRAction implements Action {
    private RobotHardware myHardware;
    private int slidePosition;
    private boolean initialized = false;
    private boolean waitForAction = false;

    public SlideMotionAsRRAction(RobotHardware robotHardware, int slidePosition, boolean waitForAction) {
        this.myHardware = robotHardware;
        this.slidePosition = slidePosition;
        this.initialized = false;
        this.waitForAction = waitForAction;
    }

    @Override
    public boolean run (@NonNull TelemetryPacket packet) {
        if (!initialized) {
            myHardware.setSlidePosition(slidePosition);
            initialized = true;
        }

        if (waitForAction) {
            return myHardware.isSlideMotorBusy();
        }

        return false;
    }
}
