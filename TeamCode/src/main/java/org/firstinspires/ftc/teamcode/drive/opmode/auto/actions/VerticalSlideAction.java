package org.firstinspires.ftc.teamcode.drive.opmode.auto.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotHardware;

public class VerticalSlideAction implements Action {
    private RobotHardware robotHardware;
    private int slidePosition;
    private int slideVelocity;
    private boolean initialized = false;
    private boolean waitForAction = false;
    private boolean shortWait = false;
    private ElapsedTime timer;

    public VerticalSlideAction(RobotHardware robotHardware, int slidePosition, boolean waitForAction) {
        this(robotHardware, slidePosition, RobotConstants.HORIZONTAL_SLIDE_VELOCITY, waitForAction, true);
    }

    public VerticalSlideAction(RobotHardware robotHardware, int slidePosition, int slideVelocity, boolean waitForAction) {
        this(robotHardware, slidePosition, slideVelocity, waitForAction, true);
    }

    public VerticalSlideAction(RobotHardware robotHardware, int slidePosition, boolean waitForAction, boolean shortWait) {
        this(robotHardware, slidePosition, RobotConstants.HORIZONTAL_SLIDE_VELOCITY, waitForAction, shortWait);
    }

    public VerticalSlideAction(RobotHardware robotHardware, int slidePosition, int slideVelocity, boolean waitForAction, boolean shortWait) {
        this.robotHardware = robotHardware;
        this.slidePosition = slidePosition;
        this.slideVelocity = slideVelocity;
        this.initialized = false;
        this.waitForAction = waitForAction;
        this.shortWait = shortWait;
    }

    @Override
    public boolean run (@NonNull TelemetryPacket packet) {
        if (!initialized) {
            robotHardware.setVerticalSlidePositionAndVelocity(slidePosition, slideVelocity);
            timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            initialized = true;
        }

        if (waitForAction) {

            if (shortWait) {

                if (Math.abs(robotHardware.getVerticalSlidePosition() - slidePosition) < RobotConstants.SLIDE_POSITION_TOLERANCE)
                    return false;   //return immediately if slide is already at the target position

                return (timer.milliseconds() < 400);
            }

            return (Math.abs(robotHardware.getVerticalSlidePosition() - slidePosition) > RobotConstants.SLIDE_POSITION_TOLERANCE);
        }

        return false;
    }
}
