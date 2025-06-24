package org.firstinspires.ftc.teamcode.drive.opmode.auto.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotHardware;

public class ResetHorizontalSlideAction implements Action {
    private RobotHardware robotHardware;
    private boolean initialized = false;

    public ResetHorizontalSlideAction(RobotHardware robotHardware) {
        this.robotHardware = robotHardware;
    }

    @Override
    public boolean run (@NonNull TelemetryPacket packet) {

        robotHardware.setHorizontalSlidePosition(robotHardware.getHorizontalSlidePosition() - RobotConstants.HORIZONTAL_SLIDE_INCREMENT);

        if (robotHardware.isHorizontalLimitSwitchPressed())
        {
            robotHardware.stopHorizontalSlideAndResetEncoder();
            return false;
        }

        return true;
    }
}
