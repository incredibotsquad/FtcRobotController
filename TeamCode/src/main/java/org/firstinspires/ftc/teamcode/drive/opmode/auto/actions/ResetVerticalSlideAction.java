package org.firstinspires.ftc.teamcode.drive.opmode.auto.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotHardware;

public class ResetVerticalSlideAction implements Action {
    private RobotHardware robotHardware;
    private boolean initialized = false;

    public ResetVerticalSlideAction(RobotHardware robotHardware) {
        this.robotHardware = robotHardware;
    }

    @Override
    public boolean run (@NonNull TelemetryPacket packet) {

        robotHardware.setVerticalSlidePositionAndVelocity(robotHardware.getVerticalSlidePosition() - RobotConstants.SLIDE_POSITION_TOLERANCE, RobotConstants.VERTICAL_SLIDE_VELOCITY);

        if (robotHardware.isVerticalLimitSwitchPressed())
        {
            robotHardware.stopVerticalSlideAndResetEncoder();
            return false;
        }

        return true;
    }
}
