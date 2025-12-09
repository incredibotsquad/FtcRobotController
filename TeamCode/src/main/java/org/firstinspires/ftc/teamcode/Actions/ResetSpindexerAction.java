package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.common.RobotHardware;


@Config
public class ResetSpindexerAction implements Action {
    private RobotHardware robotHardware;
    private boolean initialized = false;

    public static int SPINDEXER_INCREMENT = 200;

    public ResetSpindexerAction(RobotHardware robotHardware) {
        this.robotHardware = robotHardware;
    }

    @Override
    public boolean run (@NonNull TelemetryPacket packet) {

        robotHardware.setSpindexPosition(robotHardware.getSpindexPosition() - SPINDEXER_INCREMENT);

        if (robotHardware.isSpindexLimitSwitchTriggered())
        {
            robotHardware.stopSpindexAndResetEncoder();
            return false;
        }

        return true;
    }
}