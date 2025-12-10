package org.firstinspires.ftc.teamcode.Actions;

import static org.firstinspires.ftc.teamcode.subsystems.Spindex.SPINDEX_VELOCITY;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.common.RobotHardware;


@Config
public class ResetSpindexerAction implements Action {
    private RobotHardware robotHardware;
    private boolean initialized = false;

    public static int SPINDEXER_INCREMENT = 40;
    public static int SPINDEXER_RESET_VELOCITY = 1250;

    public ResetSpindexerAction(RobotHardware robotHardware) {
        this.robotHardware = robotHardware;
    }

    @Override
    public boolean run (@NonNull TelemetryPacket packet) {

        Log.i("ResetSpindexerAction", " Setting spindexer position by increment");

        robotHardware.setSpindexPositionAndVelocity(robotHardware.getSpindexPosition() - SPINDEXER_INCREMENT, SPINDEXER_RESET_VELOCITY);

        if (robotHardware.isSpindexLimitSwitchTriggered())
        {
            Log.i("ResetSpindexerAction", " spindexer limit switch triggered");

            robotHardware.stopSpindexAndResetEncoder();
            return false;
        }

        return true;
    }
}