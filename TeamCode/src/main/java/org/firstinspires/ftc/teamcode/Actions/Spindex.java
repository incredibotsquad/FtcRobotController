package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GameColors;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Config
public class Spindex implements Action {

    private class ColorIndex {
        double intakePosition;
        double launchPosition;
        GameColors color;
    }

    private RobotHardware robotHardware;
    private boolean intake;
    private boolean initialized = false;
    public static double SPINDEX_LAUNCH = 0.35;
    public static double SPINDEX_INTAKE = 0.53;
    private ElapsedTime timer;

    public Spindex(RobotHardware robotHardware, boolean intake) {
        this.robotHardware = robotHardware;
        this.intake = intake;
        this.initialized = false;
    }

    @Override
    public boolean run (@NonNull TelemetryPacket packet) {
        if (!initialized) {

            timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

            if (intake) {
                robotHardware.setSpindexPosition(SPINDEX_INTAKE);
            }
            else {
                robotHardware.setSpindexPosition(SPINDEX_LAUNCH);
            }

            initialized = true;
        }

        return (timer.milliseconds() < 200);    //tell RR to wait 200 ms for spindex position

    }
}