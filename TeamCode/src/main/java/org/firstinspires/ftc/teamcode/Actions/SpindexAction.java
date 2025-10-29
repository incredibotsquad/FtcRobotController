package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;

@Config
public class SpindexAction implements Action {

    private RobotHardware robotHardware;
    private double position;
    private boolean initialized = false;
    private ElapsedTime timer;

    public SpindexAction(RobotHardware robotHardware, double position) {
        this.robotHardware = robotHardware;
        this.position = position;
        this.initialized = false;
    }

    @Override
    public boolean run (@NonNull TelemetryPacket packet) {
        if (!initialized) {

            timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

            robotHardware.setSpindexPosition(position);

            initialized = true;
        }

        return (timer.milliseconds() < 250);    //tell RR to wait 200 ms for spindex position

    }
}