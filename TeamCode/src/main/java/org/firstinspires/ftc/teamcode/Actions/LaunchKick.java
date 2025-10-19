package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;

@Config
public class LaunchKick implements Action {
    private RobotHardware robotHardware;
    private boolean kick;
    private boolean initialized = false;
    public static double LAUNCH_KICK_RESTING = 0.95;
    public static double LAUNCH_KICK_KICKING = 0.7;
    private ElapsedTime timer;

    public LaunchKick(RobotHardware robotHardware, boolean kick) {
        this.robotHardware = robotHardware;
        this.kick = kick;
        this.initialized = false;
    }

    @Override
    public boolean run (@NonNull TelemetryPacket packet) {
        if (!initialized) {
            timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

            if (kick) {
                robotHardware.setLaunchKickPosition(LAUNCH_KICK_KICKING);
            }
            else {
                robotHardware.setLaunchKickPosition(LAUNCH_KICK_RESTING);
            }

            initialized = true;
        }

        return (timer.milliseconds() < 200);    //tell RR to wait 200 ms for kick
    }
}