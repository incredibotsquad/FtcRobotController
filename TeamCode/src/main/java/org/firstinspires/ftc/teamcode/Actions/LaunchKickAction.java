package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.RobotHardware;

@Config
public class LaunchKickAction implements Action {
    private RobotHardware robotHardware;
    private boolean kicked = false;
    private boolean reset = false;
    public static double LAUNCH_KICK_RESTING = 0.01;
    public static double LAUNCH_KICK_KICKING = 0.3;
    public static double LAUNCH_KICK_DELAY_MILLIS = 450;
    public static double LAUNCH_KICK_RESET_DELAY_MILLIS = 450;


    private ElapsedTime kickTimer;
    private ElapsedTime resetTimer;

    public LaunchKickAction(RobotHardware robotHardware) {
        this.robotHardware = robotHardware;
        this.kicked = false;
        this.reset = false;
    }

    @Override
    public boolean run (@NonNull TelemetryPacket packet) {
        if (!kicked) {
            kickTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

            robotHardware.setLaunchKickPosition(LAUNCH_KICK_KICKING);

            kicked = true;
            return true;
        }
        else if (!reset) {

            //keep looping while kick is in progress
            if (kickTimer.milliseconds() < LAUNCH_KICK_DELAY_MILLIS)
                return true;

            resetTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            robotHardware.setLaunchKickPosition(LAUNCH_KICK_RESTING);

            reset = true;
        }

        return (resetTimer.milliseconds() < LAUNCH_KICK_RESET_DELAY_MILLIS);

    }
}