package org.firstinspires.ftc.teamcode.drive.opmode.auto.roadrunner;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class HighBasketSampleDropIntakeMotionAsRRAction implements Action {
    private RobotHardware myHardware;

    private boolean initialized = false;
    private boolean waitForAction = false;
    private ElapsedTime timer;

    public HighBasketSampleDropIntakeMotionAsRRAction(RobotHardware robotHardware, boolean waitForAction) {
        this.myHardware = robotHardware;
        this.initialized = false;
        this.waitForAction = waitForAction;
    }

    @Override
    public boolean run (@NonNull TelemetryPacket packet) {
        if (!initialized) {
            initialized = true;
            timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            myHardware.ejectSampleFromIntake();
        }

        if (waitForAction) {
            int timeDuration = 250;

            //tell RR we need to keep running if duration has not elapsed
            return (timer.milliseconds() < timeDuration);
        }

        return false;
    }
}
