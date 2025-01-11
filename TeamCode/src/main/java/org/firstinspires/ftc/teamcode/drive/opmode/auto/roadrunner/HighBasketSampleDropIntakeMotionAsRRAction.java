package org.firstinspires.ftc.teamcode.drive.opmode.auto.roadrunner;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class HighBasketSampleDropIntakeMotionAsRRAction implements Action {
    private RobotHardware myHardware;

    private boolean initialized = false;

    public HighBasketSampleDropIntakeMotionAsRRAction(RobotHardware robotHardware) {
        this.myHardware = robotHardware;
        this.initialized = false;
    }

    @Override
    public boolean run (@NonNull TelemetryPacket packet) {
        if (!initialized) {
            myHardware.ejectSampleFromIntake();

            initialized = true;
        }

        return false;
    }
}
