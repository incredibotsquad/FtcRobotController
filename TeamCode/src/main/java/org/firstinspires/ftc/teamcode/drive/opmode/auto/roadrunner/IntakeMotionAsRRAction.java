package org.firstinspires.ftc.teamcode.drive.opmode.auto.roadrunner;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class IntakeMotionAsRRAction implements Action {
    private RobotHardware myHardware;
    private boolean intake;

    private boolean stop;
    private boolean initialized = false;
    private boolean waitForAction = false;
    private ElapsedTime timer;

    public IntakeMotionAsRRAction(RobotHardware robotHardware, boolean intake, boolean stop, boolean waitForAction) {
        this.myHardware = robotHardware;
        this.intake = intake;
        this.stop = stop;
        this.initialized = false;
        this.waitForAction = waitForAction;
    }

    @Override
    public boolean run (@NonNull TelemetryPacket packet) {
        if (!initialized) {
            if (stop) {
                myHardware.stopIntake();
                return false;
            }

            myHardware.operateIntake(intake);
            timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            initialized = true;
        }

        if (waitForAction) {
            int timeDuration = 250;

            //tell RR we need to keep running if duration has not elapsed
            return (timer.milliseconds() < timeDuration);
        }

        return false;
    }
}
