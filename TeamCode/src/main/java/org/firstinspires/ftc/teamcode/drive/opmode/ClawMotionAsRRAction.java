package org.firstinspires.ftc.teamcode.drive.opmode;

import android.util.Log;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class ClawMotionAsRRAction implements Action {
    private RobotHardware myHardware;
    private boolean open;
    private double openPosition;
    private double closePosition;
    private boolean initialized = false;
    private boolean waitForAction = false;
    ElapsedTime timer;

    public ClawMotionAsRRAction(RobotHardware robotHardware, boolean open, double openPosition, double closePosition, boolean waitForAction) {
        this.myHardware = robotHardware;
        this.open = open;
        this.openPosition = openPosition;
        this.closePosition = closePosition;
        this.initialized = false;
        this.waitForAction = waitForAction;
    }

    @Override
    public boolean run (@NonNull TelemetryPacket packet) {
        if (!initialized) {
            myHardware.operateClawServo(open, openPosition, closePosition);
            initialized = true;
            timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        }

        if (waitForAction) {
            boolean timerPending = timer.milliseconds() < 500;
            Log.i("=== INCREDIBOTS ===", "INSIDE CLAWMOTIONASRRACTION - WAITING FOR CLAW: " + timerPending);
            //tell RR we need to keep running if 500 ms has not elapsed
            return (timerPending);
        }

        return false;
    }
}
