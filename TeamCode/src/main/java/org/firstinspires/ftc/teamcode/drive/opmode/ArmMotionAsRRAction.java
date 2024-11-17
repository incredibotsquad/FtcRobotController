package org.firstinspires.ftc.teamcode.drive.opmode;

import android.util.Log;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class ArmMotionAsRRAction implements Action {
    private RobotHardware myHardware;
    private int armPosition;
    private int armVelocity;
    private boolean initialized = false;
    private boolean waitForAction = false;
    private ElapsedTime timer;

    public  ArmMotionAsRRAction(RobotHardware robotHardware, int armPosition, int armVelocity, boolean waitForAction) {
        this.myHardware = robotHardware;
        this.armPosition = armPosition;
        this.armVelocity = armVelocity;
        this.initialized = false;
        this.waitForAction = waitForAction;
    }

    @Override
    public boolean run (@NonNull TelemetryPacket packet) {
        if (!initialized) {
            myHardware.setClawArmPositionAndVelocity(armPosition, armVelocity);
            timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            initialized = true;
        }

        if (waitForAction) {
            Log.i("=== INCREDIBOTS ===", "|+|+|+|+|+|+|+|+|+|+|+|+|+|+: ");
            Log.i("=== INCREDIBOTS ===", "WAITING FOR ARM. MOTOR BUSY: " + myHardware.isClawArmMotorBusy());
            Log.i("=== INCREDIBOTS ===", "WAITING FOR ARM. TIMER: " + timer.milliseconds());
            Log.i("=== INCREDIBOTS ===", "|+|+|+|+|+|+|+|+|+|+|+|+|+|+: ");
            return (myHardware.isClawArmMotorBusy() && (timer.milliseconds() < 700));
        }

        return false;
    }
}
