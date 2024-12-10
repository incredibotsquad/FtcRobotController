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

    private boolean shortWait = false;
    private ElapsedTime timer;

    public ArmMotionAsRRAction(RobotHardware robotHardware, int armPosition, int armVelocity, boolean waitForAction, boolean shortWait) {
        this.myHardware = robotHardware;
        this.armPosition = armPosition;
        this.armVelocity = armVelocity;
        this.initialized = false;
        this.waitForAction = waitForAction;
        this.shortWait = shortWait;
    }
    public ArmMotionAsRRAction(RobotHardware robotHardware, int armPosition, int armVelocity, boolean waitForAction) {
        this(robotHardware, armPosition, armVelocity, waitForAction, true);
    }

    @Override
    public boolean run (@NonNull TelemetryPacket packet) {
        if (!initialized) {
            myHardware.setClawArmPositionAndVelocity(armPosition, armVelocity);
            timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            initialized = true;
        }

        if (waitForAction) {
            if (shortWait) {
                return (myHardware.isClawArmMotorBusy() && (timer.milliseconds() < 600));
            }
            else {
                return myHardware.isClawArmMotorBusy();
            }
        }

        return false;
    }
}
