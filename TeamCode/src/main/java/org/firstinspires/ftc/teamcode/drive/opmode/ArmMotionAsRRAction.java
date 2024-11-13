package org.firstinspires.ftc.teamcode.drive.opmode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.teamcode.RobotHardware;

public class ArmMotionAsRRAction implements Action {
    private RobotHardware myHardware;
    private int armPosition;
    private int armVelocity;
    private boolean initialized = false;
    private boolean waitForAction = false;

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
            initialized = true;
        }

        if (waitForAction) {
            return myHardware.isClawArmMotorBusy();
        }

        return false;
    }
}
