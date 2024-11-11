package org.firstinspires.ftc.teamcode.drive.opmode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.teamcode.RobotHardware;

public class ArmMotionAsRRAction implements Action {
    RobotHardware myHardware;
    int armPosition;
    int armVelocity;
    private boolean initialized = false;

    public Action ArmMotionAsRRAction(RobotHardware robotHardware, int armPosition, int armVelocity) {
        this.myHardware = robotHardware;
        this.armPosition = armPosition;
        this.armVelocity = armVelocity;
        this.initialized = false;
        return new ArmMotionAsRRAction();
    }

    @Override
    public boolean run (@NonNull TelemetryPacket packet) {
        if (!initialized) {
            myHardware.setClawArmPositionAndVelocity(armPosition, armVelocity);
            initialized = true;
        }
        return myHardware.isClawArmMotorBusy();
    }
}
