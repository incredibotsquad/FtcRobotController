package org.firstinspires.ftc.teamcode.drive.opmode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.teamcode.RobotHardware;

public class ClawMotionAsRRAction implements Action {
    RobotHardware myHardware;
    boolean open;
    double openPosition;
    double closePosition;
    private boolean initialized = false;

    public Action ClawMotionAsRRAction(RobotHardware robotHardware, boolean open, double openPosition, double closePosition) {
        this.myHardware = robotHardware;
        this.open = open;
        this.openPosition = openPosition;
        this.closePosition = closePosition;
        this.initialized = false;
        return new ClawMotionAsRRAction();
    }

    @Override
    public boolean run (@NonNull TelemetryPacket packet) {
        if (!initialized) {
            myHardware.operateClawServo(open, openPosition, closePosition);
            initialized = true;
        }
        return true;
    }
}
