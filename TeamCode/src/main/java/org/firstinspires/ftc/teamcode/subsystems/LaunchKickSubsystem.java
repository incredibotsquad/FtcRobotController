package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LaunchKickSubsystem extends SubsystemBase {
    private final SimpleServo kickServo;

    private static final double LAUNCH_KICK_KICK = 1;
    private static final double LAUNCH_KICK_REST = 0.35;

    private boolean isKicking;
    public LaunchKickSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        kickServo = new SimpleServo(hardwareMap, "launchKickServo", 0, 270);
    }

    public void extendKicker() {
        isKicking = true;
        kickServo.setPosition(LAUNCH_KICK_KICK);
    }

    public void retractKicker() {
        isKicking = false;
        kickServo.setPosition(LAUNCH_KICK_REST);
    }

    public boolean isKicking() {
        return isKicking;
    }

    /*
    * This method is called periodically by the CommandScheduler. Useful for updating subsystem-specific state that you don't want to offload to a Command.
    * Teams should try to be consistent within their own codebases about which responsibilities will be handled by Commands, and which will be handled here.
    * */
//    @Override
//    public void periodic() {
//        // If our target is 2200, check if we are within 50 ticks of that target
//        boolean isReady = Math.abs(getCurrentVelocity() - TARGET_VELOCITY_TICKS) < TARGET_RPM_TOLERANCE;
//
//        // (You would pass a telemetry object into the subsystem constructor to use this)
//        // telemetry.addData("Launcher Ready", isReady);
//        // telemetry.addData("Current Speed", getCurrentVelocity());
//    }
}
