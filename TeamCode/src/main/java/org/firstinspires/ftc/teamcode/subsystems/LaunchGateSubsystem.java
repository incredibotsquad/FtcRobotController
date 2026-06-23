package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LaunchGateSubsystem extends SubsystemBase {

    private final SimpleServo gateServo;


    //TODO: TUNE THESE VALUES
    private static final double LAUNCH_GATE_OPEN = 0;
    private static final double LAUNCH_GATE_CLOSED = 1;

    public LaunchGateSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {

        gateServo = new SimpleServo(hardwareMap, "launchGateServo", 0, 270);
    }

    public void openGate() {
        gateServo.setPosition(LAUNCH_GATE_OPEN);
    }

    public void closeGate() {
        gateServo.setPosition(LAUNCH_GATE_CLOSED);
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
