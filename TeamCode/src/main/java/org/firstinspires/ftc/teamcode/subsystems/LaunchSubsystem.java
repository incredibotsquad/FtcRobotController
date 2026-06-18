package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LaunchSubsystem extends SubsystemBase {
    private final MotorEx leftLaunchMotor;
    private final MotorEx rightLaunchMotor;
    private final SimpleServo gateServo;
    private static final double TARGET_RPM = 250.0;
    private static final double TARGET_RPM_TOLERANCE = 50;

    //TODO: TUNE THESE VALUES
    private static final double LAUNCH_GATE_OPEN = 0;
    private static final double LAUNCH_GATE_CLOSED = 1;

    public LaunchSubsystem(HardwareMap hardwareMap) {
        leftLaunchMotor = new MotorEx(hardwareMap, "leftLaunchMotor", Motor.GoBILDA.BARE);
        leftLaunchMotor.setRunMode(Motor.RunMode.VelocityControl);
        rightLaunchMotor = new MotorEx(hardwareMap, "rightLaunchMotor", Motor.GoBILDA.BARE);
        rightLaunchMotor.setRunMode(Motor.RunMode.VelocityControl);
        rightLaunchMotor.setInverted(true);

        gateServo = new SimpleServo(hardwareMap, "gateServo", 0, 270);
    }

    public void spinUpAtRPM() {
        //TODO: TARGET RPM SHOULD COME FROM ODOMETRY.
        double ticksPerSecond = (TARGET_RPM * Motor.GoBILDA.BARE.getCPR()) / 60.0;
        leftLaunchMotor.setVelocity(ticksPerSecond);
        rightLaunchMotor.setVelocity(ticksPerSecond);
    }

    public void stopLaunch() {
        leftLaunchMotor.setVelocity(0);
        rightLaunchMotor.setVelocity(0);
    }

    public double getCurrentVelocity() {
        return (leftLaunchMotor.getVelocity() + rightLaunchMotor.getVelocity()) / 2.0;
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
