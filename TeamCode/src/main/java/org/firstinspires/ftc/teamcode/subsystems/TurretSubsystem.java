package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretSubsystem extends SubsystemBase {
    private final SimpleServo turretServo;

    public TurretSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        turretServo = new SimpleServo(hardwareMap, "turretServo", 0, 1800);
    }

    public void setPosition(double position) {
        turretServo.setPosition(position);
    }

    public double getPosition() {
        return turretServo.getPosition();
    }

    // TODO: dummy functions from the motor version to compile the code.
    //remove once decided.
    public double getTurretAngle() { return 0; }
    public void alignToAngle(double currentAngle, double targetAngle) {}
    public void stop() {}

    /*
     * This method is called periodically by the CommandScheduler. Useful for updating subsystem-specific state that you don't want to offload to a Command.
     * Teams should try to be consistent within their own codebases about which responsibilities will be handled by Commands, and which will be handled here.
     * Excellent for sending telemetry or updating dashboards, but optional!
     * */
    @Override
    public void periodic() {

        // (You would pass a telemetry object into the subsystem constructor to use this)
        // telemetry.addData("Launcher Ready", isReady);
        // telemetry.addData("Current Speed", getCurrentVelocity());
    }
}

// MOTOR BASED CODE BELOW
//public class TurretSubsystem extends SubsystemBase {
//    private final MotorEx turretMotor;
//    private final PIDController pid;
//
//    // Tuning coefficients (proportional, integral, derivative)
//    // These must be tuned for your specific physical turret!
//    public static double kP = 0.008;
//    public static double kI = 0.0;
//    public static double kD = 0.0001;
//
//    public TurretSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
//        this.turretMotor = new MotorEx(hardwareMap, "turretMotor", Motor.GoBILDA.RPM_1150);
//        this.turretMotor.setRunMode(Motor.RunMode.RawPower);
//        this.pid = new PIDController(kP, kI, kD);
//    }
//
//    /**
//     * Calculates the PID output and spins the turret toward the target angle.
//     * @param currentAngle Degrees, read from an encoder or potentiometer
//     * @param targetAngle Degrees, calculated from odometry coordinates
//     */
//    public void alignToAngle(double currentAngle, double targetAngle) {
//        // Calculate required motor power based on error
//        double power = pid.calculate(currentAngle, targetAngle);
//
//        // Clip power for safety
//        power = Math.max(-0.5, Math.min(0.5, power));
//
//        turretMotor.set(power);
//    }
//
//    public double getTurretAngle() {
//        // Example logic: convert encoder ticks to degrees
//        // (Adjust the 0.5 factor based on your gear ratio and encoder CPR!)
//        return turretMotor.getCurrentPosition() * 0.5;
//    }
//
//    public void stop() {
//        turretMotor.stopMotor();
//    }
//
//    /*
//     * This method is called periodically by the CommandScheduler. Useful for updating subsystem-specific state that you don't want to offload to a Command.
//     * Teams should try to be consistent within their own codebases about which responsibilities will be handled by Commands, and which will be handled here.
//     * Excellent for sending telemetry or updating dashboards, but optional!
//     * */
//    @Override
//    public void periodic() {
//
//        // (You would pass a telemetry object into the subsystem constructor to use this)
//        // telemetry.addData("Launcher Ready", isReady);
//        // telemetry.addData("Current Speed", getCurrentVelocity());
//    }
//}