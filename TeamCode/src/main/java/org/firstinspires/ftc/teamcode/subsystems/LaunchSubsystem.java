package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LaunchSubsystem extends SubsystemBase {
    private final MotorEx leftLaunchMotor;
    private final MotorEx rightLaunchMotor;
    private final SimpleServo turretServo;
    private final SimpleServo hoodServo;
    private final SimpleServo alignmentIndicatorLight;

    private static final double TARGET_RPM = 250.0;
    private static final double TARGET_RPM_TOLERANCE = 50;
    public LaunchSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        leftLaunchMotor = new MotorEx(hardwareMap, "leftLaunchMotor", Motor.GoBILDA.BARE);
        leftLaunchMotor.setRunMode(Motor.RunMode.VelocityControl);
        rightLaunchMotor = new MotorEx(hardwareMap, "rightLaunchMotor", Motor.GoBILDA.BARE);
        rightLaunchMotor.setRunMode(Motor.RunMode.VelocityControl);
        rightLaunchMotor.setInverted(true);

        turretServo = new SimpleServo(hardwareMap, "turretServo", 0, 1800);
        hoodServo = new SimpleServo(hardwareMap, "launchHoodServo", 0, 270);
        alignmentIndicatorLight = new SimpleServo(hardwareMap, "alignmentIndicatorLight", 0, 270);
    }

    public void setAlignmentLightColor(double color) {
        alignmentIndicatorLight.setPosition(color);
    }

    public void setTurretPosition(double position) {
        turretServo.setPosition(position);
    }

    public double getTurretPosition() {
        // TODO: UPDATE THIS TO USE ENCODERS IF POSSIBLE
        return turretServo.getPosition();
    }

    public void setHoodPosition(double position) {
        hoodServo.setPosition(position);
    }

    public double getHoodPosition() {
        return hoodServo.getPosition();
    }

    /*
    * Spins up the flywheel to the specified RPM. Converts to TPS internally
    * */
    public void spinUpFlywheelToRPM(double targetRPM) {
        double ticksPerSecond = (targetRPM * Motor.GoBILDA.BARE.getCPR()) / 60.0;
        leftLaunchMotor.setVelocity(ticksPerSecond);
        rightLaunchMotor.setVelocity(ticksPerSecond);
    }

    public double getCurrentFlywheelTPS() {
        return (leftLaunchMotor.getVelocity() + rightLaunchMotor.getVelocity()) / 2.0;
    }

    // TODO: dummy functions from the motor version to compile the code.
    //remove once decided.
    public double getTurretAngle() { return 0; }
    public void alignTurretToAngle(double currentAngle, double targetAngle) {}
    public void stop() {
        leftLaunchMotor.setVelocity(0);
        rightLaunchMotor.setVelocity(0);
    }

    /**
     * Checks if the flywheel, turret, and hood are all within acceptable tolerances
     * to ensure a successful shot.
     *
     * @param targetRPM The RPM we are aiming for
     * @param targetTurretPos The servo position (0.0 to 1.0) the turret should be at
     * @param targetHoodPos The servo position (0.0 to 1.0) the hood should be at
     * @return true if all systems are ready
     */
    public boolean isReadyToLaunch(double targetRPM, double targetTurretPos, double targetHoodPos) {
        // 1. Check Flywheel RPM
        // We convert current TPS back to RPM for comparison
        double currentRPM = (getCurrentFlywheelTPS() * 60.0) / Motor.GoBILDA.BARE.getCPR();
        boolean flywheelReady = Math.abs(currentRPM - targetRPM) < TARGET_RPM_TOLERANCE;

        // 2. Check Turret Alignment
        // Servos move fast, but we check if it has reached the target
        // A tolerance of 0.05 is usually safe for servo positioning
        boolean turretReady = Math.abs(getTurretPosition() - targetTurretPos) < 0.05;

        // 3. Check Hood Alignment
        boolean hoodReady = Math.abs(getHoodPosition() - targetHoodPos) < 0.05;

        return flywheelReady && turretReady && hoodReady;
    }

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