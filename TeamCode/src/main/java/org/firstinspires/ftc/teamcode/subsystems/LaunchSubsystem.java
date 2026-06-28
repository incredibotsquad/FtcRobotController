package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LaunchSubsystem extends SubsystemBase {

    private final MotorEx leftLaunchMotor;
    private final MotorEx rightLaunchMotor;

//
//    private final DcMotorEx leftLaunchMotor;
//    private final DcMotorEx rightLaunchMotor;

    private final SimpleServo turretServo;
    private final SimpleServo hoodServo;
//    private final SimpleServo alignmentIndicatorLight;

    private static final double TARGET_RPM = 250.0;
    private static final double TARGET_RPM_TOLERANCE = 50;
    private static final double TURRET_POSITION_TOLERANCE = 0.05;
    private static final double HOOD_POSITION_TOLERANCE = 0.05;

    private volatile double targetRPM;
    private volatile double targetTurretPos;
    private volatile double targetHoodPos;

    public static double ROBOT_NOT_ALIGNED_TO_SHOOT_LIGHT = 0.3;    //RED
    public static double ROBOT_ALIGNED_TO_SHOOT_LIGHT = 0.5;    //GREEN
    public static double ROBOT_ALIGNMENT_NOT_POSSIBLE_LIGHT = 0;


    // Inside LaunchSubsystem
    private PIDController flywheelPID = new PIDController(0.005, 0, 0);
    // ks = static friction, kv = velocity gain (How much power to hold a speed)
    private SimpleMotorFeedforward flywheelFF = new SimpleMotorFeedforward(0, 0.00045);


    public LaunchSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        leftLaunchMotor = new MotorEx(hardwareMap, "leftLaunchMotor", Motor.GoBILDA.BARE);
        leftLaunchMotor.setRunMode(Motor.RunMode.RawPower);

        rightLaunchMotor = new MotorEx(hardwareMap, "rightLaunchMotor", Motor.GoBILDA.BARE);
        rightLaunchMotor.setRunMode(Motor.RunMode.RawPower);
        rightLaunchMotor.setInverted(true);


//        leftLaunchMotor = hardwareMap.get(DcMotorEx.class,"leftLaunchMotor");
//        leftLaunchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        rightLaunchMotor = hardwareMap.get(DcMotorEx.class,"rightLaunchMotor");
//        rightLaunchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightLaunchMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        turretServo = new SimpleServo(hardwareMap, "turretServo", 0, 1800);
        hoodServo = new SimpleServo(hardwareMap, "launchHoodServo", 0, 270);
//        alignmentIndicatorLight = new SimpleServo(hardwareMap, "alignmentIndicatorLight", 0, 270);
    }

    public void updateFlywheel(double targetRPM) {
        this.targetRPM = targetRPM;

        // Convert RPM to Ticks Per Second
        double targetTPS = (targetRPM * Motor.GoBILDA.BARE.getCPR()) / 60.0;

        // Get current velocity from encoders
        double currentTPS = getFlywheelVelocityTPS();

        // 1. Calculate PID (Correction)
        double pidOutput = flywheelPID.calculate(currentTPS, targetTPS);

        // 2. Calculate Feedforward (The "Predicted" power needed for this speed)
        double ffOutput = flywheelFF.calculate(targetTPS);

        // 3. Combine and set power (Manual mode)
        double totalPower = pidOutput + ffOutput;

        leftLaunchMotor.set(totalPower);
        rightLaunchMotor.set(totalPower);
    }


    public void setAlignmentLightColor(double color) {
//        alignmentIndicatorLight.setPosition(color);
    }

    public void setTurretPosition(double position) {
        this.targetTurretPos = position;
        turretServo.setPosition(position);
    }

    public double getTurretPosition() {
        // TODO: UPDATE THIS TO USE ENCODERS IF POSSIBLE
        return turretServo.getPosition();
    }

    public void setHoodPosition(double position) {
        this.targetHoodPos = position;
        hoodServo.setPosition(position);
    }

    public double getHoodPosition() {
        return hoodServo.getPosition();
    }

    /**
     * Allows live tuning of the PID coefficients from an OpMode.
     */
    public void setFlywheelPID(double p, double i, double d) {
        flywheelPID.setPID(p, i, d);
    }

    /**
     * Allows live tuning of the Feedforward coefficients.
     * Note: SimpleMotorFeedforward is immutable, so we create a new instance.
     *
     * @param ks Static friction (voltage to overcome friction)
     * @param kv Velocity gain (voltage per tick/second)
     */
    public void updateFeedforward(double ks, double kv) {
        flywheelFF = new SimpleMotorFeedforward(ks, kv);
    }

    /**
     * Exposes the PID controller for advanced tracking/telemetry
     */
    public PIDController getPIDController() {
        return flywheelPID;
    }

//    public void setFlywheelPIDF(double p, double i, double d, double f) {
//        leftLaunchMotor.setPIDFCoefficients(
//                DcMotor.RunMode.RUN_USING_ENCODER,
//                new PIDFCoefficients(p, i, d, f)
//        );
//
//        rightLaunchMotor.setPIDFCoefficients(
//                DcMotor.RunMode.RUN_USING_ENCODER,
//                new PIDFCoefficients(p, i, d, f)
//        );
//    }

    /*
    * Spins up the flywheel to the specified RPM. Converts to TPS internally
    * */
    public void spinUpFlywheelToRPM(double targetRPM) {
        this.targetRPM = targetRPM;
        double ticksPerSecond = (targetRPM * Motor.GoBILDA.BARE.getCPR()) / 60.0;
        leftLaunchMotor.setVelocity(ticksPerSecond);
        rightLaunchMotor.setVelocity(ticksPerSecond);
    }

    public double getFlywheelVelocityTPS() {
        return (leftLaunchMotor.getVelocity() + rightLaunchMotor.getVelocity()) / 2.0;
    }
    public double getCurrentFlywheelRPM() {
        return (getFlywheelVelocityTPS() * 60.0) / Motor.GoBILDA.BARE.getCPR();
    }

    // TODO: dummy functions from the motor version to compile the code.
    //remove once decided.
    public double getTurretAngle() { return 0; }
    public void alignTurretToAngle(double currentAngle, double targetAngle) {}
    public void stop() {
//        leftLaunchMotor.setVelocity(0);
//        rightLaunchMotor.setVelocity(0);

        this.targetRPM = 0;
        leftLaunchMotor.set(0);
        rightLaunchMotor.set(0);
    }

    /**
     * Checks if the flywheel, turret, and hood are all within acceptable tolerances
     * to ensure a successful shot.
     *
     * @return true if all systems are ready
     */
    public boolean isReadyToLaunch() {
        // 1. Check Flywheel RPM
        boolean flywheelReady = Math.abs(getCurrentFlywheelRPM() - targetRPM) < TARGET_RPM_TOLERANCE;

        // 2. Check Turret Alignment
        // Servos move fast, but we check if it has reached the target
        // A small tolerance is used for servo positioning
        boolean turretReady = Math.abs(getTurretPosition() - targetTurretPos) < TURRET_POSITION_TOLERANCE;

        // 3. Check Hood Alignment
        boolean hoodReady = Math.abs(getHoodPosition() - targetHoodPos) < HOOD_POSITION_TOLERANCE;

        return flywheelReady && turretReady && hoodReady;
    }

    /*
     * This method is called periodically by the CommandScheduler. Useful for updating subsystem-specific state that you don't want to offload to a Command.
     * Teams should try to be consistent within their own codebases about which responsibilities will be handled by Commands, and which will be handled here.
     * Excellent for sending telemetry or updating dashboards, but optional!
     * */
    @Override
    public void periodic() {
        if (targetRPM > 0) {
            updateFlywheel(targetRPM);
        } else {
            stop();
        }

        //handle the alignment indicator light
        if (isReadyToLaunch())
            setAlignmentLightColor(ROBOT_ALIGNED_TO_SHOOT_LIGHT);
        else
            setAlignmentLightColor(ROBOT_NOT_ALIGNED_TO_SHOOT_LIGHT);

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