package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class RobotHardware {

    public HardwareMap hardwareMap;
    IMU imu;
    private DcMotorEx frontRightDriveMotor;
    private DcMotorEx frontLeftDriveMotor;
    private DcMotorEx backRightDriveMotor;
    private DcMotorEx backLeftDriveMotor;
    private DcMotor odoPara;
    private DcMotor odoPerp;

    private DcMotorEx intakeMotor;

    private DcMotorEx flywheelMotor;
    private Servo launchGateServo;

    private Servo spindexServo;
    private Servo kickServo;
    private Servo visorServo;

    private ColorRangeSensor colorSensor;
    private AnalogInput verticalShoulderServoEncoder;
    private Limelight3A limelight;

    //making constructor
    public RobotHardware(HardwareMap hwMap) {

        this.hardwareMap = hwMap;

        frontRightDriveMotor = hardwareMap.get(DcMotorEx.class, "FRMotor");
        frontRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //this motor is oriented backwards, hence reversing direction
        frontLeftDriveMotor = hardwareMap.get(DcMotorEx.class, "FLMotor");
        frontLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backRightDriveMotor = hardwareMap.get(DcMotorEx.class, "BRMotor");
        backRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //this motor is oriented backwards, hence reversing direction
        backLeftDriveMotor = hardwareMap.get(DcMotorEx.class, "BLMotor");
        backLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //intake motor
        intakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //flywheel motor
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "LaunchMotor");
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //servos
        launchGateServo = hardwareMap.get(Servo.class, "LaunchGateServo");
        spindexServo = hardwareMap.get(Servo.class, "SpindexServo");
        kickServo = hardwareMap.get(Servo.class, "KickServo");
        visorServo = hardwareMap.get(Servo.class, "VisorServo");


//        imu = hardwareMap.get(IMU.class, "imu");
    }

    public LLResult GetLatestLimelightResults() {
        Log.i("=== ROBOTHARDWARE  ===", " GetLatestLimelightResults");

        LLResult result = null;
        if (limelight != null) {
            result = limelight.getLatestResult();
        }

        return result;
    }

    public void setLimelightPipeline(int pipeline) {
        if (limelight != null) {
            limelight.pipelineSwitch(pipeline);
        }
    }

    public void startLimelight() {
        if (limelight != null) {
            limelight.start();
        }
    }

    public void stopLimelight() {
        if (limelight != null) {
            limelight.stop();
        }
    }

    //sets the drive motor's powers
    public void setDriveMotorPowers(double rfPower, double lfPower, double rbPower, double lbPower) {
        frontRightDriveMotor.setPower(rfPower);
        frontLeftDriveMotor.setPower(lfPower);
        backRightDriveMotor.setPower(rbPower);
        backLeftDriveMotor.setPower(lbPower);
    }

    public void stopRobotChassis() {
        Log.i("=== ROBOTHARDWARE  ===", " stopRobotChassis");
        setDriveMotorPowers(0, 0, 0, 0);
    }

    public void stopRobotAndMechanisms() {
        Log.i("=== ROBOTHARDWARE  ===", " stopRobotAndMechanisms");
        stopRobotChassis();
        setIntakeMotorPower(0);
        setFlywheelMotorVelocityInTPS(0);
    }

    public void setIntakeMotorPower(double power) {
        Log.i("=== ROBOTHARDWARE  ===", " setIntakeMotorPower: " + power);
        intakeMotor.setPower(power);
    }

    public double getFlywheelMotorVelocityInTPS() {
        Log.i("=== ROBOTHARDWARE  ===", " getFlywheelMotorVelocityInTPS: ");
        return flywheelMotor.getVelocity();
    }

    public void setFlywheelMotorVelocityInTPS(double velocity) {
        Log.i("=== ROBOTHARDWARE  ===", " setFlywheelMotorVelocityInTPS: " + velocity);
        flywheelMotor.setVelocity(velocity);
    }

    public void setLaunchGatePosition(double position) {
        Log.i("=== ROBOTHARDWARE  ===", " setLaunchGatePosition: " + position);
        launchGateServo.setPosition(position);
    }

    public void setSpindexPosition(double position) {
        Log.i("=== ROBOTHARDWARE  ===", " setSpindexPosition: " + position);
        spindexServo.setPosition(position);
    }

    public void setLaunchKickPosition(double position) {
        Log.i("=== ROBOTHARDWARE  ===", " setLaunchKickPosition: " + position);
        kickServo.setPosition(position);
    }

    public void setLaunchVisorPosition(double position) {
        Log.i("=== ROBOTHARDWARE  ===", " setLaunchVisorPosition: " + position);
        visorServo.setPosition(position);
    }
}