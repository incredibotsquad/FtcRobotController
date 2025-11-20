package org.firstinspires.ftc.teamcode.common;

import android.util.Log;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

public class RobotHardware {

    public static double COLOR_DETECTION_RETRY_DURATION_MILLIS = 200;

    public HardwareMap hardwareMap;
    private DcMotorEx frontRightDriveMotor;
    private DcMotorEx frontLeftDriveMotor;
    private DcMotorEx backRightDriveMotor;
    private DcMotorEx backLeftDriveMotor;
    private DcMotorEx intakeMotor;
    private DcMotorEx flywheelMotor;
    private Servo launchTurretServo;
    private Servo launchVisorServo;
    private AnalogInput visorServoEncoder;
    private Servo launchKickServo;
    private Servo spindexServo;
    private AnalogInput spindexServoEncoder;
    private Servo alignmentIndicatorLight;
    private Servo colorSensorLight;

    private ColorRangeSensor colorSensor;
    private Limelight3A limelight;
    private DigitalChannel ballIntakeSensor;


    private GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

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
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "LauncherMotor");
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //servos
        launchTurretServo = hardwareMap.get(Servo.class, "LaunchTurretServo");
        launchKickServo = hardwareMap.get(Servo.class, "LaunchKickServo");

        launchVisorServo = hardwareMap.get(Servo.class, "LaunchVisorServo");
        visorServoEncoder = hardwareMap.get(AnalogInput.class, "VisorServoEncoder");

        spindexServo = hardwareMap.get(Servo.class, "SpindexServo");
        spindexServoEncoder = hardwareMap.get(AnalogInput.class, "SpindexServoEncoder");

        alignmentIndicatorLight = hardwareMap.get(Servo.class, "AlignmentIndicatorLight");
        colorSensorLight = hardwareMap.get(Servo.class, "ColorSensorLight");
        colorSensorLight.setPosition(0.5);    //turn on for color sensing

        //color sensors
        colorSensor = hardwareMap.get(ColorRangeSensor.class, "IntakeColorSensor");

        //laser sensor
        ballIntakeSensor = hardwareMap.get(DigitalChannel.class, "BallIntakeSensor");;
        ballIntakeSensor.setMode(DigitalChannel.Mode.INPUT);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)

//        odo = hardwareMap.get(GoBildaPinpointDriver.class,"PinpointOdo");
//        odo.setOffsets(20.0, -241.3, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
//        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
//        odo.resetPosAndIMU();

    }

    public Pose2D getCurrentRobotPose() {
        odo.update();
        return odo.getPosition();
    }

    public LLResult GetLatestLimelightResults() {
        Log.i("== ROBOTHARDWARE ==", " GetLatestLimelightResults");

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
    public void setDriveMotorPowers(double frPower, double flPower, double brPower, double blPower) {
        frontRightDriveMotor.setPower(frPower);
        frontLeftDriveMotor.setPower(flPower);
        backRightDriveMotor.setPower(brPower);
        backLeftDriveMotor.setPower(blPower);
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
//        Log.i("=== ROBOTHARDWARE  ===", " setIntakeMotorPower: " + power);
        intakeMotor.setPower(power);
    }

    public double getFlywheelMotorVelocityInTPS() {
//        Log.i("=== ROBOTHARDWARE  ===", " getFlywheelMotorVelocityInTPS: ");
        return flywheelMotor.getVelocity();
    }

    public void setFlywheelMotorVelocityInTPS(double velocity) {
//        Log.i("=== ROBOTHARDWARE  ===", " setFlywheelMotorVelocityInTPS: " + velocity);
        flywheelMotor.setVelocity(velocity);
    }

    public double getLaunchTurretPosition() {
        double position = launchTurretServo.getPosition();
//        Log.i("=== ROBOTHARDWARE  ===", " getLaunchTurretServoPosition: " + position);
        return position;
    }
    public void setLaunchTurretPosition(double position) {
//        Log.i("=== ROBOTHARDWARE  ===", " setLaunchTurretServoPosition: " + position);
        launchTurretServo.setPosition(position);
    }

    public double getLaunchVisorPosition() {
//        Log.i("=== ROBOTHARDWARE  ===", " getLaunchVisorPosition: ");

        double voltage = visorServoEncoder.getVoltage();
        double position = 1 - (voltage / 3.3);  //position via encoder seems to be flipped

        Log.i("=== ROBOTHARDWARE  ===", " getLaunchVisorPosition: " + position);
        return position;
    }

    public void setLaunchVisorPosition(double position) {
//        Log.i("=== ROBOTHARDWARE  ===", " setLaunchVisorServoPosition: " + position);
        launchVisorServo.setPosition(position);
    }

    public void setLaunchKickPosition(double position) {
//        Log.i("=== ROBOTHARDWARE  ===", " setLaunchKickPosition: " + position);
        launchKickServo.setPosition(position);
    }

    public double getSpindexPosition() {
//        Log.i("=== ROBOTHARDWARE  ===", " getSpindexPosition: ");

        double voltage = spindexServoEncoder.getVoltage();
        double position = 1 - (voltage / 3.3);  //position via encoder seems to be flipped

        Log.i("=== ROBOTHARDWARE  ===", " getSpindexPosition: " + position);
        return position;
    }

    public void setSpindexPosition(double position) {
//        Log.i("=== ROBOTHARDWARE  ===", " setSpindexPosition: " + position);
        spindexServo.setPosition(position);
    }

    public boolean didBallDetectionBeamBreak() {
        // Read the sensor state (true = HIGH, false = LOW)
        // HIGH means an object is detected
        boolean detected = ballIntakeSensor.getState();
//        Log.i("=== ROBOTHARDWARE  ===", " isBallPresentInIntake: " + detected);
        return detected;
    }

    public GameColors getDetectedBallColor() {
        GameColors detectedColor = GameColors.UNKNOWN;
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        do {
            NormalizedRGBA sensor1Colors = colorSensor.getNormalizedColors();

//            Log.i("Robot Hardware", "Color Sensor 1 R: " + sensor1Colors.red);
//            Log.i("Robot Hardware", "Color Sensor 1 G: " + sensor1Colors.green);
//            Log.i("Robot Hardware", "Color Sensor 1 B: " + sensor1Colors.blue);

            detectedColor = ColorClassifier.classify(sensor1Colors.red, sensor1Colors.green, sensor1Colors.blue);

        } while (detectedColor == GameColors.UNKNOWN && timer.milliseconds() < COLOR_DETECTION_RETRY_DURATION_MILLIS);

        Log.i("=== ROBOTHARDWARE  ===", " getDetectedBallColor: " + detectedColor);
        return detectedColor;
    }

    public void setAlignmentLightColor(double color) {
        alignmentIndicatorLight.setPosition(color);
    }

    public void setColorSensorLightColor(double color) {
        colorSensorLight.setPosition(color);
    }
}