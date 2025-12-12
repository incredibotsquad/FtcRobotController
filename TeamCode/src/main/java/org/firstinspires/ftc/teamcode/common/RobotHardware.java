package org.firstinspires.ftc.teamcode.common;

import static org.firstinspires.ftc.teamcode.subsystems.Spindex.SPINDEX_VELOCITY;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
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
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class RobotHardware {

    public static double COLOR_DETECTION_RETRY_DURATION_MILLIS = 200;

    public HardwareMap hardwareMap;
    private DcMotorEx frontRightDriveMotor;
    private DcMotorEx frontLeftDriveMotor;
    private DcMotorEx backRightDriveMotor;
    private DcMotorEx backLeftDriveMotor;

    private DcMotorEx intakeMotor;
    private Servo colorSensorLights;
    private DigitalChannel ballIntakeSensor;
    private ColorRangeSensor leftColorSensor;
    private ColorRangeSensor rightColorSensor;
    private ColorRangeSensor backColorSensor;


    private DcMotorEx flywheelMotor1;
    private DcMotorEx flywheelMotor2;

    private Servo launchTurretServo;
    private Servo launchVisorServo;
    private AnalogInput visorServoEncoder;
    private Servo launchKickServo;

    private DcMotorEx spindexMotor;
    private TouchSensor spindexLimitSwitch;

    private Servo alignmentIndicatorLight;
    private Servo spindexStatusLight;
    private Limelight3A limelight;


    //making constructor
    public RobotHardware(HardwareMap hwMap) {

        this.hardwareMap = hwMap;

        frontRightDriveMotor = hardwareMap.get(DcMotorEx.class, "FRMotor");
        frontRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //this motor is oriented backwards, hence reversing direction
        frontLeftDriveMotor = hardwareMap.get(DcMotorEx.class, "FLMotor");
        frontLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        backRightDriveMotor = hardwareMap.get(DcMotorEx.class, "BRMotor");
        backRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //this motor is oriented backwards, hence reversing direction
        backLeftDriveMotor = hardwareMap.get(DcMotorEx.class, "BLMotor");
        backLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //intake motor
        intakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //flywheel motor
        flywheelMotor1 = hardwareMap.get(DcMotorEx.class, "FlywheelMotor1");
        flywheelMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "FlywheelMotor2");
        flywheelMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //servos
        launchTurretServo = hardwareMap.get(Servo.class, "LaunchTurretServo");
        launchKickServo = hardwareMap.get(Servo.class, "LaunchKickServo");

        launchVisorServo = hardwareMap.get(Servo.class, "LaunchVisorServo");
        visorServoEncoder = hardwareMap.get(AnalogInput.class, "VisorServoEncoder");

        spindexMotor = hardwareMap.get(DcMotorEx.class, "SpindexMotor");
        spindexMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindexMotor.setTargetPosition(0);
        spindexMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        spindexLimitSwitch = hardwareMap.get(TouchSensor.class, "SpindexLimitSwitch");

        colorSensorLights = hardwareMap.get(Servo.class, "ColorSensorLights");
        colorSensorLights.setPosition(0.5);    //turn on for color sensing

        //color sensors
        leftColorSensor = hardwareMap.get(ColorRangeSensor.class, "LeftColorSensor");
        rightColorSensor = hardwareMap.get(ColorRangeSensor.class, "RightColorSensor");
        backColorSensor = hardwareMap.get(ColorRangeSensor.class, "BackColorSensor");

        //laser sensor
        ballIntakeSensor = hardwareMap.get(DigitalChannel.class, "BallIntakeSensor");;
        ballIntakeSensor.setMode(DigitalChannel.Mode.INPUT);

        //signal lights
        alignmentIndicatorLight = hardwareMap.get(Servo.class, "AlignmentIndicatorLight");
        spindexStatusLight = hardwareMap.get(Servo.class, "SpindexStatusLight");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
//        limelight.updateRobotOrientation(0);
    }

    public LLResult getLatestLimelightResults() {
//        Log.i("== ROBOTHARDWARE ==", " GetLatestLimelightResults");

        LLResult result = null;
        if (limelight != null) {
            result = limelight.getLatestResult();
        }

        return result;
    }

    public void updateLimelightYaw(double yaw) {
        limelight.updateRobotOrientation(yaw);
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
        stopSpindex();
    }

    public void setIntakeMotorPower(double power) {
//        Log.i("=== ROBOTHARDWARE  ===", " setIntakeMotorPower: " + power);
        intakeMotor.setPower(power);
    }

    public double getFlywheelMotorVelocityInTPS() {
//        Log.i("=== ROBOTHARDWARE  ===", " getFlywheelMotorVelocityInTPS: ");
        //motor 1 is primary - we use encoder only on that.
        return flywheelMotor2.getVelocity();
    }

    public void setFlywheelMotorVelocityInTPS(double velocity) {
//        Log.i("=== ROBOTHARDWARE  ===", " setFlywheelMotorVelocityInTPS: " + velocity);
        //motor 1 just follows motor 2 - setvelocity on both but we read only from motor 2
        flywheelMotor1.setVelocity(velocity);
        flywheelMotor2.setVelocity(velocity);
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

    public double getLaunchVisorPositionFromEncoder() {

        double voltage = visorServoEncoder.getVoltage();
        double position = 1 - (voltage / 3.3);  //position via encoder seems to be flipped

//        Log.i("=== ROBOTHARDWARE  ===", " getLaunchVisorPosition: " + position);
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

    public void stopSpindex()
    {
        Log.i("=== ROBOTHARDWARE  ===", " stopSpindex");
        spindexMotor.setPower(0);
    }

    public int getSpindexPosition () {
        int retVal = spindexMotor.getCurrentPosition();
//        Log.i("=== ROBOTHARDWARE ===", " getSpindexPosition: " + retVal);
        return retVal;
    }

    public void setSpindexPosition(int pos) {
//        Log.i("=== ROBOTHARDWARE ===", " setSpindexPosition: " + pos);
        setSpindexPositionAndVelocity(pos, SPINDEX_VELOCITY);
    }

    public void setSpindexPositionAndVelocity(int pos, int velocity) {
        spindexMotor.setTargetPosition(pos);
        spindexMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexMotor.setVelocity(velocity);
        Log.i("=== ROBOTHARDWARE ===", " setSpindexPositionAndVelocity: " + pos + " AND VELOCITY: " + velocity);
    }

    public void stopSpindexAndResetEncoder()
    {
        Log.i("=== ROBOTHARDWARE  ===", " stopSpindexAndResetEncoder");
        stopSpindex();
        spindexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setSpindexPosition(0);
    }

    public boolean isSpindexLimitSwitchTriggered()
    {
        return spindexLimitSwitch.isPressed();
    }

    public boolean isSpindexBusy() {
        boolean retVal = spindexMotor.isBusy();
//        Log.i("=== ROBOTHARDWARE  ===", " isSpindexBusy: " + retVal);
        return retVal;
    }

    public boolean didBallDetectionBeamBreak() {
        // Read the sensor state (true = HIGH, false = LOW)
        // HIGH means an object is detected
        boolean detected = ballIntakeSensor.getState();
//        Log.i("=== ROBOTHARDWARE  ===", " isBallPresentInIntake: " + detected);
        return detected;
    }

    public GameColors getDetectedBallColorFromLeftSensor() {
        GameColors detectedColor = GameColors.UNKNOWN;

        detectedColor = getDetectedColorFromSensor(leftColorSensor);
        Log.i("=== ROBOTHARDWARE  ===", " Left Detected Color: " + detectedColor);

        return detectedColor;
    }

    public GameColors getDetectedBallColorFromRightSensor() {
        GameColors detectedColor = GameColors.UNKNOWN;

        detectedColor = getDetectedColorFromSensor(rightColorSensor);
        Log.i("=== ROBOTHARDWARE  ===", " Right Detected Color: " + detectedColor);

        return detectedColor;
    }

    public GameColors getDetectedBallColorFromBackSensor() {
        GameColors detectedColor = GameColors.UNKNOWN;

        detectedColor = getDetectedColorFromSensor(backColorSensor);
        Log.i("=== ROBOTHARDWARE  ===", " Back Detected Color: " + detectedColor);

        return detectedColor;
    }

    private GameColors getDetectedColorFromSensor(ColorRangeSensor sensor) {
        GameColors detectedColor = GameColors.UNKNOWN;
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        do {
            NormalizedRGBA normalizedRGBA = sensor.getNormalizedColors();

//            Log.i("=== ROBOTHARDWARE  ===", "COLOR SENSOR NORMALIZED R: " + normalizedRGBA.red + " G: " + normalizedRGBA.green + " B: " + normalizedRGBA.blue);

            if (normalizedRGBA.green > normalizedRGBA.blue && normalizedRGBA.green > normalizedRGBA.red)  {
                detectedColor = GameColors.GREEN;
            }

            if (normalizedRGBA.blue > normalizedRGBA.green && normalizedRGBA.blue > normalizedRGBA.red) {
                detectedColor = GameColors.PURPLE;
            }

        } while (detectedColor == GameColors.UNKNOWN && timer.milliseconds() < COLOR_DETECTION_RETRY_DURATION_MILLIS);

        return detectedColor;
    }


    public void setAlignmentLightColor(double color) {
        alignmentIndicatorLight.setPosition(color);
    }

    public void setspindexStatusLightColor(double color) {
        spindexStatusLight.setPosition(color);
    }

    public void setColorSensorLightColor(double color) {
        colorSensorLights.setPosition(color);
    }
}