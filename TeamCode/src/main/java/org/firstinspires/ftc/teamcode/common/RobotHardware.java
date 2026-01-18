package org.firstinspires.ftc.teamcode.common;

import static org.firstinspires.ftc.teamcode.Actions.SpindexAction.SPINDEX_POSITION_TOLERANCE;
import static org.firstinspires.ftc.teamcode.subsystems.LaunchSystem.TURRET_VELOCITY_COARSE;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class RobotHardware {

    public static double COLOR_DETECTION_RETRY_DURATION_MILLIS = 50;
    public static double BEAM_BREAK_DETECTION_RETRY_DURATION_MILLIS = 50;

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

    private Servo spindexServo;
    private AnalogInput spindexServoEncoder;

    private Servo launchVisorServo;
    private AnalogInput visorServoEncoder;
    private Servo launchKickServo;
    private Servo leftLiftServo;
    private Servo rightLiftServo;
    private Servo alignmentIndicatorLight;
    private Servo spindexStatusLight;
    private Limelight3A limelight;

    public static double FLYWHEEL_P = 210;
    public static double FLYWHEEL_I = 0;
    public static double FLYWHEEL_D = 0;
    public static double FLYWHEEL_F = 11.0;

    public static double TURRET_P = 12.0;
    public static double TURRET_I = 3.0;
    public static double TURRET_D = 0;
    public static double TURRET_F = 0;

    public Pose2d correctedRobotPoseViaLimelight = null;

//    private GoBildaPinpointDriver pinpointDriver;

    //making constructor
    public RobotHardware(HardwareMap hwMap) {

        this.hardwareMap = hwMap;

        //set bulk read more for all hubs
//        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
//        for (LynxModule hub : allHubs) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//        }

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

        //flywheel motors
        flywheelMotor1 = hardwareMap.get(DcMotorEx.class, "FlywheelMotor1");
        flywheelMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "FlywheelMotor2");
        flywheelMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients customPIDF = new PIDFCoefficients(FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D, FLYWHEEL_F);
        flywheelMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, customPIDF);
        flywheelMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, customPIDF);

        if (CrossOpModeStorage.launchTurretMotor == null) {
            CrossOpModeStorage.launchTurretMotor = hardwareMap.get(DcMotorEx.class, "LaunchTurretMotor");
            CrossOpModeStorage.launchTurretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            CrossOpModeStorage.launchTurretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            CrossOpModeStorage.launchTurretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            customPIDF = new PIDFCoefficients(TURRET_P, TURRET_I, TURRET_D, TURRET_F);
            CrossOpModeStorage.launchTurretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, customPIDF);
        }

        //spindex and encoder
        spindexServo = hardwareMap.get(Servo.class, "SpindexServo");
        spindexServoEncoder = hardwareMap.get(AnalogInput.class, "SpindexServoEncoder");

        //visor and encoder
        launchVisorServo = hardwareMap.get(Servo.class, "LaunchVisorServo");
        visorServoEncoder = hardwareMap.get(AnalogInput.class, "VisorServoEncoder");

        //kicker
        launchKickServo = hardwareMap.get(Servo.class, "LaunchKickServo");

        //lift linear servos
        leftLiftServo = hardwareMap.get(Servo.class, "LeftLiftServo");
        rightLiftServo = hardwareMap.get(Servo.class, "RightLiftServo");

        //diffused lights
        colorSensorLights = hardwareMap.get(Servo.class, "ColorSensorLights");
        colorSensorLights.setPosition(0.5);    //turn on for color sensing

        //signal lights
        alignmentIndicatorLight = hardwareMap.get(Servo.class, "AlignmentIndicatorLight");
        spindexStatusLight = hardwareMap.get(Servo.class, "SpindexStatusLight");

        //color sensors
        leftColorSensor = hardwareMap.get(ColorRangeSensor.class, "LeftColorSensor");
        rightColorSensor = hardwareMap.get(ColorRangeSensor.class, "RightColorSensor");
        backColorSensor = hardwareMap.get(ColorRangeSensor.class, "BackColorSensor");

        //laser sensor
        ballIntakeSensor = hardwareMap.get(DigitalChannel.class, "BallIntakeSensor");;
        ballIntakeSensor.setMode(DigitalChannel.Mode.INPUT);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(250); // This sets how often we ask Limelight for data (100 times per second)

//        pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "PinpointOdo");

//        limelight.updateRobotOrientation(0);
    }

//    public void resetPinpoint() {
//        pinpointDriver.recalibrateIMU();
//        pinpointDriver.resetPosAndIMU();
//        Log.i("== ROBOTHARDWARE ==", " resetPinpoint");
//    }

    public LLResult getLatestLimelightResults() {
//        Log.i("== ROBOTHARDWARE ==", " GetLatestLimelightResults");

        LLResult result = null;
        if (limelight != null) {
            result = limelight.getLatestResult();
        }

        return result;
    }

    public void updateLimelightYawDegrees(double yaw) {
        limelight.updateRobotOrientation(yaw);
    }


    public void setLimelightPipeline(int pipeline) {
        if (limelight != null) {
            Log.i("ROBOT HARDWARE", "setLimelightPipeline. Switched to: " + pipeline);

            limelight.pipelineSwitch(pipeline);
        }
    }

    //5 pattern
    //6 is red
    //7 is blue
    public void setLimelightPipeline(AllianceColors allianceColor){
        Log.i("ROBOT HARDWARE", "setLimelightPipeline");

        switch (allianceColor) {
            case OBELISK:
                setLimelightPipeline(5);
                break;
            case RED:
                setLimelightPipeline(6);
                break;
            case BLUE:
                setLimelightPipeline(7);
                break;
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
        setFlywheelVelocityInTPS(0);
    }

    public void setIntakeMotorPower(double power) {
//        Log.i("=== ROBOTHARDWARE  ===", " setIntakeMotorPower: " + power);
        intakeMotor.setPower(power);
    }

    public double getFlywheelVelocityInTPS() {
//        Log.i("=== ROBOTHARDWARE  ===", " getFlywheelMotorVelocityInTPS: ");
        //motor 1 is primary - we use encoder only on that.
        return flywheelMotor2.getVelocity();
    }

    public void setFlywheelVelocityInTPS(double velocity) {
//        Log.i("=== ROBOTHARDWARE  ===", " setFlywheelMotorVelocityInTPS: " + velocity);
        //motor 1 just follows motor 2 - setvelocity on both but we read only from motor 2
        flywheelMotor1.setVelocity(velocity);
        flywheelMotor2.setVelocity(velocity);
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

    public double getSpindexPosition() {
        double position = spindexServo.getPosition();

//        Log.i("=== ROBOTHARDWARE  ===", " getSpindexPosition: " + position);
        return position;
    }

    public double getSpindexPositionFromEncoder() {
        double voltage = spindexServoEncoder.getVoltage();
        double position = 1 - (voltage / 3.3);  //position via encoder seems to be flipped

//        Log.i("=== ROBOTHARDWARE  ===", " getSpindexPositionFromEncoder: " + position);
        return position;
    }

    public void setSpindexPosition(double position) {
//        Log.i("=== ROBOTHARDWARE  ===", " setSpindexPosition: " + position);
        spindexServo.setPosition(position);
    }

    public boolean isSpindexBusy() {
        boolean retVal = false;

        //if position from encoder and the last setposition differ by more than delta, spindex is busy moving
        if (Math.abs(getSpindexPosition() - getSpindexPositionFromEncoder()) > SPINDEX_POSITION_TOLERANCE) {
            retVal = true;
        }

//        Log.i("=== ROBOTHARDWARE  ===", " isSpindexBusy: " + retVal);

        return retVal;
    }

    public void stopLaunchTurret()
    {
        Log.i("=== ROBOTHARDWARE  ===", " stopLaunchTurret");
        CrossOpModeStorage.launchTurretMotor.setPower(0);
    }

    public double getLaunchTurretPower() {
//        Log.i("=== ROBOTHARDWARE ===", " getLaunchTurretPower: " + power);

        return CrossOpModeStorage.launchTurretMotor.getPower();
    }

    public void setLaunchTurretPower(double power) {
//        Log.i("=== ROBOTHARDWARE ===", " setLaunchTurretPower: " + power);

        CrossOpModeStorage.launchTurretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        CrossOpModeStorage.launchTurretMotor.setPower(power);
    }

    public int getLaunchTurretPosition () {
        int retVal = CrossOpModeStorage.launchTurretMotor.getCurrentPosition();
//        Log.i("=== ROBOTHARDWARE ===", " getLaunchTurretPosition: " + retVal);
        return retVal;
    }

    public void setLaunchTurretPosition(int pos) {
//        Log.i("=== ROBOTHARDWARE ===", " setLaunchTurretPosition: " + pos);
        setLaunchTurretPositionAndVelocity(pos, TURRET_VELOCITY_COARSE);
    }

    public void setLaunchTurretPositionAndVelocity(int pos, int velocity) {
        CrossOpModeStorage.launchTurretMotor.setTargetPosition(pos);
        CrossOpModeStorage.launchTurretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        CrossOpModeStorage.launchTurretMotor.setVelocity(velocity);
//        Log.i("=== ROBOTHARDWARE ===", " setLaunchTurretPositionAndVelocity: " + pos + " AND VELOCITY: " + velocity);
    }

    public void stopLaunchTurretAndResetEncoder()
    {
        Log.i("=== ROBOTHARDWARE  ===", " stopLaunchTurretAndResetEncoder");
        stopLaunchTurret();
        CrossOpModeStorage.launchTurretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setLaunchTurretPosition(0);
    }

    public double getLiftPosition() {
        double pos = Math.min(leftLiftServo.getPosition(), rightLiftServo.getPosition());
        Log.i("=== ROBOTHARDWARE  ===", " getLiftPosition: " + pos);
        return pos;
    }

    public void setLiftPosition(double position) {
        Log.i("=== ROBOTHARDWARE  ===", " setLiftPosition: ");
        leftLiftServo.setPosition(position);
        rightLiftServo.setPosition(position);
    }

    public boolean didBallDetectionBeamBreak() {
        // Read the sensor state (true = HIGH, false = LOW)
        // HIGH means an object is detected
        boolean detected = false;
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        do {
            detected = ballIntakeSensor.getState();
        } while (timer.milliseconds() < BEAM_BREAK_DETECTION_RETRY_DURATION_MILLIS);

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

        int greenCount = 0;
        int purplecount = 0;

        do {
            NormalizedRGBA normalizedRGBA = sensor.getNormalizedColors();

//            Log.i("=== ROBOTHARDWARE  ===", "COLOR SENSOR NORMALIZED R: " + normalizedRGBA.red + " G: " + normalizedRGBA.green + " B: " + normalizedRGBA.blue);

            if (normalizedRGBA.green > normalizedRGBA.blue && normalizedRGBA.green > normalizedRGBA.red)  {
                greenCount++;
//                detectedColor = GameColors.GREEN;
            }

            if (normalizedRGBA.blue > normalizedRGBA.green && normalizedRGBA.blue > normalizedRGBA.red) {
                purplecount++;
//                detectedColor = GameColors.PURPLE;
            }

        } while (timer.milliseconds() < COLOR_DETECTION_RETRY_DURATION_MILLIS);

        if (greenCount > purplecount)
            detectedColor = GameColors.GREEN;

        if (purplecount > greenCount)
            detectedColor = GameColors.PURPLE;

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