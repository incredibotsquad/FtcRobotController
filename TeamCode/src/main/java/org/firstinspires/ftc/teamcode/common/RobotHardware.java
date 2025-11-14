package org.firstinspires.ftc.teamcode.common;

import android.util.Log;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

public class RobotHardware {

    public static double COLOR_SENSOR_DISTANCE_THRESHOLD_IN_MM = 60;

    public HardwareMap hardwareMap;
    IMU imu;
    private DcMotorEx frontRightDriveMotor;
    private DcMotorEx frontLeftDriveMotor;
    private DcMotorEx backRightDriveMotor;
    private DcMotorEx backLeftDriveMotor;
    private DcMotorEx intakeMotor;
    private DcMotorEx flywheelMotor;

    private Servo launchGateServo;
    private Servo spindexServo;
    private Servo kickServo;
    private Servo intakeLightServo;


    private ColorRangeSensor colorSensor1;
    private ColorRangeSensor colorSensor2;
    private Limelight3A limelight;
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
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "LaunchMotor");
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //servos
        launchGateServo = hardwareMap.get(Servo.class, "LaunchGateServo");
        spindexServo = hardwareMap.get(Servo.class, "SpindexServo");
        kickServo = hardwareMap.get(Servo.class, "KickServo");

        //color sensors
        colorSensor1 = hardwareMap.get(ColorRangeSensor.class, "IntakeColorSensor1");
        colorSensor2 = hardwareMap.get(ColorRangeSensor.class, "IntakeColorSensor2");

        intakeLightServo = hardwareMap.get(Servo.class, "IntakeLightServo");

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

    public void setLaunchGatePosition(double position) {
        Log.i("=== ROBOTHARDWARE  ===", " setLaunchGatePosition: " + position);
        launchGateServo.setPosition(position);
    }

    public double getSpindexPosition() {
//        Log.i("=== ROBOTHARDWARE  ===", " getSpindexPosition: ");
        return spindexServo.getPosition();
    }

    public void setSpindexPosition(double position) {
//        Log.i("=== ROBOTHARDWARE  ===", " setSpindexPosition: " + position);
        spindexServo.setPosition(position);
    }

    public void setLaunchKickPosition(double position) {
//        Log.i("=== ROBOTHARDWARE  ===", " setLaunchKickPosition: " + position);
        kickServo.setPosition(position);
    }

    public GameColors getDetectedBallColor() {
        NormalizedRGBA sensor1Colors = colorSensor1.getNormalizedColors();

        double sensor1Distance = colorSensor1.getDistance(DistanceUnit.MM);
        if ((sensor1Distance < COLOR_SENSOR_DISTANCE_THRESHOLD_IN_MM)) {

//            Log.i("Robot Hardware", "Color Sensor Distance Breached");
//            Log.i("Robot Hardware", "Color Sensor 1 R: " + sensor1Colors.red);
//            Log.i("Robot Hardware", "Color Sensor 1 G: " + sensor1Colors.green);
//            Log.i("Robot Hardware", "Color Sensor 1 B: " + sensor1Colors.blue);

            return ColorClassifier.classify(sensor1Colors.red, sensor1Colors.green, sensor1Colors.blue);
        }

        return GameColors.NONE;
    }

//    public GameColors getDetectedBallColor() {
//
//        GameColors detectedColor = GameColors.NONE;
//        double sensor1Distance = colorSensor1.getDistance(DistanceUnit.MM);
//        double sensor2Distance = colorSensor2.getDistance(DistanceUnit.MM);
//
//        if ((sensor1Distance < COLOR_SENSOR_DISTANCE_THRESHOLD_IN_MM) || (sensor2Distance < COLOR_SENSOR_DISTANCE_THRESHOLD_IN_MM))
//        {
////            Log.i("COLOR SENSORS", "PASSED DISTANCE THRESHOLD: ");
////            Log.i("COLOR SENSORS", "DISTANCE 1: " + sensor1Distance);
////            Log.i("COLOR SENSORS", "DISTANCE 2: " + sensor2Distance);
////
////
////            Log.i("COLOR SENSORS", "Sensor 1 R: " + colorSensor1.red());
////            Log.i("COLOR SENSORS", "Sensor 1 G: " + colorSensor1.green());
////            Log.i("COLOR SENSORS", "Sensor 1 B: " + colorSensor1.blue());
//
//            if ((colorSensor1.green() > colorSensor1.blue() && colorSensor1.green() > colorSensor1.red()) &&
//                    (colorSensor2.green() > colorSensor2.blue() && colorSensor2.green() > colorSensor2.red())) {
//                detectedColor = GameColors.GREEN;
//                Log.i("COLOR SENSORS", "DETECTED GREEN");
//            }
//
////            Log.i("COLOR SENSORS", "Sensor 2 R: " + colorSensor2.red());
////            Log.i("COLOR SENSORS", "Sensor 2 G: " + colorSensor2.green());
////            Log.i("COLOR SENSORS", "Sensor 2 B: " + colorSensor2.blue());
//
//            if ((colorSensor1.blue() > colorSensor1.green() && colorSensor1.blue() > colorSensor1.red()) &&
//                    (colorSensor2.blue() > colorSensor2.green() && colorSensor2.blue() > colorSensor2.red())) {
//                detectedColor = GameColors.PURPLE;
//                Log.i("COLOR SENSORS", "DETECTED PURPLE");
//            }
//        }
//
//        return detectedColor;
//    }

//        public GameColors getDetectedBallColor() {
//
//            double sensor1Distance = colorSensor1.getDistance(DistanceUnit.MM);
//            double sensor2Distance = colorSensor2.getDistance(DistanceUnit.MM);
//
//            //|| (sensor2Distance < COLOR_SENSOR_DISTANCE_THRESHOLD_IN_MM)
//
//            if ((sensor1Distance < COLOR_SENSOR_DISTANCE_THRESHOLD_IN_MM) ) {
//
//                Log.i("COLOR SENSORS", "DISTANCE BREACHED");
//
////                Log.i("COLOR SENSORS", "Sensor 1 R: " + sensor1Colors.red);
////                Log.i("COLOR SENSORS", "Sensor 1 G: " + sensor1Colors.green);
////                Log.i("COLOR SENSORS", "Sensor 1 B: " + sensor1Colors.blue);
////
////                Log.i("COLOR SENSORS", "Sensor 2 R: " + sensor2Colors.red);
////                Log.i("COLOR SENSORS", "Sensor 2 G: " + sensor2Colors.green);
////                Log.i("COLOR SENSORS", "Sensor 2 B: " + sensor2Colors.blue);
//
//
//                if ((colorSensor1.blue() > colorSensor1.green() && colorSensor1.blue() > colorSensor1.red()) &&
//                        (colorSensor2.blue() > colorSensor2.green() && colorSensor2.blue() > colorSensor2.red())) {
//
//                    Log.i("COLOR SENSORS", "DETECTED PURPLE");
//                    return GameColors.PURPLE;
//                }
//
//                if ((colorSensor1.green() > colorSensor1.blue() && colorSensor1.green() > colorSensor1.red()) &&
//                        (colorSensor2.green() > colorSensor2.blue() && colorSensor2.green() > colorSensor2.red())) {
//
//                    Log.i("COLOR SENSORS", "DETECTED GREEN");
//                    return GameColors.GREEN;
//                }
//
//                NormalizedRGBA sensor1Colors = colorSensor1.getNormalizedColors();
//                NormalizedRGBA sensor2Colors = colorSensor2.getNormalizedColors();
//
//                GameColors colorSensor1Classification = classifyGreenOrPurple(sensor1Colors.red * 255, sensor1Colors.green * 255, sensor1Colors.blue * 255);
//                GameColors colorSensor2Classification = classifyGreenOrPurple(sensor2Colors.red * 255, sensor2Colors.green * 255, sensor2Colors.blue * 255);
//
//                if (colorSensor1Classification == GameColors.NONE && colorSensor2Classification == GameColors.NONE) {
//                    Log.i("COLOR SENSORS", "DETECTED NONE");
//                    return GameColors.NONE;
//                }
//
//                Log.i("COLOR SENSOR", "DETECTED UNKNOWN");
//                return GameColors.UNKNOWN;
//            }
//
//            return GameColors.NONE;
//        }



    public void setIntakeLightColor(double color) {
        intakeLightServo.setPosition(color);
    }
}