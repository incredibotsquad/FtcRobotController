package org.firstinspires.ftc.teamcode;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
import com.qualcomm.robotcore.hardware.TouchSensor;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class RobotHardware {

    public HardwareMap hardwareMap;
    IMU imu;
    private DcMotorEx rightFrontDriveMotor;
    private DcMotorEx leftFrontDriveMotor;
    private DcMotorEx rightBackDriveMotor;
    private DcMotorEx leftBackDriveMotor;
    private DcMotor odoRight;
    private DcMotor odoLeft;
    private DcMotor odoFront;


    public static DcMotorEx horizontalSlideMotor;
    private Servo horizontalClawServo;
    private Servo horizontalWristServo;
    private Servo horizontalElbowServo;
    private Servo horizontalShoulderServo;
    private Servo horizontalTurretServo;
    private ColorRangeSensor colorSensor;
    private TouchSensor horizontalLimitSwitch;

    public static DcMotorEx verticalSlideMotor1;
    public static DcMotorEx verticalSlideMotor2;

    private Servo verticalClawServo;
    private Servo verticalWristServo;
    private Servo verticalElbowServo;
    private Servo verticalShoulderServo;
    private AnalogInput verticalShoulderServoEncoder;
    private TouchSensor verticalLimitSwitch;
    private Limelight3A limelight;


     //making constructor
     public RobotHardware(HardwareMap hwMap) {

         this.hardwareMap = hwMap;

         rightFrontDriveMotor = hardwareMap.get(DcMotorEx.class, "RFMotor");
         rightFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         rightFrontDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

         //this motor is oriented backwards, hence reversing direction
         leftFrontDriveMotor = hardwareMap.get(DcMotorEx.class, "LFMotor");
         leftFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         leftFrontDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         leftFrontDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

         rightBackDriveMotor = hardwareMap.get(DcMotorEx.class, "RBMotor");
         rightBackDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         rightBackDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

         //this motor is oriented backwards, hence reversing direction
         leftBackDriveMotor = hardwareMap.get(DcMotorEx.class, "LBMotor");
         leftBackDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         leftBackDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         leftBackDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

         odoRight = rightBackDriveMotor;
         odoLeft = leftBackDriveMotor;
         odoFront = rightFrontDriveMotor;

         // horizontal stack
         horizontalSlideMotor = hardwareMap.get(DcMotorEx.class, "HorizontalSlideMotor");
         horizontalSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         horizontalSlideMotor.setTargetPosition(0);
         horizontalSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         horizontalSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         horizontalClawServo = hardwareMap.get(Servo.class, "HorizontalClawServo");
         horizontalWristServo = hardwareMap.get(Servo.class, "HorizontalWristServo");
         horizontalElbowServo = hardwareMap.get(Servo.class, "HorizontalElbowServo");
         horizontalShoulderServo = hardwareMap.get(Servo.class, "HorizontalShoulderServo");
         horizontalTurretServo = hardwareMap.get(Servo.class, "HorizontalTurretServo");
         colorSensor = hardwareMap.get(ColorRangeSensor.class, "ColorSensor");
         horizontalLimitSwitch = hardwareMap.get(TouchSensor.class, "HorizontalLimitSwitch");


         // vertical stack
         verticalSlideMotor1 = hardwareMap.get(DcMotorEx.class, "VerticalSlideMotor1");
         verticalSlideMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         verticalSlideMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
         verticalSlideMotor1.setTargetPosition(0);
         verticalSlideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         verticalSlideMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         verticalSlideMotor2 = hardwareMap.get(DcMotorEx.class, "VerticalSlideMotor2");
         verticalSlideMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         verticalSlideMotor2.setTargetPosition(0);
         verticalSlideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         verticalSlideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         verticalClawServo = hardwareMap.get(Servo.class, "VerticalClawServo");
         verticalWristServo = hardwareMap.get(Servo.class, "VerticalWristServo");
         verticalElbowServo = hardwareMap.get(Servo.class, "VerticalElbowServo");
         verticalShoulderServo = hardwareMap.get(Servo.class, "VerticalShoulderServo");
         verticalShoulderServoEncoder = hardwareMap.get(AnalogInput.class, "AxonEncoder");

         verticalLimitSwitch = hardwareMap.get(TouchSensor.class, "VerticalLimitSwitch");

         limelight = hardwareMap.get(Limelight3A.class, "limelight");

         imu = hardwareMap.get(IMU.class, "imu");
     }

    public LLResult GetLatestLimelightResults() {
        Log.i("=== INCREDIBOTS / ROBOTHARDWARE  ===", " GetLatestLimelightResults");

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

     public void stopAllMechanisms()
     {
         Log.i("=== INCREDIBOTS / ROBOTHARDWARE  ===", " stopAllMechanisms");
         this.stopHorizontalSlide();
         this.stopVerticalSlide();
     }

     public void stopHorizontalSlide()
     {
         Log.i("=== INCREDIBOTS / ROBOTHARDWARE  ===", " stopHorizontalSlide");
         horizontalSlideMotor.setPower(0);
     }

    public void stopHorizontalSlideAndResetEncoder()
    {
        Log.i("=== INCREDIBOTS / ROBOTHARDWARE  ===", " stopHorizontalSlideAndResetEncoder");
        stopHorizontalSlide();
        horizontalSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setHorizontalSlidePosition(0);
    }

     public void stopVerticalSlide()
     {
         Log.i("=== INCREDIBOTS / ROBOTHARDWARE  ===", " stopVerticalSlide");
         verticalSlideMotor1.setPower(0);
         verticalSlideMotor2.setPower(0);
     }

    public void stopVerticalSlideAndResetEncoder()
    {
        Log.i("=== INCREDIBOTS / ROBOTHARDWARE  ===", " stopVerticalSlideAndResetEncoder");
        stopVerticalSlide();
        verticalSlideMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setVerticalSlidePosition(0);
    }

    public void stopRobotChassis() {
        Log.i("=== INCREDIBOTS / ROBOTHARDWARE  ===", " stopRobotChassis");

         rightFrontDriveMotor.setPower(0);
         leftFrontDriveMotor.setPower(0);
         rightBackDriveMotor.setPower(0);
         leftBackDriveMotor.setPower(0);
    }

    public void stopRobotAndMechanisms() {
        Log.i("=== INCREDIBOTS / ROBOTHARDWARE  ===", " stopRobotAndMechanisms");

        stopRobotChassis();
        stopAllMechanisms();
    }

     public boolean getHorizontalClawState () {
         Log.i("=== INCREDIBOTS / ROBOTHARDWARE  ===", " getHorizontalClawState");

         return horizontalClawServo.getPosition() == RobotConstants.HORIZONTAL_CLAW_OPEN;
     }

     public void setHorizontalClawState(boolean open) {
         Log.i("=== INCREDIBOTS / ROBOTHARDWARE  ===", " operateHorizontalClaw: " + open);

         if (open) {
             horizontalClawServo.setPosition(RobotConstants.HORIZONTAL_CLAW_OPEN);
         }
         else {
             horizontalClawServo.setPosition(RobotConstants.HORIZONTAL_CLAW_CLOSE);
         }
     }

     public double getHorizontalWristServoPosition() {
         Log.i("=== INCREDIBOTS / ROBOTHARDWARE  ===", " getHorizontalWristServoPosition");

         return horizontalWristServo.getPosition();
     }

     public void setHorizontalWristServoPosition(double pos) {
         Log.i("=== INCREDIBOTS / ROBOTHARDWARE  ===", " setHorizontalWristServoPosition: " + pos);
         horizontalWristServo.setPosition(pos);
     }

     public double getHorizontalElbowServoPosition() {
         Log.i("=== INCREDIBOTS / ROBOTHARDWARE  ===", " getHorizontalElbowServoPosition");
         return horizontalElbowServo.getPosition();
     }

     public void setHorizontalElbowServoPosition(double pos) {
         Log.i("=== INCREDIBOTS / ROBOTHARDWARE  ===", " setHorizontalElbowServoPosition: " + pos);
         horizontalElbowServo.setPosition(pos);
     }

     public double getHorizontalShoulderServoPosition() {
         Log.i("=== INCREDIBOTS / ROBOTHARDWARE  ===", " getHorizontalShoulderServoPosition");
         return horizontalShoulderServo.getPosition();
     }

     public void setHorizontalShoulderServoPosition(double pos) {
         Log.i("=== INCREDIBOTS / ROBOTHARDWARE  ===", " setHorizontalShoulderServo: " + pos);
         horizontalShoulderServo.setPosition(pos);
     }

     public double getHorizontalTurretServoPosition() {
         Log.i("=== INCREDIBOTS / ROBOTHARDWARE  ===", " getHorizontalTurretServoPosition");
         return horizontalTurretServo.getPosition();
     }

     public void setHorizontalTurretServoPosition(double pos) {
         Log.i("=== INCREDIBOTS / ROBOTHARDWARE  ===", " setHorizontalTurretServoPosition: " + pos);
         horizontalTurretServo.setPosition(pos);
     }

     public GameConstants.GAME_COLORS getDetectedColor(){
         GameConstants.GAME_COLORS detectedColor = GameConstants.GAME_COLORS.YELLOW;

         if (colorSensor.red() > colorSensor.green() && colorSensor.red() > colorSensor.blue()) {
             detectedColor = GameConstants.GAME_COLORS.RED;
         }

         if (colorSensor.blue() > colorSensor.green() && colorSensor.blue() > colorSensor.red()) {
             detectedColor = GameConstants.GAME_COLORS.BLUE;
         }

         Log.i("=== INCREDIBOTS / ROBOTHARDWARE  ===", " getDetectedColor: " + detectedColor.toString());

         return detectedColor;
    }

    public ColorSensorOutput getDetectedColorAndDistance() {
         ColorSensorOutput colorSensorOutput = new ColorSensorOutput(getDetectedColor(),  colorSensor.getDistance(DistanceUnit.CM));

        Log.i("=== INCREDIBOTS / ROBOTHARDWARE  ===", " getDetectedColorAndDistance: " + colorSensorOutput.detectedColor.toString() + " Distance: " + colorSensorOutput.distance);

         return colorSensorOutput;
    }

    public int getHorizontalSlidePosition() {
        Log.i("=== INCREDIBOTS / ROBOTHARDWARE  ===", " getHorizontalSlidePosition");
        return horizontalSlideMotor.getCurrentPosition();
    }

    public void setHorizontalSlidePosition(int pos) {
        setHorizontalSlidePositionAndVelocity(pos, RobotConstants.HORIZONTAL_SLIDE_VELOCITY);
        Log.i("=== INCREDIBOTS / ROBOTHARDWARE ===", " setHorizontalSlidePosition: " + pos + " AND VELOCITY: " + RobotConstants.HORIZONTAL_SLIDE_VELOCITY);
    }

    public void setHorizontalSlidePositionAndVelocity(int pos, int velocity) {
        horizontalSlideMotor.setTargetPosition(pos);
        horizontalSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        horizontalSlideMotor.setVelocity(velocity);
        Log.i("=== INCREDIBOTS / ROBOTHARDWARE ===", " setHorizontalSlidePositionAndVelocity: " + pos + " AND VELOCITY: " + velocity);
    }

    public boolean isHorizontalLimitSwitchPressed()
    {
        return horizontalLimitSwitch.isPressed();
    }

    public boolean getVerticalClawState() {
         return verticalClawServo.getPosition() == RobotConstants.VERTICAL_CLAW_OPEN;
    }

    public void setVerticalClawState(boolean open) {
        Log.i("=== INCREDIBOTS / ROBOTHARDWARE  ===", " operateVerticalClaw: " + open);

        if (open) {
            verticalClawServo.setPosition(RobotConstants.VERTICAL_CLAW_OPEN);
        }
        else {
            verticalClawServo.setPosition(RobotConstants.VERTICAL_CLAW_CLOSE);
        }
    }

    public double getVerticalWristServoPosition() {
        Log.i("=== INCREDIBOTS / ROBOTHARDWARE  ===", " getVerticalWristServoPosition");

        return verticalWristServo.getPosition();
    }

    public void setVerticalWristServoPosition(double pos) {
        Log.i("=== INCREDIBOTS / ROBOTHARDWARE  ===", " setVerticalWristServoPosition: " + pos);
        verticalWristServo.setPosition(pos);
    }

    public double getVerticalElbowServoPosition() {
        Log.i("=== INCREDIBOTS / ROBOTHARDWARE  ===", " getVerticalElbowServoPosition");
        return verticalElbowServo.getPosition();
    }

    public void setVerticalElbowServoPosition(double pos) {
        Log.i("=== INCREDIBOTS / ROBOTHARDWARE  ===", " setVerticalElbowServoPosition: " + pos);
        verticalElbowServo.setPosition(pos);
    }

    public double getVerticalShoulderServoPosition() {
        double voltage = verticalShoulderServoEncoder.getVoltage();
        double position = 1 - (voltage / 3.3);  //position via encoder seems to be flipped

        Log.i("=== INCREDIBOTS / ROBOTHARDWARE  ===", " getVerticalShoulderServo Position: " + position);
        return position;
    }

    public void setVerticalShoulderServoPosition(double pos) {
        Log.i("=== INCREDIBOTS / ROBOTHARDWARE  ===", " setVerticalShoulderServoPosition: " + pos);
        verticalShoulderServo.setPosition(pos);
    }

    //function to return the minimum of 2 motors on the vertical slide.
    public int getVerticalSlidePosition() {
        int retVal = Math.min(verticalSlideMotor1.getCurrentPosition(), verticalSlideMotor2.getCurrentPosition());
        Log.i("=== INCREDIBOTS / ROBOTHARDWARE ===", " getVerticalSlidePosition: " + retVal);
        return retVal;
    }

    public void setVerticalSlidePosition(int pos) {
        setVerticalSlidePositionAndVelocity(pos, RobotConstants.VERTICAL_SLIDE_VELOCITY);
        Log.i("=== INCREDIBOTS / ROBOTHARDWARE ===", " setVerticalSlidePosition: " + pos + " AND VELOCITY: " + RobotConstants.VERTICAL_SLIDE_VELOCITY);
    }

    public void setVerticalSlidePositionAndVelocity(int pos, int velocity) {
        verticalSlideMotor1.setTargetPosition(pos);
        verticalSlideMotor2.setTargetPosition(pos);

        verticalSlideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalSlideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        verticalSlideMotor1.setVelocity(velocity);
        verticalSlideMotor2.setVelocity(velocity);
        Log.i("=== INCREDIBOTS / ROBOTHARDWARE ===", " setVerticalSlidePositionAndVelocity: " + pos + " AND VELOCITY: " + velocity);
    }

    // function to check proximity sensors and ensure the motor stops
    public boolean isVerticalLimitSwitchPressed()
    {
        return verticalLimitSwitch.isPressed();
    }

    //returns the robots yaw as radians
    public double getRobotYawRadians() {
         return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    //returns the robots yaw as degrees
    public double getRobotYawDegrees() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    //sets the drive motor's powers
    public void setDriveMotorPowers(double rfPower, double lfPower, double rbPower, double lbPower) {
         rightFrontDriveMotor.setPower(rfPower);
         leftFrontDriveMotor.setPower(lfPower);
         rightBackDriveMotor.setPower(rbPower);
         leftBackDriveMotor.setPower(lbPower);
     }

     //resets each individual odo wheel
     public void resetRightOdo() {
         odoRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     }

     public void resetLeftOdo() {
        odoLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     }

     public void resetFrontOdo() {
        odoFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     }

    //resets all the odo wheels at the same time
     public void resetAllOdo() {
        resetRightOdo();
        resetLeftOdo();
        resetFrontOdo();
     }

    // Returns the right odo wheel value
    public double getRightOdoMM() {
         return getMMTraveled(odoRight.getCurrentPosition());
     }

    // Returns the left odo wheel value
    public double getLeftOdoMM() {
        return getMMTraveled(odoLeft.getCurrentPosition());
    }

    // returns the front odo wheel value
    public double getFrontOdoMM() {
        return getMMTraveled(odoFront.getCurrentPosition());
    }

    // Returns the distance traveled in mm
    public double getMMTraveled(int ticks) {
         return RobotConstants.MMPerTick * ticks;
    }

}