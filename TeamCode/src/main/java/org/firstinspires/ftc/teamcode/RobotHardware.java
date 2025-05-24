package org.firstinspires.ftc.teamcode;

import android.util.Log;

import org.firstinspires.ftc.teamcode.RobotConstants.*;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.opmode.IncredibotsMechanismControl;

public class RobotHardware {
    //making variables

    IMU imu;
    private DcMotorEx rightFrontDriveMotor;
    private DcMotorEx leftFrontDriveMotor;
    private DcMotorEx rightBackDriveMotor;
    private DcMotorEx leftBackDriveMotor;
    public static DcMotorEx armMotor;
    public static DcMotorEx slideMotor;
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

    public static DcMotorEx verticalSlideMotor1;
    public static DcMotorEx verticalSlideMotor2;

    private Servo verticalClawServo;
    private Servo verticalWristServo;
    private Servo verticalElbowServo;
    private Servo verticalShoulderServo;


     //making constructor
     public RobotHardware(HardwareMap hwMap) {

         rightFrontDriveMotor = hwMap.get(DcMotorEx.class, "RFMotor");
         rightFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         rightFrontDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

         //this motor is oriented backwards, hence reversing direction
         leftFrontDriveMotor = hwMap.get(DcMotorEx.class, "LFMotor");
         leftFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         leftFrontDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         leftFrontDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

         rightBackDriveMotor = hwMap.get(DcMotorEx.class, "RBMotor");
         rightBackDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         rightBackDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

         //this motor is oriented backwards, hence reversing direction
         leftBackDriveMotor = hwMap.get(DcMotorEx.class, "LBMotor");
         leftBackDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         leftBackDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         leftBackDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

         odoRight = rightBackDriveMotor;
         odoLeft = leftBackDriveMotor;
         odoFront = rightFrontDriveMotor;

         // horizontal stack
         horizontalSlideMotor = hwMap.get(DcMotorEx.class, "HorizontalSlideMotor");
         horizontalSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         horizontalSlideMotor.setTargetPosition(0);
         horizontalSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         horizontalSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         horizontalClawServo = hwMap.get(Servo.class, "HorizontalClawServo");
         horizontalWristServo = hwMap.get(Servo.class, "HorizontalWristServo");
         horizontalElbowServo = hwMap.get(Servo.class, "HorizontalElbowServo");
         horizontalShoulderServo = hwMap.get(Servo.class, "HorizontalShoulderServo");
         horizontalTurretServo = hwMap.get(Servo.class, "HorizontalTurretServo");
         colorSensor = hwMap.get(ColorRangeSensor.class, "ColorSensor");

         // vertical stack
         verticalSlideMotor1 = hwMap.get(DcMotorEx.class, "VerticalSlideMotor1");
         verticalSlideMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         verticalSlideMotor1.setTargetPosition(0);
         verticalSlideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         verticalSlideMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         verticalSlideMotor2 = hwMap.get(DcMotorEx.class, "VerticalSlideMotor2");
         verticalSlideMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         verticalSlideMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
         verticalSlideMotor2.setTargetPosition(0);
         verticalSlideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         verticalSlideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         verticalClawServo = hwMap.get(Servo.class, "VerticalClawServo");
         verticalWristServo = hwMap.get(Servo.class, "VerticalWristServo");
         verticalElbowServo = hwMap.get(Servo.class, "VerticalElbowServo");
         verticalShoulderServo = hwMap.get(Servo.class, "VerticalShoulderServo");

         imu = hwMap.get(IMU.class, "imu");
     }

     public boolean getHorizontalClawState () {
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

     public void setHorizontalShoulderServo(double pos) {
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

     public GAME_COLORS getDetectedColor(){
         Log.i("=== INCREDIBOTS / ROBOTHARDWARE  ===", " getDetectedColor");

         if (colorSensor.red() > colorSensor.green() && colorSensor.red() > colorSensor.blue()) {
             return GAME_COLORS.RED;
         }

         if (colorSensor.blue() > colorSensor.green() && colorSensor.blue() > colorSensor.red()) {
             return GAME_COLORS.BLUE;
         }

         return GAME_COLORS.OTHER;
    }

    public int getHorizontalSlidePosition() {
        return horizontalSlideMotor.getCurrentPosition();
    }

    public void setHorizontalSlidePosition(int pos) {
        horizontalSlideMotor.setTargetPosition(pos);
        horizontalSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        horizontalSlideMotor.setVelocity(RobotConstants.HORIZONTAL_SLIDE_VELOCITY);
        Log.i("=== INCREDIBOTS / ROBOTHARDWARE ===", " setHorizontalSlidePosition: " + pos + " AND VELOCITY: " + RobotConstants.HORIZONTAL_SLIDE_VELOCITY);
    }

    // function to check proximity sensors and ensure the motor stops
    public void horizontalSlideSafetyChecks()
    {}

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
        Log.i("=== INCREDIBOTS / ROBOTHARDWARE  ===", " getVerticalShoulderServoPosition");
        return verticalShoulderServo.getPosition();
    }

    public void setVerticalShoulderServo(double pos) {
        Log.i("=== INCREDIBOTS / ROBOTHARDWARE  ===", " setVerticalShoulderServo: " + pos);
        verticalShoulderServo.setPosition(pos);
    }

    //function to return the minimum of 2 motors on the vertical slide.
    public int getVerticalSlidePosition() {
        return Math.min(verticalSlideMotor1.getCurrentPosition(), verticalSlideMotor2.getCurrentPosition());
    }

    public void setVerticalSlidePosition(int pos) {
        verticalSlideMotor1.setTargetPosition(pos);
        verticalSlideMotor2.setTargetPosition(pos);

        verticalSlideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalSlideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        verticalSlideMotor1.setVelocity(RobotConstants.VERTICAL_SLIDE_VELOCITY);
        verticalSlideMotor2.setVelocity(RobotConstants.VERTICAL_SLIDE_VELOCITY);
        Log.i("=== INCREDIBOTS / ROBOTHARDWARE ===", " setVerticalSlidePosition: " + pos + " AND VELOCITY: " + RobotConstants.VERTICAL_SLIDE_VELOCITY);
    }

    // function to check proximity sensors and ensure the motor stops
    public void verticalSlideSafetyChecks()
    {
//        slideMotor.setPower(0);
//        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        setSlidePosition(0);
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