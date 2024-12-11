package org.firstinspires.ftc.teamcode;
//importing stuff

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.opmode.IncredibotsArmControl;

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
     private Servo clawServo;
     private Servo wristServo;

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

         armMotor = hwMap.get(DcMotorEx.class,"ArmMotor");
         armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
         armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         armMotor.setTargetPosition(0);
         armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         slideMotor = hwMap.get(DcMotorEx.class, "SlideMotor");
         slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
         slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         slideMotor.setTargetPosition(0);
         slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         odoRight = rightBackDriveMotor;
         odoLeft = leftBackDriveMotor;
         odoFront = rightFrontDriveMotor;

         clawServo = hwMap.get(Servo.class, "ClawServo");
         wristServo = hwMap.get(Servo.class, "WristServo");

         imu = hwMap.get(IMU.class, "imu");
     }

    private int GetSlideVelocity(int slidePos) {
        //contract faster than expanding
        return (slidePos < getSlidePos())? IncredibotsArmControl.SLIDE_VELOCITY_CONTRACTING : IncredibotsArmControl.SLIDE_VELOCITY_EXPANDING;
    }

    public void setSlidePosition(int pos) {
        setSlidePositionAndVelocity(pos, GetSlideVelocity(pos));
    }

    // Sets the slide's position and velocity
    public void setSlidePositionAndVelocity(int pos, double velocity) {
        slideMotor.setTargetPosition(pos);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setVelocity(velocity);
        Log.i("=== INCREDIBOTS / ROBOTHARDWARE ===", " SETTING SLIDE MOTOR POSITION: " + pos + " AND VELOCITY: " + velocity);
    }

    //returns if slide motor is moving currently
    public boolean isSlideMotorBusy() {
         return slideMotor.isBusy();
     }

    //returns the slide motor position
    public int getSlidePos() {
        return slideMotor.getCurrentPosition();
    }

    // Sets the left arm's position and velocity
    public void setClawArmPositionAndVelocity(int pos, double velocity) {
        armMotor.setTargetPosition(pos);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setVelocity(velocity);
        Log.i("=== INCREDIBOTS / ROBOTHARDWARE  ===", " SETTING CLAW ARM POSITION: " + pos + " AND VELOCITY: " + velocity);
    }

    //returns if the arm motor is moving currently
    public boolean isClawArmMotorBusy() {
         return armMotor.isBusy();
    }

    // Returns the claw arm position
    public int getClawArmMotorPos() {
        return armMotor.getCurrentPosition();
    }

    //decides either to open or close the claw servo
    public void operateClawServo(boolean open) {
        clawServo.setPosition(open? IncredibotsArmControl.CLAW_OPEN_POSITION: IncredibotsArmControl.CLAW_CLOSE_POSITION);
    }


    //get Claw servo position
    public double getClawServoPosition() {
         return clawServo.getPosition();
    }

    public void operateWristServo(double position) {
         wristServo.setPosition(position);
    }

    public double getWristServoPosition() {
         return wristServo.getPosition();
    }

    public void stopSlide() {
        Log.i("=== INCREDIBOTS / ROBOTHARDWARE ===", " STOPPING SLIDE ");
        setSlidePosition(getSlidePos());
    }

    public void stopClawArm() {
        Log.i("=== INCREDIBOTS / ROBOTHARDWARE ===", " STOPPING CLAW ARM ");
        setClawArmPositionAndVelocity(getClawArmMotorPos(), IncredibotsArmControl.CLAW_ARM_VELOCITY);
    }

    public void stopAndResetSlideEncoder() {
        Log.i("=== INCREDIBOTS / ROBOTHARDWARE ===", " RESETTING SLIDE ENCODER");
        slideMotor.setPower(0);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setSlidePosition(0);
    }

    public void stopAndResetArmEncoder() {
        Log.i("=== INCREDIBOTS / ROBOTHARDWARE ===", " RESETTING ARM ENCODER");
        armMotor.setPower(0);
        try {
            Thread.sleep(1000);
            Log.i("=== INCREDIBOTS / ROBOTHARDWARE ===", " RESETTING ARM ENCODER - WAIT FOR ARM FALL");
        } catch (InterruptedException e) {
            Log.i("=== INCREDIBOTS / ROBOTHARDWARE ===", " RESETTING ARM ENCODER - WAIT FOR ARM FALL");
        }
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setClawArmPositionAndVelocity(0, IncredibotsArmControl.CLAW_ARM_VELOCITY);
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