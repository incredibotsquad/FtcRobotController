package org.firstinspires.ftc.teamcode;
//importing stuff

import android.util.Log;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class RobotHardware {
    //making variables

     IMU imu;
     private DcMotorEx rightFrontDriveMotor;
     private DcMotorEx leftFrontDriveMotor;
     private DcMotorEx rightBackDriveMotor;
     private DcMotorEx leftBackDriveMotor;
     public static DcMotorEx leftArmMotor;
     public static DcMotorEx rightArmMotor;
     public static DcMotorEx rightSlideMotor;
     private DcMotor odoRight;
     private DcMotor odoLeft;
     private DcMotor odoFront;
     private Servo clawServo;
     private CRServo intakeServo;


     //making constructor
     public RobotHardware(HardwareMap hwMap) {

         rightFrontDriveMotor = hwMap.get(DcMotorEx.class, "RFMotor");
         rightFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         rightFrontDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

         //this motor is oriented backwards, hence reversing direction
         leftFrontDriveMotor = hwMap.get(DcMotorEx.class, "LFMotor");
         leftFrontDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
         leftFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         leftFrontDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

         rightBackDriveMotor = hwMap.get(DcMotorEx.class, "RBMotor");
         rightBackDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         rightBackDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

         //this motor is oriented backwards, hence reversing direction
         leftBackDriveMotor = hwMap.get(DcMotorEx.class, "LBMotor");
         leftBackDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         leftBackDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         leftBackDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

         leftArmMotor = hwMap.get(DcMotorEx.class,"LAMotor");
         leftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         leftArmMotor.setTargetPosition(0);
         leftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         leftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         rightArmMotor = hwMap.get(DcMotorEx.class, "RAMotor");
         rightArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
         rightArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         rightArmMotor.setTargetPosition(0);
         rightArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         rightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         rightSlideMotor = hwMap.get(DcMotorEx.class, "RSMotor");
         rightSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
         rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         rightSlideMotor.setTargetPosition(0);
         rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         odoRight = rightBackDriveMotor;
         odoLeft = leftBackDriveMotor;
         odoFront = rightFrontDriveMotor;

         clawServo = hwMap.get(Servo.class, "clawServo");
         intakeServo = hwMap.get(CRServo.class, "intakeServo");

         imu = hwMap.get(IMU.class, "imu");

     }

    // Sets the slide's position and velocity
     public void setSlidePositionAndVelocity(int pos, double velocity) {
         rightSlideMotor.setTargetPosition(pos);
         rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         rightSlideMotor.setVelocity(velocity);
     }

    // Sets the slide's position and velocity and wait for it to reach the position
    public void setSlidePositionAndVelocityAndWait(int pos, double velocity) {
        rightSlideMotor.setTargetPosition(pos);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideMotor.setVelocity(velocity);

        while(rightSlideMotor.isBusy()) {
            // Let the drive team see that we're waiting on the motor
            Log.i("Motor Status", "Waiting for the slide motor to reach its target");
        }
    }

    // Sets the left arm's position and velocity
    public void setClawArmPositionAndVelocity(int pos, double velocity) {
        leftArmMotor.setTargetPosition(pos);
        leftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftArmMotor.setVelocity(velocity);
    }

    // Sets the left arm's position and velocity and waits for the motor to reach its position
    public void setClawArmPositionAndVelocityAndWait(int pos, double velocity) {
        leftArmMotor.setTargetPosition(pos);
        leftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftArmMotor.setVelocity(velocity);

        while(leftArmMotor.isBusy()) {
            // Let the drive team see that we're waiting on the motor
            Log.i("Motor Status", "Waiting for the claw arm motor to reach its target");
        }
    }

    //Sets the right arm's position and velocity
    public void setSlideArmPositionAndVelocity(int pos, double velocity) {
        rightArmMotor.setTargetPosition(pos);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArmMotor.setVelocity(velocity);
    }

    //Sets the right arm's position and velocity and wait for the arm to reach the position
    public void setSlideArmPositionAndVelocityAndWait(int pos, double velocity) {
        rightArmMotor.setTargetPosition(pos);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArmMotor.setVelocity(velocity);

        while(rightArmMotor.isBusy()) {
            // Let the drive team see that we're waiting on the motor
            Log.i("== Motor Status ==", "Waiting for the slide arm motor to reach its target");
        }
    }

    //decides either to open or close the claw servo
    public void operateClawServo(boolean open, double openPosition, double closePosition) {
        clawServo.setPosition(open? openPosition: closePosition);
    }

    // function to stop the intake servo
    public void stopIntakeServo() {
        Log.i("== Servo Status ==", "Stopping Intake servo:");
        intakeServo.setPower(0);
    }

    // Decides if the intake servo should spin clockwise or counter clockwise
    public void operateIntakeServo(boolean clockwiseSpin, boolean counterClockwiseSpin) {
        Log.i("== Servo Status ==", "RobotHardware::Operate Intake servo: clockwise:" + clockwiseSpin + "counterclockwise: " + counterClockwiseSpin);

         if (clockwiseSpin) {
             Log.i("== Servo Status ==", "RobotHardware::Operate Intake servo: setting power to 1:");
            intakeServo.setPower(1);
        } else if (counterClockwiseSpin) {
             Log.i("== Servo Status ==", "RobotHardware::Operate Intake servo: setting power to -1:");
            intakeServo.setPower(-1);
        }
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
         Log.i("=== INCREDIBOTS ===","===========================");
        Log.i("=== INCREDIBOTS ===","INSIDE SET DRIVE MOTOR POWERS");
        Log.i("=== INCREDIBOTS ===","=============================");

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

    // Returns the right arm motor position
    public int getSlideArmMotorPos() {
         return rightArmMotor.getCurrentPosition();
    }

    // Returns the left arm position
    public int getClawArmMotorPos() {
        return leftArmMotor.getCurrentPosition();
    }

    public int getSlidePos() {
        return rightSlideMotor.getCurrentPosition();
    }

    public double getIntakeServoPower() { return intakeServo.getPower(); }

}