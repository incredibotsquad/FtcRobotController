package org.firstinspires.ftc.teamcode;
//importing stuff

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
     //functions
     public void setSlidePositionAndPower(int pos, double power) {
         rightSlideMotor.setPower(power);
         rightSlideMotor.setTargetPosition(pos);
         rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
     }

    public void setLeftArmPositionAndPower(int pos, double power) {
        leftArmMotor.setTargetPosition(pos);
        //leftArmMotor.setPower(power);
        leftArmMotor.setVelocity(2100);
        leftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void setRightArmPositionAndPower(int pos, double power) {
        rightArmMotor.setPower(power);
        rightArmMotor.setTargetPosition(pos);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public double getRobotYawRadians() {
         return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public double getRobotYawDegrees() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void setDriveMotorPowers(double rfPower, double lfPower, double rbPower, double lbPower) {
         rightFrontDriveMotor.setPower(rfPower);
         leftFrontDriveMotor.setPower(lfPower);
         rightBackDriveMotor.setPower(rbPower);
         leftBackDriveMotor.setPower(lbPower);
     }

     public void resetRightOdo() {
         odoRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     }

     public void resetLeftOdo() {
        odoLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     }

     public void resetFrontOdo() {
        odoFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     }


     public void resetAllOdo() {
        resetRightOdo();
        resetLeftOdo();
        resetFrontOdo();
     }

    public void operateClawServo(boolean open) {
         clawServo.setPosition(open?1:0);
    }
    public void operateIntakeServo(boolean clockwiseSpin, boolean counterClockwiseSpin) {
        if (clockwiseSpin){
            intakeServo.setPower(1);
        }
        else if (counterClockwiseSpin){
            intakeServo.setPower(-1);
        }
        else{
            intakeServo.setPower(0);
        }
    }

    public double getRightOdoMM() {
         return getMMTraveled(odoRight.getCurrentPosition());
     }

    public double getLeftOdoMM() {
        return getMMTraveled(odoLeft.getCurrentPosition());
    }

    public double getFrontOdoMM() {
        return getMMTraveled(odoFront.getCurrentPosition());
    }

    public double getMMTraveled(int ticks) {
         return RobotConstants.MMPerTick * ticks;
    }

    public int getRightArmMotorPos(){
         return rightArmMotor.getCurrentPosition();
    }

    public int getLeftArmMotorPos(){
        return leftArmMotor.getCurrentPosition();
    }

}