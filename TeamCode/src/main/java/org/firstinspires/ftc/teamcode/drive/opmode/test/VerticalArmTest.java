package org.firstinspires.ftc.teamcode.drive.opmode.test;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;

/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Config
@TeleOp(name="VerticalArmTest", group="Linear OpMode")
public class VerticalArmTest extends LinearOpMode {
    RobotHardware myHardware;

    // Declare OpMode members.
    public static double wristServoPosition;
    public static double clawServoPosition;
    public static int slidePosition;
    public static int armPosition;
    public static double armVelocity;
    public static double slideVelocity;
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private Servo wristServo;
    private Servo clawServo;
    private DcMotorEx SlideMotor;
    private DcMotorEx ArmMotor;



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        clawServo = hardwareMap.get(Servo.class, "ClawServo");
        wristServo = hardwareMap.get(Servo.class, "WristServo");
        SlideMotor = hardwareMap.get(DcMotorEx.class, "SlideMotor");
        SlideMotor.setTargetPosition(0);
        SlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        ArmMotor = hardwareMap.get(DcMotorEx.class, "ArmMotor");
        ArmMotor.setTargetPosition(0);
        ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.a) {
                Log.i("== ISHIKA ==", "GAMEPAD.A PRESSED");
                runtime.reset();
                setClawArmPositionAndVelocity(armPosition, armVelocity);
                setSlidePositionAndVelocity(slidePosition, slideVelocity);
                wristServo.setPosition(wristServoPosition);

                clawServo.setPosition(clawServoPosition);
                while(isSlideMotorBusy()) {

                    Log.i("== ISHIKA ==", "SLIDE MOTOR BUSY");

                }
                telemetry.addData("Time: ", runtime.milliseconds());
                telemetry.update();
            }
        }
    }
    public void setClawArmPositionAndVelocity(int pos, double velocity) {
        ArmMotor.setTargetPosition(pos);
        ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmMotor.setVelocity(velocity);
        Log.i("=== INCREDIBOTS ===", " !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        Log.i("=== INCREDIBOTS ===", " SETTING CLAW MOTOR POSITION AND VELOCITY");
        Log.i("=== INCREDIBOTS ===", " !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    }
    public void setSlidePositionAndVelocity(int pos, double velocity) {
        SlideMotor.setTargetPosition(pos);
        SlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SlideMotor.setVelocity(velocity);
        Log.i("=== INCREDIBOTS ===", " !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        Log.i("=== INCREDIBOTS ===", " SETTING SLIDE MOTOR POSITION AND VELOCITY");
        Log.i("=== INCREDIBOTS ===", " !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    }
    public boolean isSlideMotorBusy() {
        return SlideMotor.isBusy();
    }
}