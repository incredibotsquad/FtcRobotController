package org.firstinspires.ftc.teamcode.drive.opmode.test;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
@TeleOp(name="NewSlideTest", group="Linear OpMode")
public class NewSlideTest extends LinearOpMode {
    RobotHardware myHardware;

    // Declare OpMode members.
    public static int slidePosition;
    public static double slideVelocity;
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private DcMotorEx SlideMotor1;
    private DcMotorEx SlideMotor2;

    private Servo leftVBSerbo;
    private Servo rightVBSerbo;

    public static double armServoPosition;
    public static double wristServoPosition;
    public static double clawServoPosition;
    public static double leftVBServoPosition;
    public static double rightVBServoPosition;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        SlideMotor1 = hardwareMap.get(DcMotorEx.class, "VerticalSlideMotor1");
        SlideMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        SlideMotor1.setTargetPosition(0);
        SlideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SlideMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SlideMotor2 = hardwareMap.get(DcMotorEx.class, "VerticalSlideMotor2");
        SlideMotor2.setTargetPosition(0);
        SlideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SlideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


//        leftVBSerbo = hardwareMap.get(Servo.class, "LeftVB");
//        rightVBSerbo = hardwareMap.get(Servo.class, "RightVB");


        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.a) {
                runtime.reset();

                setSlidePositionAndVelocity(slidePosition, slideVelocity);

                telemetry.addData("Time: ", runtime.milliseconds());

                Log.i("INCREDIBOTS", "SLIDE RUNTIME: " + runtime.milliseconds());
                telemetry.update();
            }

            if (gamepad1.left_trigger > 0 && gamepad1.a) {
                leftVBSerbo.setPosition(leftVBServoPosition);
            }

            if (gamepad1.right_trigger > 0 && gamepad1.a) {
                rightVBSerbo.setPosition(rightVBServoPosition);
            }

        }
    }
    public void setSlidePositionAndVelocity(int pos, double velocity) {
        SlideMotor1.setTargetPosition(pos);
        SlideMotor2.setTargetPosition(pos);

        SlideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SlideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        SlideMotor1.setVelocity(velocity);
        SlideMotor2.setVelocity(velocity);
    }
}