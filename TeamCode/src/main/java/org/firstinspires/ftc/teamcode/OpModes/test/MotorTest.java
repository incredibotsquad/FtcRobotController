package org.firstinspires.ftc.teamcode.OpModes.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Disabled

@Disabled
@Config
@TeleOp(name="MotorTest", group="Tests")
public class MotorTest extends LinearOpMode {

    // Declare OpMode members.
    public static String motorName = "LauncherMotor";
    public static double motorPower = 0.5;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx flywheelMotor;

    @Override
    public void runOpMode() {

        while (opModeInInit()) {

            flywheelMotor = hardwareMap.get(DcMotorEx.class, "LauncherMotor");
            flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(gamepad1.aWasPressed()) {
                flywheelMotor.setPower(motorPower);
            }
//            Servo1.setPosition(0);
//            sleep(1500);
//            Servo1.setPosition(1);
//            sleep(1500);
//            Servo1.setPosition(0);
//            sleep(1500);
//            Servo1.setPosition(1);
//            sleep(1500);
//            Servo1.setPosition(0);
//            sleep(1500);
//            Servo1.setPosition(1);
//            sleep(1500);
//            Servo1.setPosition(0);
//            sleep(1500);
//            Servo1.setPosition(1);
//            sleep(1500);
//            Servo1.setPosition(0);
//            sleep(1500);
//            Servo1.setPosition(1);
//            sleep(1500);
//            Servo1.setPosition(0);
        }
    }
}
