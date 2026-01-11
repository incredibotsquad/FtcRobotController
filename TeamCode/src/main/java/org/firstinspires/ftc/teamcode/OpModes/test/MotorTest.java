package org.firstinspires.ftc.teamcode.OpModes.test;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

//@Disabled

@Disabled
@Config
@TeleOp(name="MotorTest", group="Tests")
public class MotorTest extends LinearOpMode {

    private static final Logger log = LoggerFactory.getLogger(MotorTest.class);
    // Declare OpMode members.
    public static String motorName = "LaunchTurretMotor";
    public static int motorPosition = 200;
    public static double FLYWHEEL_P = 10;
    public static double FLYWHEEL_I = 3;
    public static double FLYWHEEL_D = 0;
    public static double FLYWHEEL_F = 0;

    public static int TARGET_POS_TOLERANCE = 0;
    public static double motorVelocity = 4000;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx testMotor;

    @Override
    public void runOpMode() {

        testMotor = hardwareMap.get(DcMotorEx.class, motorName);
        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        testMotor.setTargetPosition(0);
//        spindexMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        testMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (opModeInInit()) {

        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        PIDFCoefficients customPIDF = testMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        Log.i("Motor Test", "Default P: " + customPIDF.p);
        Log.i("Motor Test", "Default I: " + customPIDF.i);
        Log.i("Motor Test", "Default D: " + customPIDF.d);
        Log.i("Motor Test", "Default F: " + customPIDF.f);

        int targetPositionTolerance = testMotor.getTargetPositionTolerance();
        TARGET_POS_TOLERANCE = targetPositionTolerance;
        Log.i("Motor Test", "Default target tolerance: " + targetPositionTolerance);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(gamepad1.aWasPressed()) {
                customPIDF = new PIDFCoefficients(FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D, FLYWHEEL_F);
                testMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, customPIDF);
                testMotor.setTargetPositionTolerance(TARGET_POS_TOLERANCE);

                testMotor.setTargetPosition(motorPosition);
                testMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                testMotor.setVelocity(motorVelocity);
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
