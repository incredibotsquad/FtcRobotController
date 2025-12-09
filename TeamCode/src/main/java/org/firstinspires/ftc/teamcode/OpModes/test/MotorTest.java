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

@Config
@TeleOp(name="MotorTest", group="Tests")
public class MotorTest extends LinearOpMode {

    // Declare OpMode members.
    public static String motorName = "LauncherMotor";
    public static int motorPosition = 50;

    public static double motorVelocity = 2000;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx spindexMotor;

    @Override
    public void runOpMode() {

        spindexMotor = hardwareMap.get(DcMotorEx.class, "SpindexMotor");
        spindexMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindexMotor.setTargetPosition(0);
//        spindexMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        spindexMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (opModeInInit()) {

        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(gamepad1.aWasPressed()) {
                spindexMotor.setTargetPosition(motorPosition);
                spindexMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                spindexMotor.setVelocity(motorVelocity);
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
