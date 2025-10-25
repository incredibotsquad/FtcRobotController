package org.firstinspires.ftc.teamcode.opmodes.test;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;


@Config
//@Disabled
@TeleOp(name="ServoWithEncoderTest", group="Linear OpMode")
public class ServoWithEncoderTest extends LinearOpMode {

    public static double SERVO1_INPUT_POS;
    public static double SERVO2_INPUT_POS;

    private Servo fedbackServo1;

    private Servo fedbackServo2;

    @Override
    public void runOpMode() {

        fedbackServo1 = hardwareMap.get(Servo.class, "SpindexServo");
        fedbackServo2 = hardwareMap.get(Servo.class, "VisorServo");

        AnalogInput analogInput = hardwareMap.get(AnalogInput.class, "ServoEncoder");

        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if(gamepad1.a) {
                fedbackServo1.setPosition(SERVO1_INPUT_POS);
            }

            if(gamepad1.b) {
                fedbackServo2.setPosition(SERVO2_INPUT_POS);
            }

            double voltage = analogInput.getVoltage();
            double position = 1 - (voltage / 3.3);
            Log.i("ServoTest: voltage: ", Double.toString(voltage));
            Log.i("ServoTest: position: ", Double.toString(position));

        }
    }
}
