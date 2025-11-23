package org.firstinspires.ftc.teamcode.OpModes.test;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Disabled
@Config
@TeleOp(name="ServoWithEncoderTest", group="Tests")
public class ServoWithEncoderTest extends LinearOpMode {

    // Declare OpMode members.
    public static double spindexServoPosition = 0.5;
    public static double visorServoPosition = 0.5;

    public static String servo1Name = "SpindexServo";
    public static String servo2Name = "LaunchVisorServo";

    public static boolean enableServo2 = false;

    private ElapsedTime runtime;
    private Servo spindexServo;
    private Servo visorServo;

    private boolean init = false;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        spindexServo = hardwareMap.get(Servo.class, servo1Name);
        visorServo = hardwareMap.get(Servo.class, servo2Name);

        AnalogInput spindexEnc = hardwareMap.get(AnalogInput.class, "SpindexServoEncoder");
        AnalogInput visorEnc = hardwareMap.get(AnalogInput.class, "VisorServoEncoder");

        // Wait for the game to start (driver presses START)
        waitForStart();

        init = false;

        runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (!init) {
                spindexServo.setPosition(spindexServoPosition);

                if (enableServo2)
                    visorServo.setPosition(visorServoPosition);

                init = true;
            }

            //poll every 10 ms
            if (runtime.milliseconds() < 10)
                continue;

            runtime.reset();

            double voltage = spindexEnc.getVoltage();
            double position = 1 - (voltage / 3.3);
            Log.i("Servo 1: voltage: ", Double.toString(voltage));
            Log.i("Servo 1: position: ", Double.toString(position));


            if (enableServo2) {
                voltage = spindexEnc.getVoltage();
                position = 1 - (voltage / 3.3);
                Log.i("Servo 2: voltage: ", Double.toString(voltage));
                Log.i("Servo 2: position: ", Double.toString(position));
            }
        }
    }
}