package org.firstinspires.ftc.teamcode.drive.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
@TeleOp(name="ServoTest", group="Linear OpMode")
public class ServoTest extends LinearOpMode {
    RobotHardware myHardware;

    // Declare OpMode members.
    public static double servo1Position = 0.35;
    public static String servo1Name = "HorizontalShoulderServo";

    public static boolean enableServo2 = true;
    public static double servo2Position = 0.475;
    public static String servo2Name = "HorizontalTurretServo";

    // Close: 0.42
    // Open: 0.55
    private ElapsedTime runtime = new ElapsedTime();
    private Servo Servo1;
    private Servo Servo2;

    @Override
    public void runOpMode() {

        while (opModeInInit()) {
            Servo1 = hardwareMap.get(Servo.class, servo1Name);
            if (enableServo2) {
                Servo2 = hardwareMap.get(Servo.class, servo2Name);
            }
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(gamepad1.a) {
                Servo1.setPosition(servo1Position);

                if (enableServo2) {
                    Servo2.setPosition(servo2Position);
                }
            }
        }
    }
}
