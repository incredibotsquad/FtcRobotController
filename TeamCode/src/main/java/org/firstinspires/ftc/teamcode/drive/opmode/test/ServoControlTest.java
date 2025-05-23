package org.firstinspires.ftc.teamcode.drive.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
@TeleOp(name="ServoTest", group="Linear OpMode")
public class ServoControlTest extends LinearOpMode {
    RobotHardware myHardware;

    // Declare OpMode members.
    public static double clawServoPosition;
    public static double wristServoPosition;

    // Close: 0.42
    // Open: 0.55
    private ElapsedTime runtime = new ElapsedTime();
    private Servo ClawServo;
    private Servo WristServo;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        ClawServo = hardwareMap.get(Servo.class, "ClawServo"); //0.1, 0.55
        WristServo = hardwareMap.get(Servo.class, "WristServo"); //resting: 0.2, specimen pick: 0.9, before snapping specimen position 0.75, picking sample: 0.83

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            ClawServo.setPosition(clawServoPosition);
            WristServo.setPosition(wristServoPosition);
        }
    }
}
/*
Control Hub
Motors:
Port 0 - LB
Port 1 - RB
Port 2 - LF
Port 3 - RF

Odometry:
Port 0: OL
Port 1: OR
Port 2: OF

Expansion Hub
Motors:
Port 0 - Arm
Port 1 - Slide

Encoders:
Port 0 - Arm
Port 1 - Slide

Servos:
Port 0 - Claw
 */
