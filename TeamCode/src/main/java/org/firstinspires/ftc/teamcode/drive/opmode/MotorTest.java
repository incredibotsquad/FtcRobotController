package org.firstinspires.ftc.teamcode.drive.opmode;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

@Config
@TeleOp(name="MotorTest", group="Linear OpMode")
public class MotorTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx Vbar;

    public static int pos = 0;

    public static double velocity = 100;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        Vbar = hardwareMap.get(DcMotorEx.class,"Vbar");
        Vbar.setDirection(DcMotorSimple.Direction.REVERSE);
        Vbar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Vbar.setTargetPosition(0);
        Vbar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Vbar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (!Vbar.isBusy()) {
                setVBarPositionAndVelocity(pos, velocity);
            }
        }
    }

    public void setVBarPositionAndVelocity(int pos, double velocity) {
        Vbar.setTargetPosition(pos);
        Vbar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Vbar.setVelocity(velocity);
    }
}
