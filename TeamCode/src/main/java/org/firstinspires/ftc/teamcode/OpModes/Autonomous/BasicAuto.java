

package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MechanismControl;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Config
@Autonomous(name="BasicAuto", group="Auto")
public class BasicAuto extends LinearOpMode {

    public static double POWER_RATIO = 0.8;

    private RobotHardware robotHardware;
    private MechanismControl mechanismControl;

    private ElapsedTime timer;

// Declare OpMode members for each of the 4 motors.
//    private DcMotor leftFrontDrive = null;
//    private DcMotor leftBackDrive = null;
//    private DcMotor rightFrontDrive = null;
//    private DcMotor rightBackDrive = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        robotHardware = new RobotHardware(this.hardwareMap);
        mechanismControl = new MechanismControl(gamepad2, robotHardware, telemetry);

        while (opModeInInit()) {


        }

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        Log.i("Timer Started", "Timer Started");

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && !isStopRequested()) {

            mechanismControl.ProcessInputs();

            if (timer.milliseconds() > 20000) {
                // Sets the drive motor powers
                robotHardware.setDriveMotorPowers(0.2, 0.2, 0.2, 0.2);
            }
            if (timer.milliseconds() > 25000){

                Log.i("Timer Finished", "Timer Finished");

                robotHardware.stopRobotChassis();

                break;

            }


            //updates telemetry
            telemetry.update();

            idle();
        }

        robotHardware.stopLimelight();
    }}