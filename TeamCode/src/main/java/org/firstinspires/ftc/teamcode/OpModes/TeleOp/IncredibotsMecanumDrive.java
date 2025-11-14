

package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.common.AllianceColors;
import org.firstinspires.ftc.teamcode.common.MechanismControl;
import org.firstinspires.ftc.teamcode.common.RobotHardware;

@Config
@TeleOp(name="IncredibotsMecanumDrive", group="Linear OpMode")
public class IncredibotsMecanumDrive extends LinearOpMode {

    public static double DRIVETRAIN_POWER_RATIO = 1;

    private RobotHardware robotHardware;
    private MechanismControl mechanismControl;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        robotHardware = new RobotHardware(this.hardwareMap);
        mechanismControl = new MechanismControl(gamepad2, robotHardware, telemetry);

        robotHardware.startLimelight();
        robotHardware.setLimelightPipeline(6);

        while (opModeInInit()) {
            if (gamepad1.xWasPressed() || gamepad2.xWasPressed()) {
                mechanismControl.setAllianceColor(AllianceColors.BLUE);
                telemetry.addData("Alliance Color", "Blue");
                telemetry.update();
            }

            if (gamepad1.bWasPressed() || gamepad2.bWasPressed()) {
                mechanismControl.setAllianceColor(AllianceColors.RED);
                telemetry.addData("Alliance Color", "Red");
                telemetry.update();
            }
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && !isStopRequested()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y  * DRIVETRAIN_POWER_RATIO;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x * DRIVETRAIN_POWER_RATIO;
            double yaw     =  gamepad1.right_stick_x * DRIVETRAIN_POWER_RATIO;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // Sets the drive motor powers
            robotHardware.setDriveMotorPowers(rightFrontPower, leftFrontPower, rightBackPower, leftBackPower);

            //control robot mechanisms
            mechanismControl.ProcessInputs();

            idle();
        }

        robotHardware.stopLimelight();
    }}