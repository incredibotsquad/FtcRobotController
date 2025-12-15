

package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import static org.firstinspires.ftc.teamcode.Actions.LaunchKickAction.LAUNCH_KICK_RESTING;
import static org.firstinspires.ftc.teamcode.subsystems.LaunchSystem.TURRET_SERVO_CENTERED;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.common.AllianceColors;
import org.firstinspires.ftc.teamcode.common.CrossOpModeStorage;
import org.firstinspires.ftc.teamcode.common.LimelightAprilTagHelper;
import org.firstinspires.ftc.teamcode.subsystems.MechanismControl;
import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ChassisControl;

@Config
@TeleOp(name="IncredibotsMecanumDrive", group="TeleOp")
public class IncredibotsMecanumDrive extends LinearOpMode {


    private RobotHardware robotHardware;
    private ChassisControl chassisControl;
    private MechanismControl mechanismControl;
    private LimelightAprilTagHelper limelightAprilTagHelper;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        robotHardware = new RobotHardware(this.hardwareMap);
        robotHardware.startLimelight();
        robotHardware.setLimelightPipeline(6);

        while (opModeInInit()) {
            if (gamepad1.xWasPressed() || gamepad2.xWasPressed()) {
                CrossOpModeStorage.allianceColor = AllianceColors.BLUE;
                telemetry.addData("Alliance Color", "Blue");
                telemetry.update();
            }

            if (gamepad1.bWasPressed() || gamepad2.bWasPressed()) {
                CrossOpModeStorage.allianceColor = AllianceColors.RED;
                telemetry.addData("Alliance Color", "Red");
                telemetry.update();
            }
        }

        this.limelightAprilTagHelper = new LimelightAprilTagHelper(robotHardware);
        this.chassisControl = new ChassisControl(gamepad1, robotHardware, this.limelightAprilTagHelper);
        this.mechanismControl = new MechanismControl(gamepad2, robotHardware, this.limelightAprilTagHelper, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robotHardware.setLaunchKickPosition(LAUNCH_KICK_RESTING);

        robotHardware.setLaunchTurretPosition(TURRET_SERVO_CENTERED);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && !isStopRequested()) {

            //control robot chassis
            chassisControl.processInputs();

            //control robot mechanisms
            mechanismControl.processInputs();

            robotHardware.keepFlywheelMotorsInSync();

            idle();
        }

        robotHardware.stopLimelight();
    }}