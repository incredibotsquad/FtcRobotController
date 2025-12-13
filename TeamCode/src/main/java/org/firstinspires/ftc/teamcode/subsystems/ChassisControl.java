package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.common.AllianceColors;
import org.firstinspires.ftc.teamcode.common.LimelightAprilTagHelper;
import org.firstinspires.ftc.teamcode.common.CrossOpModeStorage;
import org.firstinspires.ftc.teamcode.common.RobotHardware;

@Config
public class ChassisControl {
    private Gamepad gamepad1;
    private RobotHardware robotHardware;
    private LimelightAprilTagHelper limelightAprilTagHelper;
    private ElapsedTime positionCheckTimer;
    public static double POSITION_CHECK_THROTTLE_MILLIS = 200;
    private MecanumDrive mecanumDrive;
    private AllianceColors allianceColor;
    private int multiplier = 1;    //used to flip coordinates between red (1), Blue (-1)

    public double obeliskHeading = Math.toRadians(180); //this is same for both sides - do NOT need a multiplier
    public Pose2d PARK_POS;
    public static double DRIVETRAIN_POWER_RATIO = 1;
    private ChassisControl() {}

    public ChassisControl(Gamepad gamepad, RobotHardware robotHardware, LimelightAprilTagHelper limelightAprilTagHelper) {
        this.gamepad1 = gamepad;
        this.robotHardware = robotHardware;
        this.limelightAprilTagHelper = limelightAprilTagHelper;

        this.positionCheckTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        Log.i("Chassis Control", "Seeded mecanum drive with: X: " + CrossOpModeStorage.currentPose.position.x + " Y: " + CrossOpModeStorage.currentPose.position.y + " Heading: " + Math.toDegrees(CrossOpModeStorage.currentPose.heading.toDouble()));

        mecanumDrive = new MecanumDrive(this.robotHardware.hardwareMap, CrossOpModeStorage.currentPose);
        allianceColor = CrossOpModeStorage.allianceColor;

        if (allianceColor == AllianceColors.RED) {
            multiplier = -1;
        }
        PARK_POS = new Pose2d(31, 33 * multiplier, obeliskHeading);
    }

    public void processInputs() {
        moveRobotWithGamePad();

//        updateRobotPose();

        parkRobot();
//        augmentPinpointWithAprilTagData();
    }

    private void moveRobotWithGamePad() {
        mecanumDrive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                ),
                -gamepad1.right_stick_x
        ));

        mecanumDrive.updatePoseEstimate();

        Pose2d pose = mecanumDrive.localizer.getPose();
//        Log.i("Chassis Control", "position before parking: x: " + pose.position.x + " y: " + pose.position.y);

    }

//    private void moveRobotWithGamePad() {
//        double max;
//
//        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
//        double axial   = -gamepad1.left_stick_y  * DRIVETRAIN_POWER_RATIO;  // Note: pushing stick forward gives negative value
//        double lateral =  gamepad1.left_stick_x * DRIVETRAIN_POWER_RATIO;
//        double yaw     =  gamepad1.right_stick_x * DRIVETRAIN_POWER_RATIO;
//
//        // Combine the joystick requests for each axis-motion to determine each wheel's power.
//        // Set up a variable for each drive wheel to save the power level for telemetry.
//        double leftFrontPower  = axial + lateral + yaw;
//        double rightFrontPower = axial - lateral - yaw;
//        double leftBackPower   = axial - lateral + yaw;
//        double rightBackPower  = axial + lateral - yaw;
//
//        // Normalize the values so no wheel power exceeds 100%
//        // This ensures that the robot maintains the desired motion.
//        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
//        max = Math.max(max, Math.abs(leftBackPower));
//        max = Math.max(max, Math.abs(rightBackPower));
//
//        if (max > 1.0) {
//            leftFrontPower  /= max;
//            rightFrontPower /= max;
//            leftBackPower   /= max;
//            rightBackPower  /= max;
//        }
//
//        // Sets the drive motor powers
//        robotHardware.setDriveMotorPowers(rightFrontPower, leftFrontPower, rightBackPower, leftBackPower);
//    }



//    public void updateRobotPose() {
////        Log.i("Chassis Control", "updateRobotPose: using encoders to update pose in mecanumdrive");
//        mecanumDrive.updatePoseEstimate();
//    }

    public void augmentPinpointWithAprilTagData() {
        if (positionCheckTimer.milliseconds() < POSITION_CHECK_THROTTLE_MILLIS)
            return;

        positionCheckTimer.reset();

        //puppy: since Meher asked me to write it - DO NOT REMOVE THIS!

        Pose3D limelightBasedPosition = limelightAprilTagHelper.getRobotPoseFromAprilTags();
        if (limelightBasedPosition != null)
            Log.i("Chassis Control", "Limelight position: X: " + limelightBasedPosition.getPosition().x * 39.37 + " Y: " + limelightBasedPosition.getPosition().y * 39.37 + " Yaw: " + limelightBasedPosition.getOrientation().getYaw(AngleUnit.DEGREES));

        Pose2d pinpointPosition = mecanumDrive.localizer.getPose();
        if (pinpointPosition != null)
            Log.i("Chassis Control", "Pinpoint position: X: " + pinpointPosition.position.x + " Y: " + pinpointPosition.position.y + " Yaw: " + Math.toDegrees(pinpointPosition.heading.toDouble()));
    }

    public Action holdPosition() {
        return new NullAction();
    }

    public void parkRobot() {
        Action park = new NullAction();

        if (gamepad1.startWasPressed()) {

            if (PARK_POS != null) {
                Pose2d startingPose = mecanumDrive.localizer.getPose();

                Log.i("Chassis Control", "position before parking: x: " + startingPose.position.x + " y: " + startingPose.position.y);

                park = mecanumDrive.actionBuilder(startingPose)
                        .strafeToLinearHeading(PARK_POS.position, PARK_POS.heading)
                        .build();
            }

            //TODO: ADD EXTRA CORRECTION

            Actions.runBlocking(park);
        }
    }
}
