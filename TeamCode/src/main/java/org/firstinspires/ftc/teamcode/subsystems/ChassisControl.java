package org.firstinspires.ftc.teamcode.subsystems;


import static org.firstinspires.ftc.teamcode.subsystems.IntakeSystem.THREE_BALL_COLOR;
import static org.firstinspires.ftc.teamcode.subsystems.IntakeSystem.ZERO_BALL_COLOR;
import static org.firstinspires.ftc.teamcode.subsystems.LaunchSystem.TURRET_CENTERED_POSITION;
import static org.firstinspires.ftc.teamcode.subsystems.LaunchSystem.TURRET_DEGREES_PER_TICK;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Actions.LiftAction;
import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointLocalizer;
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
    public static double POSITION_CHECK_THROTTLE_MILLIS = 20;
    private MecanumDrive mecanumDrive;

//    private PinpointLocalizer pinPoint;
    private AllianceColors allianceColor;
    private int multiplier = 1;    //used to flip coordinates between red (1), Blue (-1)

    public double obeliskHeading = Math.toRadians(180); //this is same for both sides - do NOT need a multiplier
    public Pose2d PARK_POS;
    public static double DRIVETRAIN_POWER_RATIO = 1;
    private boolean hasRobotMoved = false;
    private ChassisControl() {}

    public ChassisControl(Gamepad gamepad, RobotHardware robotHardware, LimelightAprilTagHelper limelightAprilTagHelper) {
        this.gamepad1 = gamepad;
        this.robotHardware = robotHardware;
        this.limelightAprilTagHelper = limelightAprilTagHelper;

//        this.positionCheckTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

//        pinPoint = new PinpointLocalizer(this.robotHardware.hardwareMap, CrossOpModeStorage.currentPose);
//        pinPoint.setPose(CrossOpModeStorage.currentPose);
//        Log.i("Chassis Control", "Seeded localizer with: X: " + CrossOpModeStorage.currentPose.position.x + " Y: " + CrossOpModeStorage.currentPose.position.y + " Heading: " + Math.toDegrees(CrossOpModeStorage.currentPose.heading.toDouble()));

//        mecanumDrive = new MecanumDrive(this.robotHardware.hardwareMap, CrossOpModeStorage.currentPose);

        allianceColor = CrossOpModeStorage.allianceColor;

        if (allianceColor == AllianceColors.RED) {
            multiplier = -1;
        }
        PARK_POS = new Pose2d(31, 33 * multiplier, obeliskHeading);

        hasRobotMoved = false;
    }

    public void processInputs() {
        moveRobotWithGamePad();

//        trackRobotPose();

        parkRobot();
//        augmentPinpointWithAprilTagData();
//        resetRobotPosition();
    }

    private void moveRobotWithGamePad() {
        mecanumDrive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y * DRIVETRAIN_POWER_RATIO,
                        -gamepad1.left_stick_x * DRIVETRAIN_POWER_RATIO
                ),
                -gamepad1.right_stick_x * DRIVETRAIN_POWER_RATIO
        ));

        mecanumDrive.updatePoseEstimate();

        Pose2d pose = mecanumDrive.localizer.getPose();

        Log.i("Chassis Control", "Raw position update: x: " + pose.position.x + " y: " + pose.position.y + " heading: " + Math.toDegrees(pose.heading.toDouble()));

        Pose2d floorPose = new Pose2d(new Vector2d(Math.floor(pose.position.x), Math.floor(pose.position.y)), pose.heading);

        CrossOpModeStorage.currentPose = floorPose;

        Log.i("Chassis Control", "Floor position update: x: " + floorPose.position.x + " y: " + floorPose.position.y + " heading: " + Math.toDegrees(floorPose.heading.toDouble()));
    }

    public void initializeMecanumDrive() {
        Log.i("Chassis Control", "initializeMecanumDrive with: x: " + CrossOpModeStorage.currentPose.position.x + " y: " + CrossOpModeStorage.currentPose.position.y + " heading: " + Math.toDegrees(CrossOpModeStorage.currentPose.heading.toDouble()));
        mecanumDrive = new MecanumDrive(this.robotHardware.hardwareMap, CrossOpModeStorage.currentPose);
        mecanumDrive.localizer.setPose(CrossOpModeStorage.currentPose);
    }

//    private void moveRobotWithGamePad() {
//        double max;
//
//        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
//        double axial   = -gamepad1.left_stick_y  * DRIVETRAIN_POWER_RATIO;  // Note: pushing stick forward gives negative value
//        double lateral =  gamepad1.left_stick_x * DRIVETRAIN_POWER_RATIO;
//        double yaw     =  gamepad1.right_stick_x * DRIVETRAIN_POWER_RATIO;
//
//        //if robot has not moved yet but is going to move, set the initial pose once
//        if (!hasRobotMoved && (axial != 0 || lateral != 0 || yaw != 0)) {
//            Log.i("Chassis Control", "Setting pose before first move");
//            pinPoint.setPose(CrossOpModeStorage.currentPose);   //set the pose before the first move
//            hasRobotMoved = true;
//        }
//
//            // Combine the joystick requests for each axis-motion to determine each wheel's power.
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

//    public void trackRobotPose() {
//        if (!hasRobotMoved)
//            return;
//
////        Log.i("Chassis Control", "trackRobotPose: updating pose in localizer and crossopmode storage");
//        pinPoint.update();
//        CrossOpModeStorage.currentPose = pinPoint.getPose();
//        Log.i("Chassis Control", "New robot position: X: " + CrossOpModeStorage.currentPose.position.x + " Y: " + CrossOpModeStorage.currentPose.position.y + " Heading: " + Math.toDegrees(CrossOpModeStorage.currentPose.heading.toDouble()));
//    }

    public void augmentPinpointWithAprilTagData() {
//        if (positionCheckTimer.milliseconds() < POSITION_CHECK_THROTTLE_MILLIS)
//            return;
//
//        positionCheckTimer.reset();
//
//        //puppy: since Meher asked me to write it - DO NOT REMOVE THIS!
//
//        Pose2d pinpointPosition = mecanumDrive.localizer.getPose();
//        if (pinpointPosition != null)
//            Log.i("Chassis Control", "augmentPinpointWithAprilTagData : Pinpoint position: X: " + pinpointPosition.position.x + " Y: " + pinpointPosition.position.y + " Yaw: " + Math.toDegrees(pinpointPosition.heading.toDouble()));
//
//        Pose3D limelightBasedPosition = limelightAprilTagHelper.getRobotPoseFromAprilTags();
//        if (limelightBasedPosition != null) {
//            double x = limelightBasedPosition.getPosition().x * 39.37;
//            double y = limelightBasedPosition.getPosition().y * 39.37;
//            double turretHeading = limelightBasedPosition.getOrientation().getYaw(AngleUnit.RADIANS);
//            Log.i("Chassis Control", "augmentPinpointWithAprilTagData: Limelight position: X: " + x + " Y: " + y + " Yaw: " + Math.toDegrees(turretHeading));
//            Log.i("Chassis Control", "augmentPinpointWithAprilTagData: Updated Pinpoint with Limelight position");

//            //find robot heading from limelight heading
//            double currentTurretRadians = Math.toRadians(robotHardware.getLaunchTurretPosition() * TURRET_DEGREES_PER_TICK);
//            double robotHeading = turretHeading + currentTurretRadians;

//            Log.i("Chassis Control", "augmentPinpointWithAprilTagData: robot heading from turret heading: " + Math.toDegrees(robotHeading));

//            mecanumDrive.localizer.setPose(new Pose2d(x, y, robotHeading));
//        }

    }

    public Action holdPosition() {
        return new NullAction();
    }

    public void parkRobot() {
//        Action park = new NullAction();

        if (gamepad1.startWasPressed()) {

            if (robotHardware.getLiftPosition() < LiftAction.LIFT_ROBOT / 2)
                robotHardware.setLiftPosition(LiftAction.LIFT_ROBOT);
            else
                robotHardware.setLiftPosition(LiftAction.LIFT_RESET);


//            if (PARK_POS != null) {
//                Pose2d startingPose = mecanumDrive.localizer.getPose();
//
//                Log.i("Chassis Control", "position before parking: x: " + startingPose.position.x + " y: " + startingPose.position.y);
//
//                park = mecanumDrive.actionBuilder(startingPose)
//                        .strafeToLinearHeading(PARK_POS.position, PARK_POS.heading)
//                        .build();
//            }
//
//            //TODO: ADD EXTRA CORRECTION
//
//            Actions.runBlocking(park);
        }
    }

    //the goal is to do this while aligned to the back wall and looking at your own goal april tag
    public void resetRobotPosition() {
        if (gamepad1.leftBumperWasPressed() && gamepad1.rightBumperWasPressed()) {
            robotHardware.setLaunchTurretPosition(TURRET_CENTERED_POSITION);


            do {
                Log.i("Chassis Control", "resetRobotPosition: Turret still moving");
            }
            while (Math.abs(robotHardware.getLaunchTurretPosition()) - TURRET_CENTERED_POSITION > 5);

            //now we know our orientation.
            double robotHeading = 90;
            if (allianceColor == AllianceColors.BLUE)
                robotHeading = 270;

            //now get MT2 from limelight
            Pose3D robotPose = limelightAprilTagHelper.getMT2PoseFromAprilTags(robotHeading);

            Log.i("Chassis Control", "resetRobotPosition: Megatag from Limelight: X: " + robotPose.getPosition().x + " Y: " +  robotPose.getPosition().y + " Yaw: " + robotPose.getOrientation().getYaw());


            if (robotPose != null){
//                pinPoint.setPose(new Pose2d(robotPose.getPosition().x, robotPose.getPosition().y, Math.toRadians(robotHeading)));

                mecanumDrive.localizer.setPose(new Pose2d(robotPose.getPosition().x, robotPose.getPosition().y, Math.toRadians(robotHeading)));

                ElapsedTime blinkTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

                do {
                    robotHardware.setAlignmentLightColor(THREE_BALL_COLOR);
                    robotHardware.setspindexStatusLightColor(THREE_BALL_COLOR);
                } while (blinkTime.milliseconds() < 500);

                robotHardware.setAlignmentLightColor(ZERO_BALL_COLOR);
                robotHardware.setspindexStatusLightColor(ZERO_BALL_COLOR);
            }
        }
    }
}
