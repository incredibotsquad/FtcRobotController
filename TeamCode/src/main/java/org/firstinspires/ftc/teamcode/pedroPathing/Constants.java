package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem.BACK_LEFT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem.BACK_RIGHT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem.FRONT_LEFT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem.FRONT_RIGHT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem.PINPOINT_DISTANCE_UNIT;
import static org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem.PINPOINT_ENCODER_RESOLUTION;
import static org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem.PINPOINT_HARDWARE_NAME;
import static org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem.PINPOINT_X_ENCODER_DIRECTION;
import static org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem.PINPOINT_X_OFFSET_INCH;
import static org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem.PINPOINT_Y_ENCODER_DIRECTION;
import static org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem.PINPOINT_Y_OFFSET_INCH;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static PathConstraints pathConstraints = new PathConstraints(0.96, 100, 1, 1);

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.61)
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.0005, 0.025))
            .forwardZeroPowerAcceleration(-33.68622341684524)
            .lateralZeroPowerAcceleration(-65.88495423300519)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.5, 0, 0.0005, 0.05))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(1, 0.05, 0.0004, 0.6, 0.03))
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.10, 0.08159212428882408,0.0018823947511389903));

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName(FRONT_RIGHT_MOTOR_NAME)
            .leftFrontMotorName(FRONT_LEFT_MOTOR_NAME)
            .rightRearMotorName(BACK_RIGHT_MOTOR_NAME)
            .leftRearMotorName(BACK_LEFT_MOTOR_NAME)
            .xVelocity(69.02178699)
            .yVelocity(49.46574726);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(PINPOINT_X_OFFSET_INCH)
            .strafePodX(PINPOINT_Y_OFFSET_INCH)
            .distanceUnit(PINPOINT_DISTANCE_UNIT)
            .hardwareMapName(PINPOINT_HARDWARE_NAME)
            .encoderResolution(PINPOINT_ENCODER_RESOLUTION)
            .forwardEncoderDirection(PINPOINT_X_ENCODER_DIRECTION)
            .strafeEncoderDirection(PINPOINT_Y_ENCODER_DIRECTION);

    public static Follower createFollower(HardwareMap hardwareMap) {
        driveConstants.setLeftFrontMotorDirection(DcMotorSimple.Direction.REVERSE);
        driveConstants.setLeftRearMotorDirection(DcMotorSimple.Direction.REVERSE);
        driveConstants.setRightRearMotorDirection(DcMotorSimple.Direction.FORWARD);
        driveConstants.setRightFrontMotorDirection(DcMotorSimple.Direction.FORWARD);

        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}