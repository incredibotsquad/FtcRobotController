package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveSubsystem extends SubsystemBase {
    private final MecanumDrive mecanumDrive;
    private final MotorEx frontLeftMotor;
    private final MotorEx frontRightMotor;
    private final MotorEx backLeftMotor;
    private final MotorEx backRightMotor;

    public static String FRONT_LEFT_MOTOR_NAME = "frontLeftMotor";
    public static String FRONT_RIGHT_MOTOR_NAME = "frontRightMotor";
    public static String BACK_LEFT_MOTOR_NAME = "backLeftMotor";
    public static String BACK_RIGHT_MOTOR_NAME = "backRightMotor";

    public DriveSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        frontLeftMotor = new MotorEx(hardwareMap, "frontLeftMotor", Motor.GoBILDA.RPM_435);
        frontLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        frontRightMotor = new MotorEx(hardwareMap, "frontRightMotor", Motor.GoBILDA.RPM_435);
        frontRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        backLeftMotor = new MotorEx(hardwareMap, "backLeftMotor", Motor.GoBILDA.RPM_435);
        backLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        backRightMotor = new MotorEx(hardwareMap, "backRightMotor", Motor.GoBILDA.RPM_435);
        backRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        //this will auto reverse the right side motors
        mecanumDrive = new MecanumDrive(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
    }

    public void drive(double x, double y, double rotation) {
        mecanumDrive.driveRobotCentric(x, y, rotation);
    }
}
