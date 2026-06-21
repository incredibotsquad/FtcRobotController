package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveSubsystem extends SubsystemBase {
    private final MecanumDrive mecanumDrive;
    private final MotorEx frontLeftMotor;
    private final MotorEx frontRightMotor;
    private final MotorEx backLeftMotor;
    private final MotorEx backRightMotor;

    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        frontLeftMotor = new MotorEx(hardwareMap, "frontLeftMotor", Motor.GoBILDA.RPM_435);
        frontRightMotor = new MotorEx(hardwareMap, "frontRightMotor", Motor.GoBILDA.RPM_435);
        backLeftMotor = new MotorEx(hardwareMap, "backLeftMotor", Motor.GoBILDA.RPM_435);
        backRightMotor = new MotorEx(hardwareMap, "backRightMotor", Motor.GoBILDA.RPM_435);

        //this will auto reverse the right side motors
        mecanumDrive = new MecanumDrive(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
    }

    public void drive(double x, double y, double rotation) {
        mecanumDrive.driveRobotCentric(x, y, rotation);
    }
}
