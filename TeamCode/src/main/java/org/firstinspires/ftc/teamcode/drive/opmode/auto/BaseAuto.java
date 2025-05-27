package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.opmode.RobotControl;

public abstract class BaseAuto extends LinearOpMode {
    public RobotHardware robotHardware;
    public RobotControl robotControl;
    public MecanumDrive mecanumDrive;
}
