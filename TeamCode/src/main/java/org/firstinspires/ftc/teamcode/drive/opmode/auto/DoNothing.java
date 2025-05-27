package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.opmode.RobotControl;

@Autonomous(name = "DoNothing", group = "Autonomous")
public class DoNothing extends BaseAuto{

    public static double heading = Math.toRadians(90);
    public static Pose2d startPose = new Pose2d(0, 0, heading);

    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware = new RobotHardware(this.hardwareMap);
        robotControl = new RobotControl(gamepad2, robotHardware);

        mecanumDrive = new MecanumDrive(this.hardwareMap, startPose);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.update();

            break;
        }
    }
}
