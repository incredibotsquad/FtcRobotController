package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



@Config
@Autonomous(name = "IncredibotsAuto4SamplesBlue", group = "Autonomous")
public class IncredibotsAuto4SamplesBlue  extends LinearOpMode {

    RobotHardware myHardware;
    IncredibotsArmControl armControl;
    MecanumDrive drive;

    public static double heading = Math.toRadians(-90);


    public static Pose2d INIT_POS = new Pose2d(38, 62, Math.toRadians(90));
    public static Pose2d OPEN_ARM = new Pose2d(38, 40, Math.toRadians(90));
    public static Pose2d HANG_SPECIMEN = new Pose2d(5, 33, Math.toRadians(90));
    public static Pose2d BACK_POST_HANG = new Pose2d(-36, 35, Math.toRadians(90));
    public static Pose2d ALIGN_NEXT_TO_SAMP_1 = new Pose2d(-36, 9, Math.toRadians(90));
    public static Pose2d SLIDE_UNDER_SAMP_1 = new Pose2d(-42, 9, Math.toRadians(90));
    public static Pose2d PUSH_SAMP_1 = new Pose2d(-43, 60, Math.toRadians(90));
    public static Pose2d BACK_OPEN_ARM = new Pose2d(-62, 40, Math.toRadians(90));

    public static Pose2d PARK_POS = new Pose2d(22, 10, Math.toRadians(90));


    public static Pose2d DROP_SAMP = new Pose2d(53, 53, Math.toRadians(45));
    public static Pose2d COMMON_ARM_CLOSE_POS = new Pose2d(37, 37, Math.toRadians(90));

    public static Pose2d DROP_UNDER_SAMPLES = new Pose2d(37, 4, Math.toRadians(90));
    public static Pose2d ALIGN_WITH_SAMP_1 = new Pose2d(48, 38, Math.toRadians(90));
    public static Pose2d OPEN_ARM_WHILE_MOVING = new Pose2d(48, 48, Math.toRadians(90));
    public static Pose2d IN_FRONT_OF_ASCENT_ZONE = new Pose2d(37, 10, Math.toRadians(90));


    public static Pose2d ALIGN_WITH_SAMP_2 = new Pose2d(58, 38, Math.toRadians(90));
    public static Pose2d PICK_SPEC = new Pose2d(-62, 50, Math.toRadians(90));
    public static Pose2d PICK_3RD_SAMP = new Pose2d(56, 13, Math.toRadians(45));

    @Override
    public void runOpMode() throws InterruptedException {
        myHardware = new RobotHardware(this.hardwareMap);
        armControl = new IncredibotsArmControl(gamepad2, myHardware);
        drive = new MecanumDrive(this.hardwareMap, INIT_POS);

        waitForStart();

        while (opModeIsActive()) {

        }
    }
}
