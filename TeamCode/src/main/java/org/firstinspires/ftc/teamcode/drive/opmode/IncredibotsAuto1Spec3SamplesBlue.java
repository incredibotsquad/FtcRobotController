package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;


@Config
@Autonomous(name = "IncredibotsAuto1Spec3SamplesBlue", group = "Autonomous")
public class IncredibotsAuto1Spec3SamplesBlue extends LinearOpMode {

    RobotHardware myHardware;
    IncredibotsArmControl armControl;
    MecanumDrive drive;

    public static double heading = Math.toRadians(-90);


    public static Pose2d INIT_POS = new Pose2d(32, 63, heading);
    public static Pose2d HANG_SPECIMEN = new Pose2d(4, 31, heading);

    public static Pose2d BACK_POST_HANG = new Pose2d(-36, 35, heading);
    public static Pose2d ALIGN_NEXT_TO_SAMP_1 = new Pose2d(-36, 9, heading);
    public static Pose2d SLIDE_UNDER_SAMP_1 = new Pose2d(-42, 9, heading);
    public static Pose2d PUSH_SAMP_1 = new Pose2d(-43, 60, heading);
    public static Pose2d BACK_OPEN_ARM = new Pose2d(-62, 40, heading);

    public static Pose2d PARK_POS = new Pose2d(22, 10, heading);


    public static Pose2d DROP_SAMP = new Pose2d(53, 53, Math.toRadians(45));
    public static Pose2d COMMON_ARM_CLOSE_POS = new Pose2d(37, 37, heading);

    public static Pose2d DROP_UNDER_SAMPLES = new Pose2d(37, 4, heading);
    public static Pose2d ALIGN_WITH_SAMP_1 = new Pose2d(48, 38, heading);
    public static Pose2d OPEN_ARM_WHILE_MOVING = new Pose2d(48, 48, heading);
    public static Pose2d IN_FRONT_OF_ASCENT_ZONE = new Pose2d(37, 10, heading);


    public static Pose2d ALIGN_WITH_SAMP_2 = new Pose2d(58, 38, heading);
    public static Pose2d PICK_SPEC = new Pose2d(-62, 50, heading);
    public static Pose2d PICK_3RD_SAMP = new Pose2d(56, 13, Math.toRadians(45));

    private Action GetArmControlAction(int position, int velocity, boolean waitForAction) {
        return new ArmMotionAsRRAction(myHardware, position, velocity, waitForAction);
    }

    private Action GetSlideControlAction(int position, boolean waitForAction) {
        return new SlideMotionAsRRAction(myHardware, position, waitForAction);
    }

    private Action GetClawControlAction(boolean open, double openPosition, double closePosition, boolean waitForAction, boolean shortWait) {
        return new ClawMotionAsRRAction(myHardware, open, openPosition, closePosition, waitForAction, shortWait);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        myHardware = new RobotHardware(this.hardwareMap);
        armControl = new IncredibotsArmControl(gamepad2, myHardware);
        drive = new MecanumDrive(this.hardwareMap, INIT_POS);

        Action hangPreloadedSpecimen = drive.actionBuilder(INIT_POS)
                .strafeToConstantHeading(HANG_SPECIMEN.position)
                .build();

        waitForStart();

        while (opModeIsActive()) {
            Actions.runBlocking(
                    new SequentialAction(hangPreloadedSpecimen));

            break;
        }
    }
}
