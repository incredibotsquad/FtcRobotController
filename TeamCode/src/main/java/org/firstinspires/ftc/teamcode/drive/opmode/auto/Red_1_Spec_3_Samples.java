package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.opmode.IncredibotsArmControl;


@Config
@Disabled
@Autonomous(name = "Red_1_Spec_3_Samples", group = "Autonomous")
public class Red_1_Spec_3_Samples extends IncredibotsAuto {

    public static final int multiplier = -1; //used to flip coordinates between blue (1) and red (-1)

    public static double heading = Math.toRadians(-90 * multiplier);
    public static double basketHeading = Math.toRadians(-111 * multiplier);
    public static double farSampleHeading = Math.toRadians(-73 * multiplier);
    public static Pose2d INIT_POS = new Pose2d(41 * multiplier, 60.75 * multiplier, heading);
    public static Pose2d DROP_PRELOADED_SAMPLE = new Pose2d(60.5 * multiplier, 54 * multiplier, heading);

    public static Pose2d BRACE_RUNGS_FOR_SPECIMEN = new Pose2d(3.5 * multiplier, 30 * multiplier, heading);



    @Override
    public void runOpMode() throws InterruptedException {
        myHardware = new RobotHardware(this.hardwareMap);
        armControl = new IncredibotsArmControl(gamepad2, myHardware);
        drive = new MecanumDrive(this.hardwareMap, INIT_POS);

        Action hangPreloadedSpecimen = drive.actionBuilder(INIT_POS)
                .strafeToConstantHeading(BRACE_RUNGS_FOR_SPECIMEN.position)
                .build();

        waitForStart();

        while (opModeIsActive()) {
            Actions.runBlocking(
                    new SequentialAction(hangPreloadedSpecimen));

            break;
        }
    }
}
