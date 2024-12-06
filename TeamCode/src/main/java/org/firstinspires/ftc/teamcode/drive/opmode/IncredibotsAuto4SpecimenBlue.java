package org.firstinspires.ftc.teamcode.drive.opmode;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;


@Config
@Autonomous(name = "IncredibotsAuto4SpecimenBlue", group = "Autonomous")
public class IncredibotsAuto4SpecimenBlue extends IncredibotsAuto {

    public static final double heading = Math.toRadians(-90);
    public static final double reverseHeading = Math.toRadians(90);

    public static Pose2d INIT_POS = new Pose2d(-16, 60.75, heading);
    public static Vector2d SLIDE_NEXT_TO_SAMP_1 = new Vector2d(-36, 20);
    public static Vector2d SLIDE_BEHIND_SAMP_1 = new Vector2d(-44, 15);
    public static Vector2d PUSH_SAMP_1 = new Vector2d(SLIDE_BEHIND_SAMP_1.x, 52);
    public static Vector2d SLIDE_BEHIND_SAMP_2 = new Vector2d(-54, 15);
    public static Vector2d PUSH_SAMP_2 = new Vector2d(SLIDE_BEHIND_SAMP_2.x, 52);

    public static Vector2d PICK_SPECIMEN_ONE = new Vector2d(PUSH_SAMP_2.x, 56);

    //    public static Vector2d SLIDE_BEHIND_SAMP_3 = new Vector2d(-64, 15);
    //    public static Vector2d PUSH_SAMP_3 = new Vector2d(SLIDE_BEHIND_SAMP_3.x, 52);

    public static Pose2d BRACE_RUNGS_FOR_SPECIMEN_ONE = new Pose2d(-4, 30, heading);
    public static Pose2d MOVE_TO_BRACE_SUB_SPECIMEN_ONE = new Pose2d(BRACE_RUNGS_FOR_SPECIMEN_ONE.position.x, 32, heading);

    RobotHardware myHardware;
    IncredibotsArmControl armControl;
    MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        myHardware = new RobotHardware(this.hardwareMap);
        armControl = new IncredibotsArmControl(gamepad2, myHardware);
        drive = new MecanumDrive(this.hardwareMap, INIT_POS);

        //Put all the actions here, but replace the sample pushing with continuous splines below

        Action robotPreloadedSpecimen = drive.actionBuilder(INIT_POS)
                .strafeToConstantHeading(BRACE_RUNGS_FOR_SPECIMEN_ONE.position)
//                .setTangent(heading)
//                .lineToYConstantHeading(MOVE_TO_BRACE_SUB_SPECIMEN_ONE.position.y, new TranslationalVelConstraint(40), new ProfileAccelConstraint(-20, 40))
                .build();

        Action SamplePushingWithSplines = drive.actionBuilder(BRACE_RUNGS_FOR_SPECIMEN_ONE)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(SLIDE_NEXT_TO_SAMP_1, heading, new TranslationalVelConstraint(30), new ProfileAccelConstraint(-20, 40))
                .splineToConstantHeading(SLIDE_BEHIND_SAMP_1, reverseHeading, new TranslationalVelConstraint(30), new ProfileAccelConstraint(-20, 40))
                .splineToConstantHeading(PUSH_SAMP_1, reverseHeading, new TranslationalVelConstraint(30), new ProfileAccelConstraint(-20, 40))
                .splineToConstantHeading(SLIDE_BEHIND_SAMP_1, heading, new TranslationalVelConstraint(30), new ProfileAccelConstraint(-20, 40))
                .splineToConstantHeading(SLIDE_BEHIND_SAMP_2, reverseHeading, new TranslationalVelConstraint(30), new ProfileAccelConstraint(-20, 40))
                .splineToConstantHeading(PUSH_SAMP_2, reverseHeading, new TranslationalVelConstraint(30), new ProfileAccelConstraint(-20, 40))
//                .splineToConstantHeading(SLIDE_BEHIND_SAMP_3, Math.toRadians(0), new TranslationalVelConstraint(30), new ProfileAccelConstraint(-10, 30))
//                .splineToConstantHeading(PUSH_SAMP_3, Math.toRadians(90), new TranslationalVelConstraint(30), new ProfileAccelConstraint(-10, 30))
                .build();

        Action pickAndSnapSpecimenOne = drive.actionBuilder(new Pose2d(PUSH_SAMP_2, heading))
                .lineToYConstantHeading(PICK_SPECIMEN_ONE.y)
                .stopAndAdd(GetClawControlAction(false, true, false))
                .build();


        waitForStart();

        while (opModeIsActive()) {
            ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

            Actions.runBlocking(
                    new SequentialAction(
                        new ParallelAction(
                                armControl.GetHangSpecimenActionSequence(),
                                robotPreloadedSpecimen),
                        armControl.GetSnapSpecimenActionSequence()
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                        new ParallelAction(
                                armControl.GetArmVerticalActionSequence(),
                                SamplePushingWithSplines),
                        armControl.GetPickSpecimenActionSequence()
                    )
            );

            Actions.runBlocking(new SequentialAction(
                    pickAndSnapSpecimenOne
            ));

            telemetry.addData("Elapsed Time: ", timer.milliseconds());
            Log.i("=== INCREDIBOTS  ===", "ELAPSED TIME: " + timer.milliseconds());
            telemetry.update();

            //break;
        }
    }
}
