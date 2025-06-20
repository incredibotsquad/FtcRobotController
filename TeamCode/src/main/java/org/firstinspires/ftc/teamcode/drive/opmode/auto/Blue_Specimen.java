package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import android.util.Log;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GameConstants;
import org.firstinspires.ftc.teamcode.HorizontalPickupVector;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.opmode.RobotControl;
import org.firstinspires.ftc.teamcode.drive.opmode.auto.actions.VerticalClawAction;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Blue_Specimen", group = "Autonomous")
public class Blue_Specimen extends BaseAuto{
    private static final int multiplier = 1;    //used to flip coordinates between blue (1) and red (-1)

    private final SpecimenAutoCoordinates coordinates = new SpecimenAutoCoordinates(multiplier);

    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware = new RobotHardware(this.hardwareMap);
        robotControl = new RobotControl(gamepad2, robotHardware, telemetry);
        mecanumDrive = new MecanumDrive(this.hardwareMap, coordinates.INIT_POS);


        robotControl.setGameColor(GameConstants.GAME_COLORS.BLUE);
        robotHardware.startLimelight();

        //Put all the actions here, but replace the sample pushing with continuous splines below

        Action snapPreloadedSpecimen = mecanumDrive.actionBuilder(coordinates.INIT_POS)
                .strafeToConstantHeading(coordinates.BRACE_RUNGS_FOR_SPECIMEN_ONE.position)
                .build();

        Action moveToPickSample1 = mecanumDrive.actionBuilder(coordinates.INIT_POS)
                .setTangent(coordinates.heading)
                .splineToConstantHeading(coordinates.PICK_SAMPLE_1.position, coordinates.PICK_SAMPLE_1.heading)
                .build();

        Action moveToPickSample2 = mecanumDrive.actionBuilder(coordinates.PICK_SAMPLE_1)
                .strafeToConstantHeading(coordinates.PICK_SAMPLE_2.position)
                .build();

        Action moveToPickSample3 = mecanumDrive.actionBuilder(coordinates.PICK_SAMPLE_2)
                .strafeToConstantHeading(coordinates.PICK_SAMPLE_3.position)
                .build();

        Action moveToPickSpecimen = mecanumDrive.actionBuilder(coordinates.PICK_SAMPLE_3)
                .setTangent(coordinates.heading)
                .splineToLinearHeading(coordinates.PICK_SPECIMEN, coordinates.reverseHeading)
                .lineToY(coordinates.PICK_SPECIMEN_SLOW.position.y, new TranslationalVelConstraint(20), new ProfileAccelConstraint(coordinates.minAccel, 20))
                .build();

        Action moveToSnapSpecimen1 = mecanumDrive.actionBuilder(coordinates.PICK_SPECIMEN_SLOW)
                .setTangent(coordinates.heading)
                .splineToConstantHeading(coordinates.BRACE_RUNGS_FOR_SPECIMEN_ONE.position, coordinates.heading)
                .build();

        Action moveToPickSpecimen2 = mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose())
                .setTangent(coordinates.reverseHeading)
                .splineToConstantHeading(coordinates.PICK_SPECIMEN.position, coordinates.reverseHeading)
                .lineToY(coordinates.PICK_SPECIMEN_SLOW.position.y, new TranslationalVelConstraint(20), new ProfileAccelConstraint(coordinates.minAccel, 20))
                .build();

        Action moveToSnapSpecimen2 = mecanumDrive.actionBuilder(coordinates.PICK_SPECIMEN_SLOW)
                .setTangent(coordinates.heading)
                .splineToConstantHeading(coordinates.BRACE_RUNGS_FOR_SPECIMEN_TWO.position, coordinates.heading)
                .build();

        Action park = mecanumDrive.actionBuilder(coordinates.BRACE_RUNGS_FOR_SPECIMEN_FOUR)
                .setTangent(coordinates.reverseHeading)
                .splineToConstantHeading(coordinates.PARK.position, coordinates.PARK.heading)
                .build();

        waitForStart();

        while (opModeIsActive()) {
            ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

            //snap preloaded specimen
//            Actions.runBlocking(
//                    new ParallelAction(
//                            snapPreloadedSpecimen,
//                            robotControl.GetHangSpecimenActionSequence()
//                    )
//            );
//
//            Actions.runBlocking(
//                    robotControl.GetSnapSpecimenActionSequence()
//            );


//            pick sample 1
            Actions.runBlocking(
                    new ParallelAction(
                            new InstantAction(() -> {
                                List<HorizontalPickupVector> choices = new ArrayList<>();
                                choices.add(robotControl.GetHorizontalPickupVectorFromLimelightLocation(coordinates.SAMPLE1_LIMELIGHT_LOCATION));
                                robotControl.SetSampleChoices(choices);
                            }),
                            moveToPickSample1,
                            robotControl.GetEnterExitSubActionSequence(true)
                    )
            );

            Actions.runBlocking(
                    robotControl.GetPickSampleActionSequence()
            );

            Log.i("=== INCREDIBOTS / SPECIMEN AUTO ===", "ELAPSED TIME AFTER PICKING SAMPLE 1: " + timer.milliseconds());

            Actions.runBlocking(
                    new ParallelAction(
                            robotControl.GetTransferToObZoneActionSequence(),
                            moveToPickSample2,
                            new InstantAction(() -> {
                                List<HorizontalPickupVector> choices = new ArrayList<>();
                                choices.add(robotControl.GetHorizontalPickupVectorFromLimelightLocation(coordinates.SAMPLE2_LIMELIGHT_LOCATION));
                                robotControl.SetSampleChoices(choices);
                            })
                    )
            );

            Actions.runBlocking(
                    robotControl.GetPickSampleActionSequence()
            );

            Log.i("=== INCREDIBOTS / SPECIMEN AUTO ===", "ELAPSED TIME AFTER PICKING SAMPLE 2: " + timer.milliseconds());

            Actions.runBlocking(
                    new ParallelAction(
                            robotControl.GetTransferToObZoneActionSequence(),
                            moveToPickSample3,
                            new InstantAction(() -> {
                                List<HorizontalPickupVector> choices = new ArrayList<>();
                                choices.add(robotControl.GetHorizontalPickupVectorFromLimelightLocation(coordinates.SAMPLE3_LIMELIGHT_LOCATION));
                                robotControl.SetSampleChoices(choices);
                            })
                    )
            );

            Actions.runBlocking(
                    robotControl.GetPickSampleActionSequence()
            );

            Actions.runBlocking(
                    robotControl.GetTransferToObZoneActionSequence()
            );

            Log.i("=== INCREDIBOTS / SPECIMEN AUTO ===", "ELAPSED TIME AFTER PICKING SAMPLE 3: " + timer.milliseconds());

            Actions.runBlocking(
                    new ParallelAction(
                            moveToPickSpecimen,
                            robotControl.GetPickSpecimenActionSequence()
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            new VerticalClawAction(robotHardware, false, true, false),
                            new ParallelAction(
                                    robotControl.GetHangSpecimenActionSequence(),
                                    moveToSnapSpecimen1
                            )
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            robotControl.GetSnapSpecimenActionSequence(),
                            new InstantAction(()->mecanumDrive.localizer.update())  //update the pose estimate
                    )
            );

            //pick and snap specimen 2
            Actions.runBlocking(
                    new ParallelAction(
                            moveToPickSpecimen2,
                            robotControl.GetPickSpecimenActionSequence()
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            new VerticalClawAction(robotHardware, false, true, false),
                            new ParallelAction(
                                    robotControl.GetHangSpecimenActionSequence(),
                                    moveToSnapSpecimen2
                            )
                    )
            );

            Actions.runBlocking(
                    robotControl.GetSnapSpecimenActionSequence()
            );

            break;
    }

    robotHardware.stopLimelight();

}}
