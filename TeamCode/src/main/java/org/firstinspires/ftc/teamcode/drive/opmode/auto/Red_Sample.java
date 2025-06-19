package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import android.util.Log;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GameConstants;
import org.firstinspires.ftc.teamcode.HorizontalPickupVector;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.opmode.RobotControl;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Red_Sample", group = "Autonomous")
public class Red_Sample extends BaseAuto{
    private static final int multiplier = -1;    //used to flip coordinates between blue (1) and red (-1)

    private final SampleAutoCoordinates coordinates = new SampleAutoCoordinates(multiplier);

    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware = new RobotHardware(this.hardwareMap);
        robotControl = new RobotControl(gamepad2, robotHardware, telemetry);
        mecanumDrive = new MecanumDrive(this.hardwareMap, coordinates.INIT_POS);

        robotControl.setGameColor(GameConstants.GAME_COLORS.RED);
        robotHardware.startLimelight();

        //Put all the actions here, but replace the sample pushing with continuous splines below

        Action dropPreloadedSample = mecanumDrive.actionBuilder(coordinates.INIT_POS)
                .strafeToLinearHeading(coordinates.DROP_SAMPLE_IN_BASKET.position, coordinates.DROP_SAMPLE_IN_BASKET.heading)
                .build();

        Action moveToPickSample1 = mecanumDrive.actionBuilder(coordinates.DROP_SAMPLE_IN_BASKET)
                .strafeToLinearHeading(coordinates.PICK_SAMPLE_1.position, coordinates.PICK_SAMPLE_1.heading)
                .build();

        Action dropSample1 = mecanumDrive.actionBuilder(coordinates.PICK_SAMPLE_1)
                .strafeToLinearHeading(coordinates.DROP_SAMPLE_IN_BASKET.position, coordinates.DROP_SAMPLE_IN_BASKET.heading)
                .build();

        Action moveToPickSample2 = mecanumDrive.actionBuilder(coordinates.DROP_SAMPLE_IN_BASKET)
                .strafeToLinearHeading(coordinates.PICK_SAMPLE_2.position, coordinates.PICK_SAMPLE_2.heading)
                .build();

        Action dropSample2 = mecanumDrive.actionBuilder(coordinates.PICK_SAMPLE_2)
                .strafeToLinearHeading(coordinates.DROP_SAMPLE_IN_BASKET.position, coordinates.DROP_SAMPLE_IN_BASKET.heading)
                .build();

        Action moveToPickSample3 = mecanumDrive.actionBuilder(coordinates.DROP_SAMPLE_IN_BASKET)
                .strafeToLinearHeading(coordinates.PICK_SAMPLE_3.position, coordinates.PICK_SAMPLE_3.heading)
                .build();

        Action dropSample3 = mecanumDrive.actionBuilder(coordinates.PICK_SAMPLE_3)
                .strafeToLinearHeading(coordinates.DROP_SAMPLE_IN_BASKET.position, coordinates.DROP_SAMPLE_IN_BASKET.heading)
                .build();


//
//        Action park = mecanumDrive.actionBuilder(coordinates.BRACE_RUNGS_FOR_SPECIMEN_FOUR)
//                .setTangent(coordinates.reverseHeading)
//                .splineToConstantHeading(coordinates.PARK.position, coordinates.PARK.heading)
//                .build();

        waitForStart();

        while (opModeIsActive()) {

            ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

            //drop preloaded sample
            Actions.runBlocking(
                    new ParallelAction(
                            dropPreloadedSample,
                            robotControl.GetHighBasketActionSequence(false)
                    )
            );

            Log.i("=== INCREDIBOTS / SPECIMEN AUTO ===", "ELAPSED TIME AFTER PRELOADED DROP: " + timer.milliseconds());

            //pick and drop sample 1
            Actions.runBlocking(
                    new ParallelAction(
                            new InstantAction(() -> {
                                List<HorizontalPickupVector> choices = new ArrayList<>();
                                choices.add(robotControl.GetHorizontalPickupVectorFromLimelightLocation(coordinates.SAMPLE1_LIMELIGHT_LOCATION));
                                robotControl.SetSampleChoices(choices);
                            }),
                            moveToPickSample1
                    )
            );

            Actions.runBlocking(
                    robotControl.GetPickSampleActionSequence()
            );

            Actions.runBlocking(
                    new ParallelAction(
                            dropSample1,
                            robotControl.GetHighBasketActionSequence(true)
                    )
            );

            Log.i("=== INCREDIBOTS / SPECIMEN AUTO ===", "ELAPSED TIME SAMPLE 1 DROP: " + timer.milliseconds());


            //pick and drop sample 2
            Actions.runBlocking(
                    new ParallelAction(
                            new InstantAction(
                                    () -> {
                                        List<HorizontalPickupVector> choices = new ArrayList<>();
                                        choices.add(robotControl.GetHorizontalPickupVectorFromLimelightLocation(coordinates.SAMPLE2_LIMELIGHT_LOCATION));
                                        robotControl.SetSampleChoices(choices);
                                    }),
                            moveToPickSample2
                    )
            );

            Actions.runBlocking(
                    robotControl.GetPickSampleActionSequence()
            );

            Actions.runBlocking(
                    new ParallelAction(
                            dropSample2,
                            robotControl.GetHighBasketActionSequence(true)
                    )
            );

            Log.i("=== INCREDIBOTS / SPECIMEN AUTO ===", "ELAPSED TIME SAMPLE 2 DROP: " + timer.milliseconds());

            //pick and drop sample 3
            Actions.runBlocking(
                    new ParallelAction(
                            new InstantAction(() -> {
                                List<HorizontalPickupVector> choices = new ArrayList<>();
                                choices.add(robotControl.GetHorizontalPickupVectorFromLimelightLocation(coordinates.SAMPLE3_LIMELIGHT_LOCATION));
                                robotControl.SetSampleChoices(choices);
                            }),
                            moveToPickSample3
                    )
            );

            Actions.runBlocking(
                    robotControl.GetPickSampleActionSequence()
            );

            Actions.runBlocking(
                    new ParallelAction(
                            dropSample3,
                            robotControl.GetHighBasketActionSequence(true)
                    )
            );

            Log.i("=== INCREDIBOTS / SPECIMEN AUTO ===", "ELAPSED TIME SAMPLE 3 DROP: " + timer.milliseconds());

            break;
        }

        robotHardware.stopLimelight();
    }}
