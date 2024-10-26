package org.firstinspires.ftc.teamcode.drive.opmode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name = "IncredibotsAutoSpecimen", group = "Autonomous")
/* This auto opmode will do the following steps:
 1) Robot will start from the inside edge of third tile from closest vertical wall
 2) The robot will hang the preloaded specimen on the top rung
 3) The robot will then go back to the observation area to pick another specimen
 4) The robot will hang the second specimen
 5) The robot will go park in the ascent zone.
 */
public class IncredibotsAutoSpecimen extends LinearOpMode {

    //Observation starting position - 47.25 - inside edge of third tile from closest vertical wall


    public static double START_X = 8.504;
    public static double START_Y = 56.1325;
    public static double START_H = Math.toRadians(0);
    Pose2d startPose = new Pose2d(START_X, START_Y, START_H);

    public static double STEP1_X = 25.754;
    public static double STEP1_Y = 59.6325;
    public static double STEP1_H = Math.toRadians(0);
    Pose2d step1Pose = new Pose2d(STEP1_X, STEP1_Y, STEP1_H);
    //ADD ARM MOTIONS HERE

    public static double STEP2_X = 8.504;
    public static double STEP2_Y = 56.1325;
    public static double STEP2_H = Math.toRadians(0);
    Pose2d step2Pose = new Pose2d(STEP2_X, STEP2_Y, STEP2_H);
    //ADD ARM MOTIONS HERE

    public static double STEP3_X = 8.504;
    public static double STEP3_Y = 56.1325;
    public static double STEP3_H = Math.toRadians(0);
    Pose2d step3Pose = new Pose2d(STEP3_X, STEP3_Y, STEP3_H);
    //ADD ARM MOTIONS HERE

    public static double STEP4_X = 8.504;
    public static double STEP4_Y = 56.1325;
    public static double STEP4_H = Math.toRadians(0);
    Pose2d step4Pose = new Pose2d(STEP4_X, STEP4_Y, STEP4_H);
    //ADD ARM MOTIONS HERE

    public static double END_X = 8.504;
    public static double END_Y = 56.1325;
    public static double END_H = Math.toRadians(0);
    Pose2d endPose = new Pose2d(END_X, END_Y, END_H);
    //ADD ARM MOTIONS HERE


    RobotHardware myHardware;
    IncredibotsArmControl armControl;
    MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        myHardware = new RobotHardware(this.hardwareMap);
        armControl = new IncredibotsArmControl(gamepad2, myHardware);
        drive = new MecanumDrive(this.hardwareMap, startPose);

        Action dropLoadedSpecimen = drive.actionBuilder(startPose)
                .strafeTo(step1Pose.position).build();

        Action moveToPickNextSpecimen;
        Action dropSecondSpecimen;
        Action park;


        //start position 2, right next to the net zone
//        TrajectoryActionBuilder tr2 = drive.actionBuilder(initialPose2)
//                .strafeTo(new Vector2d(35, 40))
//                .turn(Math.toRadians(45))
//                //EXTEND ARM
//                .strafeTo(new Vector2d(55, 55));

        //3rd trajectory, from dropping off sample to parking in net zone
//        TrajectoryActionBuilder trcom = drive.actionBuilder(endOfActPose)
//                .strafeTo(new Vector2d(45, 45))
//                .strafeTo(new Vector2d(-55, 55))
//                .turn(Math.toRadians(-135))
//                .lineToY(62);



        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            Actions.runBlocking(
                    new SequentialAction(
                            dropLoadedSpecimen
                    )
            );
        }


//        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
//                .lineToYSplineHeading(33, Math.toRadians(0))
//                .waitSeconds(2)
//                .setTangent(Math.toRadians(90))
//                .lineToY(48)
//                .lineToX(32)
//                .setTangent(Math.toRadians(0))
//                .strafeTo(new Vector2d(44.5, 30))
//                .turn(Math.toRadians(180))
//                .lineToX(47.5)
//                .waitSeconds(3);

        //strafes in front of blue tape for net zone

        //https://rr.brott.dev/docs/v1-0/guides/centerstage-auto/
        //https://rr.brott.dev/docs/v1-0/guides/centerstage-auto/

    }
}