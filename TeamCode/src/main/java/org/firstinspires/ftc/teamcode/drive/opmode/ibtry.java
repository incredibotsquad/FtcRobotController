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
@Autonomous(name = "IncredibotsAuto", group = "Autonomous")
public class ibtry extends LinearOpMode {

    public static double INIT_X1 = -25;
    public static double INIT_Y1 = 62;
    public static double INIT_H1 = Math.toRadians(0);

    public static double INIT_X2 = 40;
    public static double INIT_Y2 = 62;
    public static double INIT_H2 = Math.toRadians(-90);

    public static double END_ACT_X3 = 55;
    public static double END_ACT_Y3 = 55;
    public static double END_ACT_H3 = Math.toRadians(45);

    public static int initPos = 0;

    RobotHardware myHardware;
    IncredibotsArmControl armControl;
    MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        myHardware = new RobotHardware(this.hardwareMap);
        armControl = new IncredibotsArmControl(gamepad2, myHardware);
        //we need to remove GAME PAD 2 (it is auto code)

        //starting position one, right at observatory zone
        Pose2d initialPose1 = new Pose2d(INIT_X1, INIT_Y1, Math.toRadians(INIT_H1));

        //starting position 2, right at the net zone
        Pose2d initialPose2 = new Pose2d(INIT_X2, INIT_Y2, Math.toRadians(INIT_H2));

        //Position from where robot is left off after each trajectory
        Pose2d endOfActPose = new Pose2d(END_ACT_X3, END_ACT_Y3, Math.toRadians(END_ACT_H3));

        drive = new MecanumDrive(this.hardwareMap, initialPose1);

        //first block/trajectory
        TrajectoryActionBuilder tr1 = drive.actionBuilder(initialPose1)
                .strafeTo(new Vector2d(35, 38))
                .turn(Math.toRadians(45))
                .strafeTo(new Vector2d(55, 55));

        //start position 2, right next to the net zone
        TrajectoryActionBuilder tr2 = drive.actionBuilder(initialPose2)
                .strafeTo(new Vector2d(35, 40))
                .turn(Math.toRadians(45))
                //EXTEND ARM
                .strafeTo(new Vector2d(55, 55));

        //3rd trajectory, from dropping off sample to parking in net zone
        TrajectoryActionBuilder trcom = drive.actionBuilder(endOfActPose)
                .strafeTo(new Vector2d(45, 45))
                .strafeTo(new Vector2d(-55, 55))
                .turn(Math.toRadians(-135))
                .lineToY(62);

        //build trajectory based on initial pos
        Action action1;

        if (initPos == 1) {
            action1 = tr1.build();
        }
        else {
            action1 = tr2.build();

        }
        Action action2 = trcom.build();

        waitForStart();

        while (opModeIsActive()) {
            Actions.runBlocking(
                    new SequentialAction(
                            action1,
                            action2
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