package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import android.util.Log;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GamePattern;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchSystem;
import org.firstinspires.ftc.teamcode.subsystems.LightSystem;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;

@Autonomous(name = "Blue_Near_Auto", group = "Autonomous")
public class BlueNearAuto extends LinearOpMode {

    public RobotHardware robotHardware;
    private Spindex spindex;
    private IntakeSystem intakeSystem;
    private LaunchSystem launchSystem;
    private LightSystem lightSystem;
    public MecanumDrive mecanumDrive;

    private static final int multiplier = -1;    //used to flip coordinates between red (1), Blue (-1)

    public double robotHeading = Math.toRadians(125 * multiplier);
    public double goalHeading = Math.toRadians(135 * multiplier);

//    public double splineHeading = Math.toRadians(-45 * multiplier);
    public double splineHeading = Math.toRadians(0);

    public double obeliskHeading = Math.toRadians(180); //this is same for both sides - do NOT need a multiplier

    public double artifactHeading = Math.toRadians(90 * multiplier);

    public int FORWARD_DELTA_TO_COLLECT_BALLS = 15;

    public Pose2d INIT_POS = new Pose2d(-54, 54 * multiplier, robotHeading);

    public Pose2d TAG_READ_POS = new Pose2d(-36, 12 * multiplier, obeliskHeading);
    public Pose2d LAUNCH_BALLS = new Pose2d(-64, 17 * multiplier, artifactHeading); //-32 / 27
//    public Pose2d LAUNCH_BALLS = new Pose2d(-30, 22 * multiplier, goalHeading); //-32 / 27
    public Pose2d COLLECT_LINE_1_START = new Pose2d(-12, 43 * multiplier, artifactHeading);
    public Pose2d COLLECT_LINE_1_END = new Pose2d(-12, (43 + FORWARD_DELTA_TO_COLLECT_BALLS) * multiplier, artifactHeading);

    public Pose2d COLLECT_LINE_2_START = new Pose2d(12, 43 * multiplier, artifactHeading);
    public Pose2d COLLECT_LINE_2_END = new Pose2d(12, (43 + FORWARD_DELTA_TO_COLLECT_BALLS) * multiplier, artifactHeading);

    public Pose2d COLLECT_LINE_3_START = new Pose2d(36, 43 * multiplier, artifactHeading);
    public Pose2d COLLECT_LINE_3_END = new Pose2d(36, (43 + FORWARD_DELTA_TO_COLLECT_BALLS) * multiplier, artifactHeading);
    public Pose2d MOVE_OFF_LINE = new Pose2d(-24, 48  * multiplier, artifactHeading);

    @Override
    public void runOpMode() throws InterruptedException {

        robotHardware = new RobotHardware(this.hardwareMap);

        this.spindex = new Spindex(robotHardware);
        this.spindex.initializeWithPPG();
        this.lightSystem = new LightSystem(robotHardware);
        this.intakeSystem = new IntakeSystem(robotHardware, this.spindex, lightSystem);
        this.launchSystem = new LaunchSystem(robotHardware, this.spindex, lightSystem);

        mecanumDrive = new MecanumDrive(this.hardwareMap, INIT_POS);

        robotHardware.startLimelight();
        robotHardware.setLimelightPipeline(6);

        Action moveToReadTag =mecanumDrive.actionBuilder(INIT_POS)
                .strafeToLinearHeading(TAG_READ_POS.position, TAG_READ_POS.heading)
                .build();

        //initialize all the paths here
        Action launchFromTagRead = mecanumDrive.actionBuilder(TAG_READ_POS)
                .strafeToLinearHeading(LAUNCH_BALLS.position, LAUNCH_BALLS.heading)
                .build();

        Action pickLine1AfterLaunch = mecanumDrive.actionBuilder(LAUNCH_BALLS)
                .setTangent(splineHeading)
                .splineToLinearHeading(COLLECT_LINE_1_START, COLLECT_LINE_1_START.heading)
                .build();

//        Action pickLine1AfterLaunch = mecanumDrive.actionBuilder(LAUNCH_BALLS)
//                .strafeToLinearHeading(COLLECT_LINE_1_START.position, COLLECT_LINE_1_START.heading)
//                .build();

        Action moveForwardToPickLine1 = mecanumDrive.actionBuilder(COLLECT_LINE_1_START)
                .setTangent(artifactHeading)
                .lineToY(COLLECT_LINE_1_END.position.y)
                .build();

        Action launchAfterPickingLine1 = mecanumDrive.actionBuilder(COLLECT_LINE_1_END)
                .strafeToLinearHeading(LAUNCH_BALLS.position, LAUNCH_BALLS.heading)
                .build();

        Action pickLine2AfterLaunch = mecanumDrive.actionBuilder(LAUNCH_BALLS)
                .strafeToLinearHeading(COLLECT_LINE_2_START.position, COLLECT_LINE_2_START.heading)
                .setTangent(artifactHeading)
                .lineToY(COLLECT_LINE_2_END.position.y)
                .build();

        Action launchAfterPickingLine2 = mecanumDrive.actionBuilder(COLLECT_LINE_2_END)
                .strafeToLinearHeading(LAUNCH_BALLS.position, LAUNCH_BALLS.heading)
                .build();

        Action pickLine3AfterLaunch = mecanumDrive.actionBuilder(LAUNCH_BALLS)
                .strafeToLinearHeading(COLLECT_LINE_3_START.position, COLLECT_LINE_3_START.heading)
                .setTangent(artifactHeading)
                .lineToY(COLLECT_LINE_3_END.position.y)
                .build();

        Action launchAfterPickingLine3 = mecanumDrive.actionBuilder(COLLECT_LINE_3_END)
                .strafeToLinearHeading(LAUNCH_BALLS.position, LAUNCH_BALLS.heading)
                .build();

        Action moveAwayFromLine = mecanumDrive.actionBuilder(LAUNCH_BALLS)
                .strafeToLinearHeading(COLLECT_LINE_1_START.position, COLLECT_LINE_1_START.heading)
                .build();

        while (opModeInInit()) {
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

            GamePattern pattern;

            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    moveToReadTag,
                                    launchSystem.getKeepWarmAction()
                            ),
                            new SleepAction(1)
                    )
            );

            pattern = launchSystem.readGamePattern();

            Actions.runBlocking(
                    new ParallelAction(
                            intakeSystem.getTurnOffAction(),
                            launchFromTagRead
                    )
            );

            Actions.runBlocking(launchSystem.getBallPatternLaunchAction(pattern));

            Actions.runBlocking(
                    new ParallelAction(
                            pickLine1AfterLaunch,
                            intakeSystem.getTurnOnAction()
                    )
            );

            //Need an action to index balls
            Actions.runBlocking(
                    new ParallelAction(
                            moveForwardToPickLine1,
                            spindex.reIndexBalls()
                    )
            );

            Actions.runBlocking(
                    new ParallelAction(
                            intakeSystem.getReverseIntakeAction(),
                            launchAfterPickingLine1
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            launchSystem.getBallPatternLaunchAction(pattern),
                            new SleepAction(1)
                    )
            );

            Log.i("RED NEAR AUTO", "Elapsed time: " + timer.seconds());

//
//            Actions.runBlocking(
//                    pickLine3AfterLaunch
//            );
//
//            Actions.runBlocking(
//                    launchAfterPickingLine3
//            );

//            Actions.runBlocking(moveAwayFromLine);

            break;
        }

        robotHardware.stopLimelight();

    }
}
