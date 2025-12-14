package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import static org.firstinspires.ftc.teamcode.Actions.LaunchFlywheelAction.FLYWHEEL_FULL_TICKS_PER_SEC;
import static org.firstinspires.ftc.teamcode.subsystems.LaunchSystem.FLYWHEEL_POWER_COEFFICIENT_FAR;
import static org.firstinspires.ftc.teamcode.subsystems.LaunchSystem.TURRET_SERVO_CENTERED;

import android.util.Log;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Actions.LaunchFlywheelAction;
import org.firstinspires.ftc.teamcode.Actions.ResetSpindexerAction;
import org.firstinspires.ftc.teamcode.Actions.SpindexAction;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.common.AllianceColors;
import org.firstinspires.ftc.teamcode.common.GameColors;
import org.firstinspires.ftc.teamcode.common.GamePattern;
import org.firstinspires.ftc.teamcode.common.LimelightAprilTagHelper;
import org.firstinspires.ftc.teamcode.common.CrossOpModeStorage;
import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchSystem;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;


@Autonomous(name = "Blue_Far_Auto_6", group = "Autonomous")
public class BlueFarAuto_6 extends BaseAuto {

    private static final int multiplier = -1;    //used to flip coordinates between red (1), Blue (-1)

    public double robotHeading = Math.toRadians(180 * multiplier);
    public double artifactHeading = Math.toRadians(90 * multiplier);
    public double splineHeading = Math.toRadians(180 * multiplier);

    public double obeliskHeading = Math.toRadians(180); //this is same for both sides - do NOT need a multiplier

    public Pose2d INIT_POS = new Pose2d(55, 16 * multiplier, robotHeading);

    public int FORWARD_DELTA_TO_COLLECT_BALL_STEP = 8; //this is repeated for each ball
    public Pose2d COLLECT_LINE_1_START = new Pose2d(36, 38 * multiplier, artifactHeading);
    public Pose2d COLLECT_LINE_1_LOW = new Pose2d(COLLECT_LINE_1_START.position.x, COLLECT_LINE_1_START.position.y + (FORWARD_DELTA_TO_COLLECT_BALL_STEP * multiplier), artifactHeading);
    public Pose2d COLLECT_LINE_1_MID = new Pose2d(COLLECT_LINE_1_LOW.position.x, COLLECT_LINE_1_LOW.position.y + ((FORWARD_DELTA_TO_COLLECT_BALL_STEP - 3) * multiplier), artifactHeading);
    public Pose2d COLLECT_LINE_1_END = new Pose2d(COLLECT_LINE_1_MID.position.x, COLLECT_LINE_1_MID.position.y + ((FORWARD_DELTA_TO_COLLECT_BALL_STEP - 3) * multiplier), artifactHeading);

    public Pose2d MOVE_OFF_LINE = new Pose2d(53, 36  * multiplier, obeliskHeading);


    @Override
    public void runOpMode() throws InterruptedException {

        robotHardware = new RobotHardware(this.hardwareMap);

        if (multiplier == 1) {
            CrossOpModeStorage.allianceColor = AllianceColors.RED;
        }
        else {
            CrossOpModeStorage.allianceColor = AllianceColors.BLUE;
        }

        this.limelightAprilTagHelper = new LimelightAprilTagHelper(robotHardware);

        this.spindex = new Spindex(robotHardware);
        this.spindex.initializeWithPPG();
        this.intakeSystem = new IntakeSystem(robotHardware, this.spindex);
        this.launchSystem = new LaunchSystem(robotHardware, this.spindex, this.limelightAprilTagHelper);

        mecanumDrive = new MecanumDrive(this.hardwareMap, INIT_POS);

        robotHardware.startLimelight();
        robotHardware.setLimelightPipeline(6);

        Action pickLine1AfterLaunch = mecanumDrive.actionBuilder(INIT_POS)
                .setTangent(splineHeading)
                .splineToLinearHeading(COLLECT_LINE_1_START, COLLECT_LINE_1_START.heading)
                .build();

        Action moveForwardToPickBall1 = mecanumDrive.actionBuilder(COLLECT_LINE_1_START)
                .setTangent(artifactHeading)
                .lineToY(COLLECT_LINE_1_LOW.position.y, new TranslationalVelConstraint(20), new ProfileAccelConstraint(-10, 10))
                .build();

        Action moveForwardToPickBall2 = mecanumDrive.actionBuilder(COLLECT_LINE_1_LOW)
                .setTangent(artifactHeading)
                .lineToY(COLLECT_LINE_1_MID.position.y, new TranslationalVelConstraint(20), new ProfileAccelConstraint(-10, 10))
                .build();

        Action moveForwardToPickBall3 = mecanumDrive.actionBuilder(COLLECT_LINE_1_MID)
                .lineToY(COLLECT_LINE_1_END.position.y, new TranslationalVelConstraint(20), new ProfileAccelConstraint(-10, 10))
                .build();

        Action launchAfterPickingLine1 = mecanumDrive.actionBuilder(COLLECT_LINE_1_END)
                .strafeToLinearHeading(INIT_POS.position, INIT_POS.heading, new TranslationalVelConstraint(50), new ProfileAccelConstraint(-50, 50))
                .build();

        Action moveAwayFromLine = mecanumDrive.actionBuilder(INIT_POS)
                .strafeToLinearHeading(MOVE_OFF_LINE.position, MOVE_OFF_LINE.heading)
                .build();

        while (opModeInInit()) {
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robotHardware.setLaunchTurretPosition(TURRET_SERVO_CENTERED);

        while (opModeIsActive() && !isStopRequested()) {
            ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

            GamePattern pattern = launchSystem.readGamePattern();

            runBlockingWithBackground(
                    new ParallelAction(
                            new InstantAction(() -> robotHardware.setLaunchTurretPosition(0.425)),
                            new LaunchFlywheelAction(robotHardware, FLYWHEEL_FULL_TICKS_PER_SEC * FLYWHEEL_POWER_COEFFICIENT_FAR),
                            spindex.moveToNextPurpleSlotAction()
                    )
            );

            runBlockingWithBackground(
                    launchSystem.getBallPatternLaunchAction(pattern)
            );

            runBlockingWithBackground(
                    new SequentialAction(
                            new ParallelAction(
                                    pickLine1AfterLaunch,
                                    intakeSystem.getTurnOnAction(false),
                                    new SpindexAction(robotHardware, spindex.storedColors.get(0).intakePosition)
                            ),
                            moveForwardToPickBall1,
                            new InstantAction(() -> {
                                ElapsedTime intakeTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
                                while (intakeTimer.milliseconds() < 500 && spindex.storedColors.get(0).ballColor == GameColors.NONE) {
                                    if (robotHardware.didBallDetectionBeamBreak()) {
                                        spindex.storedColors.get(0).ballColor = GameColors.UNKNOWN;
                                        Log.i("BLUE FAR AUTO", "RE INDEXED INDEX 0 AS UNKNOWN");
                                    } else {
                                        spindex.storedColors.get(0).ballColor = GameColors.NONE;
                                        Log.i("BLUE FAR AUTO", "RE INDEXED INDEX 0 AS NONE");
                                    }
                                }
                            }),
                            new SpindexAction(robotHardware, spindex.storedColors.get(1).intakePosition),
                            new SleepAction(0.5)
                    )
            );

            //Need an action to index balls
            runBlockingWithBackground(
                    new SequentialAction(
                            moveForwardToPickBall2,
                            new InstantAction(() -> {
                                ElapsedTime intakeTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
                                while (intakeTimer.milliseconds() < 500 && spindex.storedColors.get(1).ballColor == GameColors.NONE) {
                                    if (robotHardware.didBallDetectionBeamBreak()) {
                                        spindex.storedColors.get(1).ballColor = GameColors.UNKNOWN;
                                        Log.i("BLUE FAR AUTO", "RE INDEXED INDEX 1 AS UNKNOWN");
                                    } else {
                                        spindex.storedColors.get(1).ballColor = GameColors.NONE;
                                        Log.i("BLUE FAR AUTO", "RE INDEXED INDEX 1 AS NONE");
                                    }
                                    if (spindex.storedColors.get(0).ballColor == GameColors.UNKNOWN) {
                                        GameColors color = robotHardware.getDetectedBallColorFromLeftSensor();
                                        spindex.storedColors.get(0).ballColor = color;
                                        Log.i("BLUE FAR AUTO", "RE INDEXED INDEX 0 COLOR AS: " + color);
                                    }
                                }
                            }),
                            new SpindexAction(robotHardware, spindex.storedColors.get(2).intakePosition),
                            new SleepAction(0.75)
                    )
            );


            runBlockingWithBackground(
                    new SequentialAction(
                            moveForwardToPickBall3,
                            new InstantAction(() -> {
                                ElapsedTime intakeTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
                                while (intakeTimer.milliseconds() < 500 && spindex.storedColors.get(2).ballColor == GameColors.NONE) {
                                    if (robotHardware.didBallDetectionBeamBreak()) {
                                        spindex.storedColors.get(2).ballColor = GameColors.UNKNOWN;
                                        Log.i("BLUE FAR AUTO", "RE INDEXED INDEX 2 AS UNKNOWN");
                                    } else {
                                        spindex.storedColors.get(2).ballColor = GameColors.NONE;
                                        Log.i("BLUE FAR AUTO", "RE INDEXED INDEX 2 AS NONE");
                                    }
                                    if (spindex.storedColors.get(1).ballColor == GameColors.UNKNOWN) {
                                        GameColors color = robotHardware.getDetectedBallColorFromLeftSensor();
                                        spindex.storedColors.get(1).ballColor = color;
                                        Log.i("BLUE FAR AUTO", "RE INDEXED INDEX 1 COLOR AS: " + color);
                                    }
                                }
                            })
                    )
            );

            runBlockingWithBackground(
                    new ParallelAction(
                            intakeSystem.getReverseIntakeAction(),
                            launchAfterPickingLine1,
                            new SequentialAction(
                                    new SpindexAction(robotHardware, spindex.storedColors.get(0).intakePosition),
                                    new SleepAction(0.25),
                                    new InstantAction(() -> {
                                        if (spindex.storedColors.get(2).ballColor == GameColors.UNKNOWN) {
                                            GameColors color = robotHardware.getDetectedBallColorFromLeftSensor();
                                            spindex.storedColors.get(2).ballColor = color;
                                            Log.i("BLUE FAR AUTO", "RE INDEXED INDEX 2 COLOR AS: " + color);
                                        }
                                    })
                            )
                    )
            );

            runBlockingWithBackground(
                    launchSystem.getBallPatternLaunchAction(pattern)
            );

            runBlockingWithBackground(
                    new ParallelAction(
                            moveAwayFromLine,
                            new ResetSpindexerAction(robotHardware)

                    )
            );

            Log.i("BLUE FAR AUTO", "Elapsed time: " + timer.seconds());

            break;
        }
    }
}
