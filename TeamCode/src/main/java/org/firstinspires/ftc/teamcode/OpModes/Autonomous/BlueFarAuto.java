package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.AllianceColors;
import org.firstinspires.ftc.teamcode.common.GamePattern;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchSystem;
import org.firstinspires.ftc.teamcode.subsystems.LightSystem;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;


@Autonomous(name = "Blue_Far_Auto", group = "Autonomous")
public class BlueFarAuto extends LinearOpMode {
    public RobotHardware robotHardware;
    private Spindex spindex;
    private IntakeSystem intakeSystem;
    private LaunchSystem launchSystem;
    private LightSystem lightSystem;
    public MecanumDrive mecanumDrive;

    private static final int multiplier = -1;    //used to flip coordinates between red (1), Blue (-1)

    public double robotHeading = Math.toRadians(180 * multiplier);
    public double goalHeading = Math.toRadians(180 - (22 * multiplier));

    public double obeliskHeading = Math.toRadians(180); //this is same for both sides - do NOT need a multiplier

    public Pose2d INIT_POS = new Pose2d(58, 8 * multiplier, robotHeading);
    public Pose2d LAUNCH_BALLS = new Pose2d(48, 12 * multiplier, goalHeading);
    public Pose2d MOVE_OFF_LINE = new Pose2d(36, 24  * multiplier, obeliskHeading);


    @Override
    public void runOpMode() throws InterruptedException {

        robotHardware = new RobotHardware(this.hardwareMap);

        this.spindex = new Spindex(robotHardware);
        this.spindex.initializeWithPPG();
        this.lightSystem = new LightSystem(robotHardware);
        this.intakeSystem = new IntakeSystem(robotHardware, this.spindex);
        this.launchSystem = new LaunchSystem(robotHardware, this.spindex, lightSystem);
        this.launchSystem.setAllianceColor(AllianceColors.BLUE);

        mecanumDrive = new MecanumDrive(this.hardwareMap, INIT_POS);

        robotHardware.startLimelight();
        robotHardware.setLimelightPipeline(6);

        Action moveTolaunchBalls =mecanumDrive.actionBuilder(INIT_POS)
                .strafeToLinearHeading(LAUNCH_BALLS.position, LAUNCH_BALLS.heading)
                .build();

        Action moveAwayFromLine = mecanumDrive.actionBuilder(LAUNCH_BALLS)
                .strafeToLinearHeading(MOVE_OFF_LINE.position, MOVE_OFF_LINE.heading)
                .build();

        while (opModeInInit()) {
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

            GamePattern pattern = launchSystem.readGamePattern();

            Actions.runBlocking(
                    new ParallelAction(
                            moveTolaunchBalls,
                            launchSystem.getKeepWarmAction()
                    )
            );

            Actions.runBlocking(launchSystem.getBallPatternLaunchAction(pattern));

            Actions.runBlocking(moveAwayFromLine);

            break;
        }
    }
}
