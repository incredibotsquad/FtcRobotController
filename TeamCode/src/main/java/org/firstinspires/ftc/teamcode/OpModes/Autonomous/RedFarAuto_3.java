package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import static org.firstinspires.ftc.teamcode.Actions.LaunchFlywheelAction.FLYWHEEL_FULL_TICKS_PER_SEC;
import static org.firstinspires.ftc.teamcode.subsystems.LaunchSystem.FLYWHEEL_POWER_COEFFICIENT_FAR;
import static org.firstinspires.ftc.teamcode.subsystems.LaunchSystem.TURRET_SERVO_CENTERED;

import android.util.Log;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Actions.LaunchFlywheelAction;
import org.firstinspires.ftc.teamcode.Actions.ResetSpindexerAction;
import org.firstinspires.ftc.teamcode.common.AllianceColors;
import org.firstinspires.ftc.teamcode.common.GamePattern;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.common.LimelightAprilTagHelper;
import org.firstinspires.ftc.teamcode.common.CrossOpModeStorage;
import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchSystem;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;


@Autonomous(name = "Red_Far_Auto_3", group = "Autonomous")
public class RedFarAuto_3 extends BaseAuto {

    private static final int multiplier = 1;    //used to flip coordinates between red (1), Blue (-1)

    public double robotHeading = Math.toRadians(180 * multiplier);
    public double artifactHeading = Math.toRadians(90 * multiplier);

    public double obeliskHeading = Math.toRadians(180); //this is same for both sides - do NOT need a multiplier

    public Pose2d INIT_POS = new Pose2d(55, 16 * multiplier, robotHeading);

    public Pose2d LAUNCH_POS = new Pose2d(45, 16 * multiplier, robotHeading);

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

        Action moveToLaunchPosition = mecanumDrive.actionBuilder(INIT_POS)
                .lineToX(LAUNCH_POS.position.x)
                .build();

        Action moveAwayFromLine = mecanumDrive.actionBuilder(LAUNCH_POS)
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
                            moveToLaunchPosition,
                            new LaunchFlywheelAction(robotHardware, FLYWHEEL_FULL_TICKS_PER_SEC * FLYWHEEL_POWER_COEFFICIENT_FAR, false),
                            intakeSystem.updateBallColorsAction()
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

            Log.i("RED FAR AUTO", "Elapsed time: " + timer.seconds());

            break;
        }
    }
}
