package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import static org.firstinspires.ftc.teamcode.Actions.LaunchFlywheelAction.FLYWHEEL_FULL_TICKS_PER_SEC;
import static org.firstinspires.ftc.teamcode.subsystems.LaunchSystem.FLYWHEEL_POWER_COEFFICIENT_FAR;
import static org.firstinspires.ftc.teamcode.subsystems.LaunchSystem.TURRET_CENTERED_POSITION;

import android.util.Log;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Actions.LaunchFlywheelAction;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.common.AllianceColors;
import org.firstinspires.ftc.teamcode.common.CrossOpModeStorage;
import org.firstinspires.ftc.teamcode.common.GamePattern;
import org.firstinspires.ftc.teamcode.common.LimelightAprilTagHelper;
import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchSystem;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;


@Disabled
@Autonomous(name = "BasicMoveAuto", group = "Other")
public class BasicMoveAuto extends BaseAuto {
    private static final int multiplier = -1;    //used to flip coordinates between red (1), Blue (-1)

    public Pose2d INIT_POS = CrossOpModeStorage.currentPose;


    @Override
    public void runOpMode() throws InterruptedException {

        robotHardware = new RobotHardware(this.hardwareMap);

        mecanumDrive = new MecanumDrive(this.hardwareMap, INIT_POS);

        Action moveForward = mecanumDrive.actionBuilder(INIT_POS)
                .lineToX(INIT_POS.position.x + 2)
                .build();

        Action moveBack = mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose())
                .lineToX(INIT_POS.position.x)
                .build();

        while (opModeInInit()) {
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        while (opModeIsActive() && !isStopRequested()) {
            ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

            Actions.runBlocking(
                    moveForward
            );

            Actions.runBlocking(
                    moveBack
            );

            Log.i("Basic move auto", "Elapsed time: " + timer.seconds());
            break;
        }
    }
}
