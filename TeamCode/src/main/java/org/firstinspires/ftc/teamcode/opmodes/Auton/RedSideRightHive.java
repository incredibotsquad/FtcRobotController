package org.firstinspires.ftc.teamcode.opmodes.Auton;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.Incredibot;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.commands.LaunchBallsCommand;
import org.firstinspires.ftc.teamcode.common.AllianceColors;
import org.firstinspires.ftc.teamcode.common.CrossOpModeStorage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Configurable
@Autonomous
public class RedSideRightHive extends CommandOpMode {
    /** Longest wait for the flywheel and turret before launching anyway. */
    public static long LAUNCH_READY_TIMEOUT_MS = 1500;
    /** Wait after lowering the intake bar, before the final push into the FLOWER. */
    public static long INTAKE_BAR_LOWER_MS = 300;
    /** Longest wait at the GARDEN / a FLOWER for the robot to fill up. */
    public static long INTAKE_DURATION_MS = 1500;

    private final PathConstants paths = new PathConstants(AllianceColors.RED);
    private Incredibot robot;
    private Follower follower;
    private TelemetryManager panels;
    private boolean isScheduled = false;
    @Override
    public void initialize() {
        // Full robot so the launcher can shoot. The Follower is DriveSubsystem's, which updates it.
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        robot = new Incredibot(hardwareMap, panels);
        robot.initialize(Incredibot.OpModeType.AUTO, null, null);
        follower = robot.driveSubsystem.getFollower();

        CrossOpModeStorage.allianceColor = AllianceColors.RED; // LaunchReadinessCommand aims using this
        robot.odometrySubsystem.resetPose(
                paths.startingPoseRightHive.x(),
                paths.startingPoseRightHive.y(),
                Math.toDegrees(paths.startingPoseRightHive.heading()));
    }
    @Override
    public void run(){
        if(!isScheduled){
            schedule(getAutoRoutine());
            isScheduled = true;
        }
        super.run(); // also runs follower.update() through DriveSubsystem.periodic()
        panels.update(telemetry);
    }
    public Command getAutoRoutine() {
        return new SequentialCommandGroup(
                // Gate servo can't be commanded during INIT, so close it at START.
                new InstantCommand(robot.launchGateSubsystem::closeGate),

                // 1. Shoot the preloads from the start
                new ParallelRaceGroup(
                        new WaitUntilCommand(robot.launchSubsystem::isReadyToLaunch),
                        new WaitCommand(LAUNCH_READY_TIMEOUT_MS)
                ),
                new LaunchBallsCommand(robot.launchGateSubsystem, robot.intakeSubsystem, robot.driveSubsystem),

                // 2. Intake the garden
                new FollowPathCommand(follower, paths.intakeGarden(), true),
                new ParallelRaceGroup(
                        new WaitUntilCommand(robot.intakeSubsystem::isFull),
                        new WaitCommand(INTAKE_DURATION_MS)
                ),

                // 3. Shoot the garden pickup on the left hive
                new FollowPathCommand(follower, paths.moveToLeftHive(), true),
                new ParallelRaceGroup(
                        new WaitUntilCommand(robot.launchSubsystem::isReadyToLaunch),
                        new WaitCommand(LAUNCH_READY_TIMEOUT_MS)
                ),
                new LaunchBallsCommand(robot.launchGateSubsystem, robot.intakeSubsystem, robot.driveSubsystem),

                // 4. Intake the far flower
                new FollowPathCommand(follower, paths.intakeFarFlowerEarly(), true),
                new InstantCommand(robot.intakeBarSubsystem::lower, robot.intakeBarSubsystem),
                new WaitCommand(INTAKE_BAR_LOWER_MS),
                new FollowPathCommand(follower, paths.intakeFarFlower(), true),
                new ParallelRaceGroup(
                        new WaitUntilCommand(robot.intakeSubsystem::isFull),
                        new WaitCommand(INTAKE_DURATION_MS)
                ),
                new InstantCommand(robot.intakeBarSubsystem::raise, robot.intakeBarSubsystem),

                // 5. Shoot what was picked up at the far flower
                new ParallelRaceGroup(
                        new WaitUntilCommand(robot.launchSubsystem::isReadyToLaunch),
                        new WaitCommand(LAUNCH_READY_TIMEOUT_MS)
                ),
                new LaunchBallsCommand(robot.launchGateSubsystem, robot.intakeSubsystem, robot.driveSubsystem),

                // 6. Intake the close flower
                new FollowPathCommand(follower, paths.intakeCloseFlowerEarly(), true),
                new InstantCommand(robot.intakeBarSubsystem::lower, robot.intakeBarSubsystem),
                new WaitCommand(INTAKE_BAR_LOWER_MS),
                new FollowPathCommand(follower, paths.intakeCloseFlower(), true),
                new ParallelRaceGroup(
                        new WaitUntilCommand(robot.intakeSubsystem::isFull),
                        new WaitCommand(INTAKE_DURATION_MS)
                ),
                new InstantCommand(robot.intakeBarSubsystem::raise, robot.intakeBarSubsystem),

                // 7. Shoot the close flower pickup on the right hive
                new FollowPathCommand(follower, paths.moveToShootRightHive(), true),
                new ParallelRaceGroup(
                        new WaitUntilCommand(robot.launchSubsystem::isReadyToLaunch),
                        new WaitCommand(LAUNCH_READY_TIMEOUT_MS)
                ),
                new LaunchBallsCommand(robot.launchGateSubsystem, robot.intakeSubsystem, robot.driveSubsystem),

                // 8. Park
                new FollowPathCommand(follower, paths.parkRightSide(), true)
        );
    }
}
