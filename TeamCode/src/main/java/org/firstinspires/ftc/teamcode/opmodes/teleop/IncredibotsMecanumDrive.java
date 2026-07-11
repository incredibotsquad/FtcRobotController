package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; // Changed this
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Incredibot;
import org.firstinspires.ftc.teamcode.commands.RelocalizeCommand;
import org.firstinspires.ftc.teamcode.common.AllianceColors;
import org.firstinspires.ftc.teamcode.common.CrossOpModeStorage;

@TeleOp(name = "IncredibotsMecanumDrive", group = "TeleOp")
public class IncredibotsMecanumDrive extends LinearOpMode { // Changed this
    private Incredibot incredibot;
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;

    @Override
    public void runOpMode() throws InterruptedException {
        // 1. INITIALIZATION PHASE (Everything before the while loop)
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        incredibot = new Incredibot(
                hardwareMap,
                Incredibot.OpModeType.TELEOP,
                driverGamepad,
                operatorGamepad,
                PanelsTelemetry.INSTANCE.getTelemetry());

        incredibot.setAlliance(CrossOpModeStorage.allianceColor == AllianceColors.RED);

        // Schedule the RelocalizeCommand
        // In LinearOpMode, we schedule it once here
        RelocalizeCommand relocalizeCommand = new RelocalizeCommand(
                incredibot.limelightSubsystem,
                incredibot.odometrySubsystem,
                incredibot.driveSubsystem
        );
        relocalizeCommand.schedule();

        incredibot.odometrySubsystem.resetPose(CrossOpModeStorage.currentPose.getX(), CrossOpModeStorage.currentPose.getY(), Math.toDegrees(CrossOpModeStorage.currentPose.getHeading()));

        // 2. THE INIT-LOOP (Runs after you hit INIT, but before you hit START)
        while (opModeInInit()) {
            // Manually run the scheduler so the RelocalizeCommand can sample tags
            CommandScheduler.getInstance().run();

            telemetry.addData("Status", "READY - Relocalizing...");
            telemetry.addData("Current Pose", incredibot.odometrySubsystem.getPose().toString());

            // Update your custom telemetry panels
            PanelsTelemetry.INSTANCE.getTelemetry().update(telemetry);
        }

        // 3. START PHASE (Runs once when you hit the START button)
        waitForStart();

        // 4. MAIN TELEOP LOOP
        while (opModeIsActive() && !isStopRequested()) {
            // The scheduler runs all active commands (including Relocalize and Drive)
            CommandScheduler.getInstance().run();

            // Update your custom telemetry panels
            PanelsTelemetry.INSTANCE.getTelemetry().update(telemetry);
        }

        // 5. CLEANUP
        CommandScheduler.getInstance().reset();
    }
}