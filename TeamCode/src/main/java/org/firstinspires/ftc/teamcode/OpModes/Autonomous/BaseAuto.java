package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.common.LimelightAprilTagHelper;
import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchSystem;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;

@Config
public abstract class BaseAuto extends LinearOpMode {
    public RobotHardware robotHardware;
    public Spindex spindex;
    public LimelightAprilTagHelper limelightAprilTagHelper;
    public IntakeSystem intakeSystem;
    public LaunchSystem launchSystem;
    public MecanumDrive mecanumDrive;

    public static void runBlockingWithBackground(Action mainAction, Runnable backgroundTask) {
        boolean running = true;
        while (running && !Thread.currentThread().isInterrupted()) {
            // Run a single step of the main action
            TelemetryPacket packet = new TelemetryPacket();
            running = mainAction.run(packet);

            // Run the background task's periodic logic
            backgroundTask.run(); // This should be non-blocking and quick

            // Optional: Send telemetry packet to dashboard
             FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}