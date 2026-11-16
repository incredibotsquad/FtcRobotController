package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PowerMonitorSubsystem;

/**
 * Current draw of the drivetrain's pose hold, with nothing else running.
 *
 * Only the drive (Pedro Follower) and PowerMonitorSubsystem are created: no intake, transfer,
 * flywheel, turret, gate, intake bar or Limelight. At START the robot holds the pose it is at,
 * using the same Pedro hold as the TeleOp launch hold. Push it by hand (or with another robot) to
 * see how much current the drive motors draw fighting back.
 *
 * Gamepad 1:
 *   A - hold the pose the robot is at now (re-anchor)
 *   B - stop holding (drive motors off), to compare against idle
 *
 * Currents and hub voltages are logged with tag "PowerLog" (see PowerMonitorSubsystem); the last
 * line is also shown on the Driver Station. Pit/test tool: add @Disabled before competition.
 */
@TeleOp(name = "Test: Hold Pose Current", group = "Test")
public class HoldPoseCurrentTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        DriveSubsystem drive = new DriveSubsystem(hardwareMap, panels);
        PowerMonitorSubsystem power = new PowerMonitorSubsystem(hardwareMap);

        telemetry.addLine("Hold Pose Current test: only the drive and the power logger run.");
        telemetry.addLine("START = hold this pose. A = re-anchor hold, B = release (motors off).");
        telemetry.addLine("Push the robot and watch the currents (logcat tag PowerLog).");
        telemetry.update();
        waitForStart();

        Pose holdPose = drive.getPose();
        drive.getFollower().hold(holdPose);
        boolean holding = true;

        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                holdPose = drive.getPose();
                drive.getFollower().hold(holdPose);
                holding = true;
            }
            if (gamepad1.bWasPressed()) {
                drive.getFollower().manual(0, 0, 0);
                holding = false;
            }

            drive.periodic(); // follower.update(): reads the Pinpoint and drives the hold
            power.periodic();

            Pose pose = drive.getPose();
            double offIn = Math.hypot(pose.x() - holdPose.x(), pose.y() - holdPose.y());
            double d = pose.heading() - holdPose.heading();
            double offDeg = Math.abs(Math.toDegrees(Math.atan2(Math.sin(d), Math.cos(d))));

            telemetry.addData("Mode", holding ? "HOLDING (B = release)" : "RELEASED (A = hold here)");
            telemetry.addData("Pushed off hold by", String.format("%.1f in, %.1f deg", offIn, offDeg));
            telemetry.addData("Power", power.getLastLine());
            telemetry.update();
        }
        drive.getFollower().manual(0, 0, 0);
    }
}
