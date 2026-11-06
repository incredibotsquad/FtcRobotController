package org.firstinspires.ftc.teamcode.opmodes.teleop;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; // Changed this
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Incredibot;
import org.firstinspires.ftc.teamcode.commands.RelocalizeWithLimelightCommand;
import org.firstinspires.ftc.teamcode.common.CrossOpModeStorage;

@TeleOp(name = "IncredibotsMecanumDrive", group = "TeleOp")
public class IncredibotsMecanumDrive extends LinearOpMode { // Changed this
    private Incredibot incredibot;
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;

    // ---- turret debugging (logcat tag TurretDebug); remove once the INIT turn is understood
    public static final String TURRET_LOG_TAG = "TurretDebug";
    /** Counts TeleOp runs since the Robot Controller app started (statics survive between OpModes). */
    private static int runCount = 0;
    private Servo turretServoForLog;
    private String lastTurretHwState = "";

    @Override
    public void runOpMode() throws InterruptedException {
        // 1. INITIALIZATION PHASE (Everything before the while loop)
        runCount++;
        turretServoForLog = hardwareMap.tryGet(Servo.class, "turretServo");
        logTurret("INIT begin (before anything is built)");

        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        incredibot = new Incredibot(
                hardwareMap,
                PanelsTelemetry.INSTANCE.getTelemetry());

        // Schedule the RelocalizeCommand
        // In LinearOpMode, we schedule it once here
        logTurret("INIT robot built");

        RelocalizeWithLimelightCommand relocalizeWithLimelightCommand = new RelocalizeWithLimelightCommand(
                incredibot.limelightSubsystem,
                incredibot.odometrySubsystem,
                incredibot.driveSubsystem
        );



        // Power the intake bar servos at RAISED so the bar can't drop under gravity. The bar starts
        // physically raised, so this should not move it. G403: only re-holding a position is allowed
        // in INIT. If this makes the bar move or twitch, fix IntakeBarSubsystem's RAISED positions,
        // and until then wait for TELEOP to begin before pressing INIT.
        incredibot.intakeBarSubsystem.raise();
        logTurret("INIT after intake bar raise()");

        Log.i("IncredibotsMecanumDrive. Crossopmode pose: ", CrossOpModeStorage.currentPose.toString());

        //use cross opmode storage to ensure odometry knows where we started from.
        //odometry subsystem periodically updates this in cross oomode storage.
        incredibot.odometrySubsystem.resetPose(CrossOpModeStorage.currentPose.getX(), CrossOpModeStorage.currentPose.getY(), Math.toDegrees(CrossOpModeStorage.currentPose.getHeading()));
        logTurret("INIT after pose restore");

        //relocalize based on april tags - this only schedules it once (skipped in drive-only mode)
        if (!Incredibot.DRIVE_ONLY_MODE) {
            relocalizeWithLimelightCommand.schedule();
        }

        // 2. THE INIT-LOOP (Runs after you hit INIT, but before you hit START)
        while (opModeInInit()) {
            // Manually run the scheduler so the RelocalizeCommand can sample tags
            CommandScheduler.getInstance().run();
            logTurretIfHardwareChanged("INIT loop");

//            telemetry.addData("Status", "READY - Relocalizing...");
//            telemetry.addData("Current Pose", incredibot.odometrySubsystem.getPose().toString());

            // Update your custom telemetry panels
            PanelsTelemetry.INSTANCE.getTelemetry().update(telemetry);
        }

        // 3. START PHASE (Runs once when you hit the START button)
        waitForStart();
        logTurret("START before turret restore");

        // Restore the turret only after START: commanding the servo during INIT can move it
        // (G304.H / G403), e.g. if it hadn't reached its last commanded position when AUTO ended.
        if (!Incredibot.DRIVE_ONLY_MODE) {
            incredibot.launchSubsystem.setTurretPosition(CrossOpModeStorage.turretPosition);
        }
        logTurret("START after turret restore");

        //this is done later to avoid all the other commands from running.
        incredibot.initialize(Incredibot.OpModeType.TELEOP, driverGamepad, operatorGamepad);

        // 4. MAIN TELEOP LOOP
        while (opModeIsActive() && !isStopRequested()) {
            // The scheduler runs all active commands (including Relocalize and Drive)
            CommandScheduler.getInstance().run();

            // Update your custom telemetry panels
            PanelsTelemetry.INSTANCE.getTelemetry().update(telemetry);
        }

        logTurret("STOP");

        // 5. CLEANUP
        CommandScheduler.getInstance().reset();
    }

    // ------------------------------------------------------------------ turret debug logging

    /** Turret servo as the hub sees it: last commanded position and whether its PWM is on. */
    private String turretHwState() {
        if (turretServoForLog == null) return "turretServo NOT CONFIGURED";
        String pwm = turretServoForLog instanceof PwmControl
                ? (((PwmControl) turretServoForLog).isPwmEnabled() ? "ON" : "off")
                : "?";
        return String.format("turretCmd %.3f pwm %s", turretServoForLog.getPosition(), pwm);
    }

    private void logTurret(String event) {
        lastTurretHwState = turretHwState();
        String pose = incredibot == null ? "-" : incredibot.odometrySubsystem.getPose().toString();
        String lock = incredibot == null ? "-" : String.valueOf(incredibot.launchSubsystem.isLaunchReadinessLocked());
        Log.i(TURRET_LOG_TAG, String.format("run %d | %s | %s | storedTurret %.3f | storedPose %s | odoPose %s | lock %s | driveOnly %b | alliance %s",
                runCount, event, lastTurretHwState, CrossOpModeStorage.turretPosition,
                CrossOpModeStorage.currentPose, pose, lock, Incredibot.DRIVE_ONLY_MODE, CrossOpModeStorage.allianceColor));
    }

    /** Logs only when the turret's commanded position or PWM state changes (keeps the INIT loop quiet). */
    private void logTurretIfHardwareChanged(String event) {
        if (!turretHwState().equals(lastTurretHwState)) logTurret(event + ": turret hardware state CHANGED");
    }
}
