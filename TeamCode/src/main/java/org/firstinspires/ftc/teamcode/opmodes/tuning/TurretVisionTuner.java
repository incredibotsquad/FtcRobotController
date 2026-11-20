package org.firstinspires.ftc.teamcode.opmodes.tuning;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Incredibot;
import org.firstinspires.ftc.teamcode.commands.LaunchReadinessCommand;
import org.firstinspires.ftc.teamcode.commands.TurretVisionCorrection;
import org.firstinspires.ftc.teamcode.common.CrossOpModeStorage;
import org.firstinspires.ftc.teamcode.common.LaunchTargets;
import org.firstinspires.ftc.teamcode.subsystems.LaunchSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Turret aim tuner with the Limelight correction running, as in TeleOp. Runs the real TeleOp
 * (same Incredibot, LaunchReadinessCommand aims, TurretVisionCorrection nudges the aim from the
 * HIVE tags), so what you measure is what matches get.
 *
 * Drive (gamepad 1 sticks) to a spot, stop, let the aim settle, then correct it by eye with the
 * dpad until the turret points at the CELL center, and press A to record the spot. Repeat from
 * spots at different angles and distances (turret left, center, right; near and far), then press Y:
 * it fits the trims you needed and logs the turret constants to change.
 *
 * Gamepad 1 (gamepad 2 is the normal operator: RB launches, START locks the turret):
 *   Sticks        drive
 *   Dpad L / R    turn the aim left / right by the step (logged with everything the Limelight saw)
 *   Dpad up/down  step size (0.25 / 0.5 / 1 / 2 deg)
 *   A             aim is right here: record this spot
 *   X             undo the last recorded spot
 *   B             trim back to 0
 *   Y             fit all recorded spots and log the recommended constants
 * Moving away from a spot after adjusting without pressing A records it anyway ("auto").
 *
 * Everything goes to /sdcard/FIRST/data/turret_vision_tuning.csv (one row per event) and logcat
 * (tag TurretVisionTuner). The fit: needed trim = a + b * turret angle. a moves
 * LaunchReadinessCommand.TURRET_FORWARD_POSITION, b scales TURRET_SCALING_FACTOR. It also compares
 * the trims with what plain odometry would have needed, to show whether the Limelight correction
 * helps or hurts.
 */
@Configurable
@TeleOp(name = "TurretVisionTuner", group = "Tests")
public class TurretVisionTuner extends LinearOpMode {

    private static final String TAG = "TurretVisionTuner";

    /** Start pose set by hand (red frame), as in FlywheelTuner; false = keep the stored pose. */
    public static boolean USE_START_POSE = false;
    public static double START_X = 41.5;
    public static double START_Y = 8;
    public static double START_HEADING_DEG = 90;
    /** Leaving a spot faster than this after adjusting records it as an "auto" sample. */
    public static double LEAVE_SPEED_IN_PER_SEC = 3.0;
    /** Fit the scale term only when recorded angles span at least this much. */
    public static double MIN_ANGLE_SPAN_FOR_SCALE_DEG = 20;

    private static final double[] STEPS = {0.25, 0.5, 1.0, 2.0};
    private static final String HEADER = "session,t_ms,event,x,y,heading_deg,speed_ips,alliance,cell,target_x,target_y,"
            + "aim_from_x,aim_from_y,vision_weight,vision_dx,vision_dy,vision_reason,tags_seen,tags,est_x,est_y,"
            + "est_spread_in,est_tags,distance_in,base_angle_deg,trim_deg,aim_angle_deg,odometry_angle_deg,"
            + "turret_pos,locked,battery_v,note";

    /** One recorded spot. */
    private static final class Sample {
        double baseDeg, trimDeg, odoDeg, distance, weight;
        String kind;
    }

    private final List<Sample> samples = new ArrayList<>();
    private int stepIndex = 1;
    private double trimDeg = 0;
    private boolean adjustedSinceSample = false;
    private String pendingRow = null;
    private Sample pendingSample = null;
    /** Dpad change waiting to be logged on the next loop (after the aim is recomputed); NaN = none. */
    private double pendingDeltaDeg = Double.NaN;
    private String lastFit = "no fit yet (record spots with A, then press Y)";
    private final String session = String.format(Locale.US, "%tY%<tm%<td-%<tH%<tM%<tS", System.currentTimeMillis());
    private File csv;

    private Incredibot robot;
    private LaunchReadinessCommand readiness;

    @Override
    public void runOpMode() {
        TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx operator = new GamepadEx(gamepad2);
        robot = new Incredibot(hardwareMap, panels);

        File dir = AppUtil.ROBOT_DATA_DIR;
        //noinspection ResultOfMethodCallIgnored
        dir.mkdirs();
        csv = new File(dir, "turret_vision_tuning.csv");

        robot.intakeBarSubsystem.raise();
        if (USE_START_POSE) {
            robot.odometrySubsystem.resetPose(START_X, START_Y, START_HEADING_DEG);
        } else {
            robot.odometrySubsystem.resetPose(CrossOpModeStorage.currentPose.getX(), CrossOpModeStorage.currentPose.getY(),
                    Math.toDegrees(CrossOpModeStorage.currentPose.getHeading()));
        }
        while (opModeInInit()) {
            panels.addLine("TurretVisionTuner: runs the real TeleOp with the Limelight correction.");
            panels.addData("Start pose", CrossOpModeStorage.currentPose);
            panels.addData("Limelight correction", TurretVisionCorrection.ENABLED ? "ON" : "OFF (turn on for this test)");
            panels.addData("Log file", csv.getAbsolutePath());
            panels.update(telemetry);
        }
        if (isStopRequested()) return;

        robot.launchSubsystem.setTurretPosition(CrossOpModeStorage.turretPosition);
        robot.initialize(Incredibot.OpModeType.TELEOP, driver, operator);
        readiness = robot.launchReadinessCommand;
        readiness.setAimTrimDeg(0);
        Log.i(TAG, "session " + session + " started, Limelight correction " + (TurretVisionCorrection.ENABLED ? "on" : "off"));

        try {
            while (opModeIsActive()) {
                CommandScheduler.getInstance().run();
                if (!Double.isNaN(pendingDeltaDeg)) logAdjust(); // the aim now includes the new trim
                handleButtons();
                showTelemetry(panels);
            }
        } finally {
            if (readiness != null) readiness.setAimTrimDeg(0);
            CommandScheduler.getInstance().reset();
        }
    }

    private void handleButtons() {
        double speed = robot.odometrySubsystem.getFieldSpeedInchesPerSecond();
        if (gamepad1.dpadUpWasPressed()) stepIndex = Math.min(STEPS.length - 1, stepIndex + 1);
        if (gamepad1.dpadDownWasPressed()) stepIndex = Math.max(0, stepIndex - 1);
        if (gamepad1.dpadLeftWasPressed()) adjust(+STEPS[stepIndex]);   // left = counter-clockwise = +
        if (gamepad1.dpadRightWasPressed()) adjust(-STEPS[stepIndex]);
        if (gamepad1.bWasPressed()) {
            trimDeg = 0;
            readiness.setAimTrimDeg(0);
            writeRow("reset_trim", null, "");
        }
        if (gamepad1.aWasPressed()) record("confirm");
        if (gamepad1.xWasPressed() && !samples.isEmpty()) {
            Sample s = samples.remove(samples.size() - 1);
            writeRow("undo", null, String.format(Locale.US, "removed %s sample at %.1f deg trim %.2f", s.kind, s.baseDeg, s.trimDeg));
        }
        if (gamepad1.yWasPressed()) fit();
        // Adjusted, then drove off without A: keep the last adjusted state as a sample.
        if (adjustedSinceSample && speed > LEAVE_SPEED_IN_PER_SEC && pendingSample != null) {
            samples.add(pendingSample);
            appendLine(pendingRow.replace(",adjust,", ",auto,"));
            Log.i(TAG, String.format(Locale.US, "auto sample: angle %.1f deg, trim %.2f deg", pendingSample.baseDeg, pendingSample.trimDeg));
            adjustedSinceSample = false;
        }
    }

    private void adjust(double deltaDeg) {
        trimDeg += deltaDeg;
        readiness.setAimTrimDeg(trimDeg);
        pendingDeltaDeg = Double.isNaN(pendingDeltaDeg) ? deltaDeg : pendingDeltaDeg + deltaDeg;
    }

    private void logAdjust() {
        adjustedSinceSample = true;
        pendingSample = new Sample();
        pendingRow = writeRow("adjust", pendingSample, String.format(Locale.US, "delta %+.2f", pendingDeltaDeg));
        pendingSample.kind = "auto";
        pendingDeltaDeg = Double.NaN;
    }

    private void record(String kind) {
        Sample s = new Sample();
        writeRow(kind, s, "");
        s.kind = kind;
        if (Double.isNaN(s.baseDeg)) {
            Log.w(TAG, "not recorded: turret can't reach this angle");
            return;
        }
        samples.add(s);
        adjustedSinceSample = false;
        Log.i(TAG, String.format(Locale.US, "%s sample %d: angle %.1f deg, trim %.2f deg, %.0f in, vision %.0f%%",
                kind, samples.size(), s.baseDeg, s.trimDeg, s.distance, s.weight * 100));
    }

    /** Writes one CSV row (and fills {@code s} if given). Returns the row. */
    private String writeRow(String event, Sample s, String note) {
        Pose2d pose = robot.odometrySubsystem.getPose();
        TurretVisionCorrection vc = readiness.getVisionCorrection();
        LimelightSubsystem ll = robot.limelightSubsystem;
        StringBuilder tags = new StringBuilder();
        for (LimelightSubsystem.TagObservation o : ll.getObservations()) {
            if (tags.length() > 0) tags.append(';');
            tags.append(String.format(Locale.US, "%d:%s@%.1f/%.1f/%.1f", o.id, o.cellUp ? "UP" : "DOWN", o.forward, o.left, o.up));
        }
        LimelightSubsystem.PositionEstimate est = ll.estimateRobotPosition(pose.getHeading(), CrossOpModeStorage.allianceColor);
        Translation2d from = readiness.getLastAimFrom(), point = readiness.getLastAimPoint();
        double aim = readiness.getLastAimAngleDeg();
        double base = Double.isNaN(aim) ? Double.NaN : aim - trimDeg;
        double odo = readiness.getLastOdometryAngleDeg();
        double distance = readiness.getLastDistanceIn();
        if (s != null) {
            s.baseDeg = base;
            s.trimDeg = trimDeg;
            s.odoDeg = odo;
            s.distance = distance;
            s.weight = vc.getWeight();
        }
        String row = String.format(Locale.US,
                "%s,%d,%s,%.2f,%.2f,%.2f,%.1f,%s,%s,%s,%s,%s,%s,%.2f,%.2f,%.2f,%s,%d,%s,%s,%s,%s,%s,%.1f,%.2f,%.2f,%.2f,%.2f,%.4f,%b,%.2f,%s",
                session, System.currentTimeMillis(), event, pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()),
                robot.odometrySubsystem.getFieldSpeedInchesPerSecond(), CrossOpModeStorage.allianceColor, LaunchTargets.getLastSelectedCell(),
                num(point == null ? Double.NaN : point.getX()), num(point == null ? Double.NaN : point.getY()),
                num(from == null ? Double.NaN : from.getX()), num(from == null ? Double.NaN : from.getY()),
                vc.getWeight(), vc.getAppliedOffsetX(), vc.getAppliedOffsetY(), vc.getReason().replace(',', ';'),
                ll.getObservations().size(), tags,
                num(est == null ? Double.NaN : est.position.getX()), num(est == null ? Double.NaN : est.position.getY()),
                num(est == null ? Double.NaN : est.spreadIn), est == null ? "0" : String.valueOf(est.tagCount),
                distance, base, trimDeg, aim, odo, readiness.getLastCommandedTurret(),
                robot.launchSubsystem.isLaunchReadinessLocked(), robot.launchSubsystem.getBatteryVoltage(), note.replace(',', ';'));
        appendLine(row);
        if (!"adjust".equals(event)) Log.i(TAG, event + ": " + row);
        return row;
    }

    /** Fits trim = a + b * angle over the recorded spots and logs what to change. */
    private void fit() {
        int n = samples.size();
        if (n == 0) {
            lastFit = "no spots recorded";
            return;
        }
        double mx = 0, my = 0;
        for (Sample s : samples) { mx += s.baseDeg; my += s.trimDeg; }
        mx /= n;
        my /= n;
        double sxx = 0, sxy = 0, minA = Double.MAX_VALUE, maxA = -Double.MAX_VALUE;
        for (Sample s : samples) {
            sxx += (s.baseDeg - mx) * (s.baseDeg - mx);
            sxy += (s.baseDeg - mx) * (s.trimDeg - my);
            minA = Math.min(minA, s.baseDeg);
            maxA = Math.max(maxA, s.baseDeg);
        }
        boolean scaleFit = n >= 3 && maxA - minA >= MIN_ANGLE_SPAN_FOR_SCALE_DEG && sxx > 0;
        double b = scaleFit ? sxy / sxx : 0;
        double a = my - b * mx;

        double rmsNow = 0, rmsFit = 0, rmsOdo = 0, latNow = 0;
        for (Sample s : samples) {
            double odoTrim = LaunchReadinessCommand.normalizeDeg(s.baseDeg + s.trimDeg - s.odoDeg);
            rmsNow += s.trimDeg * s.trimDeg;
            double r = s.trimDeg - (a + b * s.baseDeg);
            rmsFit += r * r;
            rmsOdo += odoTrim * odoTrim;
            latNow += Math.abs(Math.toRadians(s.trimDeg) * s.distance);
        }
        rmsNow = Math.sqrt(rmsNow / n);
        rmsFit = Math.sqrt(rmsFit / n);
        rmsOdo = Math.sqrt(rmsOdo / n);
        latNow /= n;

        double k = LaunchReadinessCommand.turretPositionPerDegree();
        double newForward = LaunchReadinessCommand.TURRET_FORWARD_POSITION + a * k;
        double newScale = LaunchReadinessCommand.TURRET_SCALING_FACTOR * (1 + b);
        lastFit = String.format(Locale.US, "%d spots: offset %.2f deg%s; TURRET_FORWARD_POSITION %.4f -> %.4f%s | aim error %.2f deg rms now (avg %.1f in at the CELL), %.2f after fit, %.2f with odometry only",
                n, a, scaleFit ? String.format(Locale.US, ", scale %+.1f%%", b * 100) : " (scale not fitted: need 3+ spots over " + MIN_ANGLE_SPAN_FOR_SCALE_DEG + " deg)",
                LaunchReadinessCommand.TURRET_FORWARD_POSITION, newForward,
                scaleFit ? String.format(Locale.US, "; TURRET_SCALING_FACTOR %.4f -> %.4f", LaunchReadinessCommand.TURRET_SCALING_FACTOR, newScale) : "",
                rmsNow, latNow, rmsFit, rmsOdo);
        Log.i(TAG, "FIT: " + lastFit);
        Log.i(TAG, rmsNow < rmsOdo ? "Limelight correction HELPS: less trim needed than odometry alone"
                : "Limelight correction does NOT help here: odometry alone needed less trim (check the camera pose / HiveTags)");
        writeRow("fit", null, lastFit);
    }

    private void showTelemetry(TelemetryManager panels) {
        TurretVisionCorrection vc = readiness.getVisionCorrection();
        panels.addData("Trim (dpad L/R)", String.format(Locale.US, "%+.2f deg   step %.2f (dpad up/down)", trimDeg, STEPS[stepIndex]));
        panels.addData("Aim from", vc.status());
        panels.addData("Tags seen", robot.limelightSubsystem.getObservations().size());
        panels.addData("Turret angle / odometry-only", String.format(Locale.US, "%.1f / %.1f deg", readiness.getLastAimAngleDeg(), readiness.getLastOdometryAngleDeg()));
        panels.addData("Distance", String.format(Locale.US, "%.1f in", readiness.getLastDistanceIn()));
        if (robot.launchSubsystem.isLaunchReadinessLocked()) panels.addLine("TURRET LOCKED: press gamepad 2 START to unlock");
        panels.addData("Recorded spots", samples.size() + (adjustedSinceSample ? " (unrecorded adjustment: A to record)" : ""));
        panels.addLine("A record | X undo | B trim 0 | Y fit");
        panels.addLine(lastFit);
        panels.update(telemetry);
    }

    private void appendLine(String line) {
        boolean newFile = !csv.exists();
        try (FileWriter w = new FileWriter(csv, true)) {
            if (newFile) w.write(HEADER + "\n");
            w.write(line + "\n");
        } catch (IOException e) {
            Log.w(TAG, "csv write failed: " + e);
        }
    }

    private static String num(double v) {
        return Double.isNaN(v) ? "" : String.format(Locale.US, "%.2f", v);
    }
}
