package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.common.CrossOpModeStorage;
import org.firstinspires.ftc.teamcode.common.DriverBanner;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchGateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Automatic Limelight relocalization during launches (TeleOp). Runs in the background for the whole
 * match (no requirements, never interrupts anything).
 *
 * While the gate is open (a launch) and the robot is effectively stationary, it collects Limelight
 * position estimates from OUR HIVE's tags. A frame counts when it has at least MIN_TAGS tags agreeing
 * within MAX_SPREAD_IN and lies within MAX_JUMP_IN of odometry. Once SAMPLES_NEEDED frames agree
 * with their average within MAX_SAMPLE_SCATTER_IN, their average becomes the new odometry X/Y (heading
 * kept from the Pinpoint). It is applied when the launch ends, so the turret doesn't move mid-burst;
 * at most once per launch. Moving during the launch throws the samples away.
 *
 * The current state is always available from status() for the Driver Station (the manual
 * RelocalizeWithLimelightCommand reports there too, via report()). Logged with tag LaunchRelocalize.
 */
@Configurable
public class LaunchRelocalizeCommand extends CommandBase {

    /** Master switch for TeleOp (and the TeleOp-based tuners), live in Panels. */
    public static boolean ENABLED = true;
    /**
     * Same, for the autonomous OpModes (Incredibot.initAuto schedules it only when this is true at
     * START). Off by default: a pose reset changes where the next auto path starts from.
     */
    public static boolean ENABLED_IN_AUTO = false;
    public static int SAMPLES_NEEDED = 5;
    public static int MIN_TAGS = 2;
    public static double MAX_SPREAD_IN = 3.0;
    /** Every sample must be this close to the samples' average. */
    public static double MAX_SAMPLE_SCATTER_IN = 1.5;
    /** Ignore estimates farther than this from odometry (bad data). */
    public static double MAX_JUMP_IN = 24.0;
    /** Skip the reset if odometry is already this close. */
    public static double MIN_JUMP_IN = 0.5;

    private static final String TAG = "LaunchRelocalize";
    private static volatile String status = "starting";
    private static volatile String lastResult = "none yet";

    private final LimelightSubsystem limelight;
    private final OdometrySubsystem odometry;
    private final DriveSubsystem drive;
    private final LaunchGateSubsystem gate;
    private final boolean auto;

    private final List<Translation2d> samples = new ArrayList<>();
    private Translation2d pending = null;
    private boolean doneThisLaunch = false;
    private long lastFrameMs = 0;
    private String lastReject = "";

    public LaunchRelocalizeCommand(LimelightSubsystem limelight, OdometrySubsystem odometry,
                                   DriveSubsystem drive, LaunchGateSubsystem gate) {
        this(limelight, odometry, drive, gate, false);
    }

    /** {@code auto} = running in an autonomous OpMode: ENABLED_IN_AUTO is the switch instead of ENABLED. */
    public LaunchRelocalizeCommand(LimelightSubsystem limelight, OdometrySubsystem odometry,
                                   DriveSubsystem drive, LaunchGateSubsystem gate, boolean auto) {
        this.auto = auto;
        this.limelight = limelight;
        this.odometry = odometry;
        this.drive = drive;
        this.gate = gate;
        // No requirements: it only reads subsystems and resets the odometry pose.
    }

    /**
     * One line for the Driver Station. OFF: just that (and how to turn it on). ON: what it is doing now
     * and the last result.
     */
    public static String status() {
        if (!ENABLED) return "OFF (operator right stick: turn on)";
        return "ON | " + status + " | last: " + lastResult;
    }

    /** Operator right stick button: turns launch relocalization on / off, with a banner and a log line. */
    public static void toggle() {
        ENABLED = !ENABLED;
        Log.i(TAG, "turned " + (ENABLED ? "ON" : "OFF") + " by the operator");
        DriverBanner.show("LL RELOCALIZE " + (ENABLED ? "ON" : "OFF"),
                ENABLED ? "odometry updates after launches at standstill" : "odometry is not changed by the Limelight",
                ENABLED ? "#00E676" : "#FF5252");
    }

    /** Lets the manual relocalize report its result in the same status line. */
    public static void report(String result) {
        lastResult = result;
    }

    @Override
    public void initialize() {
        clear();
        status = enabled() ? "waiting for a launch at standstill" : "OFF";
    }

    @Override
    public void execute() {
        if (!enabled()) {
            clear();
            status = "off";
            return;
        }
        boolean launching = gate.isGateOpen();
        if (!launching) {
            if (pending != null) apply();
            else if (!samples.isEmpty()) {
                status = "launch ended with " + samples.size() + "/" + SAMPLES_NEEDED + " samples" + reject();
                Log.i(TAG, status);
            } else if (!status.startsWith("launch ended")) {
                status = "waiting for a launch at standstill";
            }
            samples.clear();
            doneThisLaunch = false;
            return;
        }
        if (doneThisLaunch) return;
        if (pending != null) {
            status = "sample ready: odometry updates when the launch ends";
            return;
        }
        if (!drive.isEffectivelyStationary()) {
            samples.clear();
            status = "launching but moving: no samples";
            return;
        }
        long frameMs = limelight.getFrameTimestampMs();
        if (frameMs == lastFrameMs) return;
        lastFrameMs = frameMs;

        Pose2d pose = odometry.getPose();
        LimelightSubsystem.PositionEstimate est = limelight.estimateRobotPosition(pose.getHeading(), CrossOpModeStorage.allianceColor);
        if (est == null) { lastReject = "no tags of our HIVE"; status = "sampling " + samples.size() + "/" + SAMPLES_NEEDED + reject(); return; }
        if (est.tagCount < MIN_TAGS) { lastReject = est.tagCount + " tag(s), need " + MIN_TAGS; status = "sampling " + samples.size() + "/" + SAMPLES_NEEDED + reject(); return; }
        if (est.spreadIn > MAX_SPREAD_IN) { lastReject = String.format(Locale.US, "tags disagree %.1f in", est.spreadIn); status = "sampling " + samples.size() + "/" + SAMPLES_NEEDED + reject(); return; }
        double jump = est.position.getDistance(pose.getTranslation());
        if (jump > MAX_JUMP_IN) { lastReject = String.format(Locale.US, "%.1f in from odometry", jump); status = "sampling " + samples.size() + "/" + SAMPLES_NEEDED + reject(); return; }

        samples.add(est.position);
        status = String.format(Locale.US, "sampling %d/%d (%d tags)", samples.size(), SAMPLES_NEEDED, est.tagCount);
        if (samples.size() < SAMPLES_NEEDED) return;

        double mx = 0, my = 0;
        for (Translation2d s : samples) { mx += s.getX(); my += s.getY(); }
        mx /= samples.size();
        my /= samples.size();
        double scatter = 0;
        for (Translation2d s : samples) scatter = Math.max(scatter, Math.hypot(s.getX() - mx, s.getY() - my));
        if (scatter > MAX_SAMPLE_SCATTER_IN) {
            lastReject = String.format(Locale.US, "samples scattered %.1f in", scatter);
            samples.remove(0); // slide the window and keep trying
            status = "sampling " + samples.size() + "/" + SAMPLES_NEEDED + reject();
            return;
        }
        pending = new Translation2d(mx, my);
        doneThisLaunch = true;
        status = "sample ready: odometry updates when the launch ends";
    }

    private void apply() {
        Pose2d now = odometry.getPose();
        double jump = pending.getDistance(now.getTranslation());
        if (jump < MIN_JUMP_IN) {
            lastResult = String.format(Locale.US, "odometry already within %.1f in", jump);
        } else {
            odometry.updatePoseFromLimelight(new Pose2d(pending, new Rotation2d(now.getHeading())));
            lastResult = String.format(Locale.US, "moved %.1f in to (%.1f, %.1f)", jump, pending.getX(), pending.getY());
            DriverBanner.showRelocalized(String.format(Locale.US, "Limelight, during launch (%.1f in)", jump),
                    pending.getX(), pending.getY(), Math.toDegrees(now.getHeading()));
        }
        Log.i(TAG, "relocalized after launch: " + lastResult + " (from " + now + ")");
        status = "waiting for a launch at standstill";
        pending = null;
        samples.clear();
    }

    private boolean enabled() {
        return auto ? ENABLED_IN_AUTO : ENABLED;
    }

    private String reject() {
        return lastReject.isEmpty() ? "" : " (last skip: " + lastReject + ")";
    }

    private void clear() {
        samples.clear();
        pending = null;
        doneThisLaunch = false;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
