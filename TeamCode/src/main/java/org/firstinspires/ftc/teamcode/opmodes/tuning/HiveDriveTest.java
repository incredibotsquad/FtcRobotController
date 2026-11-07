package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.common.AllianceColors;
import org.firstinspires.ftc.teamcode.common.CrossOpModeStorage;
import org.firstinspires.ftc.teamcode.common.HiveCell;
import org.firstinspires.ftc.teamcode.common.HiveState;
import org.firstinspires.ftc.teamcode.common.HiveTags;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

/**
 * Drive-around check of the Limelight HIVE TIP detection. Like the TeleOp's DRIVE_ONLY_MODE plus
 * the Limelight: only DriveSubsystem, OdometrySubsystem and LimelightSubsystem are built. Intake,
 * transfer, gate, flywheel, turret and intake bar are never constructed, so they stay unpowered.
 *
 * Unlike HiveStateTest (stationary, scores readings against a truth you enter), this one lets you
 * drive around the field and watch what the real subsystem decides from each spot:
 *  - Limelight connected / result staleness,
 *  - debounced HiveState up CELL for RED and BLUE, and how long ago each was last confirmed,
 *  - every HIVE tag in the latest frame: id, alliance, CELL, measured height vs the up/down
 *    threshold, and the UP / DOWN verdict. Tags the subsystem throws away are flagged with the
 *    reason (TOO FAR beyond MAX_TAG_DISTANCE_IN, or AMBIGUOUS within UP_DOWN_MARGIN_IN of the
 *    threshold). The filters are recomputed here with the subsystem's own constants,
 *  - robot pose and whether the robot is stationary (DriveSubsystem.isEffectivelyStationary()),
 *  - a log of the last LOG_LENGTH HiveState changes, timestamped in seconds since INIT.
 *
 * Procedure: press A to reset HiveState to the match-start state (red AUDIENCE up, blue FAR up).
 * Drive to where the HIVE is visible and check the up CELL. TIP the HIVE by hand and check a log
 * entry appears for that alliance (and only one). Repeat from other distances and angles.
 *
 * Controls (gamepad 1), after START:
 *   Sticks - drive (same as teleop: left stick translate, right stick X turn)
 *   A      - reset HiveState to the match-start state (logged)
 *   B      - clear the change log
 *
 * Odometry starts from CrossOpModeStorage.currentPose and writes the pose back to it, like teleop.
 * Nothing moves during INIT: only the Limelight is read. Panels is not legal in matches (R704).
 */
@Configurable
@TeleOp(name = "Test: HIVE Drive (Limelight)", group = "Tests")
public class HiveDriveTest extends LinearOpMode {

    /** Same as DriveRobotCommand.STICK_DEADBAND. */
    public static double STICK_DEADBAND = 0.15;
    /** Most per-tag lines to show per frame. */
    public static int MAX_TAG_LINES = 8;
    /** HiveState changes kept in the log. */
    public static int LOG_LENGTH = 6;

    private static final AllianceColors[] ALLIANCES = {AllianceColors.RED, AllianceColors.BLUE};

    private final ElapsedTime sinceInit = new ElapsedTime();
    private final LinkedList<String> changeLog = new LinkedList<>();
    private final HiveCell[] lastUpCell = new HiveCell[2];

    private TelemetryManager panels;
    private LimelightSubsystem limelight;

    @Override
    public void runOpMode() {
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        sinceInit.reset();

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        GamepadEx gp1 = new GamepadEx(gamepad1);
        DriveSubsystem drive = new DriveSubsystem(hardwareMap, panels);
        OdometrySubsystem odometry = new OdometrySubsystem(drive, panels);
        limelight = new LimelightSubsystem(hardwareMap, panels);

        // Same as teleop: odometry starts from the pose carried over in CrossOpModeStorage.
        Pose2d start = CrossOpModeStorage.currentPose;
        odometry.resetPose(start.getX(), start.getY(), Math.toDegrees(start.getHeading()));

        lastUpCell[0] = HiveState.getUpCell(AllianceColors.RED);
        lastUpCell[1] = HiveState.getUpCell(AllianceColors.BLUE);

        // INIT: read the Limelight only. Drive/odometry periodic() are not called, so the
        // follower never writes motor powers before START.
        while (opModeInInit()) {
            limelight.periodic();
            logHiveChanges();
            panels.addLine("INIT: nothing moves. After START: sticks drive, A = reset HiveState, B = clear log.");
            panels.addData("Start pose", String.format("(%.1f, %.1f) @ %.0f deg",
                    start.getX(), start.getY(), Math.toDegrees(start.getHeading())));
            addVisionLines();
            panels.update(telemetry);
        }

        while (opModeIsActive()) {
            gp1.readButtons();

            // ------------------------------------------------ buttons
            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                HiveState.resetToMatchStart();
                log("RESET to match start (red " + HiveState.RED_START_UP_CELL
                        + ", blue " + HiveState.BLUE_START_UP_CELL + " up)");
                lastUpCell[0] = HiveState.getUpCell(AllianceColors.RED);
                lastUpCell[1] = HiveState.getUpCell(AllianceColors.BLUE);
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.B)) {
                changeLog.clear();
            }

            // ------------------------------------------------ drive (same mapping as DriveRobotCommand)
            // GamepadEx.getLeftY() is already +up. Pedro: +forward, +strafe LEFT, +turn counter-clockwise.
            drive.drive(deadband(gp1.getLeftY()), deadband(-gp1.getLeftX()), deadband(-gp1.getRightX()));

            // Same order as the scheduler: drive, odometry, limelight.
            drive.periodic();     // follower.update(): reads the Pinpoint, writes motor powers
            odometry.periodic();  // copies the pose, velocity; writes CrossOpModeStorage.currentPose
            limelight.periodic(); // reads tags, updates HiveState after HIVE_CONFIRM_FRAMES
            logHiveChanges();

            // ------------------------------------------------ telemetry
            Pose2d pose = odometry.getPose();
            panels.addData("Robot pose", String.format("(%.1f, %.1f) @ %.0f deg",
                    pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading())));
            panels.addData("Stationary", drive.isEffectivelyStationary()
                    ? "yes" : String.format("no (%.1f in/s)", odometry.getFieldSpeedInchesPerSecond()));
            addVisionLines();
            panels.update(telemetry);
        }

        drive.drive(0, 0, 0);
        drive.periodic();
    }

    /** Limelight status, HiveState, per-tag verdicts and the change log. */
    private void addVisionLines() {
        LLResult result = limelight.getLatestResult();
        boolean fresh = result != null && result.isValid()
                && result.getStaleness() <= LimelightSubsystem.MAX_RESULT_AGE_MS;

        panels.addData("Limelight", !limelight.isConnected() ? "NOT CONNECTED"
                : result == null ? "no result"
                : !result.isValid() ? "result invalid"
                : "staleness " + result.getStaleness() + " ms" + (fresh ? "" : " (STALE, ignored)"));

        long now = System.nanoTime() / 1_000_000L; // same clock LimelightSubsystem stamps HiveState with
        for (AllianceColors a : ALLIANCES) {
            long updated = HiveState.getLastUpdateMs(a);
            panels.addData(a + " HIVE up CELL", HiveState.getUpCell(a)
                    + (updated == 0 ? "  (not seen since reset)"
                    : String.format("  (confirmed %.1f s ago)", (now - updated) / 1000.0)));
        }

        panels.addLine(String.format("--- HIVE tags (threshold z %.1f in, margin +/-%.1f, max dist %.0f) ---",
                HiveTags.upDownThresholdZ(), LimelightSubsystem.UP_DOWN_MARGIN_IN,
                LimelightSubsystem.MAX_TAG_DISTANCE_IN));
        List<String> tagLines = fresh ? describeTags(result) : new ArrayList<>();
        if (tagLines.isEmpty()) panels.addLine("(no HIVE tags)");
        for (String line : tagLines) panels.addLine(line);

        panels.addLine("--- HiveState changes (s since INIT) ---");
        if (changeLog.isEmpty()) panels.addLine("(none)");
        for (String entry : changeLog) panels.addLine(entry);
    }

    /** One line per HIVE tag, with the same transform and filters as LimelightSubsystem.periodic(). */
    private static List<String> describeTags(LLResult result) {
        double threshold = HiveTags.upDownThresholdZ();
        List<LLResultTypes.FiducialResult> fiducials = new ArrayList<>(result.getFiducialResults());
        fiducials.sort((a, b) -> Integer.compare(a.getFiducialId(), b.getFiducialId()));

        List<String> lines = new ArrayList<>();
        for (LLResultTypes.FiducialResult f : fiducials) {
            int id = f.getFiducialId();
            if (!HiveTags.isHiveTag(id)) continue;
            if (lines.size() >= MAX_TAG_LINES) break;

            Position p = f.getTargetPoseRobotSpace().getPosition().toUnit(DistanceUnit.INCH);
            double forward = p.x;
            double left = LimelightSubsystem.ROBOT_SPACE_Y_IS_LEFT ? p.y : -p.y;
            double up = p.z;
            double dist = Math.hypot(forward, left);

            String verdict;
            if (dist > LimelightSubsystem.MAX_TAG_DISTANCE_IN) {
                verdict = "REJECTED: too far";
            } else if (Math.abs(up - threshold) < LimelightSubsystem.UP_DOWN_MARGIN_IN) {
                verdict = "REJECTED: ambiguous height";
            } else {
                verdict = up > threshold ? "UP" : "DOWN";
            }

            lines.add(String.format("%d %s %s: z %.1f (%+.1f vs thr), dist %.0f -> %s",
                    id, HiveTags.allianceOf(id), HiveTags.cellOf(id), up, up - threshold, dist, verdict));
        }
        return lines;
    }

    /** Adds a log entry whenever a HIVE's up CELL changes (HiveState is re-confirmed every frame). */
    private void logHiveChanges() {
        for (int i = 0; i < 2; i++) {
            HiveCell now = HiveState.getUpCell(ALLIANCES[i]);
            if (now != lastUpCell[i]) {
                log(ALLIANCES[i] + " HIVE: " + lastUpCell[i] + " -> " + now + " up");
                lastUpCell[i] = now;
            }
        }
    }

    private void log(String message) {
        changeLog.addFirst(String.format("%6.1f  %s", sinceInit.seconds(), message));
        while (changeLog.size() > LOG_LENGTH) changeLog.removeLast();
    }

    private static double deadband(double v) {
        return Math.abs(v) > STICK_DEADBAND ? v : 0;
    }
}
