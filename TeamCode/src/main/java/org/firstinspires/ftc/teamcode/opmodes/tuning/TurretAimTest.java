package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.FieldPresets;
import com.bylazar.field.PanelsField;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.LaunchReadinessCommand;
import org.firstinspires.ftc.teamcode.common.AllianceColors;
import org.firstinspires.ftc.teamcode.common.CrossOpModeStorage;
import org.firstinspires.ftc.teamcode.common.HiveCell;
import org.firstinspires.ftc.teamcode.common.LaunchTargets;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;

/**
 * Test for turret aiming, using the same math as LaunchReadinessCommand. Two pose sources:
 *  - MANUAL (default, bench): the robot does NOT drive. You type a pretend robot pose into Panels
 *    (Configs tab -> TurretAimTest) and the real turret moves to where it would aim from there.
 *  - ODOMETRY (Y, or USE_ODOMETRY in Panels): drive the robot with gamepad 1 sticks (same as
 *    teleop: left stick translate, right stick X turn) and the turret tracks the goal from the
 *    live Pinpoint pose. Like teleop, odometry starts from CrossOpModeStorage.currentPose (set in
 *    INIT) and keeps writing the pose back to it. ROBOT_X / ROBOT_Y / ROBOT_HEADING_DEG mirror the
 *    live pose in this mode, so switching back to MANUAL continues from where the robot is.
 *    Target lead for moving shots is not applied (the turret aims straight at the CELL).
 * The Panels Field tab shows:
 *   - the robot as a 16 in square, with a short line from center to the front edge,
 *   - the turret direction as an arrow ("-" + ">"), computed BACK from the commanded servo
 *     position, so it shows where the servo math actually sends the turret,
 *   - the alliance's two CELLS (selected one filled) and a thin line from the robot to the target.
 * Check: the physical turret, the drawn arrow and the thin target line should all agree.
 * If the arrow is gray the target can't be reached within TURRET_MIN..TURRET_MAX.
 *
 * Coordinates are Pedro (inches, 0-144, heading CCW from +X, degrees here).
 *
 * Controls (gamepad 1). Dpad and A only work in MANUAL; they just edit ROBOT_X / ROBOT_Y / ROBOT_HEADING_DEG:
 *   Y                 - toggle MANUAL / ODOMETRY pose source
 *   Sticks            - drive (ODOMETRY only)
 *   Dpad left / right - previous / next quadrant center
 *   Dpad up / down    - heading +/- 45 deg
 *   X                 - toggle alliance
 *   A                 - toggle auto sweep: every quadrant x headings 0/90/180/270, SWEEP_STEP_MS each
 *   B                 - turret to TURRET_FORWARD_POSITION (check it points straight ahead)
 *
 * Alliance starts as CrossOpModeStorage.allianceColor. Does not spin the flywheel. Panels is not legal in matches (R704); this is a pit/bench tool.
 */
@Configurable
@TeleOp(name = "TurretAimTest", group = "Tests")
public class TurretAimTest extends LinearOpMode {

    // Live-editable in Panels.
    public static double ROBOT_X = CrossOpModeStorage.currentPose.getX();
    public static double ROBOT_Y = CrossOpModeStorage.currentPose.getY();
    public static double ROBOT_HEADING_DEG = 90;
    public static AllianceColors ALLIANCE = AllianceColors.RED;
    /** false = pose from the ROBOT_* fields (bench); true = live odometry pose, drive with sticks. */
    public static boolean USE_ODOMETRY = false;
    /** Same as DriveRobotCommand.STICK_DEADBAND. */
    public static double STICK_DEADBAND = 0.15;
    public static long SWEEP_STEP_MS = 2500;
    public static double ARROW_LENGTH_IN = 20;

    private static final double ROBOT_SIZE_IN = 16;
    private static final double[][] QUADRANT_CENTERS = {
            {36, 36}, {108, 36}, {108, 108}, {36, 108},
    };
    private static final double[] SWEEP_HEADINGS = {0, 90, 180, 270};

    private FieldManager field;

    @Override
    public void runOpMode() {
        TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
        field = PanelsField.INSTANCE.getField();
        field.setOffsets(FieldPresets.INSTANCE.getPEDRO_PATHING());

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        GamepadEx gp1 = new GamepadEx(gamepad1);
        LaunchSubsystem launcher = new LaunchSubsystem(hardwareMap, panels);
        DriveSubsystem drive = new DriveSubsystem(hardwareMap, panels);
        OdometrySubsystem odometry = new OdometrySubsystem(drive, panels);

        // Same as teleop: odometry starts from the pose carried over in CrossOpModeStorage.
        Pose2d start = CrossOpModeStorage.currentPose;
        odometry.resetPose(start.getX(), start.getY(), Math.toDegrees(start.getHeading()));
        ALLIANCE = CrossOpModeStorage.allianceColor;

        int quadrant = 0;
        boolean sweeping = false;
        int sweepStep = 0;
        ElapsedTime sweepTimer = new ElapsedTime();
        boolean forceForward = false;
        double lastPosition = Double.NaN;

        panels.addLine("MANUAL: set ROBOT_X / ROBOT_Y / ROBOT_HEADING_DEG / ALLIANCE in Panels.");
        panels.addLine("ODOMETRY (Y): drive with the sticks; start pose " + start + ".");
        panels.addLine("The turret will move on start.");
        panels.update(telemetry);
        waitForStart();

        while (opModeIsActive()) {
            gp1.readButtons();

            // ------------------------------------------------ inputs
            if (gp1.wasJustPressed(GamepadKeys.Button.Y)) {
                USE_ODOMETRY = !USE_ODOMETRY;
                sweeping = false;
            }
            boolean manual = !USE_ODOMETRY;
            if (USE_ODOMETRY) sweeping = false;
            if (manual && gp1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                sweeping = false;
                quadrant = (quadrant + 1) % QUADRANT_CENTERS.length;
                setQuadrant(quadrant);
            }
            if (manual && gp1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                sweeping = false;
                quadrant = (quadrant + QUADRANT_CENTERS.length - 1) % QUADRANT_CENTERS.length;
                setQuadrant(quadrant);
            }
            if (manual && gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                sweeping = false;
                ROBOT_HEADING_DEG = wrap360(ROBOT_HEADING_DEG + 45);
            }
            if (manual && gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                sweeping = false;
                ROBOT_HEADING_DEG = wrap360(ROBOT_HEADING_DEG - 45);
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.X)) {
                ALLIANCE = ALLIANCE == AllianceColors.RED ? AllianceColors.BLUE : AllianceColors.RED;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.B)) {
                forceForward = !forceForward;
            }
            if (manual && gp1.wasJustPressed(GamepadKeys.Button.A)) {
                sweeping = !sweeping;
                sweepStep = 0;
                applySweepStep(sweepStep);
                sweepTimer.reset();
            }
            if (sweeping && sweepTimer.milliseconds() >= SWEEP_STEP_MS) {
                sweepStep = (sweepStep + 1) % (QUADRANT_CENTERS.length * SWEEP_HEADINGS.length);
                applySweepStep(sweepStep);
                sweepTimer.reset();
            }

            // ------------------------------------------------ drive + odometry
            // Sticks drive only in ODOMETRY mode, so MANUAL stays a safe bench test.
            double forward = 0, strafe = 0, turn = 0;
            if (USE_ODOMETRY) {
                forward = deadband(gp1.getLeftY());
                strafe = deadband(-gp1.getLeftX());
                turn = deadband(-gp1.getRightX());
            }
            drive.drive(forward, strafe, turn);
            drive.periodic();    // follower.update(): reads the Pinpoint
            odometry.periodic(); // copies the pose, velocity; writes CrossOpModeStorage.currentPose

            Pose2d pose;
            if (USE_ODOMETRY) {
                pose = odometry.getPose();
                ROBOT_X = pose.getX();
                ROBOT_Y = pose.getY();
                ROBOT_HEADING_DEG = wrap360(Math.toDegrees(pose.getHeading()));
            } else {
                pose = new Pose2d(ROBOT_X, ROBOT_Y, new Rotation2d(Math.toRadians(ROBOT_HEADING_DEG)));
            }

            // ------------------------------------------------ aim (same math as LaunchReadinessCommand)
            Translation2d target = LaunchTargets.getTarget(pose, ALLIANCE);
            HiveCell cell = LaunchTargets.getLastSelectedCell();
            double relativeDeg = LaunchReadinessCommand.relativeTurretDeg(pose, target);

            double current = Double.isNaN(lastPosition) ? LaunchReadinessCommand.TURRET_FORWARD_POSITION : lastPosition;
            double position = forceForward
                    ? LaunchReadinessCommand.TURRET_FORWARD_POSITION
                    : LaunchReadinessCommand.chooseTurretPosition(relativeDeg, current);
            boolean reachable = !Double.isNaN(position);
            if (reachable) {
                launcher.setTurretPosition(position);
                lastPosition = position;
            }

            // ------------------------------------------------ draw
            double shownPosition = reachable ? position : current;
            double turretDeg = LaunchReadinessCommand.turretPositionToRelativeDeg(shownPosition);
            drawCells(cell);
            drawTargetLine(pose, target);
            drawRobot(pose);
            drawArrow(pose, Math.toRadians(ROBOT_HEADING_DEG + turretDeg), reachable ? "lime" : "gray");
            field.update();

            // ------------------------------------------------ telemetry
            double distance = pose.getTranslation().getDistance(target);
            panels.addData("Pose source (Y)", USE_ODOMETRY ? "ODOMETRY (drive with sticks)" : "MANUAL (Panels)");
            panels.addData("Mode", forceForward ? "FORWARD (B)" : sweeping ? "SWEEP step " + (sweepStep + 1) + "/16" : "tracking");
            if (USE_ODOMETRY) {
                panels.addData("Speed (in/s)", String.format("%.1f", odometry.getFieldSpeedInchesPerSecond()));
            }
            panels.addData("Robot pose", String.format("(%.1f, %.1f) @ %.0f deg", ROBOT_X, ROBOT_Y, ROBOT_HEADING_DEG));
            panels.addData("Quadrant", quadrantName(ROBOT_X, ROBOT_Y));
            panels.addData("Target", String.format("%s %s (%.1f, %.1f), %.1f in", ALLIANCE, cell, target.getX(), target.getY(), distance));
            panels.addData("Needed turret angle (deg CCW)", String.format("%.1f", relativeDeg));
            panels.addData("Servo position", reachable ? String.format("%.4f", position) : "UNREACHABLE (holding " + String.format("%.4f", current) + ")");
            panels.addData("Angle from servo position (deg)", String.format("%.1f", turretDeg));
            panels.addData("Servo range", String.format("%.2f .. %.2f, forward %.3f", LaunchSubsystem.TURRET_MIN, LaunchSubsystem.TURRET_MAX, LaunchReadinessCommand.TURRET_FORWARD_POSITION));
            panels.update(telemetry);
        }

        drive.drive(0, 0, 0);
        drive.periodic();
    }

    private static double deadband(double v) {
        return Math.abs(v) > STICK_DEADBAND ? v : 0;
    }

    // ---------------------------------------------------------------- pose presets

    private static void setQuadrant(int q) {
        ROBOT_X = QUADRANT_CENTERS[q][0];
        ROBOT_Y = QUADRANT_CENTERS[q][1];
    }

    private static void applySweepStep(int step) {
        setQuadrant(step / SWEEP_HEADINGS.length);
        ROBOT_HEADING_DEG = SWEEP_HEADINGS[step % SWEEP_HEADINGS.length];
    }

    private static String quadrantName(double x, double y) {
        String side = x < 72 ? "red side (low X)" : "blue side (high X)";
        String end = y < 72 ? "audience half" : "far half";
        return side + ", " + end;
    }

    private static double wrap360(double deg) {
        deg %= 360.0;
        return deg < 0 ? deg + 360.0 : deg;
    }

    // ---------------------------------------------------------------- drawing

    private void drawRobot(Pose2d pose) {
        double h = pose.getHeading();
        double cos = Math.cos(h), sin = Math.sin(h);
        double half = ROBOT_SIZE_IN / 2.0;
        double[][] local = {{half, half}, {-half, half}, {-half, -half}, {half, -half}};
        double[][] corners = new double[4][2];
        for (int i = 0; i < 4; i++) {
            corners[i][0] = pose.getX() + local[i][0] * cos - local[i][1] * sin;
            corners[i][1] = pose.getY() + local[i][0] * sin + local[i][1] * cos;
        }

        field.setStyle(PanelsField.INSTANCE.getTRANSPARENT(), allianceColor(), 1.0);
        for (int i = 0; i < 4; i++) {
            double[] a = corners[i], b = corners[(i + 1) % 4];
            field.moveCursor(a[0], a[1]);
            field.line(b[0], b[1]);
        }
        // Heading tick: center to the middle of the front edge.
        field.moveCursor(pose.getX(), pose.getY());
        field.line(pose.getX() + half * cos, pose.getY() + half * sin);
    }

    /** Arrow from the robot center: shaft ("-") plus two short head lines (">"). */
    private void drawArrow(Pose2d pose, double angleRad, String color) {
        double x0 = pose.getX(), y0 = pose.getY();
        double tipX = x0 + ARROW_LENGTH_IN * Math.cos(angleRad);
        double tipY = y0 + ARROW_LENGTH_IN * Math.sin(angleRad);
        double headLen = 5.0;
        double spread = Math.toRadians(150);

        field.setStyle(PanelsField.INSTANCE.getTRANSPARENT(), color, 1.5);
        field.moveCursor(x0, y0);
        field.line(tipX, tipY);
        field.moveCursor(tipX, tipY);
        field.line(tipX + headLen * Math.cos(angleRad + spread), tipY + headLen * Math.sin(angleRad + spread));
        field.moveCursor(tipX, tipY);
        field.line(tipX + headLen * Math.cos(angleRad - spread), tipY + headLen * Math.sin(angleRad - spread));
    }

    private void drawTargetLine(Pose2d pose, Translation2d target) {
        field.setStyle(PanelsField.INSTANCE.getTRANSPARENT(), "yellow", 0.5);
        field.moveCursor(pose.getX(), pose.getY());
        field.line(target.getX(), target.getY());
    }

    private void drawCells(HiveCell selected) {
        for (HiveCell c : new HiveCell[]{HiveCell.FAR, HiveCell.AUDIENCE}) {
            Translation2d p = LaunchTargets.getCellPosition(ALLIANCE, c);
            field.setStyle(c == selected ? allianceColor() : PanelsField.INSTANCE.getTRANSPARENT(), allianceColor(), 1.0);
            field.moveCursor(p.getX(), p.getY());
            field.circle(3);
        }
    }

    private static String allianceColor() {
        return ALLIANCE == AllianceColors.RED ? PanelsField.INSTANCE.getRED() : PanelsField.INSTANCE.getBLUE();
    }
}
