package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Incredibot;
import org.firstinspires.ftc.teamcode.commands.LaunchBallsCommand;
import org.firstinspires.ftc.teamcode.common.AllianceColors;
import org.firstinspires.ftc.teamcode.common.CrossOpModeStorage;
import org.firstinspires.ftc.teamcode.common.HiveCell;
import org.firstinspires.ftc.teamcode.common.HiveState;
import org.firstinspires.ftc.teamcode.common.LaunchTargets;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.ArrayList;

/**
 * Fixed autonomous from the driver's LEFT wall (red: the wall opposite the audience). Poses are the red-frame Poses values,
 * rotated 180 deg for blue like everything else:
 *  1. Start at RED_START_FAR_WALL (the driver's LEFT), drive to RED_LAUNCH_FAR_CELL_2 (LEFT spot 2)
 *     and wait there until the Limelight sees the LEFT CELL up (it starts down).
 *  2. Launch the 4 preloads.
 *  3. Intake at RED_LEFT_FLOWER (approach, bar down if SIDE_FLOWER_NEEDS_BAR, in at the drive-in
 *     speed, until full or FLOWER_WAIT_MS, back out, bar up).
 *  4. Back to the launch spot; if the CELL is still up (a Limelight reading from after arriving or
 *     up to TIP_READING_MAX_AGE_MS before), launch again.
 *  5. Park at RED_PARK_LEFT (red: the far end of the LOADING ZONE).
 * Whenever it can't launch (the CELL never comes up, it tipped back down, nothing was picked up)
 * it parks instead. In the last PARK_WINDOW_S it parks unless it is launching or on its way to
 * launch with artifacts (an intake is cut short then).
 *
 * INIT: operator gamepad (gamepad 2) B = RED, X = BLUE. Nothing moves until START.
 *
 * LeftWallAutoNoFlower runs steps 1, 2 and 5 only (wait for the CELL, launch the preloads, park).
 *
 * Timing, speed and TIP-check settings are shared with DynamicAuto (its public static fields).
 * Logged with the OpMode's class name as the tag.
 */
@Autonomous(name = "LeftWallAuto", group = "Autonomous", preselectTeleOp = "IncredibotsMecanumDrive")
public class LeftWallAuto extends LinearOpMode {

    /** Log tag: the OpMode's class name (LeftWallAuto / LeftWallAutoNoFlower). */
    private final String tag = getClass().getSimpleName();

    /** Whether to intake at the LEFT FLOWER and launch a second time. LeftWallAutoNoFlower: false. */
    protected boolean intakeAtFlower() {
        return true;
    }

    private int plannedLaunches() {
        return intakeAtFlower() ? 2 : 1;
    }

    private enum State {
        DRIVE_TO_LAUNCH, WAIT_FOR_UP, LAUNCHING,
        DRIVE_TO_INTAKE, LOWER_BAR, DRIVE_INTO_INTAKE, INTAKING, BACK_OUT_OF_INTAKE,
        DRIVE_BACK_TO_LAUNCH, CHECK_CELL,
        DRIVE_TO_PARK, DONE
    }

    private static final IntakePoint INTAKE = IntakePoint.LEFT_FLOWER;

    private Incredibot robot;
    private Follower follower;
    private Poses poses;
    private boolean mechanisms;
    private AllianceColors alliance = AllianceColors.RED;
    /** The CELL we launch into: the one that starts down. */
    private HiveCell cell;
    private LaunchPoint launchPoint;

    private State state = State.DONE;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime matchTimer = new ElapsedTime();
    private boolean followingPath = false;
    private boolean barDown = false;
    private boolean parkStarted = false;
    private boolean endBarLowered = false;
    private int launchesDone = 0;
    private long cellUpSinceMs = -1;
    private long arrivedAtLaunchMs = 0;
    private LaunchBallsCommand launchCommand;
    private String status = "";
    private String loggedStatus = "";
    private HiveCell loggedUpCell = null;

    @Override
    public void runOpMode() {
        TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
        robot = new Incredibot(hardwareMap, panels);
        follower = robot.driveSubsystem.getFollower();
        mechanisms = !Incredibot.DRIVE_ONLY_MODE;

        while (opModeInInit()) {
            if (gamepad2.bWasPressed()) alliance = AllianceColors.RED;
            if (gamepad2.xWasPressed()) alliance = AllianceColors.BLUE;
            CommandScheduler.getInstance().run(); // subsystem periodic() only; nothing moves
            dropSubsystemTelemetry(panels);
            panels.addLine("=== " + tag + " ===");
            if (!mechanisms) panels.addLine("DRIVE_ONLY_MODE: paths only, no launching or bar");
            panels.addData("Alliance (operator B red / X blue)", alliance);
            panels.addLine("Start LEFT wall, wait for the LEFT CELL to be up, launch from LEFT spot 2,");
            panels.addLine((intakeAtFlower() ? "intake at the LEFT FLOWER, launch again, " : "")
                    + "park LEFT");
            panels.update(telemetry);
        }
        if (isStopRequested()) return;

        beginMatch();
        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
            dropSubsystemTelemetry(panels);
            update();
            showStatus(panels);
        }
        CommandScheduler.getInstance().reset();
    }

    // ================================================================ start

    private void beginMatch() {
        boolean red = alliance == AllianceColors.RED;
        CrossOpModeStorage.allianceColor = alliance; // aiming, and TeleOp afterwards
        poses = new Poses(red);

        robot.initialize(Incredibot.OpModeType.AUTO, null, null); // preload count 4, HiveState reset
        follower.holdEnd.set(true);
        if (mechanisms) robot.intakeBarSubsystem.raise();

        // The driver's LEFT: start RED_START_FAR_WALL, launch RED_LAUNCH_FAR_CELL_2 into the LEFT CELL
        // (down at the start), park RED_PARK_LEFT (rotated for blue).
        launchPoint = LaunchPoint.LEFT_SPOT_2;
        cell = launchPoint.cell(red);
        LaunchTargets.setForcedCell(cell);

        Pose start = poses.start(Side.LEFT);
        robot.odometrySubsystem.resetPose(start.x(), start.y(), Math.toDegrees(start.heading()));

        matchTimer.reset();
        loggedUpCell = HiveState.getUpCell(alliance);
        log("START " + alliance + ", start pose " + fmt(start) + ", up CELL " + loggedUpCell
                + ", launching into the " + side(cell) + " CELL from " + fmt(poses.launch(launchPoint))
                + ", mechanisms " + (mechanisms ? "on" : "off (DRIVE_ONLY_MODE)"));
        driveTo(poses.launch(launchPoint), 1.0);
        setState(State.DRIVE_TO_LAUNCH, "to " + launchPoint.label() + " to wait for the " + side(cell) + " CELL");
    }

    // ================================================================ cycle

    private void update() {
        HiveCell upCell = HiveState.getUpCell(alliance);
        if (upCell != loggedUpCell) {
            log("HIVE up CELL changed " + (loggedUpCell == null ? "?" : side(loggedUpCell)) + " -> " + side(upCell) + " (Limelight)");
            loggedUpCell = upCell;
        }

        if (state != State.LAUNCHING && matchTimer.seconds() >= DynamicAuto.AUTO_LENGTH_S - DynamicAuto.END_BAR_LOWER_S) {
            lowerBarAtEnd();
        }

        if (!parkStarted && state != State.LAUNCHING && parkTime()) {
            String reason = noLaunchReason();
            if (reason != null) {
                startPark("park window, " + reason);
                return;
            }
            if (hurryToLaunch()) return;
        }

        switch (state) {
            case DRIVE_TO_LAUNCH:
                if (pathDone()) setState(State.WAIT_FOR_UP, "at " + launchPoint.label() + ", waiting for the "
                        + side(cell) + " CELL to be up");
                break;

            case WAIT_FOR_UP:
                if (!mechanisms) {
                    startPark("DRIVE_ONLY_MODE: no launching");
                } else if (HiveState.getUpCell(alliance) == cell) {
                    tryLaunch();
                } else {
                    cellUpSinceMs = -1;
                    setStatus(side(cell) + " CELL is down (up CELL " + side(HiveState.getUpCell(alliance)) + "), waiting");
                }
                break;

            case LAUNCHING:
                if (!launchCommand.isScheduled()) {
                    launchesDone++;
                    log("launch " + launchesDone + " finished after " + ms(stateTimer) + ", "
                            + robot.intakeSubsystem.getArtifactCount() + " artifacts left");
                    if (launchesDone >= plannedLaunches()) startPark("all " + plannedLaunches() + " launch(es) done");
                    else if (parkTime()) startPark("park window reached during the launch");
                    else driveToIntake();
                }
                break;

            case DRIVE_TO_INTAKE:
                if (pathDone()) {
                    if (mechanisms && INTAKE.needsBar()) {
                        robot.intakeBarSubsystem.lower();
                        barDown = true;
                        setState(State.LOWER_BAR, "at approach, lowering the bar");
                    } else {
                        driveIntoIntake();
                    }
                }
                break;

            case LOWER_BAR:
                if (robot.intakeBarSubsystem.isAtTarget() || stateTimer.milliseconds() >= DynamicAuto.BAR_LOWER_TIMEOUT_MS) {
                    log("bar " + (robot.intakeBarSubsystem.isAtTarget() ? "down" : "lower timed out") + " after " + ms(stateTimer));
                    driveIntoIntake();
                }
                break;

            case DRIVE_INTO_INTAKE:
                if (pathDone()) setState(State.INTAKING, "at the FLOWER, intaking up to "
                        + DynamicAuto.FLOWER_WAIT_MS + " ms (" + robot.intakeSubsystem.getArtifactCount() + " artifacts)");
                break;

            case INTAKING:
                if (robot.intakeSubsystem.isFullAndDoneIntaking() || stateTimer.milliseconds() >= DynamicAuto.FLOWER_WAIT_MS) {
                    log("intake done: " + (robot.intakeSubsystem.isFullAndDoneIntaking() ? "full" : "FLOWER_WAIT_MS over")
                            + " after " + ms(stateTimer) + ", " + robot.intakeSubsystem.getArtifactCount() + " artifacts");
                    leaveIntake();
                }
                break;

            case BACK_OUT_OF_INTAKE:
                if (pathDone()) {
                    raiseBar();
                    driveBackToLaunch();
                }
                break;

            case DRIVE_BACK_TO_LAUNCH:
                if (pathDone()) {
                    arrivedAtLaunchMs = nowMs();
                    cellUpSinceMs = -1;
                    setState(State.CHECK_CELL, "back at " + launchPoint.label() + ", checking the " + side(cell) + " CELL");
                }
                break;

            case CHECK_CELL:
                updateCheckCell();
                break;

            case DRIVE_TO_PARK:
                if (pathDone()) {
                    setState(State.DONE, "parked at " + String.format("%.1f s", matchTimer.seconds()));
                    lowerBarAtEnd();
                }
                break;

            case DONE:
                break;
        }
    }

    /** Back at the launch spot: launch only if a recent Limelight reading says the CELL is still up. */
    private void updateCheckCell() {
        if (robot.intakeSubsystem.getArtifactCount() <= 0) {
            startPark("nothing picked up: no second launch");
            return;
        }
        if (stateTimer.milliseconds() < DynamicAuto.TIP_CHECK_MS) {
            setStatus("looking at the HIVE (TIP_CHECK_MS " + DynamicAuto.TIP_CHECK_MS + ")");
            return;
        }
        long lastReadingMs = HiveState.getLastUpdateMs(alliance);
        boolean fresh = lastReadingMs > 0 && lastReadingMs >= arrivedAtLaunchMs - DynamicAuto.TIP_READING_MAX_AGE_MS;
        if (!fresh && stateTimer.milliseconds() < DynamicAuto.TIP_CHECK_TIMEOUT_MS) {
            setStatus("no recent Limelight reading of our HIVE, waiting (TIP_CHECK_TIMEOUT_MS "
                    + DynamicAuto.TIP_CHECK_TIMEOUT_MS + ")");
            return;
        }
        boolean up = HiveState.getUpCell(alliance) == cell;
        if (!fresh) {
            Log.w(tag, String.format("[%5.2fs] CELL check UNCONFIRMED: no recent Limelight reading, %s",
                    matchTimer.seconds(), DynamicAuto.LAUNCH_IF_TIP_UNCONFIRMED && up
                            ? "launching anyway (LAUNCH_IF_TIP_UNCONFIRMED)" : "not launching"));
            if (!DynamicAuto.LAUNCH_IF_TIP_UNCONFIRMED) up = false;
        }
        if (up) {
            tryLaunch();
        } else {
            startPark(side(cell) + " CELL is down (or unconfirmed): can't launch again");
        }
    }

    /** CELL is up: launch once the launcher is ready, or after READY_TIMEOUT_MS anyway. */
    private void tryLaunch() {
        if (cellUpSinceMs < 0) cellUpSinceMs = nowMs();
        boolean ready = robot.launchSubsystem.isReadyToLaunch();
        if (!ready && nowMs() - cellUpSinceMs < DynamicAuto.READY_TIMEOUT_MS) {
            setStatus(side(cell) + " CELL is up, waiting for launcher" + (robot.launchReadinessCommand == null ? ""
                    : " (" + robot.launchReadinessCommand.getNotReadyReason() + ")"));
            return;
        }
        launchCommand = new LaunchBallsCommand(robot.launchGateSubsystem, robot.intakeSubsystem,
                robot.driveSubsystem, robot.intakeBarSubsystem);
        launchCommand.schedule();
        setState(State.LAUNCHING, "launch " + (launchesDone + 1) + ": " + side(cell) + " CELL is up, launcher "
                + (ready ? "ready" : "NOT ready after READY_TIMEOUT_MS, firing anyway"));
    }

    private void driveToIntake() {
        driveTo(poses.intakeApproach(INTAKE), 1.0);
        setState(State.DRIVE_TO_INTAKE, "to the " + INTAKE.label(alliance == AllianceColors.RED) + " approach");
    }

    private void driveIntoIntake() {
        driveTo(poses.intake(INTAKE), DynamicAuto.INTAKE_DRIVE_IN_SPEED_FRACTION);
        setState(State.DRIVE_INTO_INTAKE, "driving in at " + DynamicAuto.INTAKE_DRIVE_IN_SPEED_FRACTION + " speed");
    }

    /** Done intaking: back straight out first if the bar is down, else straight to the launch spot. */
    private void leaveIntake() {
        if (barDown) {
            driveTo(poses.intakeApproach(INTAKE), 1.0);
            setState(State.BACK_OUT_OF_INTAKE, "bar is down: backing out to the approach pose");
        } else {
            driveBackToLaunch();
        }
    }

    private void driveBackToLaunch() {
        driveTo(poses.launch(launchPoint), 1.0);
        setState(State.DRIVE_BACK_TO_LAUNCH, "back to " + launchPoint.label() + " with "
                + robot.intakeSubsystem.getArtifactCount() + " artifacts");
    }

    // ================================================================ park

    private boolean parkTime() {
        return matchTimer.seconds() >= DynamicAuto.AUTO_LENGTH_S - DynamicAuto.PARK_WINDOW_S;
    }

    /** Why it can't launch soon (park now), or null if it can. */
    private String noLaunchReason() {
        if (!mechanisms) return "mechanisms off";
        if (launchesDone >= plannedLaunches()) return "all launches done";
        if (robot.intakeSubsystem.getArtifactCount() <= 0) return "no artifacts to launch";
        if ((state == State.WAIT_FOR_UP || state == State.CHECK_CELL) && HiveState.getUpCell(alliance) != cell) {
            return side(cell) + " CELL is down";
        }
        return null;
    }

    /** Park window with artifacts: cut the intake short and go launch. Returns true if it changed state. */
    private boolean hurryToLaunch() {
        switch (state) {
            case DRIVE_TO_INTAKE:
                log("park window: skipping the intake to go launch");
                driveBackToLaunch();
                return true;
            case LOWER_BAR:
            case DRIVE_INTO_INTAKE:
            case INTAKING:
                log("park window: cutting the intake short to go launch");
                leaveIntake();
                return true;
            default:
                return false;
        }
    }

    private void startPark(String reason) {
        parkStarted = true;
        raiseBar();
        driveTo(poses.park(Side.LEFT), 1.0);
        setState(State.DRIVE_TO_PARK, "parking LEFT: " + reason);
    }

    private void raiseBar() {
        if (barDown) {
            robot.intakeBarSubsystem.raise();
            barDown = false;
        }
    }

    /** End of auto: lower the intake bar once (see DynamicAuto.END_BAR_LOWER_S). */
    private void lowerBarAtEnd() {
        if (!mechanisms || endBarLowered) return;
        endBarLowered = true;
        robot.intakeBarSubsystem.lower();
        barDown = true;
        log("end of auto: lowering the intake bar");
    }

    // ================================================================ paths

    /** Straight line from the current pose, heading turning linearly; speedFraction < 1 caps the speed. */
    private void driveTo(Pose target, double speedFraction) {
        Pose start = follower.pose();
        if (start.distance(target) < DynamicAuto.MIN_PATH_LENGTH_IN) {
            follower.hold(target);
            followingPath = false;
            return;
        }
        Path path = Paths.line(start, target).linear(start, target);
        boolean limited = speedFraction > 0 && speedFraction < 1;
        if (limited) path = path.with(Constants.foresightConfig.maxPathSpeed.at(speedFraction));
        follower.follow(path);
        followingPath = true;
        log("  path line " + fmt(start) + " -> " + fmt(target) + String.format(", %.1f in", start.distance(target))
                + (limited ? String.format(", max %.2f of top speed", speedFraction) : ""));
    }

    private boolean pathDone() {
        if (!followingPath) return true;
        double t = stateTimer.milliseconds();
        if (t < DynamicAuto.PATH_MIN_MS) return false;
        if (!follower.isBusy() || follower.remainingDistance() < DynamicAuto.PATH_END_TOLERANCE_IN) {
            log("  path done in " + ms(stateTimer) + ", pose " + fmt(follower.pose()));
            return true;
        }
        if (t > DynamicAuto.PATH_TIMEOUT_MS) {
            Log.w(tag, String.format("[%5.2fs]   PATH TIMEOUT in %s, %.1f in short, pose %s",
                    matchTimer.seconds(), state, follower.remainingDistance(), fmt(follower.pose())));
            return true;
        }
        return false;
    }

    // ================================================================ telemetry / logging

    private static void dropSubsystemTelemetry(TelemetryManager panels) {
        if (!DynamicAuto.SHOW_TELEMETRY) panels.setLines(new ArrayList<>());
    }

    private void showStatus(TelemetryManager panels) {
        if (DynamicAuto.SHOW_TELEMETRY) {
            panels.addData("Time left (s)", String.format("%.1f", Math.max(0, DynamicAuto.AUTO_LENGTH_S - matchTimer.seconds())));
            panels.addData("State", state + ": " + status);
            panels.addData("Up CELL (Limelight)", side(HiveState.getUpCell(alliance)));
            panels.addData("Artifacts", robot.intakeSubsystem.getArtifactCount());
        }
        panels.update(telemetry);
    }

    private void setState(State next, String why) {
        log(state + " -> " + next + ": " + why + " | pose " + fmt(follower.pose()) + ", "
                + robot.intakeSubsystem.getArtifactCount() + " artifacts, up CELL " + side(HiveState.getUpCell(alliance)));
        state = next;
        status = why;
        loggedStatus = why;
        stateTimer.reset();
    }

    private void setStatus(String s) {
        status = s;
        if (!s.equals(loggedStatus)) {
            log("  " + state + ": " + s);
            loggedStatus = s;
        }
    }

    private void log(String msg) {
        Log.i(tag, String.format("[%5.2fs] %s", matchTimer.seconds(), msg));
    }

    /** A physical CELL (HiveState / LaunchTargets) as the driver's LEFT / RIGHT. */
    private Side side(HiveCell c) {
        return Side.of(c, alliance == AllianceColors.RED);
    }

    private static String fmt(Pose p) {
        return String.format("(%.1f, %.1f, %.0f°)", p.x(), p.y(), Math.toDegrees(p.heading()));
    }

    private static String ms(ElapsedTime t) {
        return String.format("%.0f ms", t.milliseconds());
    }

    private static long nowMs() {
        return System.nanoTime() / 1_000_000L;
    }
}
