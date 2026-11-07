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
 * "RightWall-L-G-L-F-P": fixed autonomous from the driver's RIGHT wall (red: the audience wall) (Launch, Garden, Launch, Flower, Park).
 * Poses are the red-frame Poses values, rotated 180 deg for blue like everything else:
 *  1. Start at RED_START_AUDIENCE_WALL, drive to RED_LAUNCH_AUDIENCE_CELL_2 and launch the 4
 *     preloads into that CELL (up at the start: red AUDIENCE CELL, blue FAR CELL).
 *  2. GARDEN: approach, bar down, in at the drive-in speed until full or INTAKE_TIMEOUT_MS, back out,
 *     rock ROCK_COUNT times (stops when full), bar up.
 *  3. Back to the same launch spot and check for a TIP (a Limelight reading from after arriving or
 *     up to TIP_READING_MAX_AGE_MS before). Not tipped: launch. Tipped / unconfirmed: no launch, and
 *     straight to park (no FLOWER: the robot still holds artifacts and may not possess more than 4).
 *     Same at step 1 if the CELL is already down.
 *  4. RIGHT FLOWER (alliance wall): approach, bar down if WALL_FLOWER_NEEDS_BAR, in, until full or
 *     FLOWER_WAIT_MS, back out, bar up. Skipped if the robot is already full.
 *  5. Park at RED_PARK_RIGHT (red: the audience end of the LOADING ZONE).
 * Intakes are skipped when the robot is already full. In the last PARK_WINDOW_S it parks unless it
 * is launching or can still launch at step 3 (then a GARDEN intake is cut short to go launch).
 *
 * INIT: operator gamepad (gamepad 2) B = RED, X = BLUE. Nothing moves until START.
 * Timing, speed, rocking and TIP-check settings are DynamicAuto's public static fields.
 * Logged with the class name as the tag.
 *
 * RightWallLGLFLP ("RightWall-L-G-L-F-L-P", launchAfterFlower() true) adds step 4b: after the FLOWER, back to
 * the launch spot, TIP check, launch if not tipped; then park.
 */
@Autonomous(name = "RightWall-L-G-L-F-P", group = "Autonomous", preselectTeleOp = "IncredibotsMecanumDrive")
public class RightWallLGLFP extends LinearOpMode {

    /** Log tag: the OpMode's class name (RightWallLGLFP / RightWallLGLFLP). */
    private final String tag = getClass().getSimpleName();

    /**
     * After the RIGHT FLOWER, go back to the launch spot, check for a TIP and launch again before
     * parking (RightWallLGLFLP). false = park straight from the FLOWER (RightWall-L-G-L-F-P).
     */
    protected boolean launchAfterFlower() {
        return false;
    }

    private String title() {
        return launchAfterFlower() ? "RightWall-L-G-L-F-L-P" : "RightWall-L-G-L-F-P";
    }

    private enum State {
        DRIVE_TO_LAUNCH, AT_LAUNCH, LAUNCHING,
        DRIVE_TO_INTAKE, LOWER_BAR, DRIVE_INTO_INTAKE, INTAKING, BACK_OUT_OF_INTAKE,
        ROCK_FORWARD, ROCK_BACK,
        DRIVE_TO_PARK, DONE
    }

    private Incredibot robot;
    private Follower follower;
    private Poses poses;
    private boolean mechanisms;
    private AllianceColors alliance = AllianceColors.RED;
    /** The CELL we launch into (up at the start). */
    private HiveCell cell;
    private LaunchPoint launchPoint;

    private State state = State.DONE;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime matchTimer = new ElapsedTime();
    private boolean followingPath = false;
    private boolean barDown = false;
    private boolean parkStarted = false;
    private boolean endBarLowered = false;
    /** 1 = the preload visit, 2 = back after the GARDEN (TIP check first). */
    private int launchVisit = 0;
    /** The intake point being worked (GARDEN, then RIGHT_FLOWER). */
    private IntakePoint intake;
    private long cellUpSinceMs = -1;
    private long arrivedAtLaunchMs = 0;
    private boolean tipChecked = false;
    private int rocksDone = 0;
    private Pose rockBase;
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
            panels.addLine("=== " + title() + " ===");
            if (!mechanisms) panels.addLine("DRIVE_ONLY_MODE: paths only, no launching or bar");
            panels.addData("Alliance (operator B red / X blue)", alliance);
            panels.addLine("Start RIGHT wall, launch from RIGHT spot 2,");
            panels.addLine("GARDEN, if not tipped: launch + RIGHT FLOWER"
                    + (launchAfterFlower() ? ", if not tipped: launch" : "") + "; park RIGHT");
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

        // The driver's RIGHT: start RED_START_AUDIENCE_WALL, launch RED_LAUNCH_AUDIENCE_CELL_2 into the
        // RIGHT CELL (up at the start), park RED_PARK_RIGHT (rotated for blue).
        launchPoint = LaunchPoint.RIGHT_SPOT_2;
        cell = launchPoint.cell(red);
        LaunchTargets.setForcedCell(cell);

        Pose start = poses.start(Side.RIGHT);
        robot.odometrySubsystem.resetPose(start.x(), start.y(), Math.toDegrees(start.heading()));

        matchTimer.reset();
        loggedUpCell = HiveState.getUpCell(alliance);
        log("START " + alliance + ", start pose " + fmt(start) + ", launching into the " + side(cell) + " CELL from "
                + fmt(poses.launch(launchPoint)) + ", mechanisms " + (mechanisms ? "on" : "off (DRIVE_ONLY_MODE)"));
        driveToLaunch();
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
                if (pathDone()) {
                    arrivedAtLaunchMs = nowMs();
                    cellUpSinceMs = -1;
                    tipChecked = false;
                    setState(State.AT_LAUNCH, "at " + launchPoint.label() + (launchVisit >= 2 ? ", checking for a TIP" : ""));
                }
                break;

            case AT_LAUNCH:
                updateAtLaunch();
                break;

            case LAUNCHING:
                if (!launchCommand.isScheduled()) {
                    log("launch " + launchVisit + " finished after " + ms(stateTimer) + ", "
                            + robot.intakeSubsystem.getArtifactCount() + " artifacts left");
                    afterLaunchVisit(true);
                }
                break;

            case DRIVE_TO_INTAKE:
                if (pathDone()) {
                    if (mechanisms && intake.needsBar()) {
                        robot.intakeBarSubsystem.lower();
                        barDown = true;
                        setState(State.LOWER_BAR, "at the " + intakeLabel() + " approach, lowering the bar");
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
                if (pathDone()) setState(State.INTAKING, "at the " + intakeLabel() + ", intaking up to "
                        + intakeTimeoutMs() + " ms (" + robot.intakeSubsystem.getArtifactCount() + " artifacts)");
                break;

            case INTAKING:
                if (robot.intakeSubsystem.isFullAndDoneIntaking() || stateTimer.milliseconds() >= intakeTimeoutMs()) {
                    log("intake done: " + (robot.intakeSubsystem.isFullAndDoneIntaking() ? "full" : "timed out")
                            + " after " + ms(stateTimer) + ", " + robot.intakeSubsystem.getArtifactCount() + " artifacts");
                    leaveIntake();
                }
                break;

            case BACK_OUT_OF_INTAKE:
                if (pathDone()) {
                    rocksDone = 0;
                    if (intake == IntakePoint.GARDEN && DynamicAuto.ROCK_COUNT > 0 && !robot.intakeSubsystem.isFull()
                            && !(parkTime() && noLaunchReason() == null)) {
                        Pose p = follower.pose();
                        double snapped = Math.toRadians(90.0 * Math.round(Math.toDegrees(p.heading()) / 90.0));
                        rockBase = new Pose(p.x(), p.y(), snapped);
                        rockForward();
                    } else {
                        finishIntake();
                    }
                }
                break;

            case ROCK_FORWARD:
                if (stopRockingIfFull()) break;
                if (pathDone(DynamicAuto.ROCK_END_TOLERANCE_IN)) {
                    driveTo(rockBase, DynamicAuto.ROCK_SPEED_FRACTION);
                    setState(State.ROCK_BACK, "rock " + (rocksDone + 1) + "/" + DynamicAuto.ROCK_COUNT + " back");
                }
                break;

            case ROCK_BACK:
                if (stopRockingIfFull()) break;
                if (pathDone(DynamicAuto.ROCK_END_TOLERANCE_IN)) {
                    rocksDone++;
                    if (rocksDone < DynamicAuto.ROCK_COUNT) rockForward();
                    else {
                        log("rocked " + rocksDone + " times, " + robot.intakeSubsystem.getArtifactCount() + " artifacts");
                        finishIntake();
                    }
                }
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

    /** At the launch spot: visit 1 launches if the CELL is up; later visits check for a TIP first. */
    private void updateAtLaunch() {
        if (!mechanisms) {
            afterLaunchVisit(false);
            return;
        }
        if (robot.intakeSubsystem.getArtifactCount() <= 0) {
            log("nothing to launch");
            afterLaunchVisit(false);
            return;
        }
        boolean up = HiveState.getUpCell(alliance) == cell;
        if (launchVisit >= 2 && !tipChecked) {
            if (stateTimer.milliseconds() < DynamicAuto.TIP_CHECK_MS) {
                setStatus("checking for a TIP (TIP_CHECK_MS " + DynamicAuto.TIP_CHECK_MS + ")");
                return;
            }
            long lastReadingMs = HiveState.getLastUpdateMs(alliance);
            boolean fresh = lastReadingMs > 0 && lastReadingMs >= arrivedAtLaunchMs - DynamicAuto.TIP_READING_MAX_AGE_MS;
            if (!fresh && stateTimer.milliseconds() < DynamicAuto.TIP_CHECK_TIMEOUT_MS) {
                setStatus("no recent Limelight reading of our HIVE, waiting (TIP_CHECK_TIMEOUT_MS "
                        + DynamicAuto.TIP_CHECK_TIMEOUT_MS + ")");
                return;
            }
            tipChecked = true;
            if (fresh) {
                log("TIP check: " + side(cell) + " CELL " + (up ? "up -> not tipped" : "down -> tipped")
                        + " (reading " + (nowMs() - lastReadingMs) + " ms old)");
            } else {
                Log.w(tag, String.format("[%5.2fs] TIP check UNCONFIRMED: no recent Limelight reading, %s",
                        matchTimer.seconds(), DynamicAuto.LAUNCH_IF_TIP_UNCONFIRMED && up
                                ? "launching anyway (LAUNCH_IF_TIP_UNCONFIRMED)" : "assuming it tipped"));
                if (!DynamicAuto.LAUNCH_IF_TIP_UNCONFIRMED) up = false;
            }
            if (!up) {
                log("HIVE tipped: no launch " + launchVisit);
                afterLaunchVisit(false);
                return;
            }
        } else if (!up && launchVisit == 1) {
            log(side(cell) + " CELL is already down: no preload launch");
            afterLaunchVisit(false);
            return;
        }

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
        setState(State.LAUNCHING, "launch " + launchVisit + ": " + side(cell) + " CELL is up, launcher "
                + (ready ? "ready" : "NOT ready after READY_TIMEOUT_MS, firing anyway"));
    }

    /**
     * After the launch spot: launched -> visit 1 GARDEN / visit 2 RIGHT FLOWER / visit 3 park; not
     * launched with artifacts aboard -> park.
     */
    private void afterLaunchVisit(boolean launched) {
        if (parkTime()) {
            startPark("park window reached at the launch spot");
            return;
        }
        // Didn't launch with artifacts aboard: no more intaking (never possess more than 4).
        if (!launched && robot.intakeSubsystem.getArtifactCount() > 0) {
            startPark("didn't launch with " + robot.intakeSubsystem.getArtifactCount()
                    + " artifacts aboard: no more intaking");
            return;
        }
        if (launchVisit >= 3) {
            startPark("all launches done");
            return;
        }
        startIntake(launchVisit == 1 ? IntakePoint.GARDEN : IntakePoint.RIGHT_FLOWER);
    }

    /** Done at an intake point: back to the launch spot, except FLOWER without launchAfterFlower() -> park. */
    private void finishIntake() {
        raiseBar();
        if (intake == IntakePoint.GARDEN || launchAfterFlower()) driveToLaunch();
        else startPark("done at the " + intakeLabel());
    }

    private void startIntake(IntakePoint p) {
        intake = p;
        if (robot.intakeSubsystem.isFull()) {
            log("full (" + robot.intakeSubsystem.getArtifactCount() + "): skipping the " + intakeLabel());
            finishIntake();
            return;
        }
        driveTo(poses.intakeApproach(p), 1.0);
        setState(State.DRIVE_TO_INTAKE, "to the " + intakeLabel() + " approach");
    }

    private void driveIntoIntake() {
        driveTo(poses.intake(intake), DynamicAuto.INTAKE_DRIVE_IN_SPEED_FRACTION);
        setState(State.DRIVE_INTO_INTAKE, "driving in at " + DynamicAuto.INTAKE_DRIVE_IN_SPEED_FRACTION + " speed");
    }

    /** Done intaking: back straight out first if the bar is down. */
    private void leaveIntake() {
        if (barDown) {
            driveTo(poses.intakeApproach(intake), 1.0);
            setState(State.BACK_OUT_OF_INTAKE, "bar is down: backing out to the approach pose");
        } else {
            finishIntake();
        }
    }

    private void driveToLaunch() {
        launchVisit++;
        driveTo(poses.launch(launchPoint), 1.0);
        setState(State.DRIVE_TO_LAUNCH, "to " + launchPoint.label() + " (visit " + launchVisit + ", "
                + robot.intakeSubsystem.getArtifactCount() + " artifacts)");
    }

    /** From the approach pose, ROCK_DISTANCE_IN toward the intake pose along the snapped heading. */
    private void rockForward() {
        double h = rockBase.heading();
        double ux = Math.round(Math.cos(h));
        double uy = Math.round(Math.sin(h));
        Pose target = new Pose(rockBase.x() + ux * DynamicAuto.ROCK_DISTANCE_IN,
                rockBase.y() + uy * DynamicAuto.ROCK_DISTANCE_IN, h);
        driveTo(target, DynamicAuto.ROCK_SPEED_FRACTION);
        setState(State.ROCK_FORWARD, "rock " + (rocksDone + 1) + "/" + DynamicAuto.ROCK_COUNT + " forward, bar down");
    }

    private boolean stopRockingIfFull() {
        if (!robot.intakeSubsystem.isFull()) return false;
        log("full (" + robot.intakeSubsystem.getArtifactCount() + ") while rocking: stopping");
        follower.hold(follower.pose());
        followingPath = false;
        finishIntake();
        return true;
    }

    private long intakeTimeoutMs() {
        return intake == IntakePoint.GARDEN ? DynamicAuto.INTAKE_TIMEOUT_MS : DynamicAuto.FLOWER_WAIT_MS;
    }

    private String intakeLabel() {
        return intake.label(alliance == AllianceColors.RED);
    }

    // ================================================================ park

    private boolean parkTime() {
        return matchTimer.seconds() >= DynamicAuto.AUTO_LENGTH_S - DynamicAuto.PARK_WINDOW_S;
    }

    /**
     * Why it can't launch soon (park now), or null if it can: the preload visit, an intake trip
     * that returns to launch (GARDEN; FLOWER with launchAfterFlower()) and the later launch visits
     * can still end in a launch (with artifacts and the CELL not known to be down).
     */
    private String noLaunchReason() {
        if (!mechanisms) return "mechanisms off";
        if (robot.intakeSubsystem.getArtifactCount() <= 0) return "no artifacts to launch";
        boolean atLaunch = state == State.DRIVE_TO_LAUNCH || state == State.AT_LAUNCH;
        boolean preloadVisit = launchVisit == 1 && intake == null && atLaunch;
        boolean laterVisit = launchVisit >= 2 && atLaunch;
        if (!preloadVisit && !onTripToLaunch() && !laterVisit) return "no launch left";
        if (preloadVisit) {
            if (state == State.AT_LAUNCH && HiveState.getUpCell(alliance) != cell) return side(cell) + " CELL is down";
            return null;
        }
        boolean tipCheckPending = state != State.AT_LAUNCH || !tipChecked;
        if (!tipCheckPending && HiveState.getUpCell(alliance) != cell) return side(cell) + " CELL is down";
        return null;
    }

    /** On an intake trip that comes back to launch: GARDEN after visit 1, FLOWER after visit 2 (launchAfterFlower()). */
    private boolean onTripToLaunch() {
        if (state == State.DRIVE_TO_LAUNCH || state == State.AT_LAUNCH) return false;
        return (launchVisit == 1 && intake == IntakePoint.GARDEN)
                || (launchVisit == 2 && intake == IntakePoint.RIGHT_FLOWER && launchAfterFlower());
    }

    /** Park window on an intake trip that comes back to launch, with artifacts: cut it short and go launch. */
    private boolean hurryToLaunch() {
        if (!onTripToLaunch()) return false;
        switch (state) {
            case DRIVE_TO_INTAKE:
                log("park window: skipping the " + intakeLabel() + " to go launch");
                finishIntake();
                return true;
            case LOWER_BAR:
            case DRIVE_INTO_INTAKE:
            case INTAKING:
                log("park window: cutting the " + intakeLabel() + " short to go launch");
                leaveIntake();
                return true;
            case ROCK_FORWARD:
            case ROCK_BACK:
                log("park window: stopping the rocking to go launch");
                follower.hold(follower.pose());
                followingPath = false;
                finishIntake();
                return true;
            default:
                return false;
        }
    }

    private void startPark(String reason) {
        parkStarted = true;
        raiseBar();
        driveTo(poses.park(Side.RIGHT), 1.0);
        setState(State.DRIVE_TO_PARK, "parking RIGHT: " + reason);
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
        return pathDone(DynamicAuto.PATH_END_TOLERANCE_IN);
    }

    private boolean pathDone(double endToleranceIn) {
        if (!followingPath) return true;
        double t = stateTimer.milliseconds();
        if (t < DynamicAuto.PATH_MIN_MS) return false;
        if (!follower.isBusy() || follower.remainingDistance() < endToleranceIn) {
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
