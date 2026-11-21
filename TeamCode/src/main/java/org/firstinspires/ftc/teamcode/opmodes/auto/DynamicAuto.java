package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.algorithm.ForesightConfig;
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

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Deque;
import java.util.List;

/**
 * Cycle autonomous: launch, intake, come back, launch again, moving to the next launch point only
 * once the Limelight has seen the current CELL TIP. Configured on the Driver Station in INIT
 * ({@link AutoPlan}): alliance, start side, a sequence of launch points and a sequence of intake
 * points, each reached by a straight line or a spline.
 *
 * LEFT / RIGHT (start, launch points, CELLS, park) are as seen by the drivers from behind their
 * alliance wall ({@link Side}), so the same plan means the same thing for red and blue: RIGHT is
 * red's audience-wall side, and blue's matching side (physically the far wall). Only LB / RB
 * changes between alliances. GARDEN and FLOWERS are named per alliance already.
 *
 * The cycle:
 *  1. Drive to the current launch point (the first one, with the 4 preloads).
 *  2. At the launch point:
 *     - its CELL is up (HiveState, kept current by the Limelight): wait for the launcher to be
 *       ready (up to READY_TIMEOUT_MS) and launch;
 *     - its CELL is down and we have already launched here: the HIVE tipped, so the current launch
 *       point is done; go to step 1 with the next launch point;
 *     - its CELL is down and we haven't launched here yet: wait for it to come up.
 *  3. After a launch, drive to the next intake point, intake (bar down if the point needs it) until
 *     full or INTAKE_TIMEOUT_MS. If the bar is down, back straight out to the approach pose; at the
 *     GARDEN also rock forward and back ROCK_COUNT times (bar still down, so the bar keeps the
 *     elements in front of the intake while they get pulled in); then raise it. Then go back to the
 *     current launch point (step 2). The GARDEN currently uses no bar (Poses.GARDEN_NEEDS_BAR false):
 *     it drives in, then rocks from the intake pose (back ROCK_DISTANCE_IN, forward again) up to
 *     ROCK_COUNT times, stopping as soon as the count reads 4; no back-out.
 *     With 0 artifacts it never heads for a launch point (REFILL_BEFORE_LAUNCH): it goes to the
 *     closest unused intake point first.
 * With FINAL_LAUNCH_AT_LEFT_SPOT_3, the trip after the LAST intake point goes to LEFT spot 3
 * (straight line) for the final launch instead of the current launch point.
 * When a sequence runs out (no next launch or intake point), the robot parks.
 *
 * Park: the OpMode keeps its own AUTO_LENGTH_S clock from START. In the last PARK_WINDOW_S
 * seconds it drives to the LOADING ZONE if it can't launch soon (no artifacts, no launch point
 * left, or waiting at a launch point whose CELL is down); otherwise launching wins: it cuts any intake short and goes to launch,
 * parking after the launch if time remains.
 *
 * INIT controls (gamepad 1 or 2); nothing is commanded until START (G304.H / G403). At START the
 * intake bar is commanded up and held there except at intake points that need it:
 *   LB / RB          - alliance RED / BLUE
 *   Dpad up / down   - start on the RIGHT (red: audience wall) / LEFT (red: far wall)
 *   L / R stick press - park at the LEFT / RIGHT end of the LOADING ZONE
 *   Left stick up/down  - delay after START, +/- 0.5 s
 *   Right stick up/down - delay after the LAST point of the sequence being edited, +/- 0.5 s
 *                         (launch point: waits after each launch there; intake point: after intaking)
 *   L / R trigger    - between launch points across the HIVE, go AROUND it / THROUGH it
 *                      (spline through Poses.RED_CONTROL_AROUND_HIVE_AUDIENCE/_FAR / straight lines via
 *                      Poses.RED_THROUGH_HIVE_WAYPOINT_AUDIENCE and _FAR;
 *                      replaces the leg's own line/spline for that move only)
 *   Y                - switch between editing the LAUNCH and the INTAKE sequence
 *   Dpad left/right  - choose a point
 *   A / X            - append it, reached by a straight LINE / a SPLINE
 *   Back             - remove the last point of the sequence being edited
 *   B                - clear the sequence being edited
 */
@Configurable
@Autonomous(name = "DynamicAuto", group = "Autonomous", preselectTeleOp = "IncredibotsMecanumDrive")
public class DynamicAuto extends LinearOpMode {

    // ---------------------------------------------------------------- tunables (Panels)
    public static double AUTO_LENGTH_S = 30;
    /** Park when this little time is left (unless launching). */
    public static double PARK_WINDOW_S = 3;
    public static boolean PARK_WITH_SPLINE = false;

    /** Longest wait for LaunchSubsystem.isReadyToLaunch() once the CELL is up; then it fires anyway. */
    public static long READY_TIMEOUT_MS = 1500;
    /**
     * Settle before launching (2026-11-20): the robot was still moving 1-3 in when a launch started,
     * which can shake artifacts out. A launch also waits until the pose has stayed within
     * SETTLE_MAX_MOVE_IN and SETTLE_MAX_TURN_DEG for SETTLE_MS (SettleCheck), giving up after
     * SETTLE_TIMEOUT_MS. READY_TIMEOUT_MS still fires anyway. Also used by LeftWallAuto.
     */
    public static boolean SETTLE_BEFORE_LAUNCH = true;
    public static double SETTLE_MAX_MOVE_IN = 0.5;
    public static double SETTLE_MAX_TURN_DEG = 1.0;
    public static long SETTLE_MS = 150;
    public static long SETTLE_TIMEOUT_MS = 1000;
    /** Back at a launch point we've already launched from: look at the HIVE this long before deciding if it tipped. */
    public static long TIP_CHECK_MS = 300;
    /**
     * Back at a launch point we've already launched from, the TIP decision needs a Limelight reading
     * of our HIVE taken after arriving (HiveState is only the last known state). Wait up to this
     * long for one.
     */
    public static long TIP_CHECK_TIMEOUT_MS = 500;
    /**
     * The TIP check also accepts a Limelight reading of our HIVE taken up to this long BEFORE
     * arriving (e.g. on the drive in, when the tags are often in view and drop out at the spot).
     * 0 = only readings taken after arriving.
     */
    public static long TIP_READING_MAX_AGE_MS = 1000;
    /**
     * No fresh reading by TIP_CHECK_TIMEOUT_MS: true = launch anyway (the CELL may have tipped),
     * false = treat the CELL as tipped and move on to the next launch point (or park if it was the
     * last one). Default false: launching at a CELL that may be down risks a G417 violation.
     */
    public static boolean LAUNCH_IF_TIP_UNCONFIRMED = false;

    public static long INTAKE_TIMEOUT_MS = 1200;
    public static long BAR_LOWER_TIMEOUT_MS = 600;

    /** A path counts as done once this close to its end (or when Pedro says it's done). */
    public static double PATH_END_TOLERANCE_IN = 2.0;
    /** Ignore "done" for this long after starting a path (lets the follower latch onto it). */
    public static long PATH_MIN_MS = 250;
    /** Give up on a path after this long (e.g. blocked by another robot). */
    public static long PATH_TIMEOUT_MS = 5000;
    /**
     * After the park path, stop holding the park pose (zero power, motors in brake) once Pedro says
     * the path is done or PARK_RELEASE_MS after arriving, whichever is first. false = hold until the
     * OpMode ends (jiggles against the wall). Also used by LeftWallAuto.
     */
    public static boolean RELEASE_HOLD_AFTER_PARK = true;
    public static long PARK_RELEASE_MS = 500;
    /** Closer than this, just hold the target pose instead of building a path. */
    public static double MIN_PATH_LENGTH_IN = 1.0;
    /**
     * Speed for the straight drive from an intake point's approach pose into the intake pose, as a
     * fraction of the drive's tuned top forward speed (Constants.foresightConfig
     * maxAchievableForwardVelocity). 1 = no limit.
     */
    public static double INTAKE_DRIVE_IN_SPEED_FRACTION = 0.4;
    /**
     * At an intake point's approach pose, turn in place until facing the approach heading within
     * this before lowering the bar and driving in (e.g. after a path whose heading ended off).
     * Gives up after PATH_TIMEOUT_MS.
     */
    public static double INTAKE_APPROACH_HEADING_TOLERANCE_DEG = 10;

    /**
     * After backing out of an intake point with the bar down: rock forward (toward the intake pose)
     * and back to the approach pose this many times before raising the bar. 0 = no rocking.
     */
    public static int ROCK_COUNT = 2;
    /**
     * FLOWERS don't rock: the bar funnels the POLLEN into the intake, so the robot stays at the
     * FLOWER (intake pose) this long, or until full, instead of INTAKE_TIMEOUT_MS. Then it backs out
     * to the approach pose and raises the bar.
     */
    public static long FLOWER_WAIT_MS = 1200;
    /**
     * How far forward each rock goes. Rocks run along one field axis only: X when the robot faces
     * 0/180 deg, Y when it faces 90/270 (heading snapped to the nearest 90), from where the robot
     * stopped after backing out, so the other coordinate never changes.
     */
    public static double ROCK_DISTANCE_IN = 4.0;
    /** Rock speed as a fraction of the drive's top forward speed, like INTAKE_DRIVE_IN_SPEED_FRACTION. */
    public static double ROCK_SPEED_FRACTION = 0.4;
    /** A rock leg counts as done this close to its end (smaller than PATH_END_TOLERANCE_IN; the legs are short). */
    public static double ROCK_END_TOLERANCE_IN = 0.75;
    /**
     * Driving into an intake point or rocking: the leg counts as done once the robot has moved slower
     * than STALL_SPEED_IN_PER_SEC for STALL_MS (pushing against the wall, the POLLEN or a FLOWER), not
     * just at the path's end. The GARDEN intake pose is past the wall on purpose, so without this the
     * drive-in pushed for the whole PATH_TIMEOUT_MS (5 s) before intaking / rocking. Also used by
     * LeftWallAuto.
     */
    public static double STALL_SPEED_IN_PER_SEC = 2.0;
    /**
     * Backing out of an intake point with the bar down: the back-out counts as done this far from the
     * approach pose, so the next path starts while the robot is still moving instead of after it has
     * slowed to a stop at the approach pose (that stop looked like a wait at the FLOWER). The bar is
     * clear of the FLOWER by then. Also used by LeftWallAuto.
     */
    public static double BACK_OUT_END_TOLERANCE_IN = 6.0;
    public static long STALL_MS = 300;

    /**
     * true = after the LAST intake point in the plan, the final launch is at LEFT spot 3
     * (Poses.RED_LAUNCH_FAR_CELL_3), reached by a straight line with linear heading, instead of
     * going back to the plan's current launch point. Launches there only if the LEFT CELL is up,
     * like any first visit; then parks. false = the plan's launch point as before.
     */
    public static boolean FINAL_LAUNCH_AT_LEFT_SPOT_3 = true;
    /**
     * LEFT spot 3 is partly inside the LOADING ZONE, so parking from there is no move at all: when
     * the auto would park while the robot is within LEFT_SPOT_3_PARK_RADIUS_IN of it, it just holds
     * and is done (2026-11-20). false = drive to the plan's park pose as before.
     */
    public static boolean LEFT_SPOT_3_COUNTS_AS_PARK = true;
    public static double LEFT_SPOT_3_PARK_RADIUS_IN = 6.0;
    /**
     * Never drive to a launch point empty (2026-11-20): when the robot would head for a launch point
     * with 0 artifacts (an intake point that picked nothing up, or moving on after a TIP), it goes
     * to the CLOSEST unused intake point first, then on to that launch point. With no intake point
     * left it parks. false = follow the plan order regardless.
     */
    public static boolean REFILL_BEFORE_LAUNCH = true;
    /**
     * true (as on 2026-11-07, and what LeftWallAuto does): Pedro holds the pose at
     * every path end and wherever the auto stays put. false = no hold: a path end stops with the
     * motors braked (Follower.holdEnd off, Mecanum stop = brake) and every "hold here" stops instead,
     * except the turn in place at an intake approach. Tried false on 2026-11-20 for the wobble; it
     * didn't fix it.
     */
    public static boolean HOLD_POSITION = true;
    /**
     * To the rear-wall FLOWER from the audience half (e.g. RIGHT spot 2): always a spline through
     * Poses.RED_CONTROL_SIDE_FLOWER_FROM_AUDIENCE, whatever the plan leg says; a line hits the HIVE
     * frame (2026-11-20). false = the plan leg's line / spline as before.
     */
    public static boolean SIDE_FLOWER_SPLINE_FROM_AUDIENCE = true;

    /**
     * true = show informational telemetry: the subsystems' readouts (drive, odometry, intake,
     * launcher, Limelight) in INIT and during the run, plus the run status (state, points, time
     * left). false = INIT shows only the plan editor, and the run shows nothing. The log
     * (tag DynamicAuto) is written either way.
     */
    public static boolean SHOW_TELEMETRY = false;

    private static final String TAG = "DynamicAuto";

    private enum State {
        DRIVE_TO_LAUNCH, AT_LAUNCH, LAUNCHING,
        DRIVE_TO_INTAKE, LOWER_BAR, DRIVE_INTO_INTAKE, INTAKING, BACK_OUT_OF_INTAKE,
        ROCK_FORWARD, ROCK_BACK,
        WAIT, DRIVE_TO_PARK, DONE
    }

    private enum Editing { LAUNCH, INTAKE }

    // ---------------------------------------------------------------- run state
    private Incredibot robot;
    private Follower follower;
    private Poses poses;
    private boolean mechanisms;

    private List<AutoPlan.Leg<LaunchPoint>> launchLegs;
    private List<AutoPlan.Leg<IntakePoint>> intakeLegs;
    private int launchIndex = 0;
    private int intakeIndex = 0;
    /** We have launched from the current launch point at least once. */
    private boolean launchedHere = false;

    private State state = State.DONE;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime matchTimer = new ElapsedTime();
    private boolean followingPath = false;
    private final SettleCheck settle = new SettleCheck();
    private long cellUpSinceMs = -1;
    /** nowMs() when the robot reached the current launch point. */
    private long arrivedAtLaunchMs = 0;
    /** The TIP check at this visit has been decided (so the logs and waits run once). */
    private boolean tipChecked = false;
    /** The TIP check at this visit got a fresh Limelight reading. */
    private boolean tipConfirmed = false;
    private boolean barDown = false;
    /** Rocks finished at the current intake point. */
    private int rocksDone = 0;
    /** Where rocking starts and returns to (pose after backing out, heading snapped to 90 deg). */
    private Pose rockBase;
    /** Rocking from the intake pose (GARDEN without the bar) rather than from the approach pose. */
    private boolean rockFromIntake = false;
    /** stateTimer time the robot became slower than STALL_SPEED_IN_PER_SEC this state, or -1. */
    private double slowSinceMs = -1;
    private LaunchBallsCommand launchCommand;
    /** DRIVE_TO_LAUNCH through the HIVE: waypoints still to visit, then the launch pose (last). */
    private final Deque<Pose> launchRoute = new ArrayDeque<>();
    private boolean parkStarted = false;
    private boolean parkHoldReleased = false;
    /** The end-of-auto bar lowering has been commanded. */
    /** WAIT state: how long, and what to do after. */
    private long waitMs = 0;
    private Runnable afterWait;
    private String status = "";
    /** Last status / up CELL written to the log, so waiting states log once, not every loop. */
    private String loggedStatus = "";
    private HiveCell loggedUpCell = null;

    @Override
    public void runOpMode() {
        Constants.foresightConfig.maxPathSpeed.set(ForesightConfig.Constraint.NONE);
        TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
        robot = new Incredibot(hardwareMap, panels);
        // The 4 preloads are in before INIT: count them now so the count light shows 4 during INIT
        // (Incredibot.initAuto() sets it again at START). Otherwise the sensor-4 leftover rule reads 1.
        robot.intakeSubsystem.setArtifactCount(org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem.MAX_ARTIFACTS);
        follower = robot.driveSubsystem.getFollower();
        follower.holdEnd.set(HOLD_POSITION); // this OpMode's follower only (a new one each OpMode)
        mechanisms = !Incredibot.DRIVE_ONLY_MODE;

        configureDuringInit(panels);
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

    // ================================================================ INIT: plan editing

    private void configureDuringInit(TelemetryManager panels) {
        Editing editing = Editing.LAUNCH;
        int launchChoice = 0;
        int intakeChoice = 0;
        LaunchPoint[] launchOptions = LaunchPoint.values();
        IntakePoint[] intakeOptions = IntakePoint.values();

        while (opModeInInit()) {
            if (pressed(Button.LB)) AutoPlan.alliance = AllianceColors.RED;
            if (pressed(Button.RB)) AutoPlan.alliance = AllianceColors.BLUE;
            if (pressed(Button.DPAD_UP)) AutoPlan.startSide = Side.RIGHT;
            if (pressed(Button.DPAD_DOWN)) AutoPlan.startSide = Side.LEFT;
            if (pressed(Button.LEFT_STICK)) AutoPlan.parkSide = Side.LEFT;
            if (pressed(Button.RIGHT_STICK)) AutoPlan.parkSide = Side.RIGHT;
            AutoPlan.startDelayMs = AutoPlan.clampDelay(AutoPlan.startDelayMs + stickSteps(true) * AutoPlan.DELAY_STEP_MS);
            int lastDelaySteps = stickSteps(false);
            if (triggerPressed(true)) AutoPlan.hiveRouteAround = true;
            if (triggerPressed(false)) AutoPlan.hiveRouteAround = false;
            if (pressed(Button.Y)) editing = editing == Editing.LAUNCH ? Editing.INTAKE : Editing.LAUNCH;

            int step = (pressed(Button.DPAD_RIGHT) ? 1 : 0) - (pressed(Button.DPAD_LEFT) ? 1 : 0);
            boolean addLine = pressed(Button.A);
            boolean addSpline = pressed(Button.X);
            boolean removeLast = pressed(Button.BACK);
            boolean clear = pressed(Button.B);

            if (editing == Editing.LAUNCH) {
                launchChoice = Math.floorMod(launchChoice + step, launchOptions.length);
                if (addLine || addSpline) AutoPlan.addLaunch(launchOptions[launchChoice], addSpline);
                if (removeLast) AutoPlan.removeLast(AutoPlan.launchPoints);
                AutoPlan.adjustLastDelay(AutoPlan.launchPoints, lastDelaySteps);
                if (clear) AutoPlan.launchPoints.clear();
            } else {
                intakeChoice = Math.floorMod(intakeChoice + step, intakeOptions.length);
                if (addLine || addSpline) AutoPlan.addIntake(intakeOptions[intakeChoice], addSpline);
                if (removeLast) AutoPlan.removeLast(AutoPlan.intakePoints);
                AutoPlan.adjustLastDelay(AutoPlan.intakePoints, lastDelaySteps);
                if (clear) AutoPlan.intakePoints.clear();
            }

            CommandScheduler.getInstance().run(); // subsystem periodic() only (lights, pose); nothing moves
            dropSubsystemTelemetry(panels);

            boolean red = AutoPlan.isRed();
            String choice = editing == Editing.LAUNCH
                    ? launchOptions[launchChoice].label()
                    : describe(intakeOptions[intakeChoice], red);
            panels.addLine("=== DYNAMIC AUTO (cycle) ===");
            if (!mechanisms) panels.addLine("DRIVE_ONLY_MODE: paths only, no launching or bar");
            panels.addData("Alliance (LB red / RB blue)", AutoPlan.alliance);
            panels.addData("Start (dpad up RIGHT / down LEFT)", AutoPlan.startSide);
            panels.addData("Start delay (left stick up/down)", String.format("%.1f s", AutoPlan.startDelayMs / 1000.0));
            panels.addData("Park (L/R stick press)", AutoPlan.parkLabel());
            panels.addLine("LEFT / RIGHT: as seen by the drivers, from behind your alliance wall");
            panels.addData("Launch-to-launch (L/R trigger)", AutoPlan.hiveRouteLabel());
            panels.addData("Up CELL at start", sideName(startUpCell()));
            panels.addData("Final launch (after last intake)", FINAL_LAUNCH_AT_LEFT_SPOT_3
                    ? LaunchPoint.LEFT_SPOT_3.label() + " (line)" : "plan's launch point");
            panels.addLine("-------------------------------");
            panels.addData("Editing (Y)", editing);
            panels.addData("Point (dpad L/R)", choice);
            panels.addLine("A = add (line), X = add (spline), Back = remove last, B = clear");
            panels.addLine("Right stick up/down = delay after the last point");
            panels.addLine("------ LAUNCH points ------");
            listLaunch(panels);
            panels.addLine("------ INTAKE points ------");
            listIntake(panels, red);
            String warning = planWarning();
            if (!warning.isEmpty()) panels.addLine(">> " + warning);
            panels.update(telemetry);
        }
    }

    private void listLaunch(TelemetryManager panels) {
        if (AutoPlan.launchPoints.isEmpty()) panels.addLine("(none)");
        for (int i = 0; i < AutoPlan.launchPoints.size(); i++) {
            AutoPlan.Leg<LaunchPoint> leg = AutoPlan.launchPoints.get(i);
            panels.addLine((i + 1) + ". " + leg.point.label() + " [" + leg.moveLabel() + "]");
        }
    }

    private void listIntake(TelemetryManager panels, boolean red) {
        if (AutoPlan.intakePoints.isEmpty()) panels.addLine("(none)");
        for (int i = 0; i < AutoPlan.intakePoints.size(); i++) {
            AutoPlan.Leg<IntakePoint> leg = AutoPlan.intakePoints.get(i);
            panels.addLine((i + 1) + ". " + describe(leg.point, red) + " [" + leg.moveLabel() + "]");
        }
    }

    private static String describe(IntakePoint p, boolean red) {
        return p.label(red) + (p.needsBar() ? " (bar)" : "");
    }

    private static String planWarning() {
        if (AutoPlan.launchPoints.isEmpty()) return "No launch points: the robot will only park.";
        if (AutoPlan.launchPoints.get(0).point.cell(AutoPlan.isRed()) != startUpCell()) {
            return "First launch CELL is down at the start: the robot waits there until it is up.";
        }
        return "";
    }

    /** A physical CELL (HiveState / LaunchTargets) as the driver's LEFT / RIGHT. */
    private static String sideName(HiveCell cell) {
        return Side.of(cell, AutoPlan.isRed()).toString();
    }

    /** The up CELL of our HIVE as LEFT / RIGHT. */
    private static String upSide() {
        return sideName(HiveState.getUpCell(AutoPlan.alliance));
    }

    private static HiveCell startUpCell() {
        return AutoPlan.isRed() ? HiveState.RED_START_UP_CELL : HiveState.BLUE_START_UP_CELL;
    }

    // ================================================================ START

    private void beginMatch() {
        boolean red = AutoPlan.isRed();
        CrossOpModeStorage.allianceColor = AutoPlan.alliance; // TeleOp aims at our HIVE afterwards
        poses = new Poses(red);
        launchLegs = new ArrayList<>(AutoPlan.launchPoints);
        intakeLegs = new ArrayList<>(AutoPlan.intakePoints);

        robot.initialize(Incredibot.OpModeType.AUTO, null, null); // preload count 4, HiveState reset
        follower.holdEnd.set(true);

        // Hold the intake bar up from START; it stays up except while at an intake point that
        // needs it. Not done in INIT: the bar servos stay unpowered until START.
        if (mechanisms) robot.intakeBarSubsystem.raise();

        Pose start = poses.start(AutoPlan.startSide);
        robot.odometrySubsystem.resetPose(start.x(), start.y(), Math.toDegrees(start.heading()));

        matchTimer.reset();
        log("START " + AutoPlan.alliance + ", start " + AutoPlan.startSide + ", start pose " + fmt(start)
                + ", up CELL " + upSide() + " (physical " + HiveState.getUpCell(AutoPlan.alliance) + ")"
                + String.format(", start delay %.1f s", AutoPlan.startDelayMs / 1000.0)
                + ", park " + AutoPlan.parkLabel() + ", launch-to-launch " + AutoPlan.hiveRouteLabel()
                + ", mechanisms " + (mechanisms ? "on" : "off (DRIVE_ONLY_MODE)"));
        log("PLAN launch: " + launchPlanText() + " | intake: " + intakePlanText(red));
        loggedUpCell = HiveState.getUpCell(AutoPlan.alliance);
        if (launchLegs.isEmpty()) {
            setState(State.DONE, "no launch points: waiting to park");
        } else {
            waitThen(AutoPlan.startDelayMs, "after START", this::driveToLaunchPoint);
        }
    }

    // ================================================================ cycle

    private void update() {
        HiveCell upCell = HiveState.getUpCell(AutoPlan.alliance);
        if (upCell != loggedUpCell) {
            log("HIVE up CELL changed " + (loggedUpCell == null ? "?" : sideName(loggedUpCell)) + " -> "
                    + sideName(upCell) + " (Limelight)");
            loggedUpCell = upCell;
        }

        if (!parkStarted && state != State.LAUNCHING && parkTime()) {
            if (launchTakesPriority()) {
                // Launching beats parking: carry artifacts on to the launch point, cutting any intake short.
                if (hurryToLaunch()) return;
            } else {
                startPark("park window: " + String.format("%.1f", AUTO_LENGTH_S - matchTimer.seconds())
                        + " s left (PARK_WINDOW_S " + PARK_WINDOW_S + "), " + noLaunchReason());
                return;
            }
        }

        switch (state) {
            case DRIVE_TO_LAUNCH:
                if (!launchRoute.isEmpty() && pathDone()) {
                    // Through the HIVE: on to the next waypoint, or the launch pose.
                    Pose next = launchRoute.poll();
                    log("  waypoint reached, on to " + (launchRoute.isEmpty() ? "the launch pose" : "the next waypoint"));
                    driveTo(next, null, false);
                    stateTimer.reset(); // per-leg PATH_MIN_MS / PATH_TIMEOUT_MS
                    break;
                }
                if (pathDone()) {
                    cellUpSinceMs = -1;
                    arrivedAtLaunchMs = nowMs();
                    tipChecked = false;
                    tipConfirmed = false;
                    setState(State.AT_LAUNCH, "arrived at " + currentLaunch().point.label()
                            + (launchedHere ? ", already launched here: will check for a TIP" : ", first visit"));
                }
                break;

            case AT_LAUNCH:
                updateAtLaunchPoint();
                break;

            case LAUNCHING:
                if (!launchCommand.isScheduled()) {
                    launchedHere = true;
                    log("launch finished after " + ms(stateTimer) + ", " + robot.intakeSubsystem.getArtifactCount()
                            + " artifacts left");
                    if (parkTime()) startPark("park window reached during the launch");
                    else waitThen(currentLaunch().delayMs, "after launching at " + currentLaunch().point.label(),
                            this::driveToIntakePoint);
                }
                break;

            case DRIVE_TO_INTAKE:
                if (followingPath && !pathDone()) break;
                followingPath = false; // path over (Pedro holds its end pose): turn in place if needed
                if (approachHeadingReached()) {
                    if (mechanisms && currentIntake().point.needsBar()) {
                        robot.intakeBarSubsystem.lower();
                        barDown = true;
                        setState(State.LOWER_BAR, "at approach, " + currentIntake().point + " needs the bar: lowering it");
                    } else {
                        driveIntoIntakePoint();
                    }
                }
                break;

            case LOWER_BAR:
                if (robot.intakeBarSubsystem.isAtTarget() || stateTimer.milliseconds() >= BAR_LOWER_TIMEOUT_MS) {
                    log("bar " + (robot.intakeBarSubsystem.isAtTarget() ? "down" : "lower timed out (BAR_LOWER_TIMEOUT_MS)")
                            + " after " + ms(stateTimer));
                    driveIntoIntakePoint();
                }
                break;

            case DRIVE_INTO_INTAKE:
                // At a FLOWER, a full count while still driving in ends the drive-in: INTAKING sees full
                // on the next loop and leaves straight away (it used to push on until the path ended).
                if (isFlower(currentIntake().point) && robot.intakeSubsystem.isFull()) {
                    log("full (" + robot.intakeSubsystem.getArtifactCount() + ") while driving into the FLOWER");
                    setState(State.INTAKING, "full during the drive-in: leaving the FLOWER");
                } else if (pathDone() || stalled()) setState(State.INTAKING, "at intake pose, intaking"
                        + (isFlower(currentIntake().point) ? " (FLOWER: waiting up to " + FLOWER_WAIT_MS + " ms)" : "") + " ("
                        + robot.intakeSubsystem.getArtifactCount() + " artifacts)");
                break;

            case INTAKING:
                boolean flower = isFlower(currentIntake().point);
                long intakeTimeoutMs = flower ? FLOWER_WAIT_MS : INTAKE_TIMEOUT_MS;
                // At a FLOWER, move on as soon as the count reads full: the intake's FULL_RUN_ON_MS run-on
                // keeps pulling the last one in while the robot backs out (it used to wait out that 1 s here).
                boolean full = flower ? robot.intakeSubsystem.isFull() : robot.intakeSubsystem.isFullAndDoneIntaking();
                if (full || stateTimer.milliseconds() >= intakeTimeoutMs) {
                    log("intake done: " + (full ? "full"
                            : flower ? "FLOWER wait over (FLOWER_WAIT_MS)" : "timed out (INTAKE_TIMEOUT_MS)")
                            + " after " + ms(stateTimer) + ", "
                            + robot.intakeSubsystem.getArtifactCount() + " artifacts");
                    if (barDown) {
                        // Back straight out to the approach pose with the bar still down, then raise it.
                        driveTo(poses.intakeApproach(currentIntake().point), null, false);
                        setState(State.BACK_OUT_OF_INTAKE, "bar is down: backing out to the approach pose");
                    } else if (!flower && !full && ROCK_COUNT > 0 && !(parkTime() && launchTakesPriority())) {
                        // GARDEN without the bar: the robot is at the intake pose against the wall, so each
                        // rock goes back ROCK_DISTANCE_IN and forward to the intake pose again.
                        startRocking(true);
                    } else {
                        if (!flower) log("no rocking: " + (full ? "full" : ROCK_COUNT <= 0 ? "ROCK_COUNT is 0" : "park window"));
                        finishIntakePoint();
                    }
                }
                break;

            case BACK_OUT_OF_INTAKE:
                if (pathDone(BACK_OUT_END_TOLERANCE_IN)) {
                    rocksDone = 0;
                    if (parkTime() && launchTakesPriority()) {
                        log("park window: skipping the rocking to go launch");
                        finishIntakePoint();
                        break;
                    }
                    if (isFlower(currentIntake().point)) {
                        finishIntakePoint(); // FLOWERS don't rock: bar up, on to the launch point
                        break;
                    }
                    if (ROCK_COUNT <= 0 || robot.intakeSubsystem.isFull()) {
                        if (ROCK_COUNT > 0) log("full (" + robot.intakeSubsystem.getArtifactCount() + "): skipping the rocking");
                        finishIntakePoint();
                        break;
                    }
                    startRocking(false);
                }
                break;

            case ROCK_FORWARD:
                if (stopRockingIfFull()) break;
                if (pathDone(ROCK_END_TOLERANCE_IN) || stalled()) {
                    if (rockFromIntake) rockDone(); // forward is the second leg of a rock from the intake pose
                    else rockBack();
                }
                break;

            case ROCK_BACK:
                if (stopRockingIfFull()) break;
                if (pathDone(ROCK_END_TOLERANCE_IN) || stalled()) {
                    if (rockFromIntake) rockForward();
                    else rockDone();
                }
                break;

            case WAIT:
                if (stateTimer.milliseconds() >= waitMs) {
                    Runnable next = afterWait;
                    afterWait = null;
                    log("delay done after " + ms(stateTimer));
                    next.run();
                }
                break;

            case DRIVE_TO_PARK:
                if (pathDone()) {
                    setState(State.DONE, "parked at " + String.format("%.1f s", matchTimer.seconds()));
                }
                break;

            case DONE:
                // Parked: stop holding. Pedro's end-of-path hold would keep correcting toward the park
                // pose until the OpMode ends, and it jiggled there (a park pose ~1 in off the wall, after a
                // full auto's odometry drift, pushed into the wall). Zero power + brake instead.
                if (parkStarted && !parkHoldReleased && RELEASE_HOLD_AFTER_PARK
                        && (!follower.isBusy() || stateTimer.milliseconds() >= PARK_RELEASE_MS)) {
                    follower.manual(0, 0, 0);
                    parkHoldReleased = true;
                    log("parked: hold released (zero power, brake) | pose " + fmt(follower.pose()));
                }
                break;
        }
    }

    private void updateAtLaunchPoint() {
        HiveCell cell = currentLaunch().point.cell(AutoPlan.isRed());

        // Back after intaking: give the Limelight a moment on the HIVE before deciding.
        // HiveState only holds the last confirmed reading; require one taken since arriving here.
        boolean cellUp = HiveState.getUpCell(AutoPlan.alliance) == cell;
        if (launchedHere && !tipChecked) {
            if (stateTimer.milliseconds() < TIP_CHECK_MS) {
                setStatus("checking " + sideName(cell) + " CELL for a TIP (TIP_CHECK_MS " + TIP_CHECK_MS + ")");
                return;
            }
            long lastReadingMs = HiveState.getLastUpdateMs(AutoPlan.alliance);
            boolean fresh = lastReadingMs > 0 && lastReadingMs >= arrivedAtLaunchMs - TIP_READING_MAX_AGE_MS;
            if (!fresh && stateTimer.milliseconds() < TIP_CHECK_TIMEOUT_MS) {
                setStatus("no Limelight reading of our HIVE since " + TIP_READING_MAX_AGE_MS
                        + " ms before arriving, waiting (TIP_CHECK_TIMEOUT_MS " + TIP_CHECK_TIMEOUT_MS + ")");
                return;
            }
            tipChecked = true;
            tipConfirmed = fresh;
            String seen = robot.limelightSubsystem.describeHive(AutoPlan.alliance);
            if (fresh) {
                long ageMs = nowMs() - lastReadingMs;
                log("TIP check: Limelight confirms " + upSide() + " CELL up -> "
                        + (cellUp ? "not tipped" : "tipped") + " (reading " + ageMs + " ms old"
                        + (lastReadingMs < arrivedAtLaunchMs ? ", from before arriving" : "") + ") | " + seen);
            } else {
                Log.w(TAG, String.format("[%5.2fs] TIP check UNCONFIRMED: no Limelight reading of the %s HIVE in %d ms, %s | %s",
                        matchTimer.seconds(), AutoPlan.alliance, TIP_CHECK_TIMEOUT_MS,
                        LAUNCH_IF_TIP_UNCONFIRMED ? "launching anyway (LAUNCH_IF_TIP_UNCONFIRMED)" : "assuming it tipped",
                        seen));
                if (!LAUNCH_IF_TIP_UNCONFIRMED) cellUp = false;
            }
        }

        if (cellUp) {
            if (cellUpSinceMs < 0) {
                cellUpSinceMs = nowMs();
                settle.reset();
            }
            boolean settled = settle.update(follower.pose(), nowMs());
            boolean ready = robot.launchSubsystem.isReadyToLaunch();
            if ((ready && settled) || nowMs() - cellUpSinceMs >= READY_TIMEOUT_MS) {
                log("  settle: " + settle.describe(nowMs()));
                startLaunch(ready);
            } else if (!ready) {
                setStatus(sideName(cell) + " CELL is up, waiting for launcher" + (robot.launchReadinessCommand == null ? ""
                        : " (" + robot.launchReadinessCommand.getNotReadyReason() + ")"));
            } else {
                setStatus(sideName(cell) + " CELL is up, launcher ready, waiting for the robot to settle (" + settle.describe(nowMs()) + ")");
            }
        } else if (launchedHere) {
            log(sideName(cell) + " CELL is down after launching here: HIVE tipped, done with launch point "
                    + (launchIndex + 1) + "/" + launchLegs.size());
            launchIndex++;
            launchedHere = false;
            if (launchIndex >= launchLegs.size()) startPark("no launch points left in the plan");
            else if (!refillFirst("moving on after the TIP")) driveToNextLaunchPoint(cell);
        } else {
            cellUpSinceMs = -1;
            setStatus("first visit and " + sideName(cell) + " CELL is down (up CELL " + upSide()
                    + "): waiting for it to be up");
        }
    }

    private void startLaunch(boolean ready) {
        launchCommand = new LaunchBallsCommand(robot.launchGateSubsystem, robot.intakeSubsystem,
                robot.driveSubsystem, robot.intakeBarSubsystem, robot.launchSubsystem);
        launchCommand.schedule();
        String why = currentLaunch().point.side + " CELL is up and launcher "
                + (ready ? "ready" : "NOT ready after READY_TIMEOUT_MS " + READY_TIMEOUT_MS + ", firing anyway ("
                + (robot.launchReadinessCommand == null ? "?" : robot.launchReadinessCommand.getNotReadyReason()) + ")");
        setState(State.LAUNCHING, "launching at " + currentLaunch().point.label() + ": " + why);
    }

    /**
     * From the launch point just finished (its CELL {@code fromCell}) to the next one. Across the
     * HIVE (other CELL), the spline goes through the AROUND / THROUGH control point picked in INIT;
     * to the same CELL, the leg's own line/spline as usual.
     */
    private void driveToNextLaunchPoint(HiveCell fromCell) {
        AutoPlan.Leg<LaunchPoint> leg = currentLaunch();
        if (leg.point.cell(AutoPlan.isRed()) == fromCell) {
            driveToLaunchPoint();
            return;
        }
        LaunchTargets.setForcedCell(leg.point.cell(AutoPlan.isRed()));
        Pose target = poses.launch(leg.point);
        launchRoute.clear();
        if (AutoPlan.hiveRouteAround) {
            driveThrough(target, poses.hiveAroundControls(follower.pose()), true, 1.0);
        } else {
            // Straight lines: waypoint, waypoint, launch pose. Turns to the launch heading on the first leg.
            Pose[] waypoints = poses.hiveThroughWaypoints(follower.pose(), target.heading());
            launchRoute.addAll(Arrays.asList(waypoints).subList(1, waypoints.length));
            launchRoute.add(target);
            log("  THROUGH the HIVE via " + fmt(waypoints[0]) + " and " + fmt(waypoints[1]));
            driveTo(waypoints[0], null, false);
        }
        setState(State.DRIVE_TO_LAUNCH, "to launch point " + (launchIndex + 1) + "/" + launchLegs.size() + " "
                + leg.point.label() + " from the " + sideName(fromCell) + " CELL, " + AutoPlan.hiveRouteLabel());
    }

    private void driveToLaunchPoint() {
        AutoPlan.Leg<LaunchPoint> leg = currentLaunch();
        launchRoute.clear();
        LaunchTargets.setForcedCell(leg.point.cell(AutoPlan.isRed())); // turret aims at it on the way
        driveTo(poses.launch(leg.point), poses.launchControl(leg.point), leg.spline);
        setState(State.DRIVE_TO_LAUNCH, "to launch point " + (launchIndex + 1) + "/" + launchLegs.size() + " "
                + leg.point.label() + " [" + leg.moveLabel() + "]" + (launchedHere ? ", back after intaking" : ""));
    }

    private void driveToIntakePoint() {
        if (intakeIndex >= intakeLegs.size()) {
            startPark("no intake points left in the plan");
            return;
        }
        AutoPlan.Leg<IntakePoint> leg = currentIntake();
        if (SIDE_FLOWER_SPLINE_FROM_AUDIENCE && leg.point == IntakePoint.LEFT_FLOWER && poses.onAudienceHalf(follower.pose())) {
            // A line (or the usual control point) from the audience half clips the HIVE frame.
            driveTo(poses.intakeApproach(leg.point), poses.sideFlowerFromAudienceControl(), true);
            setState(State.DRIVE_TO_INTAKE, "to intake point " + (intakeIndex + 1) + "/" + intakeLegs.size() + " "
                    + describe(leg.point, AutoPlan.isRed()) + " approach [spline via "
                    + fmt(poses.sideFlowerFromAudienceControl()) + ", clear of the HIVE frame]");
            return;
        }
        driveTo(poses.intakeApproach(leg.point), poses.intakeControl(leg.point), leg.spline);
        setState(State.DRIVE_TO_INTAKE, "to intake point " + (intakeIndex + 1) + "/" + intakeLegs.size() + " "
                + describe(leg.point, AutoPlan.isRed()) + " approach [" + leg.moveLabel() + "]");
    }

    /**
     * True once the robot faces the current intake point's approach heading (within
     * INTAKE_APPROACH_HEADING_TOLERANCE_DEG), or PATH_TIMEOUT_MS has passed in DRIVE_TO_INTAKE.
     * The follower is holding the approach pose, so it turns in place meanwhile.
     */
    private boolean approachHeadingReached() {
        Pose approach = poses.intakeApproach(currentIntake().point);
        double d = approach.heading() - follower.pose().heading();
        double errDeg = Math.abs(Math.toDegrees(Math.atan2(Math.sin(d), Math.cos(d))));
        if (errDeg <= INTAKE_APPROACH_HEADING_TOLERANCE_DEG) return true;
        if (stateTimer.milliseconds() > PATH_TIMEOUT_MS) {
            Log.w(TAG, String.format("[%5.2fs]   TURN TIMEOUT at the approach pose: %.0f deg off, going on",
                    matchTimer.seconds(), errDeg));
            return true;
        }
        if (!follower.holding()) follower.hold(approach); // e.g. path timed out: hold here and turn
        setStatus(String.format("at the approach, turning to %.0f deg (%.0f deg off)",
                Math.toDegrees(approach.heading()), errDeg));
        return false;
    }

    private void driveIntoIntakePoint() {
        driveTo(poses.intake(currentIntake().point), null, false, INTAKE_DRIVE_IN_SPEED_FRACTION);
        setState(State.DRIVE_INTO_INTAKE, "driving straight in at " + INTAKE_DRIVE_IN_SPEED_FRACTION + " speed");
    }

    /**
     * Starts rocking from where the robot is (heading snapped to the nearest 90). fromIntake = at the
     * intake pose (GARDEN without the bar): each rock goes back ROCK_DISTANCE_IN, then forward to here.
     * Otherwise (backed out to the approach pose with the bar down): forward ROCK_DISTANCE_IN, then back.
     */
    private void startRocking(boolean fromIntake) {
        rocksDone = 0;
        rockFromIntake = fromIntake;
        Pose p = follower.pose();
        double snapped = Math.toRadians(90.0 * Math.round(Math.toDegrees(p.heading()) / 90.0));
        rockBase = new Pose(p.x(), p.y(), snapped);
        if (fromIntake) rockBack();
        else rockForward();
    }

    /** Rock base moved {@code distance} along the snapped heading: exactly +/-X (0/180) or +/-Y (90/270). */
    private Pose rockOffset(double distance) {
        double h = rockBase.heading();
        return new Pose(rockBase.x() + Math.round(Math.cos(h)) * distance,
                rockBase.y() + Math.round(Math.sin(h)) * distance, h);
    }

    private String rockAxis() {
        return Math.round(Math.cos(rockBase.heading())) != 0 ? "X" : "Y";
    }

    /** Forward leg: from the approach pose ROCK_DISTANCE_IN toward the intake, or back to the intake pose. */
    private void rockForward() {
        driveTo(rockFromIntake ? rockBase : rockOffset(ROCK_DISTANCE_IN), null, false, rockSpeed());
        setState(State.ROCK_FORWARD, String.format("rock %d/%d forward %s along %s%s", rocksDone + 1, ROCK_COUNT,
                rockFromIntake ? "to the intake pose" : String.format("%.1f in", ROCK_DISTANCE_IN), rockAxis(),
                barDown ? ", bar down" : ""));
    }

    /** Back leg: back to the approach pose, or ROCK_DISTANCE_IN back from the intake pose. */
    private void rockBack() {
        driveTo(rockFromIntake ? rockOffset(-ROCK_DISTANCE_IN) : rockBase, null, false, rockSpeed());
        setState(State.ROCK_BACK, String.format("rock %d/%d back %s along %s", rocksDone + 1, ROCK_COUNT,
                rockFromIntake ? String.format("%.1f in from the intake pose", ROCK_DISTANCE_IN) : "to the approach pose",
                rockAxis()));
    }

    /** One rock finished: the next one, or done with this intake point. */
    private void rockDone() {
        rocksDone++;
        if (rocksDone < ROCK_COUNT) {
            if (rockFromIntake) rockBack();
            else rockForward();
        } else {
            log("rocked " + rocksDone + " times, " + robot.intakeSubsystem.getArtifactCount() + " artifacts");
            finishIntakePoint();
        }
    }

    /** Full mid-rock: stop rocking right where the robot is and finish the intake point. */
    private boolean stopRockingIfFull() {
        if (!robot.intakeSubsystem.isFull()) return false;
        log("full (" + robot.intakeSubsystem.getArtifactCount() + ") during rock " + (rocksDone + 1) + "/" + ROCK_COUNT
                + " " + (state == State.ROCK_FORWARD ? "forward" : "back") + ": stopping the rocking");
        holdHere(follower.pose());
        followingPath = false;
        finishIntakePoint();
        return true;
    }

    private static boolean isFlower(IntakePoint p) {
        return p != IntakePoint.GARDEN;
    }

    private static double rockSpeed() {
        return ROCK_SPEED_FRACTION;
    }

    /** Done at this intake point: bar up, then back to the same launch point to check for a TIP. */
    private void finishIntakePoint() {
        log("done with intake point " + (intakeIndex + 1) + "/" + intakeLegs.size() + (barDown ? ", raising bar" : ""));
        raiseBar();
        long delay = currentIntake().delayMs;
        String what = "after intake point " + (intakeIndex + 1);
        intakeIndex++;
        boolean lastIntake = intakeIndex >= intakeLegs.size();
        waitThen(delay, what, () -> {
            if (refillFirst("intake point " + intakeIndex + " picked nothing up")) return;
            if (lastIntake && FINAL_LAUNCH_AT_LEFT_SPOT_3) driveToFinalLaunchPoint();
            else driveToLaunchPoint();
        });
    }

    /**
     * REFILL_BEFORE_LAUNCH: with 0 artifacts, go to the closest unused intake point instead of a
     * launch point (it is moved up to be the next intake point; after it the usual return to the
     * current launch point follows). No intake point left: park. Returns true if it took over.
     */
    private boolean refillFirst(String why) {
        if (!REFILL_BEFORE_LAUNCH || !mechanisms || robot.intakeSubsystem.getArtifactCount() > 0) return false;
        if (intakeIndex >= intakeLegs.size()) {
            startPark("no artifacts and no intake points left (" + why + ")");
            return true;
        }
        Pose here = follower.pose();
        int best = intakeIndex;
        double bestDist = Double.MAX_VALUE;
        for (int i = intakeIndex; i < intakeLegs.size(); i++) {
            Pose a = poses.intakeApproach(intakeLegs.get(i).point);
            double d = Math.hypot(a.x() - here.x(), a.y() - here.y());
            if (d < bestDist) {
                bestDist = d;
                best = i;
            }
        }
        if (best != intakeIndex) intakeLegs.add(intakeIndex, intakeLegs.remove(best));
        log(String.format("0 artifacts (%s): to the closest unused intake point, %s (%.0f in away), before launch point %s",
                why, describe(currentIntake().point, AutoPlan.isRed()), bestDist,
                launchIndex < launchLegs.size() ? currentLaunch().point.label() : "-"));
        driveToIntakePoint();
        return true;
    }

    /**
     * After the last intake point: the final launch is at LEFT spot 3. Added to the end of this
     * run's launch sequence as a straight-line leg (linear heading, see driveThrough) and made the
     * current launch point, so the usual arrive / CELL-up / launch / park logic runs there.
     */
    private void driveToFinalLaunchPoint() {
        launchLegs.add(new AutoPlan.Leg<>(LaunchPoint.LEFT_SPOT_3, false));
        launchIndex = launchLegs.size() - 1;
        launchedHere = false;
        log("last intake point done: final launch at " + LaunchPoint.LEFT_SPOT_3.label() + " "
                + fmt(poses.launch(LaunchPoint.LEFT_SPOT_3)) + " (FINAL_LAUNCH_AT_LEFT_SPOT_3)");
        driveToLaunchPoint();
    }

    /** Holds still for {@code ms} (a plan delay), then runs {@code next}. ms 0 runs it now. */
    private void waitThen(long ms, String what, Runnable next) {
        if (ms <= 0) {
            next.run();
            return;
        }
        holdHere(follower.pose());
        followingPath = false;
        waitMs = ms;
        afterWait = next;
        setState(State.WAIT, String.format("waiting %.1f s (delay %s)", ms / 1000.0, what));
    }

    /** "Stay here": Pedro's hold with HOLD_POSITION, else stop with the motors braked. */
    private void holdHere(Pose p) {
        if (HOLD_POSITION) follower.hold(p);
        else follower.stop();
    }

    private void startPark(String reason) {
        parkStarted = true;
        launchRoute.clear();
        raiseBar();
        Pose spot3 = poses.launch(LaunchPoint.LEFT_SPOT_3);
        Pose here = follower.pose();
        double fromSpot3 = Math.hypot(here.x() - spot3.x(), here.y() - spot3.y());
        if (LEFT_SPOT_3_COUNTS_AS_PARK && fromSpot3 <= LEFT_SPOT_3_PARK_RADIUS_IN) {
            holdHere(here);
            followingPath = false;
            setState(State.DONE, String.format("parked at LEFT spot 3 (%.1f in from it, partly in the LOADING ZONE), no park move: %s",
                    fromSpot3, reason));
            return;
        }
        driveTo(poses.park(AutoPlan.parkSide), poses.parkControl(AutoPlan.parkSide), PARK_WITH_SPLINE);
        setState(State.DRIVE_TO_PARK, "parking " + AutoPlan.parkLabel() + ": " + reason);
    }

    private void raiseBar() {
        if (barDown) {
            robot.intakeBarSubsystem.raise();
            barDown = false;
        }
    }

    /**
     * At the launch point, launcher ready, and our CELL confirmed up: on the first visit by
     * HiveState (known start state or Limelight), on a return visit by a fresh TIP-check reading.
     */
    /**
     * Launching beats parking: in the park window the robot keeps going while it holds artifacts
     * and the plan still has a launch point. It parks only when there is nothing left to launch.
     */
    private boolean launchTakesPriority() {
        return noLaunchReason() == null;
    }

    /**
     * Why the robot can't launch soon (so it should park in the park window), or null if it can:
     * it holds artifacts, the plan has a launch point, and it isn't stuck at a launch point whose
     * CELL is down (waiting there for it to come up can't end in a launch before time runs out).
     * Still driving to a launch point counts as able: the CELL may be confirmed up on arrival.
     */
    private String noLaunchReason() {
        if (!mechanisms) return "mechanisms off";
        if (robot.intakeSubsystem.getArtifactCount() <= 0) return "no artifacts to launch";
        if (launchIndex >= launchLegs.size()) return "no launch points left";
        if (state == State.AT_LAUNCH) {
            HiveCell cell = currentLaunch().point.cell(AutoPlan.isRed());
            boolean tipCheckPending = launchedHere && !tipChecked;
            if (!tipCheckPending && HiveState.getUpCell(AutoPlan.alliance) != cell) {
                return "at " + currentLaunch().point.label() + " but its CELL is down (up CELL "
                        + upSide() + ")";
            }
        }
        return null;
    }

    /**
     * Park window with artifacts to launch: if the robot is intaking, stop and head for the launch
     * point (backing out first if the bar is down). Driving to / at the launch point, backing out,
     * and plan delays carry on as normal. Returns true if it changed state this loop.
     */
    private boolean hurryToLaunch() {
        switch (state) {
            case DRIVE_TO_INTAKE:
                log("park window with " + robot.intakeSubsystem.getArtifactCount()
                        + " artifacts: skipping this intake point to go launch");
                finishIntakePoint();
                return true;
            case LOWER_BAR:
            case DRIVE_INTO_INTAKE:
            case INTAKING:
                log("park window with " + robot.intakeSubsystem.getArtifactCount()
                        + " artifacts: cutting the intake short to go launch");
                if (barDown) {
                    driveTo(poses.intakeApproach(currentIntake().point), null, false);
                    setState(State.BACK_OUT_OF_INTAKE, "bar is down: backing out to the approach pose, then launch");
                } else {
                    finishIntakePoint();
                }
                return true;
            case ROCK_FORWARD:
            case ROCK_BACK:
                log("park window with " + robot.intakeSubsystem.getArtifactCount()
                        + " artifacts: stopping the rocking to go launch");
                holdHere(follower.pose());
                followingPath = false;
                finishIntakePoint();
                return true;
            default:
                return false;
        }
    }

    private boolean parkTime() {
        return matchTimer.seconds() >= AUTO_LENGTH_S - PARK_WINDOW_S;
    }

    private AutoPlan.Leg<LaunchPoint> currentLaunch() {
        return launchLegs.get(launchIndex);
    }

    private AutoPlan.Leg<IntakePoint> currentIntake() {
        return intakeLegs.get(intakeIndex);
    }

    private void setState(State next, String why) {
        log(state + " -> " + next + ": " + why + " | pose " + fmt(follower.pose()) + ", "
                + robot.intakeSubsystem.getArtifactCount() + " artifacts, up CELL " + upSide());
        state = next;
        status = why;
        loggedStatus = why;
        stateTimer.reset();
        slowSinceMs = -1;
    }

    /** Status inside a state (e.g. what it's waiting for); logged only when it changes. */
    private void setStatus(String s) {
        status = s;
        if (!s.equals(loggedStatus)) {
            log("  " + state + ": " + s);
            loggedStatus = s;
        }
    }

    /** All lines share the "DynamicAuto" tag and start with the match time. */
    private void log(String msg) {
        Log.i(TAG, String.format("[%5.2fs] %s", matchTimer.seconds(), msg));
    }

    private static String fmt(Pose p) {
        return String.format("(%.1f, %.1f, %.0f°)", p.x(), p.y(), Math.toDegrees(p.heading()));
    }

    private static String ms(ElapsedTime t) {
        return String.format("%.0f ms", t.milliseconds());
    }

    private String launchPlanText() {
        if (launchLegs.isEmpty()) return "(none)";
        StringBuilder b = new StringBuilder();
        for (int i = 0; i < launchLegs.size(); i++) {
            if (i > 0) b.append(", ");
            b.append(i + 1).append(". ").append(launchLegs.get(i).point.label())
                    .append(" [").append(launchLegs.get(i).moveLabel()).append("]");
        }
        return b.toString();
    }

    private String intakePlanText(boolean red) {
        if (intakeLegs.isEmpty()) return "(none)";
        StringBuilder b = new StringBuilder();
        for (int i = 0; i < intakeLegs.size(); i++) {
            if (i > 0) b.append(", ");
            b.append(i + 1).append(". ").append(describe(intakeLegs.get(i).point, red))
                    .append(" [").append(intakeLegs.get(i).moveLabel()).append("]");
        }
        return b.toString();
    }

    // ================================================================ paths

    /**
     * Drives from where the robot is now to {@code target}: a straight line, or a spline bending
     * through {@code control}. Heading turns linearly to the target heading. Pedro holds the end.
     */
    private void driveTo(Pose target, Pose control, boolean spline) {
        driveTo(target, control, spline, 1.0);
    }

    /**
     * Same, with the path's target speed capped at {@code speedFraction} of the drive's max
     * achievable velocity for this path only. Pedro's maxPathSpeed is that fraction (Foresight uses
     * min(v, maxPathSpeed * maxAchievableVelocity)), not in/s. 1 = no limit.
     */
    private void driveTo(Pose target, Pose control, boolean spline, double speedFraction) {
        driveThrough(target, control == null ? new Pose[0] : new Pose[] {control}, spline, speedFraction);
    }

    /** Same, with any number of spline control points (in travel order). */
    private void driveThrough(Pose target, Pose[] controls, boolean spline, double speedFraction) {
        Pose start = follower.pose();
        if (start.distance(target) < MIN_PATH_LENGTH_IN) {
            log("  already within " + MIN_PATH_LENGTH_IN + " in of " + fmt(target) + ": holding, no path");
            holdHere(target);
            followingPath = false;
            return;
        }
        boolean curve = spline && controls.length > 0;
        Path path;
        if (curve) {
            Pose[] pts = new Pose[controls.length + 2];
            pts[0] = start;
            System.arraycopy(controls, 0, pts, 1, controls.length);
            pts[pts.length - 1] = target;
            path = Paths.curve(pts);
        } else {
            path = Paths.line(start, target);
        }
        path = path.linear(start, target);
        boolean limited = speedFraction > 0 && speedFraction < 1;
        if (limited) {
            // Applied while this path runs, reverted afterwards.
            path = path.with(Constants.foresightConfig.maxPathSpeed.at(speedFraction));
        }
        follower.follow(path);
        followingPath = true;
        log("  path " + (curve ? "spline" + (controls.length > 1 ? " (" + controls.length + " control points)" : "") : "line")
                + " " + fmt(start) + " -> " + fmt(target)
                + String.format(", %.1f in", start.distance(target))
                + (limited ? String.format(", max %.2f of top speed", speedFraction) : ""));
    }

    private boolean pathDone() {
        return pathDone(PATH_END_TOLERANCE_IN);
    }

    /**
     * True once the robot has been slower than STALL_SPEED_IN_PER_SEC for STALL_MS this state (after
     * PATH_MIN_MS, so it has had time to start moving): stuck against the wall or a FLOWER.
     */
    private boolean stalled() {
        double t = stateTimer.milliseconds();
        if (t < PATH_MIN_MS || follower.velocity().linSpeed() >= STALL_SPEED_IN_PER_SEC) {
            slowSinceMs = -1;
            return false;
        }
        if (slowSinceMs < 0) slowSinceMs = t;
        if (t - slowSinceMs < STALL_MS) return false;
        log("  stalled (under " + STALL_SPEED_IN_PER_SEC + " in/s for " + STALL_MS + " ms) in " + state + " after "
                + ms(stateTimer) + String.format(", %.1f in short", follower.remainingDistance()) + ", pose " + fmt(follower.pose()));
        return true;
    }

    private boolean pathDone(double endToleranceIn) {
        if (!followingPath) return true;
        double t = stateTimer.milliseconds();
        if (t < PATH_MIN_MS) return false;
        if (!follower.isBusy() || follower.remainingDistance() < endToleranceIn) {
            log("  path done (" + (!follower.isBusy() ? "follower finished" : "within " + endToleranceIn + " in of the end")
                    + ") in " + ms(stateTimer) + ", pose " + fmt(follower.pose()));
            return true;
        }
        if (t > PATH_TIMEOUT_MS) {
            Log.w(TAG, String.format("[%5.2fs]   PATH TIMEOUT in %s after %d ms, %.1f in short, pose %s",
                    matchTimer.seconds(), state, PATH_TIMEOUT_MS, follower.remainingDistance(), fmt(follower.pose())));
            return true;
        }
        return false;
    }

    // ================================================================ telemetry

    /** Unless SHOW_TELEMETRY, discards the lines the subsystems' periodic() just added. */
    private static void dropSubsystemTelemetry(TelemetryManager panels) {
        if (!SHOW_TELEMETRY) panels.setLines(new ArrayList<>());
    }

    private void showStatus(TelemetryManager panels) {
        if (!SHOW_TELEMETRY) {
            panels.update(telemetry); // clears the screen
            return;
        }
        boolean red = AutoPlan.isRed();
        panels.addData("Time left (s)", String.format("%.1f", Math.max(0, AUTO_LENGTH_S - matchTimer.seconds())));
        panels.addData("State", state + ": " + status);
        panels.addData("Launch point", launchLegs.isEmpty() ? "-"
                : launchIndex >= launchLegs.size() ? "done"
                : (launchIndex + 1) + "/" + launchLegs.size() + " " + currentLaunch().point.label()
                + (launchedHere ? " (launched here)" : ""));
        panels.addData("Intake point", intakeIndex >= intakeLegs.size() ? "done"
                : (intakeIndex + 1) + "/" + intakeLegs.size() + " " + currentIntake().point.label(red));
        panels.addData("Up CELL (Limelight)", upSide());
        panels.addData("Artifacts", robot.intakeSubsystem.getArtifactCount());
        panels.update(telemetry);
    }

    private static long nowMs() {
        return System.nanoTime() / 1_000_000L;
    }

    // ================================================================ input: either gamepad, edge-triggered

    private enum Button { LB, RB, DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT, A, B, X, Y, BACK, LEFT_STICK, RIGHT_STICK }

    private final boolean[] triggerWasDown = new boolean[2];
    private final int[] stickWasDir = new int[2];

    /**
     * Edge-triggered flick of either gamepad's left (true) or right (false) stick: +1 when pushed
     * up past half, -1 down, else 0.
     */
    private int stickSteps(boolean left) {
        double y = left ? pickStronger(gamepad1.left_stick_y, gamepad2.left_stick_y)
                : pickStronger(gamepad1.right_stick_y, gamepad2.right_stick_y);
        int dir = y < -0.5 ? 1 : y > 0.5 ? -1 : 0; // stick y is negative when pushed up
        int i = left ? 0 : 1;
        int step = dir != stickWasDir[i] ? dir : 0;
        stickWasDir[i] = dir;
        return step;
    }

    private static double pickStronger(double a, double b) {
        return Math.abs(a) >= Math.abs(b) ? a : b;
    }

    /** Edge-triggered press of either gamepad's left (true) or right (false) trigger. */
    private boolean triggerPressed(boolean left) {
        boolean down = (left ? Math.max(gamepad1.left_trigger, gamepad2.left_trigger)
                : Math.max(gamepad1.right_trigger, gamepad2.right_trigger)) > 0.5;
        int i = left ? 0 : 1;
        boolean pressed = down && !triggerWasDown[i];
        triggerWasDown[i] = down;
        return pressed;
    }

    private boolean pressed(Button b) {
        // Bitwise | so both gamepads' edge flags are consumed every loop.
        switch (b) {
            case LB: return gamepad1.leftBumperWasPressed() | gamepad2.leftBumperWasPressed();
            case RB: return gamepad1.rightBumperWasPressed() | gamepad2.rightBumperWasPressed();
            case DPAD_UP: return gamepad1.dpadUpWasPressed() | gamepad2.dpadUpWasPressed();
            case DPAD_DOWN: return gamepad1.dpadDownWasPressed() | gamepad2.dpadDownWasPressed();
            case DPAD_LEFT: return gamepad1.dpadLeftWasPressed() | gamepad2.dpadLeftWasPressed();
            case DPAD_RIGHT: return gamepad1.dpadRightWasPressed() | gamepad2.dpadRightWasPressed();
            case A: return gamepad1.aWasPressed() | gamepad2.aWasPressed();
            case B: return gamepad1.bWasPressed() | gamepad2.bWasPressed();
            case X: return gamepad1.xWasPressed() | gamepad2.xWasPressed();
            case Y: return gamepad1.yWasPressed() | gamepad2.yWasPressed();
            case LEFT_STICK: return gamepad1.leftStickButtonWasPressed() | gamepad2.leftStickButtonWasPressed();
            case RIGHT_STICK: return gamepad1.rightStickButtonWasPressed() | gamepad2.rightStickButtonWasPressed();
            default: return gamepad1.backWasPressed() | gamepad2.backWasPressed();
        }
    }
}
