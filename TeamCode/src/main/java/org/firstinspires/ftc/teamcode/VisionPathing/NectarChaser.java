package org.firstinspires.ftc.teamcode.VisionPathing;

import static org.firstinspires.ftc.teamcode.VisionPathing.VisionConstants.*;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.bylazar.field.FieldManager;
import com.bylazar.field.FieldPresets;
import com.bylazar.field.PanelsField;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.api.Paths;
import com.pedropathing.config.Modifier;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.VisionPathing.VisionSubsystem.Ball;
import org.firstinspires.ftc.teamcode.VisionPathing.VisionSubsystem.BallType;
import org.firstinspires.ftc.teamcode.pedro.Constants;   // the team's Pedro Pathing Constants (pedro/Constants.java)

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Drives to Nectar with Pedro Pathing and steers AROUND Pollen.
 *
 * WHY AVOID POLLEN: G407 says a robot may not CONTROL more than 4 scoring elements, and the
 * manual's definition of CONTROL includes pushing a ball "in a preferred direction (herding)".
 * So the robot must neither intake Pollen nor plow it. Every path is checked against a
 * corridor as wide as the intake (INTAKE_WIDTH, ~16 in) / chassis plus POLLEN_CLEARANCE.
 *
 * WHAT IT DOES
 *   SEARCH  hold still (or slowly spin, SEARCH_TURN_POWER) until a reachable Nectar of
 *           TARGET_ALLIANCE's color is tracked. Up to MAX_CANDIDATES of the nearest are actually
 *           path-planned; the cheapest path wins (not simply the closest ball).
 *   FACE    turn in place toward it, so the camera gets a good look at it AND at the Pollen
 *           around it before committing to a path.
 *   SETTLE  wait SETTLE_S for fresh readings, then plan.
 *   DRIVE   follow the path. The intake comes on for the last INTAKE_ON_WITHIN inches.
 *           Re-plans (at most every REPLAN_MIN_S) if new Pollen blocks the rest of the path or
 *           the target estimate moves. The last ~11 in, where the camera can't see the ball,
 *           are driven on ODOMETRY: the ball's field position is remembered, not guessed.
 *   COLLECT hold the end pose with the intake running for COLLECT_DWELL_S, then look again.
 *
 * THE PATH
 *   For a range of approach directions (straight-in first, then +-15, +-30, ...):
 *     1. Final leg: a straight line into the ball along that direction, FINAL_APPROACH_LEN long,
 *        ending with robot center END_CENTER_TO_BALL short of the ball. The space in front of the
 *        intake at the end must be free of Pollen too (else it gets eaten along with the Nectar).
 *     2. Travel: a smooth cubic Bezier from the robot (leaving the way it faces) to the start of
 *        that leg (arriving already lined up). If Pollen is in the way, a detour waypoint is
 *        placed beside the blocking ball, on either side, up to MAX_DETOURS deep; each option is
 *        re-checked against every obstacle by sampling the actual curve.
 *   Cheapest = shortest total length + APPROACH_TURN_PENALTY per degree off straight-in.
 *
 * CONTROLS
 *   init:  X = calibrate camera on one Pollen    B = calibrate on one Nectar (alliance color)
 *          (ball straight ahead, CAL_FLOOR_DIST along the floor from the point under the lens)
 *   run:   hold A = stop; the sticks drive the robot. Release = start looking again.
 *
 * NEEDS
 *   - The team's Pedro Constants (teamcode/pedro/Constants.java: create(HardwareMap) builds the
 *     Pinpoint + Mecanum + Foresight follower, and foresightConfig is its ForesightConfig). If
 *     those ever get renamed, only createFollower() and finalLegModifiers() below need changing.
 *   - This OpMode builds its OWN follower, so don't run it alongside DriveSubsystem (that one
 *     also owns the drive motors and Pinpoint). In team OpModes, use DriveSubsystem's follower.
 *   - The robot center Pedro reports must be the robot center the VisionConstants offsets are
 *     measured from.
 *   - START_X / START_Y / START_HEADING_DEG to be where the robot really starts if you turn on
 *     USE_FIELD_BOUNDS. Otherwise only relative positions matter and any start pose works.
 *
 * The planner (Planner / Plan / Obstacle below) is pure geometry and has no hardware in it, so
 * an autonomous can reuse it: Planner.plan(pose, ballX, ballY, obstacles) -> Planner.buildPath().
 */
@TeleOp(name = "Nectar Chaser (avoids Pollen)", group = "VisionPathing")
public class NectarChaser extends CommandOpMode {

    private enum State { SEARCH, FACE, SETTLE, DRIVE, COLLECT, STOPPED, DONE }

    private Follower follower;
    private VisionSubsystem vision;
    private DcMotor intake;
    private Telemetry telem;
    private FieldManager field;

    private State state = State.SEARCH;
    private String note = "";
    private int targetId = -1;
    private Plan plan = null;
    private Pose facePose = null;
    private int collected = 0;
    private boolean searchNow = true;
    private boolean lastSearchBlocked = false;

    private final ElapsedTime clock = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime chaseTimer = new ElapsedTime();
    private final ElapsedTime replanTimer = new ElapsedTime();
    private final ElapsedTime searchTimer = new ElapsedTime();
    private final Map<Integer, Double> skipUntil = new HashMap<>();   // ball id -> clock seconds

    // =====================================================================
    // ===================== PEDRO HOOKS (edit if needed) ==================
    // =====================================================================

    /** Builds the follower from your Pedro Constants. */
    private Follower createFollower() {
        return Constants.create(hardwareMap);
    }

    /**
     * Slows the straight drive-in so the intake gets a clean bite.
     * If this doesn't compile, your Constants names its ForesightConfig differently: rename it
     * here, or return new Modifier[0] to drive in at normal speed.
     */
    private Modifier[] finalLegModifiers() {
        return new Modifier[] { Constants.foresightConfig.maxPathSpeed.at(FINAL_LEG_SPEED) };
    }

    // =====================================================================
    // ============================ LIFECYCLE ==============================
    // =====================================================================

    @Override
    public void initialize() {
        // FTCLib's scheduler is a singleton that outlives OpModes. Clear whatever the last
        // OpMode left registered, or its periodic() keeps running against dead hardware.
        CommandScheduler.getInstance().reset();

        telem = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        telem.setMsTransmissionInterval(100);
        telem.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        follower = createFollower();
        follower.setPose(new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG)));

        vision = new VisionSubsystem(hardwareMap);       // registers itself with the scheduler
        vision.setRobotPoseSupplier(follower::pose);

        intake = INTAKE_MOTOR.isEmpty() ? null : hardwareMap.tryGet(DcMotor.class, INTAKE_MOTOR);

        field = PanelsField.INSTANCE.getField();
        field.setOffsets(FieldPresets.INSTANCE.getDEFAULT_FTC());
    }

    /**
     * Replaces CommandOpMode's loop to add an init loop (calibration, live preview) and a
     * guaranteed cleanup. Order every loop: odometry -> vision -> decisions.
     */
    @Override
    public void runOpMode() throws InterruptedException {
        try {
            initialize();

            boolean lastX = false, lastB = false;
            while (opModeInInit()) {
                follower.update();
                run();                                   // FTCLib scheduler -> vision.periodic()

                boolean x = gamepad1.x, b = gamepad1.b;
                if (x && !lastX) vision.startCalibration(BallType.POLLEN);
                if (b && !lastB) vision.startCalibration(
                        TARGET_ALLIANCE == 1 ? BallType.BLUE_NECTAR : BallType.RED_NECTAR);
                lastX = x;
                lastB = b;

                initTelemetry();
            }
            if (isStopRequested()) return;

            clock.reset();
            enterSearch("started");

            while (opModeIsActive()) {
                follower.update();                       // 1. where are we
                run();                                   // 2. what do we see (uses that pose)
                updateChase();                           // 3. what do we do
                drawField();
                chaseTelemetry();
            }
        } finally {
            shutdown();
        }
    }

    private void shutdown() {
        setIntake(false);
        if (follower != null) {
            follower.stop();
            follower.drivetrain.stop();
        }
        if (vision != null) vision.close();
        reset();                                         // clear the FTCLib scheduler
    }

    // =====================================================================
    // ========================== STATE MACHINE ============================
    // =====================================================================

    private void updateChase() {
        // Hold A: stop chasing, sticks drive. Checked first so it always wins.
        if (gamepad1.a) {
            if (state != State.STOPPED) {
                dropTarget();
                enter(State.STOPPED, "A held: manual drive");
            }
            follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            setIntake(false);
            return;
        }

        switch (state) {
            case STOPPED: enterSearch("A released"); break;
            case SEARCH:  doSearch();  break;
            case FACE:    doFace();    break;
            case SETTLE:  doSettle();  break;
            case DRIVE:   doDrive();   break;
            case COLLECT: doCollect(); break;
            case DONE:    setIntake(false); break;
        }
    }

    private void doSearch() {
        setIntake(false);
        if (!searchNow && searchTimer.seconds() < REPLAN_MIN_S) return;   // planning ~5x/s is plenty
        searchNow = false;
        searchTimer.reset();

        Candidate best = chooseTarget();
        if (best == null) {
            if (SEARCH_TURN_POWER != 0) follower.manual(0, 0, SEARCH_TURN_POWER);
            note = lastSearchBlocked ? "Nectar seen, but every path would touch Pollen"
                    : "no Nectar in range";
            return;
        }

        targetId = best.ball.id;
        vision.highlight(targetId);
        chaseTimer.reset();
        startFacing(best.ball);
    }

    private void startFacing(Ball b) {
        setIntake(false);
        Pose pose = follower.pose();
        double want = Math.atan2(b.fieldY - pose.y(), b.fieldX - pose.x());
        if (Math.abs(Angle.error(pose.heading(), want)) <= Math.toRadians(FACE_TOL_DEG)) {
            planAndGo("driving to Nectar #" + b.id);       // already looking at it
            return;
        }
        facePose = new Pose(pose.x(), pose.y(), want);
        follower.hold(facePose);
        enter(State.FACE, String.format("turning %.0f deg to face Nectar #%d",
                Math.toDegrees(Angle.error(pose.heading(), want)), b.id));
    }

    private void doFace() {
        Ball t = vision.getBall(targetId);
        if (t == null) { enterSearch("target lost while turning"); return; }

        Pose pose = follower.pose();
        double want = Math.atan2(t.fieldY - pose.y(), t.fieldX - pose.x());
        // The estimate sharpens as the ball comes into view: follow it if it moved noticeably
        if (Math.abs(Angle.error(facePose.heading(), want)) > Math.toRadians(3)) {
            facePose = new Pose(facePose.x(), facePose.y(), want);
            follower.hold(facePose);
        }

        boolean aimed = Math.abs(Angle.error(pose.heading(), want)) <= Math.toRadians(FACE_TOL_DEG);
        if (aimed || stateTimer.seconds() > FACE_TIMEOUT_S) {
            enter(State.SETTLE, aimed ? "aimed, taking a look" : "turn timed out, taking a look");
        }
    }

    private void doSettle() {
        if (vision.getBall(targetId) == null) { enterSearch("target lost"); return; }
        if (stateTimer.seconds() >= SETTLE_S) planAndGo("driving to Nectar #" + targetId);
    }

    /** Plan to the locked target from here and start following, or give up on it for a moment. */
    private void planAndGo(String why) {
        Ball t = vision.getBall(targetId);
        if (t == null) { enterSearch("target lost"); return; }

        Plan p = Planner.plan(follower.pose(), t.fieldX, t.fieldY, obstaclesFor(t.id));
        if (p == null) {
            skip(t.id);
            enterSearch("every path to #" + t.id + " touches Pollen, skipping it");
            return;
        }
        plan = p;
        if (p.trivial) {
            follower.hold(follower.pose());
            enter(State.COLLECT, "already at Nectar #" + t.id);
            return;
        }
        follow(p);
        enter(State.DRIVE, why);
    }

    private void doDrive() {
        Ball t = vision.getBall(targetId);
        if (t == null) { enterSearch("target disappeared (taken or knocked away)"); return; }
        if (chaseTimer.seconds() > CHASE_TIMEOUT_S) {
            skip(t.id);
            enterSearch("timed out on #" + t.id);
            return;
        }

        Pose pose = follower.pose();
        double toEnd = pose.toVector2D().distance(plan.end);
        setIntake(toEnd <= INTAKE_ON_WITHIN);

        if (!follower.following()) {                     // path done; Pedro now holds the end pose
            enter(State.COLLECT, "at Nectar #" + t.id + ", intaking");
            return;
        }

        // ---- Re-plan only when something actually changed, and not too often ----
        if (replanTimer.seconds() < REPLAN_MIN_S) return;
        replanTimer.reset();

        List<Obstacle> obs = obstaclesFor(t.id);
        boolean moved = t.isVisibleNow() && toEnd > NO_REPLAN_INSIDE
                && Math.hypot(t.fieldX - plan.targetX, t.fieldY - plan.targetY) > REPLAN_TARGET_SHIFT;
        boolean blocked = !Planner.remainingClear(plan, pose, obs);
        if (!moved && !blocked) return;

        Plan p = Planner.plan(pose, t.fieldX, t.fieldY, obs);
        if (p == null) {
            if (blocked) {
                skip(t.id);
                enterSearch("Pollen now blocks every path to #" + t.id);
            }
            return;                                      // moved but no new path: old one is still clear
        }
        plan = p;
        if (p.trivial) {
            follower.hold(pose);
            enter(State.COLLECT, "at Nectar #" + t.id + ", intaking");
            return;
        }
        follow(p);
        note = blocked ? "Pollen in the way: re-planned" : "target estimate moved: re-planned";
    }

    private void doCollect() {
        setIntake(true);
        if (stateTimer.seconds() < COLLECT_DWELL_S) return;

        // No ball sensor, so "collected" is assumed. If it's still sitting there, vision will
        // see it again as a new ball and the robot will go back for it.
        vision.forget(targetId);
        collected++;
        int id = targetId;
        dropTarget();

        if (MAX_TO_COLLECT > 0 && collected >= MAX_TO_COLLECT) {
            setIntake(false);
            follower.hold(follower.pose());
            enter(State.DONE, "collected " + collected + ", done (MAX_TO_COLLECT)");
            return;
        }
        enterSearch("collected #" + id + ", looking for the next one");
    }

    // =====================================================================
    // ============================ HELPERS ================================
    // =====================================================================

    /** Plans the best reachable Nectar among the nearest few. Null if none. */
    private Candidate chooseTarget() {
        lastSearchBlocked = false;
        Pose pose = follower.pose();
        double now = clock.seconds();
        Candidate best = null;
        int planned = 0;

        for (Ball b : vision.getNectar(TARGET_ALLIANCE)) {          // nearest first
            if (b.distance() > MAX_TARGET_DIST) break;
            if (planned >= MAX_CANDIDATES) break;
            Double until = skipUntil.get(b.id);
            if (until != null && until > now) continue;

            planned++;
            Plan p = Planner.plan(pose, b.fieldX, b.fieldY, obstaclesFor(b.id));
            if (p == null) { lastSearchBlocked = true; continue; }

            // Stick with the previous target unless another one is clearly cheaper
            double cost = p.cost - (b.id == targetId ? SWITCH_MARGIN : 0);
            if (best == null || cost < best.cost) best = new Candidate(b, cost);
        }
        return best;
    }

    /**
     * Everything the robot must not touch: Pollen, plus the other alliance's Nectar.
     * Each obstacle is padded by its position uncertainty, so a far, shaky estimate gets more
     * room than a close, solid one.
     */
    private List<Obstacle> obstaclesFor(int targetId) {
        List<Obstacle> out = new ArrayList<>();
        addObstacles(out, BallType.POLLEN, targetId);
        if (AVOID_OTHER_NECTAR) {
            if (!VisionSubsystem.isAllianceNectar(BallType.RED_NECTAR, TARGET_ALLIANCE))
                addObstacles(out, BallType.RED_NECTAR, targetId);
            if (!VisionSubsystem.isAllianceNectar(BallType.BLUE_NECTAR, TARGET_ALLIANCE))
                addObstacles(out, BallType.BLUE_NECTAR, targetId);
        }
        return out;
    }

    private void addObstacles(List<Obstacle> out, BallType type, int targetId) {
        for (Ball b : vision.getBalls(type, AVOID_UNCONFIRMED)) {
            if (b.id == targetId) continue;
            out.add(new Obstacle(b.fieldX, b.fieldY, type.radius() + Math.min(b.sigma, 3.0)));
        }
    }

    private void follow(Plan p) {
        follower.follow(Planner.buildPath(p, finalLegModifiers()));
        replanTimer.reset();
    }

    private void enter(State s, String why) {
        state = s;
        note = why;
        stateTimer.reset();
    }

    private void enterSearch(String why) {
        plan = null;
        setIntake(false);
        follower.hold(follower.pose());
        searchNow = true;
        enter(State.SEARCH, why);
    }

    private void dropTarget() {
        targetId = -1;
        plan = null;
        vision.highlight(-1);
    }

    private void skip(int id) {
        skipUntil.put(id, clock.seconds() + Math.max(BLOCKED_RETRY_S, 0.1));
    }

    private void setIntake(boolean on) {
        if (intake != null) intake.setPower(on ? INTAKE_POWER : 0);
    }

    // =====================================================================
    // =========================== TELEMETRY ===============================
    // =====================================================================

    private String allianceName() {
        return TARGET_ALLIANCE == 0 ? "RED Nectar" : TARGET_ALLIANCE == 1 ? "BLUE Nectar" : "ANY Nectar";
    }

    private void initTelemetry() {
        telem.addLine("NECTAR CHASER  (init)");
        telem.addLine("X = calibrate on Pollen    B = calibrate on Nectar");
        telem.addData("chasing", allianceName());
        telem.addData("intake motor", intake == null ? "none" : INTAKE_MOTOR);
        Pose p = follower.pose();
        telem.addData("pose", "x %.1f  y %.1f  h %.1f deg", p.x(), p.y(), Math.toDegrees(p.heading()));
        vision.addTelemetry(telem);
        telem.update();
    }

    private void chaseTelemetry() {
        telem.addData("state", state);
        telem.addData("note", note);
        telem.addData("chasing", allianceName());
        telem.addData("collected", MAX_TO_COLLECT > 0 ? collected + " / " + MAX_TO_COLLECT : "" + collected);

        Ball t = vision.getBall(targetId);
        if (t != null) {
            telem.addData("target", "#%d %s  %.1f in  %+.0f deg  +/-%.1f in  %s",
                    t.id, t.type.letter(), t.distance(), t.bearingDeg(), t.sigma,
                    t.isVisibleNow() ? "seen" : "remembered");
        }
        Pose pose = follower.pose();
        if (plan != null && plan.end != null) {
            telem.addData("plan", "%.0f in, %d detour(s), approach %+.0f deg, %.1f in to go",
                    plan.length, plan.waypoints.size(), plan.approachOffsetDeg,
                    pose.toVector2D().distance(plan.end));
        }
        telem.addData("pose", "x %.1f  y %.1f  h %.1f deg", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telem.addData("follower", follower.mode());
        vision.addTelemetry(telem);
        telem.addLine("hold A = stop + drive with sticks");
        telem.update();
    }

    /**
     * Panels field view: robot (green), Pollen (yellow), Nectar (red/blue), target (purple ring),
     * planned path (white). Drawn in Pedro coordinates shifted to the field center; the picture
     * may be rotated relative to Panels' field image, but everything is consistent with itself.
     */
    private void drawField() {
        Pose pose = follower.pose();
        String clear = PanelsField.INSTANCE.getTRANSPARENT();

        field.setStyle(clear, "#4CAF50", 1.0);
        moveTo(pose.x(), pose.y());
        field.circle(ROBOT_WIDTH / 2.0);
        moveTo(pose.x(), pose.y());
        lineTo(pose.x() + Math.cos(pose.heading()) * FRONT_EXTENT, pose.y() + Math.sin(pose.heading()) * FRONT_EXTENT);

        for (BallType type : BallType.values()) {
            String color = type == BallType.POLLEN ? "#FFD600" : type == BallType.RED_NECTAR ? "#F44336" : "#2196F3";
            for (Ball b : vision.getBalls(type, true)) {
                field.setStyle(b.confirmed ? color : clear, color, 0.5);
                moveTo(b.fieldX, b.fieldY);
                field.circle(type.radius());
                if (b.id == targetId) {
                    field.setStyle(clear, "#7C4DFF", 1.0);
                    moveTo(b.fieldX, b.fieldY);
                    field.circle(type.radius() + 2);
                }
            }
        }

        if (plan != null && plan.samples.size() > 1) {
            field.setStyle(clear, "#FFFFFF", 0.5);
            List<Vector2D> s = plan.samples;
            for (int i = 0; i + 1 < s.size(); i += 3) {
                Vector2D a = s.get(i), b = s.get(Math.min(i + 3, s.size() - 1));
                moveTo(a.x(), a.y());
                lineTo(b.x(), b.y());
            }
        }
        field.update();
    }

    private void moveTo(double x, double y) {
        field.moveCursor(x - FIELD_SIZE / 2.0, y - FIELD_SIZE / 2.0);
    }

    private void lineTo(double x, double y) {
        field.line(x - FIELD_SIZE / 2.0, y - FIELD_SIZE / 2.0);
    }

    private static final class Candidate {
        final Ball ball;
        final double cost;
        Candidate(Ball ball, double cost) { this.ball = ball; this.cost = cost; }
    }

    // =====================================================================
    // ============================= PLANNER ===============================
    // Pure geometry in Pedro field coordinates. No hardware, no state.
    // =====================================================================

    /** Something to stay away from: a ball center and its (padded) radius, inches. */
    static final class Obstacle {
        final double x, y, r;
        Obstacle(double x, double y, double r) { this.x = x; this.y = y; this.r = r; }
        Vector2D pos() { return Vector2D.cartesian(x, y); }
    }

    /** A planned route to one ball. */
    static final class Plan {
        double targetX, targetY;               // ball center planned against
        double approachHeading;                // robot heading on the final leg (rad)
        double approachOffsetDeg;              // how far that is from straight-in
        Vector2D dir;                          // unit vector of approachHeading
        Vector2D stage, end;                   // start / end of the straight final leg (robot center)
        List<Vector2D[]> cubics = new ArrayList<>();      // travel: Bezier control points {p0, p1, p2, p3}
        List<Vector2D> waypoints = new ArrayList<>();     // detour points
        List<Vector2D> samples = new ArrayList<>();       // robot-center points along the whole route
        double length, cost;
        boolean trivial;                       // robot is already at the end point
    }

    private static final class Travel {
        List<Vector2D[]> cubics = new ArrayList<>();
        List<Vector2D> waypoints = new ArrayList<>();
        List<Vector2D> samples = new ArrayList<>();
        double length;
    }

    static final class Planner {
        private Planner() {}

        /** Re-checks of a path already being driven get this much slack, so it doesn't flicker. */
        private static final double RECHECK_SLACK = 0.75;

        /** Half the width the robot sweeps: the wider of intake and chassis. */
        static double halfWidth() {
            return Math.max(INTAKE_WIDTH, ROBOT_WIDTH) / 2.0 + Math.abs(INTAKE_CENTER_RIGHT);
        }

        /** How far an obstacle's center must stay from the robot-center path. */
        static double clearance(Obstacle o) {
            return halfWidth() + o.r + POLLEN_CLEARANCE;
        }

        /**
         * Best route that puts the intake on the ball at (tx, ty) without touching any obstacle,
         * or null if there isn't one.
         */
        static Plan plan(Pose robot, double tx, double ty, List<Obstacle> obstacles) {
            Vector2D s = robot.toVector2D();
            Vector2D target = Vector2D.cartesian(tx, ty);
            Vector2D toTarget = target.minus(s);
            double direct = toTarget.magnitude() < 1e-6 ? robot.heading() : toTarget.theta();
            // No route can be shorter than this, so it bounds how far it's worth searching
            double lowerBound = Math.max(0, toTarget.magnitude() - END_CENTER_TO_BALL - Math.abs(INTAKE_CENTER_RIGHT));

            double step = Math.max(1.0, APPROACH_STEP_DEG);
            int steps = (int) Math.floor(Math.min(180.0, Math.max(0.0, APPROACH_MAX_DEG)) / step + 1e-9);
            Plan best = null;

            for (int k = 0; k <= steps; k++) {
                double offset = k * step;
                if (best != null && lowerBound + APPROACH_TURN_PENALTY * offset >= best.cost) break;
                boolean oneSided = k == 0 || offset >= 180.0 - 1e-9;
                for (int sign = 1; sign >= (oneSided ? 1 : -1); sign -= 2) {
                    double off = sign * offset;
                    Plan p = planApproach(robot, s, target, direct + Math.toRadians(off), off, obstacles);
                    if (p != null && (best == null || p.cost < best.cost)) best = p;
                }
            }
            if (best != null) {
                best.targetX = tx;
                best.targetY = ty;
            }
            return best;
        }

        /** Is the part of the plan still ahead of the robot clear of these obstacles? */
        static boolean remainingClear(Plan plan, Pose robot, List<Obstacle> obstacles) {
            if (plan == null || plan.trivial) return true;
            Vector2D r = robot.toVector2D();
            int nearest = 0;
            double bestD = Double.POSITIVE_INFINITY;
            for (int i = 0; i < plan.samples.size(); i++) {
                double d = plan.samples.get(i).distance(r);
                if (d < bestD) { bestD = d; nearest = i; }
            }
            List<Vector2D> rest = new ArrayList<>();
            rest.add(r);
            rest.addAll(plan.samples.subList(nearest, plan.samples.size()));
            return firstBlocker(rest, obstacles, robot, RECHECK_SLACK) == null
                    && frontBlocker(plan.end, plan.dir, obstacles, RECHECK_SLACK) == null;
        }

        /** The Pedro path for a plan: tangent-heading curves, then a straight constant-heading drive-in. */
        static Path buildPath(Plan plan, Modifier... finalLegModifiers) {
            List<Path> parts = new ArrayList<>();
            for (Vector2D[] c : plan.cubics) {
                parts.add(Paths.curve(c[0], c[1], c[2], c[3]).tangent());   // drive forward, camera leads
            }
            Path fin = Paths.line(plan.stage, plan.end).constant(plan.approachHeading);
            if (finalLegModifiers != null && finalLegModifiers.length > 0) fin = fin.with(finalLegModifiers);
            parts.add(fin);
            return parts.size() == 1 ? parts.get(0) : Paths.path(parts.toArray(new Path[0]));
        }

        // ------------------------------------------------------------------

        /** One approach direction: final leg along `heading`, travel to its start. */
        private static Plan planApproach(Pose robot, Vector2D s, Vector2D target, double heading,
                                         double offDeg, List<Obstacle> obs) {
            Vector2D dir = Vector2D.unit(heading);
            Vector2D right = Vector2D.cartesian(dir.y(), -dir.x());
            Vector2D end = target.minus(dir.times(END_CENTER_TO_BALL)).minus(right.times(INTAKE_CENTER_RIGHT));
            Vector2D stage = end.minus(dir.times(FINAL_APPROACH_LEN));

            // Whatever is in front of the intake at the end gets eaten too
            if (frontBlocker(end, dir, obs, 0) != null) return null;
            if (USE_FIELD_BOUNDS && (!inField(end) || !inField(stage))) return null;

            Plan p = new Plan();
            p.approachHeading = heading;
            p.approachOffsetDeg = offDeg;
            p.dir = dir;
            p.end = end;
            double penalty = APPROACH_TURN_PENALTY * Math.abs(offDeg);

            if (s.distance(end) < 1.5) {                         // already there
                p.trivial = true;
                p.stage = s;
                p.samples.add(s);
                p.cost = penalty;
                return p;
            }

            // Already on (or next to) the final leg: just drive the rest of it
            Vector2D fromStage = s.minus(stage);
            double along = fromStage.dot(dir), lateral = Math.abs(fromStage.dot(right));
            boolean facing = Math.abs(Angle.error(robot.heading(), heading)) < Math.toRadians(15);
            if (s.distance(stage) < 2.0
                    || (along >= -1.0 && along <= FINAL_APPROACH_LEN && lateral < 1.5 && facing)) {
                p.stage = s;
                p.samples.addAll(sampleLine(s, end));
                if (firstBlocker(p.samples, obs, robot, 0) != null) return null;
                p.length = s.distance(end);
                p.cost = p.length + penalty;
                return p;
            }

            // The final leg itself must be clear before any travel is worth planning
            List<Vector2D> finalLeg = sampleLine(stage, end);
            if (firstBlocker(finalLeg, obs, null, 0) != null) return null;

            Travel[] best = new Travel[1];
            List<Vector2D> pts = new ArrayList<>();
            pts.add(stage);
            explore(robot, pts, dir, obs, finalLeg, 0, best);
            if (best[0] == null) return null;

            p.stage = stage;
            p.cubics = best[0].cubics;
            p.waypoints = best[0].waypoints;
            p.samples = best[0].samples;
            p.length = best[0].length + FINAL_APPROACH_LEN;
            p.cost = p.length + penalty;
            return p;
        }

        /**
         * Depth-first search over "go around the blocking ball on the left / on the right",
         * up to MAX_DETOURS waypoints. Keeps the shortest clear route in best[0].
         */
        private static void explore(Pose robot, List<Vector2D> pts, Vector2D endDir, List<Obstacle> obs,
                                    List<Vector2D> finalLeg, int depth, Travel[] best) {
            Vector2D s = robot.toVector2D();
            if (best[0] != null && polylineLength(s, pts) >= best[0].length) return;   // can't win

            Travel t = buildTravel(robot, pts, endDir);
            List<Vector2D> all = new ArrayList<>(t.samples);
            all.addAll(finalLeg.subList(1, finalLeg.size()));

            if (USE_FIELD_BOUNDS && !allInField(t.samples)) return;
            Obstacle blocker = firstBlocker(all, obs, robot, 0);
            if (blocker == null) {
                if (best[0] == null || t.length < best[0].length) {
                    t.samples = all;
                    best[0] = t;
                }
                return;
            }
            if (depth >= MAX_DETOURS) return;

            // Put a waypoint beside the blocker, square to the leg it blocks
            int leg = nearestLeg(s, pts, blocker.pos());
            Vector2D a = leg == 0 ? s : pts.get(leg - 1);
            Vector2D b = pts.get(leg);
            Vector2D legVec = b.minus(a);
            if (legVec.magnitude() < 1e-6) return;
            Vector2D left = legVec.normalized().perpendicularLeft();
            double off = clearance(blocker) + DETOUR_MARGIN;

            // Try the side the ball is NOT on first: it's usually the shorter way around
            double ballSide = blocker.pos().minus(a).dot(left);
            double first = ballSide >= 0 ? -1 : 1;
            for (double side = first, n = 0; n < 2; n++, side = -side) {
                Vector2D via = blocker.pos().plus(left.times(side * off));
                if (USE_FIELD_BOUNDS && !inField(via)) continue;
                if (via.distance(a) < 3.0 || via.distance(b) < 3.0) continue;
                List<Vector2D> next = new ArrayList<>(pts);
                next.add(leg, via);
                explore(robot, next, endDir, obs, finalLeg, depth + 1, best);
            }
        }

        /**
         * Smooth route from the robot through pts (last one = final-leg start). Cubic Bezier per
         * leg; tangents are continuous at every waypoint, it leaves the way the robot faces (unless
         * that's more than 60 deg off) and arrives lined up with the final leg.
         */
        private static Travel buildTravel(Pose robot, List<Vector2D> pts, Vector2D endDir) {
            Vector2D s = robot.toVector2D();
            List<Vector2D> all = new ArrayList<>();
            all.add(s);
            all.addAll(pts);
            int n = all.size();

            Vector2D heading = Vector2D.unit(robot.heading());
            Vector2D firstChord = all.get(1).minus(s);
            firstChord = firstChord.magnitude() < 1e-6 ? heading : firstChord.normalized();

            Vector2D[] tan = new Vector2D[n];
            tan[0] = heading.dot(firstChord) > 0.5 ? heading : firstChord;
            tan[n - 1] = endDir;
            for (int i = 1; i < n - 1; i++) {
                Vector2D d = all.get(i + 1).minus(all.get(i - 1));
                tan[i] = d.magnitude() < 1e-6 ? firstChord : d.normalized();
            }

            Travel t = new Travel();
            t.waypoints = new ArrayList<>(pts.subList(0, pts.size() - 1));
            t.samples.add(s);
            double spacing = Math.max(0.25, SAMPLE_SPACING);
            for (int i = 0; i < n - 1; i++) {
                Vector2D p0 = all.get(i), p3 = all.get(i + 1);
                double h = p0.distance(p3) / 3.0;
                Vector2D p1 = p0.plus(tan[i].times(h));
                Vector2D p2 = p3.minus(tan[i + 1].times(h));
                t.cubics.add(new Vector2D[] { p0, p1, p2, p3 });
                int count = Math.max(6, (int) Math.ceil(p0.distance(p3) * 1.25 / spacing));
                for (int k = 1; k <= count; k++) t.samples.add(cubic(p0, p1, p2, p3, k / (double) count));
            }
            t.length = polyLength(t.samples);
            return t;
        }

        /**
         * First obstacle that any sample point comes too close to, or null.
         * If start is given: obstacles already beside or behind the robot (within touching range)
         * are ignored for the first half robot length, because the robot is already next to them
         * and driving forward moves away. After that they are checked like everything else.
         */
        static Obstacle firstBlocker(List<Vector2D> samples, List<Obstacle> obs, Pose start, double slack) {
            boolean[] besideAtStart = new boolean[obs.size()];
            if (start != null) {
                Vector2D s0 = start.toVector2D();
                Vector2D fwd = Vector2D.unit(start.heading());
                Vector2D left = fwd.perpendicularLeft();
                for (int i = 0; i < obs.size(); i++) {
                    Vector2D d = obs.get(i).pos().minus(s0);
                    double along = d.dot(fwd), lateral = Math.abs(d.dot(left));
                    besideAtStart[i] = along < FRONT_EXTENT && along > -ROBOT_LENGTH
                            && lateral < clearance(obs.get(i)) + 1.0;
                }
            }
            double grace = ROBOT_LENGTH / 2.0 + 2.0;
            double arc = 0;
            Vector2D prev = null;
            for (Vector2D p : samples) {
                if (prev != null) arc += p.distance(prev);
                prev = p;
                for (int i = 0; i < obs.size(); i++) {
                    if (besideAtStart[i] && arc < grace) continue;
                    Obstacle o = obs.get(i);
                    if (p.distance(o.pos()) < clearance(o) - slack) return o;
                }
            }
            return null;
        }

        /** Anything in the box in front of the intake when the robot stops at `end` facing `dir`. */
        static Obstacle frontBlocker(Vector2D end, Vector2D dir, List<Obstacle> obs, double slack) {
            Vector2D left = dir.perpendicularLeft();
            for (Obstacle o : obs) {
                Vector2D d = o.pos().minus(end);
                double along = d.dot(dir), lateral = Math.abs(d.dot(left));
                double pad = o.r + POLLEN_CLEARANCE - slack;
                if (along >= 0 && along <= FRONT_EXTENT + pad && lateral <= halfWidth() + pad) return o;
            }
            return null;
        }

        /** Index (into pts) of the leg {start->pts[0], pts[0]->pts[1], ...} nearest to point q. */
        private static int nearestLeg(Vector2D s, List<Vector2D> pts, Vector2D q) {
            int best = 0;
            double bestD = Double.POSITIVE_INFINITY;
            Vector2D a = s;
            for (int i = 0; i < pts.size(); i++) {
                Vector2D b = pts.get(i);
                double d = pointToSegment(q, a, b);
                if (d < bestD) { bestD = d; best = i; }
                a = b;
            }
            return best;
        }

        private static double pointToSegment(Vector2D q, Vector2D a, Vector2D b) {
            Vector2D ab = b.minus(a);
            double len2 = ab.dot(ab);
            if (len2 < 1e-9) return q.distance(a);
            double u = Math.max(0, Math.min(1, q.minus(a).dot(ab) / len2));
            return q.distance(a.plus(ab.times(u)));
        }

        private static List<Vector2D> sampleLine(Vector2D a, Vector2D b) {
            List<Vector2D> out = new ArrayList<>();
            int count = Math.max(2, (int) Math.ceil(a.distance(b) / Math.max(0.25, SAMPLE_SPACING)));
            for (int k = 0; k <= count; k++) out.add(a.plus(b.minus(a).times(k / (double) count)));
            return out;
        }

        private static Vector2D cubic(Vector2D p0, Vector2D p1, Vector2D p2, Vector2D p3, double t) {
            double u = 1 - t;
            double b0 = u * u * u, b1 = 3 * u * u * t, b2 = 3 * u * t * t, b3 = t * t * t;
            return Vector2D.cartesian(
                    b0 * p0.x() + b1 * p1.x() + b2 * p2.x() + b3 * p3.x(),
                    b0 * p0.y() + b1 * p1.y() + b2 * p2.y() + b3 * p3.y());
        }

        private static double polyLength(List<Vector2D> pts) {
            double len = 0;
            for (int i = 1; i < pts.size(); i++) len += pts.get(i).distance(pts.get(i - 1));
            return len;
        }

        private static double polylineLength(Vector2D s, List<Vector2D> pts) {
            double len = 0;
            Vector2D a = s;
            for (Vector2D b : pts) { len += a.distance(b); a = b; }
            return len;
        }

        private static boolean inField(Vector2D p) {
            return p.x() >= WALL_MARGIN && p.x() <= FIELD_SIZE - WALL_MARGIN
                    && p.y() >= WALL_MARGIN && p.y() <= FIELD_SIZE - WALL_MARGIN;
        }

        private static boolean allInField(List<Vector2D> pts) {
            for (Vector2D p : pts) if (!inField(p)) return false;
            return true;
        }
    }
}
