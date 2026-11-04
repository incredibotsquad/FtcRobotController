package org.firstinspires.ftc.teamcode.Vision;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RepeatCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Vision.PollenTrackerSubsystem.TrackedBall;
import org.firstinspires.ftc.teamcode.Vision.PollenTrackerSubsystem.TrackedGroup;
import org.firstinspires.ftc.teamcode.commands.DriveRobotCommand;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * Example: read PollenTrackerSubsystem and drive to Pollen with Pedro Pathing.
 *
 * The Pedro version of moveToBall. Instead of turn/drive P controllers and a timed blind
 * finish, each chase is a Pedro path to the target's FIELD position. When the balls drop out
 * of the camera's view up close, Pedro keeps driving to that remembered point on odometry.
 *
 * SETUP
 *   DriveSubsystem is created first (its periodic() runs follower.update()), then the tracker,
 *   so vision always sees this loop's pose. The tracker gets the pose from drive::getPose.
 *   DriveSubsystem stays the only owner of the Pedro follower.
 *
 * CONTROLS (gamepad 1)
 *   init:  X = calibrate the camera on one Pollen (straight ahead, CAL_FLOOR_DIST away)
 *   run:   A = start chasing (one target after another)   B = stop
 *          sticks drive the robot whenever it isn't chasing (the team's DriveRobotCommand)
 *   Nothing that can move the robot is set up before START.
 *
 * ONE CHASE (DriveToPollenCommand)
 *   1. pick the best group (or the nearest ball, TARGET_GROUPS = false) and lock onto it
 *   2. FACE:   follower.hold() a pose turned toward it, so the camera gets a good look
 *   3. SETTLE: wait SETTLE_S for steady readings
 *   4. FOLLOW: Paths.line(robot -> STOP_DIST short of the target).facingPoint(target), at
 *              CHASE_SPEED. Re-plans while the target is still in view and its estimate moves.
 *   5. done when !follower.isBusy() (Pedro's "finished" check), then hold, wait REARM_S, repeat.
 *
 * It only drives. See the "INTAKE:" comments for where intake code would go.
 */
@Configurable
@TeleOp(name = "Pollen Pedro Example", group = "Vision")
public class PollenPedroExample extends CommandOpMode {

    // ---- Target ----
    public static boolean TARGET_GROUPS = true;      // true = best group (blobTrack), false = nearest ball
    public static double MAX_CHASE_DIST = 0;         // ignore targets further than this (in); 0 = no limit

    // ---- Driving ----
    public static double STOP_DIST = 6.0;            // robot center stops this far short of the aim point (in)
    public static double CHASE_SPEED = 0.6;          // fraction of the robot's max speed
    public static double FACE_TOL_DEG = 5.0;         // facing the target this well -> settle
    public static double FACE_TIMEOUT_S = 1.5;
    public static double SETTLE_S = 0.25;

    // ---- Re-planning while driving ----
    public static double REPLAN_SHIFT = 2.0;         // target estimate moved this much (in) -> new path
    public static double REPLAN_MIN_S = 0.3;         // at most this often
    public static double NO_REPLAN_INSIDE = 14.0;    // this close, the camera is losing it: trust the plan

    // ---- Safety ----
    public static double LOST_S = 0.5;               // target unseen this long before driving -> re-pick
    public static double MIN_FOLLOW_S = 0.2;         // let a new path latch before asking isBusy()
    public static double CHASE_TIMEOUT_S = 8.0;
    public static double REARM_S = 0.5;              // pause between chases

    // ---- Start pose (only relative positions matter for this example) ----
    public static double START_X = 72, START_Y = 72, START_HEADING_DEG = 90;

    private DriveSubsystem drive;
    private PollenTrackerSubsystem vision;
    private TelemetryManager telemetryM;
    private DriveToPollenCommand chase;
    private RepeatCommand chaseLoop;

    @Override
    public void initialize() {
        reset();                                                    // clear any old scheduler state
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        drive = new DriveSubsystem(hardwareMap, telemetryM);          // 1st: updates the pose
        vision = new PollenTrackerSubsystem(hardwareMap, telemetryM); // 2nd: sees this loop's pose
        vision.setPoseSupplier(drive::getPose);
        drive.setPose(new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG)));
    }

    /** Runs once at START. Default commands and buttons are bound here, never during init. */
    private void onStart() {
        GamepadEx driver = new GamepadEx(gamepad1);
        drive.setDefaultCommand(new DriveRobotCommand(drive, driver));

        chase = new DriveToPollenCommand(drive, vision);
        chaseLoop = new RepeatCommand(new SequentialCommandGroup(
                chase,
                new WaitCommand((long) (REARM_S * 1000))));

        driver.getGamepadButton(GamepadKeys.Button.A).whenPressed(new Runnable() {
            @Override
            public void run() {
                if (!chaseLoop.isScheduled()) chaseLoop.schedule();
            }
        });
        driver.getGamepadButton(GamepadKeys.Button.B).whenPressed(new Runnable() {
            @Override
            public void run() {
                chaseLoop.cancel();                                 // sticks take over again
            }
        });
    }

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            initialize();

            // INIT: CommandOpMode normally skips the scheduler here, but the camera needs it
            // (exposure, calibration). Nothing drives: no default command, no buttons yet.
            boolean lastX = false;
            while (opModeInInit()) {
                boolean x = gamepad1.x;
                if (x && !lastX) vision.startCalibration();
                lastX = x;

                run();
                telemetryM.addLine("INIT: X = calibrate on ONE Pollen straight ahead");
                telemetryM.addData("Calibration", vision.getCalibrationStatus());
                telemetryM.update(telemetry);
            }
            waitForStart();
            if (isStopRequested()) return;

            onStart();
            while (opModeIsActive()) {
                run();
                telemetryM.addData("Chase", chaseLoop.isScheduled() ? chase.status() : "off  (A = start, B = stop)");
                telemetryM.update(telemetry);
            }
        } finally {
            if (vision != null) vision.close();
            reset();
        }
    }

    // =====================================================================
    // ===================== ONE CHASE, AS A COMMAND =======================
    // =====================================================================

    /**
     * Turns to the target, then drives to it on a Pedro path. Finishes on arrival, when
     * there's nothing to chase, or on timeout. Always ends holding the robot still.
     */
    static class DriveToPollenCommand extends CommandBase {
        private enum Phase { FACE, SETTLE, FOLLOW, DONE }

        private final DriveSubsystem drive;
        private final PollenTrackerSubsystem vision;

        private Phase phase = Phase.DONE;
        private int lockedId = -1;           // TARGET_GROUPS: the group's lead ball; else the ball
        private int lockedCount = 0;
        private double targetX, targetY;     // field frame, last good estimate
        private double plannedX, plannedY;   // target the current path was built for
        private double holdX, holdY, faceHeading;   // the turn-in-place hold
        private String status = "";

        private final ElapsedTime chaseTimer = new ElapsedTime();
        private final ElapsedTime phaseTimer = new ElapsedTime();
        private final ElapsedTime replanTimer = new ElapsedTime();
        private final ElapsedTime seenTimer = new ElapsedTime();

        DriveToPollenCommand(DriveSubsystem drive, PollenTrackerSubsystem vision) {
            this.drive = drive;
            this.vision = vision;
            addRequirements(drive);          // NOT vision: it's only read
        }

        String status() {
            return status;
        }

        @Override
        public void initialize() {
            chaseTimer.reset();
            holdHere();                      // whatever the sticks were doing, stop
            if (!lockOnBestTarget()) {
                finish("waiting for Pollen");
                return;
            }
            startFacing();
        }

        @Override
        public void execute() {
            if (phase == Phase.DONE) return;
            if (chaseTimer.seconds() > CHASE_TIMEOUT_S) {
                finish("timed out");
                return;
            }
            boolean seen = refreshTarget();  // updates targetX/Y if the locked target is tracked

            switch (phase) {
                case FACE:
                case SETTLE:
                    if (!seen && seenTimer.seconds() > LOST_S) {
                        // Lost it before driving: take the best target now, or give up
                        if (!lockOnBestTarget()) {
                            finish("lost the target");
                            return;
                        }
                        startFacing();
                        return;
                    }
                    if (phase == Phase.FACE) {
                        double want = headingToTarget();
                        // Estimate moved enough to change the aim: re-aim the hold (timer keeps running)
                        if (Math.abs(Angle.normalizeSigned(want - faceHeading)) > Math.toRadians(FACE_TOL_DEG)) {
                            aimHold(want);
                        }
                        double errDeg = Math.toDegrees(Angle.normalizeSigned(want - drive.getPose().heading()));
                        if (Math.abs(errDeg) <= FACE_TOL_DEG || phaseTimer.seconds() > FACE_TIMEOUT_S) {
                            phase = Phase.SETTLE;
                            phaseTimer.reset();
                            status = "settling";
                        }
                    } else if (phaseTimer.seconds() >= SETTLE_S) {
                        planAndFollow();
                    }
                    break;

                case FOLLOW:
                    // Still in view and the estimate moved: aim the path at the better estimate.
                    // Up close the camera can't see it; the path already ends at the right spot.
                    if (seen && distanceToTarget() > NO_REPLAN_INSIDE && replanTimer.seconds() > REPLAN_MIN_S
                            && Math.hypot(targetX - plannedX, targetY - plannedY) > REPLAN_SHIFT) {
                        planAndFollow();
                        break;
                    }
                    Follower f = drive.getFollower();
                    if (phaseTimer.seconds() > MIN_FOLLOW_S && !f.isBusy()) {
                        finish(String.format("arrived (%d ball target)", lockedCount));
                        // INTAKE: this is where a real chase would intake / wait for the ball sensor
                    }
                    break;

                default:
                    break;
            }
        }

        @Override
        public boolean isFinished() {
            return phase == Phase.DONE;
        }

        @Override
        public void end(boolean interrupted) {
            holdHere();
            if (interrupted) status = "stopped";
            // INTAKE: stop the intake here
        }

        // -------------------------------------------------------------- steps

        /** Turn in place toward the target: hold the current spot with the target's heading. */
        private void startFacing() {
            Pose pose = drive.getPose();
            holdX = pose.x();
            holdY = pose.y();
            aimHold(headingToTarget());
            phase = Phase.FACE;
            phaseTimer.reset();
            status = String.format("turning to %d ball target", lockedCount);
        }

        private void aimHold(double heading) {
            faceHeading = heading;
            drive.getFollower().hold(new Pose(holdX, holdY, heading));
        }

        private void planAndFollow() {
            Pose pose = drive.getPose();
            double[] end = endPoint(pose.x(), pose.y(), targetX, targetY, STOP_DIST);
            if (end == null) {
                finish("already there");
                return;
            }
            Path path = Paths.line(pose, new Pose(end[0], end[1], end[2]))
                    .facingPoint(Vector2D.cartesian(targetX, targetY))       // camera stays on it
                    .with(Constants.foresightConfig.maxPathSpeed.at(CHASE_SPEED));
            drive.getFollower().follow(path);
            plannedX = targetX;
            plannedY = targetY;
            if (phase != Phase.FOLLOW) phaseTimer.reset();
            phase = Phase.FOLLOW;
            replanTimer.reset();
            status = String.format("driving to %d ball target, %.1f in", lockedCount, distanceToTarget());
            // INTAKE: start the intake here
        }

        private void finish(String why) {
            phase = Phase.DONE;
            status = why;
        }

        private void holdHere() {
            drive.getFollower().hold(drive.getPose());
        }

        // -------------------------------------------------------------- target

        /** Locks onto the best target right now. False if there is none (in range). */
        private boolean lockOnBestTarget() {
            if (TARGET_GROUPS) {
                TrackedGroup g = vision.getTargetGroup();
                if (g == null || !inRange(g.distance())) return false;
                lockedId = g.leadId;
                lockedCount = g.count;
                targetX = g.fieldX;
                targetY = g.fieldY;
            } else {
                TrackedBall b = vision.getNearestBall();
                if (b == null || !inRange(b.distance())) return false;
                lockedId = b.id;
                lockedCount = 1;
                targetX = b.fieldX;
                targetY = b.fieldY;
            }
            seenTimer.reset();
            return true;
        }

        /**
         * Follows the locked target as it is re-measured. Returns true if it was SEEN in the
         * latest frame. When it's not tracked any more, the last good position is kept.
         */
        private boolean refreshTarget() {
            boolean seen;
            if (TARGET_GROUPS) {
                TrackedGroup g = vision.getGroupContaining(lockedId);
                if (g == null) return false;
                lockedId = g.leadId;         // the nearest ball can change as we drive
                lockedCount = g.count;
                targetX = g.fieldX;
                targetY = g.fieldY;
                seen = g.seenThisFrame;
            } else {
                TrackedBall b = vision.getBall(lockedId);
                if (b == null) return false;
                targetX = b.fieldX;
                targetY = b.fieldY;
                seen = b.seenThisFrame;
            }
            if (seen) seenTimer.reset();
            return seen;
        }

        private static boolean inRange(double dist) {
            return MAX_CHASE_DIST <= 0 || dist <= MAX_CHASE_DIST;
        }

        private double headingToTarget() {
            Pose pose = drive.getPose();
            return Math.atan2(targetY - pose.y(), targetX - pose.x());
        }

        private double distanceToTarget() {
            Pose pose = drive.getPose();
            return Math.hypot(targetX - pose.x(), targetY - pose.y());
        }
    }

    /**
     * Where the robot center should stop: stopDist short of (tx, ty) on the line from the robot,
     * facing the target. Returns {x, y, heading} (field, radians), or null if already that close.
     */
    static double[] endPoint(double rx, double ry, double tx, double ty, double stopDist) {
        double dx = tx - rx, dy = ty - ry;
        double d = Math.hypot(dx, dy);
        if (d <= stopDist + 0.5) return null;
        double ux = dx / d, uy = dy / d;
        return new double[] { tx - ux * stopDist, ty - uy * stopDist, Math.atan2(dy, dx) };
    }
}
