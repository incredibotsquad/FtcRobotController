package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.common.CrossOpModeStorage;
import org.firstinspires.ftc.teamcode.common.LaunchTargets;
import org.firstinspires.ftc.teamcode.subsystems.LaunchSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;

/**
 * Default command for the launcher. Every loop it:
 *  1. picks the target CELL (LaunchTargets) from the robot pose,
 *  2. optionally leads the target for shooting while moving,
 *  3. aims the turret (choosing the reachable angle closest to where the turret is now),
 *  4. runs the flywheel at the RPM for that distance (always, so shots are never waiting on spin-up),
 *  5. reports whether a valid shot exists via LaunchSubsystem.setShotSolutionReady().
 *
 * Firing is NOT done here: LaunchBallsCommand / AutoFireCommand open the gate once
 * LaunchSubsystem.isReadyToLaunch() is true.
 *
 * When LaunchSubsystem.isLaunchReadinessLocked() is set, the last turret position and RPM
 * are held (flywheel control keeps running).
 *
 * You need to tune these (all live in Panels):
 * - SHOT_DISTANCES_IN / SHOT_RPMS: placeholder values. Fill them in from test shots.
 * - TURRET_FORWARD_POSITION: the servo position where the turret points straight forward. It defaults
 *   to TURRET_MID.
 * - TURRET_DIRECTION: flip it to −1 if the turret turns the wrong way.
 * - BALL_SPEED_IN_PER_SEC and MAX_STATIONARY_SHOT_SPEED_IN_PER_SEC.
 *
 */
@Configurable
public class LaunchReadinessCommand extends CommandBase {

    // ------------------------------------------------------------------ shot table
    /**
     * Distance (inches, robot center to CELL) -> flywheel RPM. PLACEHOLDERS: fill from test shots.
     * Must be the same length, sorted by distance. Values in between are linearly interpolated.
     */
    public static double[] SHOT_DISTANCES_IN = {24, 48, 72, 96, 120};
    public static double[] SHOT_RPMS = {900, 1000, 1100, 1250, 1400};

    // ------------------------------------------------------------------ turret geometry
    /** Servo position at which the turret points straight out the front of the robot. */
    public static double TURRET_FORWARD_POSITION = LaunchSubsystem.TURRET_MID;
    /** +1 if increasing servo position turns the turret counter-clockwise (seen from above), else -1. */
    public static double TURRET_DIRECTION = 1.0;

    /** After the turret target moves more than this (servo units), wait TURRET_SETTLE_MS before shooting. */
    public static double TURRET_SETTLE_THRESHOLD = 0.01;
    /** Servos give no position feedback, so readiness waits this long after a large move. */
    public static long TURRET_SETTLE_MS = 150;

    // ------------------------------------------------------------------ moving shots
    /** Toggled by the operator. When on, the target is led by robot velocity x flight time. */
    public static boolean ENABLE_MOVING_SHOT_COMPENSATION = false;
    /** Average horizontal ball speed, used to estimate flight time. Tune from video/testing. */
    public static double BALL_SPEED_IN_PER_SEC = 200.0;
    /** With compensation off, no shot is offered above this robot speed. */
    public static double MAX_STATIONARY_SHOT_SPEED_IN_PER_SEC = 4.0;

    private final LaunchSubsystem launcher;
    private final OdometrySubsystem odometry;
    private final TelemetryManager telemetry;

    private double lastTurretPosition = Double.NaN;
    private double lastRpm = 0;
    private boolean lastSolutionValid = false;
    private boolean lastShotPossible = true;
    private long turretMovedAtMs = 0;
    private String notReadyReason = "";

    public LaunchReadinessCommand(LaunchSubsystem launcher, OdometrySubsystem odometry, TelemetryManager telemetry) {
        this.launcher = launcher;
        this.odometry = odometry;
        this.telemetry = telemetry;
        // Odometry is only read, so it is not a requirement.
        addRequirements(launcher);
    }

    @Override
    public void initialize() {
        lastTurretPosition = CrossOpModeStorage.turretPosition;
        turretMovedAtMs = nowMs();
    }

    @Override
    public void execute() {
        if (launcher.isLaunchReadinessLocked()) {
            // Hold the last solution; the flywheel controller still needs updating every loop.
            launcher.updateFlywheel(lastRpm);
            launcher.setShotSolutionReady(lastSolutionValid);
            launcher.setShotPossible(lastShotPossible);
            telemetry.addData("Launch: ", "LOCKED (rpm " + Math.round(lastRpm) + ")");
            return;
        }

        Pose2d pose = odometry.getPose();
        Translation2d robot = pose.getTranslation();
        Translation2d target = LaunchTargets.getTarget(pose, CrossOpModeStorage.allianceColor);

        // ---------------------------------------------------- lead the target when moving
        double speed = odometry.getFieldSpeedInchesPerSecond();
        Translation2d aimPoint = target;
        if (ENABLE_MOVING_SHOT_COMPENSATION && BALL_SPEED_IN_PER_SEC > 0) {
            Translation2d velocity = odometry.getFieldVelocity();
            // Two passes: flight time depends on distance, which depends on the lead.
            for (int i = 0; i < 2; i++) {
                double flightTime = robot.getDistance(aimPoint) / BALL_SPEED_IN_PER_SEC;
                aimPoint = target.minus(velocity.times(flightTime));
            }
        }

        double dx = aimPoint.getX() - robot.getX();
        double dy = aimPoint.getY() - robot.getY();
        double distance = Math.hypot(dx, dy);

        // ---------------------------------------------------- turret
        double bearingDeg = Math.toDegrees(Math.atan2(dy, dx));
        double relativeDeg = normalizeDeg(bearingDeg - Math.toDegrees(pose.getHeading()));
        double turretPosition = chooseTurretPosition(relativeDeg);
        boolean turretReachable = !Double.isNaN(turretPosition);

        if (turretReachable) {
            if (Double.isNaN(lastTurretPosition)
                    || Math.abs(turretPosition - lastTurretPosition) > TURRET_SETTLE_THRESHOLD) {
                turretMovedAtMs = nowMs();
            }
            launcher.setTurretPosition(turretPosition);
            lastTurretPosition = turretPosition;
        }
        boolean turretSettled = nowMs() - turretMovedAtMs >= TURRET_SETTLE_MS;

        // ---------------------------------------------------- flywheel (always running)
        double rpm = rpmForDistance(distance);
        launcher.updateFlywheel(rpm);
        lastRpm = rpm;

        // ---------------------------------------------------- shot validity
        boolean inRange = distance >= SHOT_DISTANCES_IN[0]
                && distance <= SHOT_DISTANCES_IN[SHOT_DISTANCES_IN.length - 1];
        boolean slowEnough = ENABLE_MOVING_SHOT_COMPENSATION || speed <= MAX_STATIONARY_SHOT_SPEED_IN_PER_SEC;

        if (!turretReachable) notReadyReason = "turret can't reach";
        else if (!inRange) notReadyReason = "out of range";
        else if (!slowEnough) notReadyReason = "moving too fast";
        else if (!turretSettled) notReadyReason = "turret settling";
        else notReadyReason = "";

        lastSolutionValid = notReadyReason.isEmpty();
        lastShotPossible = turretReachable && inRange;
        launcher.setShotSolutionReady(lastSolutionValid);
        launcher.setShotPossible(lastShotPossible);

        // ---------------------------------------------------- telemetry
        telemetry.addData("Launch: Target cell", CrossOpModeStorage.allianceColor + " " + LaunchTargets.getLastSelectedCell());
        telemetry.addData("Launch: Distance (in)", Math.round(distance * 10) / 10.0);
        telemetry.addData("Launch: Turret angle (deg)", Math.round(relativeDeg));
        telemetry.addData("Launch: Turret position", turretReachable ? turretPosition : -1);
        telemetry.addData("Launch: RPM target / actual", Math.round(rpm) + " / " + Math.round(launcher.getCurrentFlywheelRPM()));
        telemetry.addData("Launch: Solution", lastSolutionValid ? "READY" : notReadyReason);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public String getNotReadyReason() {
        return notReadyReason;
    }

    // ------------------------------------------------------------------ helpers

    /**
     * The turret travels more than 360 degrees, so an angle may be reachable two ways.
     * Returns the reachable servo position closest to the current one, or NaN if none.
     */
    private double chooseTurretPosition(double relativeDeg) {
        double perDegree = TURRET_DIRECTION * LaunchSubsystem.GEAR_RATIO / LaunchSubsystem.TOTAL_SERVO_RANGE;
        double current = Double.isNaN(lastTurretPosition) ? TURRET_FORWARD_POSITION : lastTurretPosition;

        double best = Double.NaN;
        for (int wrap = -1; wrap <= 1; wrap++) {
            double candidate = TURRET_FORWARD_POSITION + (relativeDeg + 360.0 * wrap) * perDegree;
            if (candidate < LaunchSubsystem.TURRET_MIN || candidate > LaunchSubsystem.TURRET_MAX) continue;
            if (Double.isNaN(best) || Math.abs(candidate - current) < Math.abs(best - current)) {
                best = candidate;
            }
        }
        return best;
    }

    /** Linear interpolation over the shot table, clamped to its ends so the flywheel always has a target. */
    static double rpmForDistance(double distance) {
        int n = Math.min(SHOT_DISTANCES_IN.length, SHOT_RPMS.length);
        if (n == 0) return 0;
        if (distance <= SHOT_DISTANCES_IN[0]) return SHOT_RPMS[0];
        for (int i = 1; i < n; i++) {
            if (distance <= SHOT_DISTANCES_IN[i]) {
                double t = (distance - SHOT_DISTANCES_IN[i - 1]) / (SHOT_DISTANCES_IN[i] - SHOT_DISTANCES_IN[i - 1]);
                return SHOT_RPMS[i - 1] + t * (SHOT_RPMS[i] - SHOT_RPMS[i - 1]);
            }
        }
        return SHOT_RPMS[n - 1];
    }

    private static double normalizeDeg(double deg) {
        deg %= 360.0;
        if (deg > 180.0) deg -= 360.0;
        if (deg <= -180.0) deg += 360.0;
        return deg;
    }

    private static long nowMs() {
        return System.nanoTime() / 1_000_000L;
    }
}
