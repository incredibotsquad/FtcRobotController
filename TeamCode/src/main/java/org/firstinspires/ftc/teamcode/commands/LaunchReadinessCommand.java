package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.common.DebugTelemetry;

import android.util.Log;

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
 * Firing is NOT done here: LaunchBallsCommand opens the gate on the operator's press and trusts
 * this command to have the launcher ready. isReadyToLaunch() drives the alignment light, so the
 * operator should fire on green.
 *
 * When LaunchSubsystem.isLaunchReadinessLocked() is set, the only change is that the turret is
 * driven to LaunchSubsystem.TURRET_MID and kept there instead of tracking: a fully manual turret
 * mode where the driver aims by turning the robot. The target, flywheel RPM and shot validity are
 * still computed live every loop, but readiness ignores turret aim (no on-target or reachability
 * check). It only waits for the turret to finish the move to TURRET_MID.
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
     * Outside the table the nearest end RPM is used, and the shot still counts as ready (no
     * waiting for the robot to get into range).
     */
    public static double[] SHOT_DISTANCES_IN = {42, 43.5, 44, 47, 48.75, 52, 66, 88};
    public static double[] SHOT_RPMS = {2515, 2515, 2515, 2668, 2668, 2731, 2878, 3170};
    /** Out-of-table distance already logged (logs once each time the robot leaves the table range). */
    private boolean loggedOutOfRange = false;

    // ------------------------------------------------------------------ turret geometry
    /** Servo position at which the turret points straight out the front of the robot. */
    public static double TURRET_FORWARD_POSITION = LaunchSubsystem.TURRET_MID;
    /** +1 if increasing servo position turns the turret counter-clockwise (seen from above), else -1. */
    public static double TURRET_DIRECTION = 1.0;
    /**
     * Turret travel correction: multiplies the servo position change per degree of aim.
     * The goBILDA 5-turn servo covers its 1800 deg over a 500-2500 us pulse, but the hub's default
     * servo pulse range is 600-2400 us, so positions 0..1 only reach 90% of that (~1620 deg) and every
     * move falls 10% short without this. 1 / 0.9 = 1.111. Last season's tuned value was 1.1.
     * Tune with TurretAimTest: the error should no longer grow with the turret angle.
     * 1.0 = no correction (old behavior).
     */
    public static double TURRET_SCALING_FACTOR = 1;

    /**
     * Servos give no position feedback, so readiness estimates where the turret is: the estimate
     * moves toward the commanded position at TURRET_SERVO_RPM. The turret counts as settled
     * once the estimate is within TURRET_SETTLE_THRESHOLD of the command and has stayed there for
     * TURRET_SETTLE_MS (for the overshoot/wobble at the end of a move). The wait therefore scales
     * with how far the turret has to travel, and a retarget mid-move continues from the estimate.
     */
    public static double TURRET_SETTLE_THRESHOLD = 0.01;
    public static long TURRET_SETTLE_MS = 150;
    /**
     * Servo output shaft speed under load (RPM). The goBILDA 5-turn is ~230 RPM at 6 V unloaded;
     * 200 assumes some loss under the turret's load. 200 RPM = 1200 deg/s = ~0.74 servo position
     * units/s (1200 / 1800 x TURRET_SCALING_FACTOR, see turretSpeedPosPerSec()), so a full
     * TURRET_MIN -> TURRET_MAX move (0.8) is ~1.1 s and a 360 deg turret flip is ~1.0 s.
     * Too high and the light goes green mid-swing.
     * 0 or less = treat moves as instant (only TURRET_SETTLE_MS is waited).
     */
    public static double TURRET_SERVO_RPM = 200;

    /**
     * TURRET_SERVO_RPM converted to servo position units per second. Scaled like the aim math: each
     * position unit is only TOTAL_SERVO_RANGE / TURRET_SCALING_FACTOR degrees of servo travel.
     */
    public static double turretSpeedPosPerSec() {
        return TURRET_SERVO_RPM * 360.0 / 60.0 / LaunchSubsystem.TOTAL_SERVO_RANGE * TURRET_SCALING_FACTOR;
    }

    // ------------------------------------------------------------------ moving shots
    /** Toggled by the operator. When on, the target is led by robot velocity x flight time. */
    public static boolean ENABLE_MOVING_SHOT_COMPENSATION = false;
    /** Average horizontal ball speed, used to estimate flight time. Tune from video/testing. */
    public static double BALL_SPEED_IN_PER_SEC = 190;
    /** With compensation off, no shot is offered above this robot speed. */
    public static double MAX_STATIONARY_SHOT_SPEED_IN_PER_SEC = 4.0;

    private final LaunchSubsystem launcher;
    private final OdometrySubsystem odometry;
    private final TelemetryManager telemetry;

    /** Last commanded turret position. */
    private double lastTurretPosition = Double.NaN;
    /** Where the turret probably is right now (see TURRET_SERVO_RPM). */
    private double estimatedTurretPosition = Double.NaN;
    private long lastEstimateMs = 0;
    /** When the estimate last arrived at the commanded position. */
    private long turretArrivedAtMs = 0;
    private String notReadyReason = "";
    /** Distance (in) and RPM target from the last execute(), for FlywheelTuner. NaN before the first. */
    private double lastDistanceIn = Double.NaN;
    private double lastRpmTarget = Double.NaN;
    /** Turret debug: log the first aim decision of each run (logcat tag TurretDebug). */
    private boolean loggedFirstAim = false;

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
        estimatedTurretPosition = lastTurretPosition;
        lastEstimateMs = nowMs();
        turretArrivedAtMs = lastEstimateMs;
        loggedFirstAim = false;
        Log.i("TurretDebug", String.format("LaunchReadiness initialize: lastTurretPosition %.3f (from storage)", lastTurretPosition));
    }

    @Override
    public void execute() {
        boolean turretLocked = launcher.isLaunchReadinessLocked();

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
        lastDistanceIn = distance;

        // ---------------------------------------------------- turret
        double relativeDeg = relativeTurretDeg(pose, aimPoint);
        if (turretLocked) {
            // Locked: park the turret at TURRET_MID. Done before choosing the aim position so the
            // wrap (the turret can reach some angles two ways) is chosen relative to TURRET_MID.
            double mid = LaunchSubsystem.TURRET_MID;
            launcher.setTurretPosition(mid);
            lastTurretPosition = mid;
        }

        double turretPosition = chooseTurretPosition(relativeDeg);
        boolean turretReachable = !Double.isNaN(turretPosition);

        if (!turretLocked && turretReachable) {
            launcher.setTurretPosition(turretPosition);
            lastTurretPosition = turretPosition;
        }
        boolean turretSettled = updateTurretSettled();

        if (!loggedFirstAim) {
            loggedFirstAim = true;
            Log.i("TurretDebug", String.format(
                    "LaunchReadiness first aim: pose %s | cell %s %s at %s | aimPoint %s | relDeg %.1f | chosen %.3f | locked %b | fwdPos %.3f dir %.0f ratio %.3f min %.2f max %.2f",
                    pose, CrossOpModeStorage.allianceColor, LaunchTargets.getLastSelectedCell(), target, aimPoint,
                    relativeDeg, turretPosition, turretLocked, TURRET_FORWARD_POSITION, TURRET_DIRECTION,
                    LaunchSubsystem.GEAR_RATIO, LaunchSubsystem.TURRET_MIN, LaunchSubsystem.TURRET_MAX));
        }

        // ---------------------------------------------------- flywheel (always running)
        double rpm = rpmForDistance(distance);
        lastRpmTarget = rpm;
        launcher.updateFlywheel(rpm, distance);

        // ---------------------------------------------------- shot validity
        // Outside the table: rpmForDistance() already snapped to the end RPM; shoot with it.
        boolean inRange = distance >= SHOT_DISTANCES_IN[0]
                && distance <= SHOT_DISTANCES_IN[SHOT_DISTANCES_IN.length - 1];
        if (!inRange && !loggedOutOfRange) {
            Log.i("LaunchReadiness", String.format("distance %.1f in is outside the shot table (%.0f-%.0f): using %.0f RPM",
                    distance, SHOT_DISTANCES_IN[0], SHOT_DISTANCES_IN[SHOT_DISTANCES_IN.length - 1], rpm));
        }
        loggedOutOfRange = !inRange;
        boolean slowEnough = ENABLE_MOVING_SHOT_COMPENSATION || speed <= MAX_STATIONARY_SHOT_SPEED_IN_PER_SEC;

        // Locked = manual turret: where the turret points is the driver's job, so aim isn't checked.
        boolean turretAimOk = turretLocked || turretReachable;

        if (!turretAimOk) notReadyReason = "turret can't reach";
        else if (!slowEnough) notReadyReason = "moving too fast";
        else if (!turretSettled) notReadyReason = "turret settling";
        else notReadyReason = "";

        boolean solutionValid = notReadyReason.isEmpty();
        launcher.setShotSolutionReady(solutionValid);
        launcher.setShotPossible(turretAimOk);

        // ---------------------------------------------------- telemetry
        if (DebugTelemetry.ENABLED) telemetry.addData("Launch: Target cell", CrossOpModeStorage.allianceColor + " " + LaunchTargets.getLastSelectedCell());
        if (DebugTelemetry.ENABLED) telemetry.addData("Launch: Distance (in)", Math.round(distance * 10) / 10.0
                + (inRange ? "" : " (outside table, end RPM)"));
        if (DebugTelemetry.ENABLED) telemetry.addData("Launch: Turret angle (deg)", Math.round(relativeDeg));
        if (DebugTelemetry.ENABLED) telemetry.addData("Launch: Turret position", turretLocked ? LaunchSubsystem.TURRET_MID + " (LOCKED)" : String.valueOf(turretReachable ? turretPosition : -1));
        if (DebugTelemetry.ENABLED) telemetry.addData("Launch: RPM target / actual", Math.round(rpm) + " / " + Math.round(launcher.getCurrentFlywheelRPM()));
        if (DebugTelemetry.ENABLED) telemetry.addData("Launch: Solution", solutionValid ? "READY" : notReadyReason);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    /** Robot-to-aim-point distance (in) used for the last RPM / gain lookup. NaN before the first loop. */
    public double getLastDistanceIn() {
        return lastDistanceIn;
    }

    /** RPM target from the shot table on the last loop. NaN before the first loop. */
    public double getLastRpmTarget() {
        return lastRpmTarget;
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
        return chooseTurretPosition(relativeDeg,
                Double.isNaN(lastTurretPosition) ? TURRET_FORWARD_POSITION : lastTurretPosition);
    }

    /** Servo position change per degree of turret rotation (CCW positive when TURRET_DIRECTION = +1). */
    public static double turretPositionPerDegree() {
        return TURRET_DIRECTION * LaunchSubsystem.GEAR_RATIO / LaunchSubsystem.TOTAL_SERVO_RANGE * TURRET_SCALING_FACTOR;
    }

    /**
     * Angle (deg, CCW from robot forward, in (-180, 180]) the turret must point to face {@code aimPoint}.
     */
    public static double relativeTurretDeg(Pose2d pose, Translation2d aimPoint) {
        double dx = aimPoint.getX() - pose.getX();
        double dy = aimPoint.getY() - pose.getY();
        double bearingDeg = Math.toDegrees(Math.atan2(dy, dx));
        return normalizeDeg(bearingDeg - Math.toDegrees(pose.getHeading()));
    }

    /** Inverse of the aiming math: turret angle (deg, CCW from robot forward) for a servo position. */
    public static double turretPositionToRelativeDeg(double position) {
        return (position - TURRET_FORWARD_POSITION) / turretPositionPerDegree();
    }

    /**
     * Pure version of the turret choice: the reachable servo position for {@code relativeDeg}
     * closest to {@code current}, or NaN if no wrap lands inside [TURRET_MIN, TURRET_MAX].
     */
    public static double chooseTurretPosition(double relativeDeg, double current) {
        double perDegree = turretPositionPerDegree();

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

    /**
     * Moves the turret position estimate toward the last command at TURRET_SERVO_RPM and
     * returns true once it has arrived and TURRET_SETTLE_MS has passed since arriving.
     */
    private boolean updateTurretSettled() {
        long now = nowMs();
        double dtSec = (now - lastEstimateMs) / 1000.0;
        lastEstimateMs = now;

        if (Double.isNaN(lastTurretPosition)) return false;
        if (Double.isNaN(estimatedTurretPosition)) estimatedTurretPosition = lastTurretPosition;

        double error = lastTurretPosition - estimatedTurretPosition;
        double speed = turretSpeedPosPerSec();
        double step = speed * dtSec;
        if (speed <= 0 || Math.abs(error) <= step) {
            estimatedTurretPosition = lastTurretPosition;
        } else {
            estimatedTurretPosition += Math.signum(error) * step;
        }

        boolean arrived = Math.abs(lastTurretPosition - estimatedTurretPosition) <= TURRET_SETTLE_THRESHOLD;
        if (!arrived) turretArrivedAtMs = now; // still travelling: the settle clock starts on arrival
        return arrived && now - turretArrivedAtMs >= TURRET_SETTLE_MS;
    }

    /** Estimated current turret servo position (for telemetry / tests). */
    public double getEstimatedTurretPosition() {
        return estimatedTurretPosition;
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

    public static double normalizeDeg(double deg) {
        deg %= 360.0;
        if (deg > 180.0) deg -= 360.0;
        if (deg <= -180.0) deg += 360.0;
        return deg;
    }

    private static long nowMs() {
        return System.nanoTime() / 1_000_000L;
    }
}
