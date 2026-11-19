package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.common.DebugTelemetry;

import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.Map;

/**
 * Drivetrain, run entirely through Pedro Pathing's {@link Follower}.
 *
 * The Follower is the single owner of the drive motors AND the Pinpoint (its PinpointLocalizer,
 * configured in pedro/Constants). OdometrySubsystem reads the pose from here instead of talking to
 * the Pinpoint itself. periodic() calls follower.update() every loop, which reads the Pinpoint and
 * drives the motors for the current mode:
 *   IDLE   - motors at 0 (the state after construction, so nothing moves during INIT)
 *   MANUAL - driver powers from {@link #drive(double, double, double)}
 *   HOLD   - closed-loop hold of a fixed pose (launch hold, below)
 *   FOLLOW - following a path (autonomous)
 *
 * Launch hold: while a launch runs, LaunchBallsCommand calls requestLaunchHold(). DriveRobotCommand
 * then holds the robot's pose with Pedro's hold controller so defenders can't shove it off target,
 * and releases it if the driver moves a stick (breakout). LAUNCH_HOLD_ENABLED = false turns the
 * feature off immediately, including mid-hold, and leaves the driver in manual control.
 *
 * Hold power cap: while the launch hold is active, every drive wheel's power is limited to
 * HOLD_MAX_POWER (see {@link CappedDrivetrain}). Pushing against a defender at full power stalls all
 * four drive motors (~25 A from the battery measured), which helps blow the 20 A main fuse.
 *
 * Hold current cutoff: if the 4 drive motors together draw more than HOLD_CUTOFF_AMPS for
 * HOLD_CUTOFF_MS during the launch hold, the hold is dropped for the rest of that launch (motors to 0,
 * driver back in manual control), the same as a stick breakout. Logged with tag "DriveSubsystem".
 */
@Configurable
public class DriveSubsystem extends SubsystemBase {

    /** Master switch for the launch hold. false = always manual driving. Live in Panels. */
    public static boolean LAUNCH_HOLD_ENABLED = true;

    /**
     * Max drive wheel power (0-1) while the launch hold is active. 1.0 = no cap. Lower = less current
     * when a defender pushes, but the robot gets pushed off its spot more easily. Live in Panels.
     */
    public static double HOLD_MAX_POWER = 0.5;

    /**
     * Drop the launch hold when the 4 drive motors together draw more than this many amps. 0 = off.
     * Live in Panels.
     *
     * This is MOTOR current, not battery current:
     *  - Motor current = current through each motor's own wires, read at its hub port
     *    (DcMotorEx.getCurrent; Pedro's Mecanum.currentAmps() adds up the 4 drive motors). It says how
     *    hard the motors are pushing: more current = more torque. PowerLog prints it per motor
     *    ("frontLeftMotor 6.5 A"); this setting compares against frontLeft + frontRight + backLeft + backRight.
     *  - Hub (battery) current = what the battery feeds into each hub (LynxModule.getCurrent). PowerLog
     *    prints it as "Control Hub ... A", "Expansion Hub 2 ... A" and "hubs total". The 20 A main fuse
     *    only sees this one.
     *
     * Battery current is about (motor power x motor current). At part power the motor driver connects
     * the battery only part of the time, while the motor's current keeps flowing through the driver the
     * whole time, so the battery gives less than the motors read. Measured on our robot:
     *  - holding at low power after a push: motors 6.1 A, battery 1.4 A
     *  - hard push at full power (1.0): motors ~28 A, battery ~26 A
     *
     * With HOLD_MAX_POWER = 0.5 the battery gives about half of this number:
     *  7 A -> ~3.5 A battery, 10 A -> ~5 A, 14 A -> ~7 A.
     * Launch budget for the 20 A fuse: flywheel recovering ~6-8 A + transfer ~2 A + hubs/servos/Limelight
     * ~1 A = ~10-11 A, leaving ~9 A of battery current for the drive.
     * If HOLD_MAX_POWER goes up, the battery share goes up too (at 1.0 it is about the full number), so
     * lower this cutoff when raising the cap.
     */
    public static double HOLD_CUTOFF_AMPS = 14.0;

    /**
     * How long the current must stay above HOLD_CUTOFF_AMPS before the hold is dropped.
     * 0 = drop on the first reading over the limit (one loop, ~10-20 ms, is the fastest software can react).
     */
    public static double HOLD_CUTOFF_MS = 0;

    /** Below these speeds the robot counts as stationary (used to gate Limelight relocalization). */
    public static double STATIONARY_SPEED_IN_PER_SEC = 1.0;
    public static double STATIONARY_TURN_RAD_PER_SEC = 0.1;

    private final Follower follower;
    private final CappedDrivetrain cappedDrivetrain;
    private final TelemetryManager telemetry;

    private boolean launchHoldRequested = false;
    private boolean launchHoldBrokenOut = false;

    private long holdOverCurrentSinceNs = -1;
    private double lastHoldAmps = 0;

    public DriveSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetry = telemetry;
        // Constants.create builds the Mecanum drivetrain + PinpointLocalizer (resets the Pinpoint, ~0.5 s).
        // The Mecanum is wrapped so its power can be capped during the launch hold.
        final CappedDrivetrain[] wrapped = new CappedDrivetrain[1];
        follower = Constants.create(hardwareMap, d -> wrapped[0] = new CappedDrivetrain(d));
        cappedDrivetrain = wrapped[0];
    }

    /** For autonomous path following. */
    public Follower getFollower() {
        return follower;
    }

    // ---------------------------------------------------------------- manual driving

    /**
     * Robot-centric driver control. Pedro convention, powers in [-1, 1]:
     * forward = +front, strafe = +LEFT, turn = +counter-clockwise.
     * Ignored while a launch hold is active.
     */
    public void drive(double forward, double strafe, double turn) {
        follower.manual(forward, strafe, turn);
    }

    // ---------------------------------------------------------------- launch hold

    /** Called by LaunchBallsCommand when a launch starts. The hold engages once the sticks are centered. */
    public void requestLaunchHold() {
        launchHoldRequested = true;
        launchHoldBrokenOut = false;
    }

    /** Called by LaunchBallsCommand when the launch ends. Returns to manual control. */
    public void releaseLaunchHold() {
        launchHoldRequested = false;
        launchHoldBrokenOut = false;
        if (follower.holding()) {
            follower.manual(0, 0, 0);
        }
    }

    /** The driver moved a stick during the hold: stay manual for the rest of this launch. */
    public void breakOutOfLaunchHold() {
        if (follower.holding()) {
            Log.i("DriveSubsystem", "Driver broke out of launch hold");
        }
        launchHoldBrokenOut = true;
        follower.manual(0, 0, 0);
    }

    /** True when DriveRobotCommand should be holding instead of driving manually. */
    public boolean shouldHoldForLaunch() {
        return LAUNCH_HOLD_ENABLED && launchHoldRequested && !launchHoldBrokenOut;
    }

    public boolean isHolding() {
        return follower.holding();
    }

    /** Starts holding the current pose (no-op if already holding). */
    public void holdCurrentPose() {
        if (!follower.holding()) {
            Pose pose = follower.pose();
            Log.i("DriveSubsystem", "Launch hold engaged at " + pose);
            follower.hold(pose);
        }
    }

    // ---------------------------------------------------------------- pose / motion

    /** Current pose from the Pinpoint (Pedro frame: inches, heading radians). */
    public Pose getPose() {
        return follower.pose();
    }

    public void setPose(Pose pose) {
        follower.setPose(pose);
        // A held pose from before the reset would make the robot drive to the old coordinates.
        if (follower.holding()) {
            follower.hold(pose);
        }
    }

    public Velocity getVelocity() {
        return follower.velocity();
    }

    /** Used to gate Limelight relocalization. */
    public boolean isEffectivelyStationary() {
        Velocity v = follower.velocity();
        return v.linSpeed() < STATIONARY_SPEED_IN_PER_SEC && Math.abs(v.omega) < STATIONARY_TURN_RAD_PER_SEC;
    }

    @Override
    public void periodic() {
        checkHoldCurrent();

        // Cap only while actually holding for a launch; path following and manual driving stay at full power.
        boolean capHold = shouldHoldForLaunch() && follower.holding();
        cappedDrivetrain.setMaxPower(capHold ? HOLD_MAX_POWER : 1.0);

        follower.update();

        if (org.firstinspires.ftc.teamcode.Incredibot.DRIVE_ONLY_MODE) {
            if (DebugTelemetry.ENABLED) telemetry.addData("MODE", "DRIVE ONLY (turret locked at mid; intake, launcher, gate, Limelight off)");
        }

        if (DebugTelemetry.ENABLED) telemetry.addData("Drive: mode", follower.mode()
                + (shouldHoldForLaunch() ? " (launch hold)" : launchHoldBrokenOut ? " (broke out)" : ""));
        if (DebugTelemetry.ENABLED) telemetry.addData("Drive: max power", cappedDrivetrain.getMaxPower());
        if (DebugTelemetry.ENABLED) telemetry.addData("Drive: hold amps", String.format("%.1f", lastHoldAmps));
    }

    /**
     * During the launch hold, reads the drive motor current (4 extra hub reads per loop, only while
     * holding) and drops the hold if it stays above HOLD_CUTOFF_AMPS for HOLD_CUTOFF_MS.
     */
    private void checkHoldCurrent() {
        if (HOLD_CUTOFF_AMPS <= 0 || !shouldHoldForLaunch() || !follower.holding()) {
            holdOverCurrentSinceNs = -1;
            lastHoldAmps = 0;
            return;
        }
        double amps = cappedDrivetrain.currentAmps();
        lastHoldAmps = amps;
        if (Double.isNaN(amps) || amps <= HOLD_CUTOFF_AMPS) {
            holdOverCurrentSinceNs = -1;
            return;
        }
        long now = System.nanoTime();
        if (holdOverCurrentSinceNs < 0) {
            holdOverCurrentSinceNs = now;
        }
        double overMs = (now - holdOverCurrentSinceNs) / 1e6;
        if (overMs >= HOLD_CUTOFF_MS) {
            Log.w("DriveSubsystem", String.format("Launch hold dropped: drive current %.1f A > %.1f A for %.0f ms",
                    amps, HOLD_CUTOFF_AMPS, overMs));
            holdOverCurrentSinceNs = -1;
            launchHoldBrokenOut = true;
            follower.manual(0, 0, 0);
        }
    }

    /**
     * Wraps Pedro's drivetrain and limits the largest wheel power to maxPower. Mecanum wheel powers are
     * sums and differences of forward/strafe/turn, so scaling all three by the same factor scales every
     * wheel by that factor and keeps the direction of the push.
     */
    static class CappedDrivetrain implements Drivetrain {
        private final Drivetrain inner;
        private double maxPower = 1.0;

        CappedDrivetrain(Drivetrain inner) {
            this.inner = inner;
        }

        void setMaxPower(double maxPower) {
            this.maxPower = Math.max(0.0, Math.min(1.0, maxPower));
        }

        double getMaxPower() {
            return maxPower;
        }

        /** Sum of the 4 drive motors' current in amps (NaN if the drivetrain isn't Pedro's Mecanum). */
        double currentAmps() {
            return inner instanceof Mecanum ? ((Mecanum) inner).currentAmps() : Double.NaN;
        }

        @Override
        public void drive(DrivePowers powers, boolean manual) {
            if (maxPower < 1.0) {
                // Largest wheel power this command would produce (Mecanum normalizes to at most 1.0).
                double f = powers.forward(), s = powers.strafe(), t = powers.turn();
                double largestWheel = Math.max(Math.max(Math.abs(f - s - t), Math.abs(f + s - t)),
                        Math.max(Math.abs(f + s + t), Math.abs(f - s + t)));
                double effective = Math.min(largestWheel, 1.0);
                if (effective > maxPower) {
                    double k = maxPower / effective;
                    powers = new DrivePowers(f * k, s * k, t * k);
                }
            }
            inner.drive(powers, manual);
        }

        @Override
        public double maxScaling(DrivePowers a, DrivePowers b) {
            return inner.maxScaling(a, b);
        }

        @Override
        public void stop() {
            inner.stop();
        }

        @Override
        public void stop(boolean b) {
            inner.stop(b);
        }

        @Override
        public Map<String, Object> debug() {
            return inner.debug();
        }

        @Override
        public double interpolateVelocity(double a, double b, double c) {
            return inner.interpolateVelocity(a, b, c);
        }
    }
}
