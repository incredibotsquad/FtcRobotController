package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.Constants;

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
 */
@Configurable
public class DriveSubsystem extends SubsystemBase {

    /** Master switch for the launch hold. false = always manual driving. Live in Panels. */
    public static boolean LAUNCH_HOLD_ENABLED = true;

    /** Below these speeds the robot counts as stationary (used to gate Limelight relocalization). */
    public static double STATIONARY_SPEED_IN_PER_SEC = 1.0;
    public static double STATIONARY_TURN_RAD_PER_SEC = 0.1;

    private final Follower follower;
    private final TelemetryManager telemetry;

    private boolean launchHoldRequested = false;
    private boolean launchHoldBrokenOut = false;

    public DriveSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetry = telemetry;
        // Constants.create builds the Mecanum drivetrain + PinpointLocalizer (resets the Pinpoint, ~0.5 s).
        follower = Constants.create(hardwareMap);
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
        follower.update();

        if (org.firstinspires.ftc.teamcode.Incredibot.DRIVE_ONLY_MODE) {
            telemetry.addData("MODE", "DRIVE ONLY (turret locked at mid; intake, launcher, gate, Limelight off)");
        }

        telemetry.addData("Drive: mode", follower.mode()
                + (shouldHoldForLaunch() ? " (launch hold)" : launchHoldBrokenOut ? " (broke out)" : ""));
    }
}
