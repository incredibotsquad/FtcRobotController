package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.common.DebugTelemetry;

import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Artifact counting
 * -----------------
 * Sensor 1 is the entrance, sensor 4 the top (just below the gate). Both are goBILDA laser distance
 * sensors in analog mode, each read by a LaserBallCounter (episode, seam and duration rules; thresholds
 * re-tuned with LaserIntakeTuner). Count = artifacts that entered - artifacts that left:
 *  - Entered: balls sensor 1 sees arrive while the intake runs forward (not while launching: the
 *    launch kick shuffles the stack past sensor 1).
 *  - Ejected: balls that have passed sensor 1 while the intake runs in reverse (also unjams).
 *  - Launched: balls that have passed sensor 4 while launching with the transfer forward. A ball
 *    resting on sensor 4 when the launch starts is re-timed then (LaserBallCounter.rebase), so its
 *    resting time doesn't look like two balls; likewise sensor 1 when the intake starts reversing.
 *    SENSOR4_DURATION_RULE is off by default: counting one ball too many at the exit would read
 *    empty with one left, and the intake would then take a 5th (G407). Counting too few only makes
 *    the launch run to MAX_FIRE_MS, where confirmEmptyAfterLaunch() fixes it.
 *  - Jam: sensor 1 blocked for JAM_MS while intaking means the artifact can't move up: full.
 *  - Sensor 2 full check (beam break, SENSOR2_FULL_ENABLED): with 4 POLLEN inside, the last one
 *    rests at sensor 2 (POLLEN is smaller than NECTAR, so a stack of 3 sits above it). Sensor 2
 *    blocked for SENSOR2_FULL_MS while the transfer runs forward (a ball that could move up would
 *    have) and not launching or reversing means full: the count is raised to MAX_ARTIFACTS (never
 *    lowered). That last ball is already inside, so there's no FULL_RUN_ON_MS run-on: the intake
 *    stops at once and the transfer TRANSFER_FULL_DELAY_MS later.
 *  - Sensor 4 leftover (SENSOR4_LEFTOVER_ENABLED): with the gate closed nothing gets past sensor 4,
 *    so sensor 4 blocked for SENSOR4_LEFTOVER_MS while not launching or reversing means at least one
 *    artifact inside. Catches a launch that fed n-1 of n but counted n exits (a seam miscount) and
 *    ended at 0: the one left rests at (or rides up to) sensor 4 once the gate closes, and the count
 *    goes 0 -> 1. Never lowers the count.
 *  - Sensor 3 minimum (beam break, SENSOR3_MIN_ENABLED, off until checked on the robot): sensor 3
 *    blocked the same way means at least SENSOR3_MIN_COUNT inside.
 *
 * When the count reaches MAX_ARTIFACTS while intaking, the intake stops automatically. On entering
 * FULL, SmartIntakeCommand first runs the intake motor (not the transfer) in reverse for
 * FULL_EJECT_MS (startFullEject()) to push out a 5th artifact straddling the intake (G407). Balls
 * leaving past sensor 1 then aren't subtracted: the 4 inside are in the transfer, so anything pushed
 * out is the straddler (counted on arrival and clamped at 4, or never counted).
 *
 * confirmEmptyAfterLaunch (called by LaunchBallsCommand when it hits MAX_FIRE_MS): if the transfer
 * has run forward with sensors 1 and 4 clear for at least EMPTY_CONFIRM_MS, any artifact inside
 * would have been carried past sensor 4 by then, so the robot is empty: count = 0.
 *
 * Every count change is logged (tag "IntakeCount") with its reason; laser episodes are logged with
 * tag "LaserBallCounter".
 *
 * Stall detection and unjamming
 * -----------------------------
 * Only the intake motor is watched (the transfer is not), while it is commanded forward and not
 * launching (LaunchBallsCommand drives the motors then). A stall is high current (STALL_SIGNAL = CURRENT,
 * works without encoders), near-zero encoder speed (VELOCITY, needs the encoder cable), or both at
 * once (BOTH, the most robust: a loaded but still-turning roller isn't a stall) lasting
 * STALL_TIME_MS, ignoring the first STALL_STARTUP_MS after the motor starts. The current is
 * low-pass filtered (STALL_CURRENT_FILTER_MS), so short spikes from an artifact squeezing through
 * the rollers don't count. Then the intake
 * motor reverses at UNJAM_REVERSE_POWER for UNJAM_REVERSE_MS (backs the stuck artifact out of the
 * rollers; the transfer keeps going), then goes forward again.
 * More than UNJAM_MAX_TRIES unjams within UNJAM_WINDOW_MS: the intake motor gives up and stays stopped
 * for UNJAM_GIVE_UP_MS (stops it burning out against a real jam), then tries again.
 * Commands' motor requests are remembered and reapplied after an unjam; a stop or reverse request
 * (e.g. full, ReverseIntakeCommand) cancels an unjam at once. Logged with tag "IntakeStall".
 */
@Configurable
public class IntakeSubsystem extends SubsystemBase {

    public static int MAX_ARTIFACTS = 4;

    /**
     * Laser background distance (mm) with the sensor clear, measured on the robot (shown in
     * Test: Beam Break Sensors). 0 = take the first reading at INIT, which is wrong if a ball is in
     * front then (e.g. AUTO preloads resting on sensor 4), so measure and set these.
     */
    public static double SENSOR1_BASELINE_MM = 108.2;
    public static double SENSOR4_BASELINE_MM = 98.9;
    /** Duration rule (a long deep episode = 2 balls) at the entrance: tuned on, errs toward overcounting. */
    public static boolean SENSOR1_DURATION_RULE = true;
    /** Duration rule at the exit: off, since overcounting exits risks a 5th artifact (see class comment). */
    public static boolean SENSOR4_DURATION_RULE = false;

    /**
     * Sensor 2 full check (see class comment): blocked this long with the transfer forward means the
     * 4th artifact is resting there. Must be longer than a ball takes to ride past sensor 2.
     */
    public static boolean SENSOR2_FULL_ENABLED = true;
    public static long SENSOR2_FULL_MS = 250;
    /** Sensor 4 leftover (see class comment): blocked this long with the gate closed = at least 1 inside. */
    public static boolean SENSOR4_LEFTOVER_ENABLED = true;
    public static long SENSOR4_LEFTOVER_MS = 150;
    /** Sensor 3 minimum (see class comment): off until checked where the balls rest with 3 inside. */
    public static boolean SENSOR3_MIN_ENABLED = false;
    public static int SENSOR3_MIN_COUNT = 3;
    public static long SENSOR3_MIN_MS = 250;

    /** Sensors 2/3 (digital beam breaks): set false if they read LOW when blocked. */
    public static boolean BLOCKED_READS_HIGH = true;

    /** Sensors 2/3: a sensor must hold a new state this long before it counts (filters flicker). */
    public static long DEBOUNCE_MS = 15;
    /** Sensor 1 blocked this long while intaking means the artifact can't move up: full. */
    public static long JAM_MS = 400;
    /**
     * confirmEmptyAfterLaunch(): how long the transfer must have run forward, while launching, with
     * sensors 1 and 4 clear before the robot is taken as empty. Must be longer than an artifact
     * takes to ride from the intake up past sensor 4.
     */
    public static long EMPTY_CONFIRM_MS = 1000;
    /**
     * When intaking fills the robot, keep the intake and transfer running this much longer so the
     * last artifact is pulled all the way in (the count hits MAX as it reaches sensor 1, before it is
     * fully inside). 0 = stop immediately (old behavior). Keep short: running on while full could
     * pick up a 5th artifact (G407).
     */
    public static long FULL_RUN_ON_MS = 1000;
    /**
     * Extra time the TRANSFER keeps running after the intake motor stops when intaking fills the
     * robot (on top of FULL_RUN_ON_MS), so the stack settles up against the gate. The intake motor
     * is already off, so nothing new is pulled in. 0 = transfer stops with the intake (old behavior).
     */
    public static long TRANSFER_FULL_DELAY_MS = 250;
    /** On reaching full: reverse the intake motor (not the transfer) this long to push out a straddler. 0 = off. */
    public static long FULL_EJECT_MS = 500;
    public static double FULL_EJECT_POWER = 1.0;

    // ---------------------------------------------------------------- stall detection (see class comment)
    public enum StallSignal { CURRENT, VELOCITY, BOTH }

    public static boolean STALL_DETECTION_ENABLED = false;
    /**
     * CURRENT works on any wiring; VELOCITY needs the motor's encoder cable plugged in; BOTH (high
     * current AND low speed) needs the encoder too and is the least likely to false-trigger.
     */
    public static StallSignal STALL_SIGNAL = StallSignal.CURRENT;
    /**
     * CURRENT / BOTH: stalled at or above this FILTERED draw (a goBILDA 5203 stalls around 9 A at
     * 12 V, less on a low battery; free-running about 0.5-2 A; pulling an artifact in, 3-6 A).
     */
    public static double STALL_CURRENT_A = 7.5;
    /**
     * CURRENT / BOTH: time constant of the current low-pass filter. Spikes much shorter than this
     * barely move it. 0 = no filtering (raw readings).
     */
    public static long STALL_CURRENT_FILTER_MS = 150;
    /** CURRENT: how often to read current (each read is a separate hub call, not bulk-cached). */
    public static long CURRENT_READ_INTERVAL_MS = 40;
    /** VELOCITY / BOTH: stalled at or below this encoder speed (ticks/s; free speed is about 2800). */
    public static double STALL_MAX_VELOCITY_TPS = 250;
    /** Only watch a motor commanded at least this much forward. */
    public static double STALL_MIN_POWER = 0.5;
    /** Ignore this long after a motor starts (startup current spike, zero speed while spinning up). */
    public static long STALL_STARTUP_MS = 300;
    /** Stall condition must hold this long, without a break. */
    public static long STALL_TIME_MS = 500;

    public static double UNJAM_REVERSE_POWER = 0.8;
    public static long UNJAM_REVERSE_MS = 250;
    public static int UNJAM_MAX_TRIES = 3;
    public static long UNJAM_WINDOW_MS = 4000;
    public static long UNJAM_GIVE_UP_MS = 1500;

    public static double ZERO_BALL_COLOR = 0;
    public static double ONE_BALL_COLOR = 0.29;   // RED
    public static double TWO_BALL_COLOR = 0.388;  // YELLOW
    public static double THREE_BALL_COLOR = 0.622; // BLUE
    public static double FOUR_BALL_COLOR = 0.5;   // GREEN

    private final MotorEx intakeMotor;
    private final MotorEx transferMotor;
    private final SimpleServo artifactCountLight;
    private final LaserBallCounter laser1;
    private final BeamBreak beamBreak2;
    private final BeamBreak beamBreak3;
    private final LaserBallCounter laser4;
    /** Laser totals already applied to the count. */
    private int s1SeenApplied, s1GoneApplied, s4GoneApplied;
    private boolean wasLaunching = false;
    private boolean wasReversing = false;
    /** Since when sensor 4 has been blocked while not launching or reversing; -1 = not now. */
    private long s4IdleBlockedSinceMs = -1;

    private final TelemetryManager telemetry;

    private final StallGuard intakeGuard;

    private int artifactCount = 0;
    /** While launching: since when the transfer has run forward with all sensors clear; -1 = not now. */
    private long feedClearSinceMs = -1;
    /** Intake keeps running until this time after intaking filled the robot (see FULL_RUN_ON_MS). */
    private long fullRunOnUntilMs = 0;
    /** Transfer keeps running until this time after intaking filled the robot (see TRANSFER_FULL_DELAY_MS). */
    private long transferRunOnUntilMs = 0;
    /** Full-eject intake reverse runs until this time (see FULL_EJECT_MS). */
    private long fullEjectUntilMs = 0;
    private boolean launching = false;
    /** Transfer power to use while launching instead of the default; NaN = no override. */
    private double launchTransferOverride = Double.NaN;
    /** Intake motor power to use while launching instead of off; NaN = no override (intake off). */
    private double launchIntakeOverride = Double.NaN;

    public IntakeSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetry = telemetry;
        intakeMotor = new MotorEx(hardwareMap, "intakeMotor", Motor.GoBILDA.BARE);
        intakeMotor.setInverted(true);
        intakeMotor.setRunMode(Motor.RunMode.RawPower);

        transferMotor = new MotorEx(hardwareMap, "transferMotor", Motor.GoBILDA.RPM_435);
        transferMotor.setInverted(true);
        transferMotor.setRunMode(Motor.RunMode.RawPower);

        laser1 = new LaserBallCounter(hardwareMap, "ballSensor1", SENSOR1_BASELINE_MM);
        beamBreak2 = new BeamBreak(hardwareMap, "ballSensor2");
        beamBreak3 = new BeamBreak(hardwareMap, "ballSensor3");
        laser4 = new LaserBallCounter(hardwareMap, "ballSensor4", SENSOR4_BASELINE_MM);

        artifactCountLight = new SimpleServo(hardwareMap, "artifactCountLight", 0, 270);

        intakeGuard = new StallGuard("intake", intakeMotor);
    }

    // ---------------------------------------------------------------- motor control

    /** Starts intake and transfer, unless already full. */
    public void startIntakeAndTransfer() {
        if (isFullAndDoneIntaking()) {
            intakeGuard.request(0);
            transferMotor.set(isFullAndTransferDone() ? 0 : 1); // transfer runs on TRANSFER_FULL_DELAY_MS
            return;
        }
        intakeGuard.request(1);
        transferMotor.set(1);
    }

    /** Stops intake and transfer. */
    public void stopIntakeAndTransfer() {
        intakeGuard.request(0);
        transferMotor.set(0);
    }

    /** Runs intake and transfer in reverse to eject artifacts. */
    public void reverseIntakeAndTransfer() {
        intakeGuard.request(-1);
        transferMotor.set(-1);
    }

    public boolean isOn() {
        return intakeMotor.get() > 0 || transferMotor.get() > 0;
    }

    public void setIntakeMotorPower(double power) {
        intakeGuard.request(power);
    }

    public void setTransferMotorPower(double power) {
        transferMotor.set(power);
    }

    public void stopIntakeMotorOnly() {
        intakeGuard.request(0);
    }

    public void stopTransferMotorOnly() {
        transferMotor.set(0);
    }

    /** The intake motor is currently reversing to clear a stall, or has given up after too many. */
    public boolean isUnjamming() {
        return intakeGuard.isOverriding();
    }

    // ---------------------------------------------------------------- counting

    /**
     * Call with true while feeding artifacts into the launcher (gate open, transfer running),
     * so artifacts leaving past sensor 4 are subtracted and the auto-stop doesn't fight the feed.
     */
    public void setLaunching(boolean launching) {
        this.launching = launching;
        if (launching) fullEjectUntilMs = 0;
    }

    /**
     * Full: reverses the intake motor (transfer untouched) for FULL_EJECT_MS to push out an artifact
     * straddling the intake. With FULL_EJECT_MS = 0 the intake just stops. Ends early on a launch.
     */
    public void startFullEject() {
        if (FULL_EJECT_MS <= 0) {
            stopIntakeMotorOnly();
            return;
        }
        fullEjectUntilMs = nowMs() + FULL_EJECT_MS;
        intakeGuard.request(-Math.abs(FULL_EJECT_POWER));
        Log.i("IntakeCount", "full: intake reversing " + FULL_EJECT_MS + " ms to push out a straddler, sensors " + sensorString());
    }

    /** The full-eject reverse from startFullEject() is still running. */
    public boolean isFullEjecting() {
        return nowMs() < fullEjectUntilMs;
    }

    /**
     * Lets a launch command take over the transfer power while launching (e.g. a brief reverse to
     * give a lone ball a run-up). SmartIntakeCommand applies it in its LAUNCHING state.
     */
    public void setLaunchTransferOverride(double power) {
        launchTransferOverride = power;
    }

    public void clearLaunchTransferOverride() {
        launchTransferOverride = Double.NaN;
    }

    public boolean hasLaunchTransferOverride() {
        return !Double.isNaN(launchTransferOverride);
    }

    public double getLaunchTransferOverride() {
        return launchTransferOverride;
    }

    /**
     * Lets a launch command run the intake motor while launching (normally it is off so nothing new
     * is pulled in mid-shot). Used during the launch kick so the intake holds the bottom artifact
     * in while the transfer reverses. SmartIntakeCommand applies it in its LAUNCHING state.
     */
    public void setLaunchIntakeOverride(double power) {
        launchIntakeOverride = power;
    }

    public void clearLaunchIntakeOverride() {
        launchIntakeOverride = Double.NaN;
    }

    public boolean hasLaunchIntakeOverride() {
        return !Double.isNaN(launchIntakeOverride);
    }

    public double getLaunchIntakeOverride() {
        return launchIntakeOverride;
    }

    public int getArtifactCount() {
        return artifactCount;
    }

    public boolean isFull() {
        return artifactCount >= MAX_ARTIFACTS;
    }

    /**
     * Full, and past the FULL_RUN_ON_MS run-on that pulls the last artifact fully in. This is what
     * stops the intake; isFull() turns true as soon as the last artifact reaches sensor 1.
     */
    public boolean isFullAndDoneIntaking() {
        return isFull() && nowMs() >= fullRunOnUntilMs;
    }

    /** Full, and past both run-ons (FULL_RUN_ON_MS + TRANSFER_FULL_DELAY_MS): the transfer stops too. */
    public boolean isFullAndTransferDone() {
        return isFull() && nowMs() >= transferRunOnUntilMs;
    }

    /** Manually override the count, e.g. preloads at the start of auto. */
    public void setArtifactCount(int count) {
        changeCount(count, "set by code (setArtifactCount)");
    }

    public boolean isSensor1Blocked() {
        return laser1.isBlocked();
    }

    public boolean isSensor2Blocked() {
        return beamBreak2.isBlocked();
    }

    public boolean isSensor3Blocked() {
        return beamBreak3.isBlocked();
    }

    public boolean isSensor4Blocked() {
        return laser4.isBlocked();
    }

    /** Laser distance readings (mm), and their baselines, for test OpModes. */
    public double getSensor1DistanceMm() {
        return laser1.getDistanceMm();
    }

    public double getSensor4DistanceMm() {
        return laser4.getDistanceMm();
    }

    public double getSensor1BaselineMm() {
        return laser1.getBaselineMm();
    }

    public double getSensor4BaselineMm() {
        return laser4.getBaselineMm();
    }

    private void updateArtifactCount() {
        long now = nowMs();
        boolean wasFull = isFull();
        double intakePower = intakeMotor.get();
        double transferPower = transferMotor.get();
        boolean reversing = intakePower < 0;

        // Re-time a ball resting on a sensor when it starts to move, so its resting time isn't
        // taken for two balls: sensor 4 when a launch starts, sensor 1 when the intake reverses.
        if (launching && !wasLaunching) laser4.rebase(now);
        if (reversing && !wasReversing) laser1.rebase(now);
        wasLaunching = launching;
        wasReversing = reversing;

        laser1.setDurationRule(SENSOR1_DURATION_RULE);
        laser4.setDurationRule(SENSOR4_DURATION_RULE);
        laser1.update(now);
        laser4.update(now);
        beamBreak2.update(now);
        beamBreak3.update(now);

        int entered = laser1.getTotalSeen() - s1SeenApplied;
        int ejected = laser1.getTotalGone() - s1GoneApplied;
        int launched = laser4.getTotalGone() - s4GoneApplied;
        s1SeenApplied = laser1.getTotalSeen();
        s1GoneApplied = laser1.getTotalGone();
        s4GoneApplied = laser4.getTotalGone();

        // Entrance. Not while launching: the launch kick runs the intake while the transfer
        // shuffles the stack up and down past sensor 1, which isn't a new artifact.
        if (!launching && entered > 0 && intakePower > 0) {
            changeCount(artifactCount + entered, "sensor 1: " + entered + " entered, intake forward");
        }
        if (!launching && ejected > 0 && reversing && isFullEjecting()) {
            Log.i("IntakeCount", "sensor 1: " + ejected + " pushed out by the full eject (straddler, not subtracted)");
        } else if (!launching && ejected > 0 && reversing) {
            changeCount(artifactCount - ejected, "sensor 1: " + ejected + " ejected, intake reversed");
        }

        // Exit into the launcher
        if (launching && launched > 0 && transferPower > 0) {
            changeCount(artifactCount - launched, "sensor 4: " + launched + " launched");
        }

        // Entrance blocked and not moving while intaking -> nowhere left to go
        // (not while launching: during the kick the intake holds an artifact on sensor 1 on purpose)
        if (!launching && intakePower > 0 && laser1.isBlocked() && laser1.msInState(now) >= JAM_MS) {
            changeCount(MAX_ARTIFACTS, "sensor 1 blocked " + JAM_MS + " ms while intaking (jam = full)");
        }

        // Empty-confirmation timer for confirmEmptyAfterLaunch().
        if (launching && transferPower > 0 && laserSensorsClear()) {
            if (feedClearSinceMs < 0) feedClearSinceMs = now;
        } else {
            feedClearSinceMs = -1;
        }

        // Intaking just filled the robot: run on briefly so the last artifact gets fully inside.
        // (Not for setArtifactCount(), e.g. AUTO preloads, or while launching.)
        if (!wasFull && isFull() && intakePower > 0 && !launching) {
            fullRunOnUntilMs = now + Math.max(0, FULL_RUN_ON_MS);
            transferRunOnUntilMs = fullRunOnUntilMs + Math.max(0, TRANSFER_FULL_DELAY_MS);
        }

        // Leftover after a launch: with the gate closed, a ball at sensor 4 is still inside.
        // Own timer (not laser4.msInState), so it starts only once the launch has ended.
        if (!launching && !reversing && laser4.isBlocked()) {
            if (s4IdleBlockedSinceMs < 0) s4IdleBlockedSinceMs = now;
        } else {
            s4IdleBlockedSinceMs = -1;
        }
        if (SENSOR4_LEFTOVER_ENABLED && artifactCount == 0 && s4IdleBlockedSinceMs >= 0
                && now - s4IdleBlockedSinceMs >= SENSOR4_LEFTOVER_MS) {
            changeCount(1, "sensor 4 blocked " + SENSOR4_LEFTOVER_MS + " ms with the gate closed (ball left after launch)");
        }

        // Resting balls at sensors 2/3 (after the run-on check above: no run-on for these, the
        // ball is already inside). Only raises the count.
        boolean settled = !launching && !reversing && transferPower > 0;
        if (SENSOR3_MIN_ENABLED && settled && artifactCount < SENSOR3_MIN_COUNT
                && beamBreak3.isBlocked() && beamBreak3.msInState(now) >= SENSOR3_MIN_MS) {
            changeCount(SENSOR3_MIN_COUNT, "sensor 3 blocked " + SENSOR3_MIN_MS + " ms with transfer forward");
        }
        if (SENSOR2_FULL_ENABLED && settled && !isFull()
                && beamBreak2.isBlocked() && beamBreak2.msInState(now) >= SENSOR2_FULL_MS) {
            changeCount(MAX_ARTIFACTS, "sensor 2 blocked " + SENSOR2_FULL_MS + " ms with transfer forward (4th resting there)");
            fullRunOnUntilMs = now;
            transferRunOnUntilMs = now + Math.max(0, TRANSFER_FULL_DELAY_MS);
        }
    }

    /** Sets the count (clamped to 0..MAX) and logs the change with its reason. */
    private void changeCount(int newCount, String reason) {
        newCount = clamp(newCount);
        if (newCount == artifactCount) return;
        Log.i("IntakeCount", String.format("%d -> %d: %s | sensors %s, intake %.1f, transfer %.1f%s",
                artifactCount, newCount, reason, sensorString(), intakeMotor.get(), transferMotor.get(),
                launching ? ", launching" : ""));
        artifactCount = newCount;
    }

    /** Sensors 1-4 as X (blocked) / - (clear). */
    private String sensorString() {
        return (laser1.isBlocked() ? "X" : "-") + (beamBreak2.isBlocked() ? "X" : "-")
                + (beamBreak3.isBlocked() ? "X" : "-") + (laser4.isBlocked() ? "X" : "-");
    }

    private static int clamp(int count) {
        return Math.max(0, Math.min(count, MAX_ARTIFACTS));
    }

    private static long nowMs() {
        return System.nanoTime() / 1_000_000L;
    }

    // ---------------------------------------------------------------- status light

    public void updateStatusLight(int artifactCount) {
        double color;
        switch (artifactCount) {
            case 1:
                color = ONE_BALL_COLOR;
                break;
            case 2:
                color = TWO_BALL_COLOR;
                break;
            case 3:
                color = THREE_BALL_COLOR;
                break;
            case 4:
                color = FOUR_BALL_COLOR;
                break;
            default:
                color = ZERO_BALL_COLOR;
                break;
        }
        artifactCountLight.setPosition(color);
    }

    @Override
    public void periodic() {
        updateArtifactCount();
        updateStallGuards();

        // Auto-stop when full (only while intaking, not while feeding the launcher)
        // The transfer stops TRANSFER_FULL_DELAY_MS after the intake motor.
        if (isFullAndDoneIntaking() && !launching) {
            if (intakeMotor.get() > 0) stopIntakeMotorOnly();
            if (isFullAndTransferDone() && transferMotor.get() > 0) stopTransferMotorOnly();
        }

        updateStatusLight(artifactCount);

        if (DebugTelemetry.ENABLED) telemetry.addData("Artifact Count: ", artifactCount);
        if (DebugTelemetry.ENABLED) telemetry.addData("Sensors 1/2/3/4: ", sensorString());
        if (DebugTelemetry.ENABLED) telemetry.addData("Laser 1 / 4 (mm): ", String.format("%.0f (base %.0f) / %.0f (base %.0f)",
                laser1.getDistanceMm(), laser1.getBaselineMm(), laser4.getDistanceMm(), laser4.getBaselineMm()));
        if (DebugTelemetry.ENABLED) telemetry.addData("Transfer Power: ", transferMotor.get());
        if (DebugTelemetry.ENABLED) telemetry.addData("Intake stall", intakeGuard.status());
    }

    // ---------------------------------------------------------------- stall detection

    private void updateStallGuards() {
        long now = nowMs();
        intakeGuard.update(now, STALL_DETECTION_ENABLED && !launching);
    }

    /** Sensors 1 and 4 (the lasers). */
    private boolean laserSensorsClear() {
        return !laser1.isBlocked() && !laser4.isBlocked();
    }

    /**
     * End of a launch that timed out with artifacts still counted: if the transfer has fed forward
     * with sensors 1 and 4 clear for EMPTY_CONFIRM_MS, the robot is really empty, so the count goes to 0
     * (the count was high). Returns true if it reset the count.
     */
    public boolean confirmEmptyAfterLaunch() {
        if (artifactCount == 0 || feedClearSinceMs < 0) {
            if (artifactCount > 0) {
                Log.i("IntakeCount", "kept " + artifactCount + " after launch: sensors " + sensorString()
                        + (transferMotor.get() > 0 ? "" : ", transfer not feeding forward"));
            }
            return false;
        }
        long clearMs = nowMs() - feedClearSinceMs;
        if (clearMs < EMPTY_CONFIRM_MS) {
            Log.i("IntakeCount", String.format("kept %d after launch: sensors clear with transfer forward only %d ms (EMPTY_CONFIRM_MS %d)",
                    artifactCount, clearMs, EMPTY_CONFIRM_MS));
            return false;
        }
        changeCount(0, "launch timed out, transfer fed forward with all sensors clear for " + clearMs + " ms (empty)");
        return true;
    }

    /** Watches the intake motor for a stall and runs the reverse-and-retry unjam. */
    private static final class StallGuard {
        private final String name;
        private final MotorEx motor;

        /** Last power a command asked for; reapplied after an unjam. */
        private double requested = 0;
        private long forwardSinceMs = -1;
        private long stalledSinceMs = -1;
        private long unjamUntilMs = 0;
        private long giveUpUntilMs = 0;
        private int tries = 0;
        private long firstTryMs = 0;
        /** Filtered current (NaN until the first reading after the motor starts) and the last raw reading. */
        private double currentA = Double.NaN;
        private double rawCurrentA = Double.NaN;
        private long currentReadAtMs = 0;

        StallGuard(String name, MotorEx motor) {
            this.name = name;
            this.motor = motor;
        }

        /** A command's power request. Stop/reverse requests win over an unjam; forward ones wait for it. */
        void request(double power) {
            requested = power;
            if (power <= 0 && isOverriding()) {
                unjamUntilMs = 0;
                giveUpUntilMs = 0;
            }
            if (!isOverriding()) motor.set(power);
        }

        boolean isOverriding() {
            long now = nowMs();
            return now < unjamUntilMs || now < giveUpUntilMs;
        }

        void update(long now, boolean watch) {
            // Not watched (launching / detection off): drop any unjam so the launch gets its motors.
            if (!watch && isOverriding()) {
                unjamUntilMs = 0;
                giveUpUntilMs = 0;
                tries = 0;
                motor.set(requested);
                Log.i("IntakeStall", name + ": unjam cancelled (launching or detection off)");
            }
            // Finish an unjam / give-up period: back to what the command asked for.
            if (unjamUntilMs != 0 && now >= unjamUntilMs) {
                unjamUntilMs = 0;
                forwardSinceMs = now; // startup grace again
                stalledSinceMs = -1;
                motor.set(requested);
                Log.i("IntakeStall", name + ": unjam reverse done, back to " + requested);
            }
            if (giveUpUntilMs != 0 && now >= giveUpUntilMs) {
                giveUpUntilMs = 0;
                tries = 0;
                forwardSinceMs = now;
                stalledSinceMs = -1;
                motor.set(requested);
                Log.i("IntakeStall", name + ": give-up pause over, trying again");
            }
            if (isOverriding()) return;

            if (!watch || requested < STALL_MIN_POWER) {
                forwardSinceMs = -1;
                stalledSinceMs = -1;
                currentA = Double.NaN;
                return;
            }
            if (forwardSinceMs < 0) forwardSinceMs = now;
            if (now - forwardSinceMs < STALL_STARTUP_MS) {
                currentA = Double.NaN; // don't let the startup spike seed the filter
                return;
            }

            if (!isStalled(now)) {
                stalledSinceMs = -1;
                return;
            }
            if (stalledSinceMs < 0) stalledSinceMs = now;
            if (now - stalledSinceMs < STALL_TIME_MS) return;

            // Stalled: reverse briefly, or give up for a while after too many tries.
            stalledSinceMs = -1;
            if (tries == 0 || now - firstTryMs > UNJAM_WINDOW_MS) {
                tries = 0;
                firstTryMs = now;
            }
            tries++;
            String signal = describeSignal();
            if (tries > UNJAM_MAX_TRIES) {
                giveUpUntilMs = now + UNJAM_GIVE_UP_MS;
                motor.set(0);
                Log.w("IntakeStall", String.format("%s: STALLED (%s) %d times in %d ms, giving up: stopped for %d ms",
                        name, signal, tries - 1, now - firstTryMs, UNJAM_GIVE_UP_MS));
            } else {
                unjamUntilMs = now + UNJAM_REVERSE_MS;
                motor.set(-Math.abs(UNJAM_REVERSE_POWER));
                Log.i("IntakeStall", String.format("%s: STALLED (%s for %d ms), unjam %d/%d: reversing %d ms at %.2f",
                        name, signal, STALL_TIME_MS, tries, UNJAM_MAX_TRIES, UNJAM_REVERSE_MS, UNJAM_REVERSE_POWER));
            }
        }

        private boolean isStalled(long now) {
            switch (STALL_SIGNAL) {
                case VELOCITY: return slow();
                case BOTH: return highCurrent(now) && slow();
                default: return highCurrent(now);
            }
        }

        private boolean slow() {
            return Math.abs(motor.getVelocity()) <= STALL_MAX_VELOCITY_TPS;
        }

        /** Reads current every CURRENT_READ_INTERVAL_MS and low-passes it (STALL_CURRENT_FILTER_MS). */
        private boolean highCurrent(long now) {
            if (Double.isNaN(currentA) || now - currentReadAtMs >= CURRENT_READ_INTERVAL_MS) {
                rawCurrentA = motor.motorEx.getCurrent(CurrentUnit.AMPS);
                if (Double.isNaN(currentA) || STALL_CURRENT_FILTER_MS <= 0) {
                    currentA = rawCurrentA;
                } else {
                    double dt = now - currentReadAtMs;
                    currentA += (rawCurrentA - currentA) * (1 - Math.exp(-dt / STALL_CURRENT_FILTER_MS));
                }
                currentReadAtMs = now;
            }
            return currentA >= STALL_CURRENT_A;
        }

        private String describeSignal() {
            String amps = Double.isNaN(currentA) ? "-" : String.format("%.1f A filtered (%.1f raw)", currentA, rawCurrentA);
            String speed = String.format("%.0f ticks/s", Math.abs(motor.getVelocity()));
            switch (STALL_SIGNAL) {
                case VELOCITY: return speed;
                case BOTH: return amps + ", " + speed;
                default: return amps;
            }
        }

        String status() {
            long now = nowMs();
            if (now < unjamUntilMs) return "UNJAMMING";
            if (now < giveUpUntilMs) return "GAVE UP";
            return "ok (" + describeSignal() + ")";
        }
    }

    // ---------------------------------------------------------------- debounced digital sensor (2, 3)

    private static class BeamBreak {
        private final DigitalChannel channel;
        private boolean stableBlocked;
        private boolean lastRaw;
        private long rawChangedAtMs;
        private long stableSinceMs;

        BeamBreak(HardwareMap hardwareMap, String name) {
            channel = hardwareMap.get(DigitalChannel.class, name);
            channel.setMode(DigitalChannel.Mode.INPUT);
            long now = nowMs();
            lastRaw = readRaw();
            stableBlocked = lastRaw;
            rawChangedAtMs = now;
            stableSinceMs = now;
        }

        private boolean readRaw() {
            return channel.getState() == BLOCKED_READS_HIGH;
        }

        /** Reads the sensor; returns true if the debounced state changed this loop. */
        boolean update(long now) {
            boolean raw = readRaw();
            if (raw != lastRaw) {
                lastRaw = raw;
                rawChangedAtMs = now;
            }
            if (raw != stableBlocked && now - rawChangedAtMs >= DEBOUNCE_MS) {
                stableBlocked = raw;
                stableSinceMs = now;
                return true;
            }
            return false;
        }

        boolean isBlocked() {
            return stableBlocked;
        }

        long msInState(long now) {
            return now - stableSinceMs;
        }
    }
}
