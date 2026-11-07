package org.firstinspires.ftc.teamcode.subsystems;

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
 * Sensor 1 is the lowest (entrance), sensor 4 the highest. Every artifact passes sensor 1
 * and travels up to the highest free slot, so the stack fills top-down: 4, 3, 2, then 1.
 *
 * Because artifacts are different sizes, the slot sensors alone can't be trusted to give
 * an exact count. Instead:
 *  1. Primary count = events at sensor 1:
 *       - intake forward + sensor 1 becomes blocked  -> +1 (artifact entered)
 *       - intake reversed + sensor 1 becomes clear    -> -1 (artifact ejected)
 *     Plus, while launching: sensor 4 becomes clear  -> -1 (artifact left the top)
 *  2. Correction, once nothing is moving through the entrance (sensor 1 clear for SETTLE_MS):
 *       - sensor 4 clear                 -> count = 0 (first artifact always reaches the top)
 *       - sensors 4/3/2 blocked from top -> count is at least that many
 *       - upper bound (USE_SLOT_BASED_UPPER_BOUND):
 *           true:  count is at most (blocked slots from the top) + 1, i.e. one artifact the
 *                  sensors can't see (e.g. a 4th resting between sensors 1 and 2)
 *           false: count is at most MAX - 1 (old rule: assumes a 4th artifact rests on sensor 1)
 *  3. If sensor 1 stays blocked for JAM_MS while intaking, the artifact can't move up -> full.
 *
 * When the count reaches MAX_ARTIFACTS while intaking, the intake stops automatically.
 *
 * G407 (control at most 4): an undercount makes the intake pull in a 5th artifact, an overcount
 * only stops it early. So by default (ALLOW_SETTLED_DECREASE = false) the settle corrections in
 * step 2 may only RAISE the count. It only goes down on real exit events: sensor 1 clearing with
 * the intake reversed, or sensor 4 clearing while launching. A ball stuck low after a launch, which
 * the slot sensors can't see, then still counts.
 * One exception (confirmEmptyAfterLaunch, called by LaunchBallsCommand when it hits MAX_FIRE_MS):
 * if the transfer has run forward with all four sensors clear for at least EMPTY_CONFIRM_MS, any
 * artifact inside would have been carried past sensor 4 by then, so the robot is empty: count = 0.
 *
 * Every count change is logged (tag "IntakeCount") with its reason.
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

    /** Set false if your beam breaks read LOW (false) when the beam is broken. */
    public static boolean BLOCKED_READS_HIGH = true;

    /** A sensor must hold a new state this long before it counts (filters flicker). */
    public static long DEBOUNCE_MS = 15;
    /** Must be longer than the time for an artifact to travel from sensor 1 to the top. */
    public static long SETTLE_MS = 500;
    /** Sensor 1 blocked this long while intaking means the artifact can't move up: full. */
    public static long JAM_MS = 400;
    /** Use sensors 2-4 as a lower bound when settled. Disable if one large artifact can block two sensors. */
    public static boolean USE_SLOT_SENSORS_AS_LOWER_BOUND = true;
    /**
     * Settled upper bound on the count. true = (blocked slots from the top) + 1, which keeps a full
     * stack at 4 when the 4th artifact doesn't reach sensor 1. false = the old MAX_ARTIFACTS - 1
     * cap, which drops a full stack to 3 in that case. Experimental: flip live in Panels to compare.
     */
    public static boolean USE_SLOT_BASED_UPPER_BOUND = true;
    /**
     * Let the settle corrections LOWER the count (sensor 4 clear -> 0, and the upper bound above).
     * false (default) = they can only raise it, so a hidden artifact can't make the intake take a
     * 5th (G407). Cost: a double count stays until the next launch, and the robot stops one short.
     */
    public static boolean ALLOW_SETTLED_DECREASE = false;
    /**
     * confirmEmptyAfterLaunch(): how long the transfer must have run forward, while launching, with
     * all four sensors clear before the robot is taken as empty. Must be longer than an artifact
     * takes to ride from the intake up past sensor 4.
     */
    public static long EMPTY_CONFIRM_MS = 1000;
    /**
     * When intaking fills the robot, keep the intake and transfer running this much longer so the
     * last artifact is pulled all the way in (the count hits MAX as it reaches sensor 1, before it is
     * fully inside). 0 = stop immediately (old behavior). Keep short: running on while full could
     * pick up a 5th artifact (G407).
     */
    public static long FULL_RUN_ON_MS = 250;

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
    private final BeamBreak beamBreak1;
    private final BeamBreak beamBreak2;
    private final BeamBreak beamBreak3;
    private final BeamBreak beamBreak4;

    private final TelemetryManager telemetry;

    private final StallGuard intakeGuard;

    private int artifactCount = 0;
    /** A skipped settle decrease was logged; reset whenever the count changes so it logs once per count. */
    private boolean loggedSkippedDecrease = false;
    /** While launching: since when the transfer has run forward with all sensors clear; -1 = not now. */
    private long feedClearSinceMs = -1;
    /** Intake keeps running until this time after intaking filled the robot (see FULL_RUN_ON_MS). */
    private long fullRunOnUntilMs = 0;
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

        beamBreak1 = new BeamBreak(hardwareMap, "ballSensor1");
        beamBreak2 = new BeamBreak(hardwareMap, "ballSensor2");
        beamBreak3 = new BeamBreak(hardwareMap, "ballSensor3");
        beamBreak4 = new BeamBreak(hardwareMap, "ballSensor4");

        artifactCountLight = new SimpleServo(hardwareMap, "artifactCountLight", 0, 270);

        intakeGuard = new StallGuard("intake", intakeMotor);
    }

    // ---------------------------------------------------------------- motor control

    /** Starts intake and transfer, unless already full. */
    public void startIntakeAndTransfer() {
        if (isFullAndDoneIntaking()) {
            stopIntakeAndTransfer();
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

    /** Manually override the count, e.g. preloads at the start of auto. */
    public void setArtifactCount(int count) {
        changeCount(count, "set by code (setArtifactCount)");
    }

    public boolean isSensor1Blocked() {
        return beamBreak1.isBlocked();
    }

    public boolean isSensor2Blocked() {
        return beamBreak2.isBlocked();
    }

    public boolean isSensor3Blocked() {
        return beamBreak3.isBlocked();
    }

    public boolean isSensor4Blocked() {
        return beamBreak4.isBlocked();
    }

    private void updateArtifactCount() {
        long now = nowMs();
        boolean wasFull = isFull();

        boolean s1Changed = beamBreak1.update(now);
        beamBreak2.update(now);
        beamBreak3.update(now);
        boolean s4Changed = beamBreak4.update(now);

        double intakePower = intakeMotor.get();

        // 1. Event counting at the entrance. Not while launching: the launch kick runs the intake
        // while the transfer shuffles the stack up and down past sensor 1, which isn't a new artifact.
        if (s1Changed && !launching) {
            if (beamBreak1.isBlocked() && intakePower > 0) {
                changeCount(artifactCount + 1, "sensor 1 blocked, intake forward (entered)");
            } else if (!beamBreak1.isBlocked() && intakePower < 0) {
                changeCount(artifactCount - 1, "sensor 1 cleared, intake reversed (ejected)");
            }
        }

        // Artifact leaving the top into the launcher
        if (s4Changed && !beamBreak4.isBlocked() && launching && transferMotor.get() > 0) {
            changeCount(artifactCount - 1, "sensor 4 cleared while launching (launched)");
        }

        // 2. Correct drift once the entrance has been quiet for a while
        boolean settled = !launching
                && !beamBreak1.isBlocked()
                && beamBreak1.msInState(now) >= SETTLE_MS;

        if (settled) {
            if (!beamBreak4.isBlocked() && beamBreak4.msInState(now) >= SETTLE_MS) {
                settledDecrease(0, "settled: sensor 4 clear (empty)");
            } else {
                if (USE_SLOT_SENSORS_AS_LOWER_BOUND && contiguousSlotsFromTop() > artifactCount) {
                    changeCount(contiguousSlotsFromTop(), "settled: " + contiguousSlotsFromTop() + " slot(s) blocked from the top");
                }
                if (USE_SLOT_BASED_UPPER_BOUND) {
                    // At most one artifact the slot sensors can't see (below the lowest blocked one).
                    settledDecrease(contiguousSlotsFromTop() + 1, "settled: upper bound (slots from top + 1)");
                } else {
                    // Old rule: a 4th artifact would be resting on sensor 1, which is clear
                    settledDecrease(MAX_ARTIFACTS - 1, "settled: upper bound MAX - 1");
                }
            }
        }

        // 3. Entrance blocked and not moving while intaking -> nowhere left to go
        // (not while launching: during the kick the intake holds an artifact on sensor 1 on purpose)
        if (!launching && intakePower > 0 && beamBreak1.isBlocked() && beamBreak1.msInState(now) >= JAM_MS) {
            changeCount(MAX_ARTIFACTS, "sensor 1 blocked " + JAM_MS + " ms while intaking (jam = full)");
        }

        // Empty-confirmation timer for confirmEmptyAfterLaunch().
        if (launching && transferMotor.get() > 0 && allSensorsClear()) {
            if (feedClearSinceMs < 0) feedClearSinceMs = now;
        } else {
            feedClearSinceMs = -1;
        }

        // Intaking just filled the robot: run on briefly so the last artifact gets fully inside.
        // (Not for setArtifactCount(), e.g. AUTO preloads, or while launching.)
        if (!wasFull && isFull() && intakePower > 0 && !launching) {
            fullRunOnUntilMs = now + Math.max(0, FULL_RUN_ON_MS);
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
        loggedSkippedDecrease = false;
    }

    /** A settle correction that would lower the count: applied only if ALLOW_SETTLED_DECREASE. */
    private void settledDecrease(int newCount, String reason) {
        if (newCount >= artifactCount) return;
        if (ALLOW_SETTLED_DECREASE) {
            changeCount(newCount, reason);
        } else if (!loggedSkippedDecrease) {
            Log.i("IntakeCount", String.format("kept %d (would be %d: %s; ALLOW_SETTLED_DECREASE off) | sensors %s",
                    artifactCount, Math.max(0, newCount), reason, sensorString()));
            loggedSkippedDecrease = true;
        }
    }

    private String sensorString() {
        return (beamBreak1.isBlocked() ? "X" : "-") + (beamBreak2.isBlocked() ? "X" : "-")
                + (beamBreak3.isBlocked() ? "X" : "-") + (beamBreak4.isBlocked() ? "X" : "-");
    }

    /** Number of blocked slot sensors counting down from the top (4, then 3, then 2). */
    private int contiguousSlotsFromTop() {
        if (!beamBreak4.isBlocked()) return 0;
        if (!beamBreak3.isBlocked()) return 1;
        if (!beamBreak2.isBlocked()) return 2;
        return 3;
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
        if (isFullAndDoneIntaking() && !launching && intakeMotor.get() > 0) {
            stopIntakeAndTransfer();
        }

        updateStatusLight(artifactCount);

        telemetry.addData("Artifact Count: ", artifactCount);
        telemetry.addData("Beam breaks 1/2/3/4: ",
                (beamBreak1.isBlocked() ? "X" : "-")
                        + (beamBreak2.isBlocked() ? "X" : "-")
                        + (beamBreak3.isBlocked() ? "X" : "-")
                        + (beamBreak4.isBlocked() ? "X" : "-"));
        telemetry.addData("Transfer Power: ", transferMotor.get());
        telemetry.addData("Intake stall", intakeGuard.status());
    }

    // ---------------------------------------------------------------- stall detection

    private void updateStallGuards() {
        long now = nowMs();
        intakeGuard.update(now, STALL_DETECTION_ENABLED && !launching);
    }

    private boolean allSensorsClear() {
        return !beamBreak1.isBlocked() && !beamBreak2.isBlocked() && !beamBreak3.isBlocked() && !beamBreak4.isBlocked();
    }

    /**
     * End of a launch that timed out with artifacts still counted: if the transfer has fed forward
     * with every sensor clear for EMPTY_CONFIRM_MS, the robot is really empty, so the count goes to 0
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

    // ---------------------------------------------------------------- debounced sensor

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
