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
 *
 * Every count change is logged (tag "IntakeCount") with its reason.
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
     * When intaking fills the robot, keep the intake and transfer running this much longer so the
     * last artifact is pulled all the way in (the count hits MAX as it reaches sensor 1, before it is
     * fully inside). 0 = stop immediately (old behavior). Keep short: running on while full could
     * pick up a 5th artifact (G407).
     */
    public static long FULL_RUN_ON_MS = 250;

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

    private int artifactCount = 0;
    /** A skipped settle decrease was logged; reset whenever the count changes so it logs once per count. */
    private boolean loggedSkippedDecrease = false;
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
    }

    // ---------------------------------------------------------------- motor control

    /** Starts intake and transfer, unless already full. */
    public void startIntakeAndTransfer() {
        if (isFullAndDoneIntaking()) {
            stopIntakeAndTransfer();
            return;
        }
        intakeMotor.set(1);
        transferMotor.set(1);
    }

    /** Stops intake and transfer. */
    public void stopIntakeAndTransfer() {
        intakeMotor.set(0);
        transferMotor.set(0);
    }

    /** Runs intake and transfer in reverse to eject artifacts. */
    public void reverseIntakeAndTransfer() {
        intakeMotor.set(-1);
        transferMotor.set(-1);
    }

    public boolean isOn() {
        return intakeMotor.get() > 0 || transferMotor.get() > 0;
    }

    public void setIntakeMotorPower(double power) {
        intakeMotor.set(power);
    }

    public void setTransferMotorPower(double power) {
        transferMotor.set(power);
    }

    public void stopIntakeMotorOnly() {
        intakeMotor.set(0);
    }

    public void stopTransferMotorOnly() {
        transferMotor.set(0);
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
