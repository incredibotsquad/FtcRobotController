package org.firstinspires.ftc.teamcode.subsystems;

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
 *       - sensor 1 clear                 -> count is at most MAX - 1 (4th artifact rests on sensor 1)
 *  3. If sensor 1 stays blocked for JAM_MS while intaking, the artifact can't move up -> full.
 *
 * When the count reaches MAX_ARTIFACTS while intaking, the intake stops automatically.
 */
@Configurable
public class IntakeSubsystem extends SubsystemBase {

    public static int MAX_ARTIFACTS = 4;

    /** Set false if your beam breaks read LOW (false) when the beam is broken. */
    public static boolean BLOCKED_READS_HIGH = true;

    /** A sensor must hold a new state this long before it counts (filters flicker). */
    public static long DEBOUNCE_MS = 15;
    /** Must be longer than the time for an artifact to travel from sensor 1 to the top. */
    public static long SETTLE_MS = 750;
    /** Sensor 1 blocked this long while intaking means the artifact can't move up: full. */
    public static long JAM_MS = 400;
    /** Use sensors 2-4 as a lower bound when settled. Disable if one large artifact can block two sensors. */
    public static boolean USE_SLOT_SENSORS_AS_LOWER_BOUND = true;

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
        if (isFull()) {
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

    /** Manually override the count, e.g. preloads at the start of auto. */
    public void setArtifactCount(int count) {
        artifactCount = clamp(count);
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

        boolean s1Changed = beamBreak1.update(now);
        beamBreak2.update(now);
        beamBreak3.update(now);
        boolean s4Changed = beamBreak4.update(now);

        double intakePower = intakeMotor.get();

        // 1. Event counting at the entrance. Not while launching: the launch kick runs the intake
        // while the transfer shuffles the stack up and down past sensor 1, which isn't a new artifact.
        if (s1Changed && !launching) {
            if (beamBreak1.isBlocked() && intakePower > 0) {
                artifactCount++;
            } else if (!beamBreak1.isBlocked() && intakePower < 0) {
                artifactCount--;
            }
        }

        // Artifact leaving the top into the launcher
        if (s4Changed && !beamBreak4.isBlocked() && launching && transferMotor.get() > 0) {
            artifactCount--;
        }

        // 2. Correct drift once the entrance has been quiet for a while
        boolean settled = !launching
                && !beamBreak1.isBlocked()
                && beamBreak1.msInState(now) >= SETTLE_MS;

        if (settled) {
            if (!beamBreak4.isBlocked() && beamBreak4.msInState(now) >= SETTLE_MS) {
                artifactCount = 0;
            } else {
                if (USE_SLOT_SENSORS_AS_LOWER_BOUND) {
                    artifactCount = Math.max(artifactCount, contiguousSlotsFromTop());
                }
                // 4th artifact would be resting on sensor 1, which is clear
                artifactCount = Math.min(artifactCount, MAX_ARTIFACTS - 1);
            }
        }

        // 3. Entrance blocked and not moving while intaking -> nowhere left to go
        // (not while launching: during the kick the intake holds an artifact on sensor 1 on purpose)
        if (!launching && intakePower > 0 && beamBreak1.isBlocked() && beamBreak1.msInState(now) >= JAM_MS) {
            artifactCount = MAX_ARTIFACTS;
        }

        artifactCount = clamp(artifactCount);
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
        if (isFull() && !launching && intakeMotor.get() > 0) {
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
