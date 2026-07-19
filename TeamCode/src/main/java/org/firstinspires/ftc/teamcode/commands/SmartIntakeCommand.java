package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.AllianceColors;
import org.firstinspires.ftc.teamcode.common.CrossOpModeStorage;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchGateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchKickSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;

import java.util.NavigableMap;
import java.util.TreeMap;

@Configurable
public class SmartIntakeCommand extends CommandBase {
    public static double STABLE_STORAGE_MS = 15;
    public static double DELAYED_STOP_MS = 500;
    public static double KICKER_PULSE_MS = 200;
    public static double INTAKE_MOTOR_STOP_DELAY_MS = 1500; // Time in ms to keep intake motor running after transfer stops
    public static double KICK_STABILITY_MS = 300; // NEW: Duration to wait before kicking
    public static boolean TURN_OFF_INTAKE = false;
    private final IntakeSubsystem intakeSubsystem;
    private final LaunchGateSubsystem launchGate;
    private final LaunchKickSubsystem launchKick;
    private final OdometrySubsystem odometry;

    private final ElapsedTime kickTimer = new ElapsedTime();
    private final ElapsedTime kickStabilityTimer = new ElapsedTime(); // NEW
    private ElapsedTime stopDelayTimer;

    private int lastRawCount = -1;
    private int stableCount = 0;
    private final ElapsedTime countStabilityTimer = new ElapsedTime();
    // Inside the class...
    private double TARGET_X;
    private double TARGET_Y;
    private static final NavigableMap<Double, Double> TRANSFER_POWER_TABLE = new TreeMap<>();

    static {
        // Distance (in) -> Motor Power (0.0 to 1.0)
        TRANSFER_POWER_TABLE.put(120.0, 1.0);   // full speed in near zones
        TRANSFER_POWER_TABLE.put(126.0, 0.725);  // Far zone APEX
        TRANSFER_POWER_TABLE.put(131.0, 0.725); // Far zone close to goal
        TRANSFER_POWER_TABLE.put(136.0, 0.7); // Far zone backwall brace
        TRANSFER_POWER_TABLE.put(143.0, 0.65); // Far zone far from goal
    }

    public SmartIntakeCommand(IntakeSubsystem intakeSubsystem, LaunchGateSubsystem launchGate, LaunchKickSubsystem launchKick, OdometrySubsystem odometry) {
        this.intakeSubsystem = intakeSubsystem;
        this.launchGate = launchGate;
        this.launchKick = launchKick;
        this.odometry = odometry;

        if (CrossOpModeStorage.allianceColor == AllianceColors.BLUE) {
            TARGET_X = CrossOpModeStorage.BLUE_TARGET_X;
            TARGET_Y = CrossOpModeStorage.BLUE_TARGET_Y;
        }
        else {
            TARGET_X = CrossOpModeStorage.RED_TARGET_X;
            TARGET_Y = CrossOpModeStorage.RED_TARGET_Y;
        }

        addRequirements(intakeSubsystem, launchKick);
    }

    @Override
    public void execute() {
        // --- 1. STABLE COUNT & LIGHT LOGIC ---
        int currentRawCount = intakeSubsystem.getArtifactCount();

        if (currentRawCount != lastRawCount) {
            countStabilityTimer.reset();
            lastRawCount = currentRawCount;
        }

        if (countStabilityTimer.milliseconds() > STABLE_STORAGE_MS) {
            stableCount = currentRawCount;
        }

        intakeSubsystem.updateStatusLight(stableCount);

        // --- 2. INTAKE MOTOR CONTROL ---
        boolean isLaunching = launchGate.isGateOpen();
        boolean isConfirmedFull = (stableCount == 3);

        if (isLaunching || !isConfirmedFull) {
            if (isLaunching) {
                // DYNAMIC POWER DURING LAUNCH
                double transferPower = getTransferPowerBasedOnDistance();
                intakeSubsystem.setTransferMotorPower(transferPower);
                intakeSubsystem.setIntakeMotorPower(1);
            }
            else {
                if(!TURN_OFF_INTAKE)
                    intakeSubsystem.startIntake();
            }
            stopDelayTimer = null;
        } else {
            // 3 Balls Detected: Begin sequential shutdown
            if (stopDelayTimer == null) {
                stopDelayTimer = new ElapsedTime();
            }
            // Phase 1: Stop the Transfer motor first
            if (stopDelayTimer.milliseconds() > DELAYED_STOP_MS) {
                intakeSubsystem.stopTransferMotorOnly();
            }

            // Phase 2: Stop the Intake motor after a longer delay
            if (stopDelayTimer.milliseconds() > INTAKE_MOTOR_STOP_DELAY_MS) {
                intakeSubsystem.stopIntakeMotorOnly();
            }
        }

        // --- 3. AUTO-KICK LOGIC ---
        boolean low = intakeSubsystem.isLowSensorBlocked();
        boolean mid = intakeSubsystem.isMidSensorBlocked();
        boolean high = intakeSubsystem.isHighSensorBlocked();

        boolean isStuck = isLaunching && high && !mid && !low;

        if (isStuck) {
            // Check if the stuck state has been stable for X ms
            if (kickStabilityTimer.milliseconds() > KICK_STABILITY_MS) {
                Log.i("SmartIntakeCommand", "Kick firing");
                launchKick.extendKicker();
                kickTimer.reset();
            }
        } else {
            // Reset stability if the condition is no longer met
            kickStabilityTimer.reset();

            if (kickTimer.milliseconds() > KICKER_PULSE_MS) {
                Log.i("SmartIntakeCommand", "Kick reset");
                launchKick.retractKicker();
            }
        }
    }

    private double getTransferPowerBasedOnDistance() {
        double distance = odometry.getPose().getTranslation()
                .getDistance(new Translation2d(TARGET_X, TARGET_Y));

        Double lowKey = TRANSFER_POWER_TABLE.floorKey(distance);
        Double highKey = TRANSFER_POWER_TABLE.ceilingKey(distance);

        if (lowKey == null) return TRANSFER_POWER_TABLE.get(highKey);
        if (highKey == null) return TRANSFER_POWER_TABLE.get(lowKey);
        if (lowKey.equals(highKey)) return TRANSFER_POWER_TABLE.get(lowKey);

        double lowPower = TRANSFER_POWER_TABLE.get(lowKey);
        double highPower = TRANSFER_POWER_TABLE.get(highKey);

        // Linear Interpolation
        return lowPower + ((distance - lowKey) * (highPower - lowPower) / (highKey - lowKey));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}