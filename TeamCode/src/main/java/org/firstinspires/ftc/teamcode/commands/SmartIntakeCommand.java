package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchGateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchKickSubsystem;

@Configurable
public class SmartIntakeCommand extends CommandBase {
    public static double STABLE_STORAGE_MS = 200;
    public static double DELAYED_STOP_MS = 500;
    public static double KICKER_PULSE_MS = 200;
    public static double KICK_STABILITY_MS = 300; // NEW: Duration to wait before kicking
    public static boolean TURN_OFF_INTAKE = false;
    private final IntakeSubsystem intakeSubsystem;
    private final LaunchGateSubsystem launchGate;
    private final LaunchKickSubsystem launchKick;

    private final ElapsedTime kickTimer = new ElapsedTime();
    private final ElapsedTime kickStabilityTimer = new ElapsedTime(); // NEW
    private ElapsedTime stopDelayTimer;

    private int lastRawCount = -1;
    private int stableCount = 0;
    private final ElapsedTime countStabilityTimer = new ElapsedTime();

    public SmartIntakeCommand(IntakeSubsystem intakeSubsystem, LaunchGateSubsystem launchGate, LaunchKickSubsystem launchKick) {
        this.intakeSubsystem = intakeSubsystem;
        this.launchGate = launchGate;
        this.launchKick = launchKick;

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
            if(!TURN_OFF_INTAKE)
                intakeSubsystem.startIntake();

            stopDelayTimer = null;
        } else {
            if (stopDelayTimer == null) {
                stopDelayTimer = new ElapsedTime();
            } else if (stopDelayTimer.milliseconds() > DELAYED_STOP_MS) {
                intakeSubsystem.stopIntake();
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

    @Override
    public boolean isFinished() {
        return false;
    }
}