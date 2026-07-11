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
    public  static double DELAYED_STOP_MS = 500;
    public static double KICKER_PULSE_MS = 200; // Time kicker stays extended
    private final IntakeSubsystem intakeSubsystem;
    private final LaunchGateSubsystem launchGate;
    private final LaunchKickSubsystem launchKick;
    private ElapsedTime stableTimer;
    private ElapsedTime kickTimer = new ElapsedTime();
    private ElapsedTime stopDelayTimer;

    public SmartIntakeCommand(IntakeSubsystem intakeSubsystem, LaunchGateSubsystem launchGate, LaunchKickSubsystem launchKick) {
        this.intakeSubsystem = intakeSubsystem;
        this.launchGate = launchGate;
        this.launchKick = launchKick;

        addRequirements(intakeSubsystem, launchKick);
    }


    @Override
    public void execute() {
        boolean isLaunching = launchGate.isGateOpen();

        // --- 1. SENSOR STABILITY LOGIC ---
        boolean low = intakeSubsystem.isLowSensorBlocked();
        boolean mid = intakeSubsystem.isMidSensorBlocked();
        boolean high = intakeSubsystem.isHighSensorBlocked();

        // Start stableTimer if all three are blocked, reset it if they aren't
        if (low && mid && high) {
            if (stableTimer == null) {
                stableTimer = new ElapsedTime();
            }
        } else {
            stableTimer = null;
        }

        // A "Confirmed Full" state requires all 3 sensors to be blocked for 200ms
        boolean isConfirmedFull = (stableTimer != null && stableTimer.milliseconds() > STABLE_STORAGE_MS);

        // --- 2. INTAKE MOTOR CONTROL ---
        if (isLaunching || !isConfirmedFull) {
            // Run intake if we are shooting OR if we aren't confirmed full yet
            intakeSubsystem.startIntake();
            stopDelayTimer = null;
            Log.i("SmartIntakeCommand", "Staring intake: ");
        } else {
            // We are Confirmed Full and not launching
            if (stopDelayTimer == null) {
                stopDelayTimer = new ElapsedTime();
            } else if (stopDelayTimer.milliseconds() > DELAYED_STOP_MS) {
                // Only stop after the extra "seating" time
                intakeSubsystem.stopIntake();
                Log.i("SmartIntakeCommand", "Stopping intake: ");
            }
        }

        // --- 3. AUTO-KICK LOGIC (Timer is Essential here) ---
        // If launching and ONLY the top ball is left (stuck in dead zone)
        if (isLaunching && high && !mid && !low) {
            launchKick.extendKicker();
            kickTimer.reset(); // Keep resetting while the condition is true
        } else {
            // Condition is no longer met (either ball moved or we stopped launching)
            // BUT we wait for the pulse timer to finish before retracting
            if (kickTimer.milliseconds() > KICKER_PULSE_MS) {
                launchKick.retractKicker();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false; // This stays active throughout the match
    }
}