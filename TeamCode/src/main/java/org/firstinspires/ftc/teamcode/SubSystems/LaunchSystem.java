package org.firstinspires.ftc.teamcode.SubSystems;

import android.util.Log;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.SubsystemGroup;

public class LaunchSystem extends SubsystemGroup {
    public boolean isOn;

    public static final LaunchSystem INSTANCE = new LaunchSystem();

    private LaunchSystem() {
        super(
                LaunchFlywheel.INSTANCE,
                LaunchGate.INSTANCE,
                LaunchKicker.INSTANCE,
                LaunchVisor.INSTANCE
        );

        isOn = false;
    }

    public Command turnOff = LaunchFlywheel.INSTANCE.turnOff
            .and(new InstantCommand(() -> isOn = false))
            .and(new InstantCommand(() -> Log.i("LAUNCH", "TURNED OFF")))
            .named("Launch System Off");

    public Command turnOn = LaunchFlywheel.INSTANCE.turnOn
            .and(new InstantCommand(() -> isOn = true))
            .and(new InstantCommand(() -> Log.i("LAUNCH", "TURNED ON")))
            .named("Launch System On");

    private Command launchSetup = turnOn
            .and(new InstantCommand(Spindexer.INSTANCE::moveToNextFullSlot))
            .and(LaunchKicker.INSTANCE.rest)
            .named("Launch System: Launch Setup");

    private Command launchSetupGreen = turnOn
            .and(new InstantCommand(Spindexer.INSTANCE::moveToNextGreenSlot))
            .and(LaunchKicker.INSTANCE.rest)
            .named("Launch System: Launch Setup Green");

    private Command launchSetupPurple = turnOn
            .and(new InstantCommand(Spindexer.INSTANCE::moveToNextPurpleSlot))
            .and(LaunchKicker.INSTANCE.rest)
            .named("Launch System: Launch Setup Purple");

    private final Command launch = LaunchGate.INSTANCE.open
            .then(LaunchKicker.INSTANCE.kick)
            .and(new InstantCommand(Spindexer.INSTANCE::clearCurrentBall))
            .thenWait(0.25)
            .named("Launch System: Launch");

    private final Command launchTeardown = LaunchKicker.INSTANCE.rest
            .and(LaunchGate.INSTANCE.close)
            .named("Launch System: Launch Teardown");

    public Command launchOne = launchSetup
            .then(launch)
            .then(launchTeardown)
            .named("Launch System: Launch one");

    public Command launchThree = launchOne
            .then(launchOne)
            .then(launchOne)
            .named("Launch System: Launch Three");

    public Command launchGreen = launchSetupGreen
            .then(launch)
            .then(launchTeardown)
            .named("Launch System: Launch Green");

    public Command launchPurple = launchSetupPurple
            .then(launch)
            .then(launchTeardown)
            .named("Launch System: Launch Purple");
}
