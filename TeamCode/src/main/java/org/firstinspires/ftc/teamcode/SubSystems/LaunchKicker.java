package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.config.Config;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

@Config
public class LaunchKicker implements Subsystem {
    public static double KICKER_KICK = 0.7;
    public static double KICKER_REST = 0.95;

    public static final LaunchKicker INSTANCE = new LaunchKicker();
    private LaunchKicker() {}

    private final ServoEx launchKickServo = new ServoEx("LaunchGateServo");

    public Command kick = new SetPosition(launchKickServo, KICKER_KICK).requires(launchKickServo);
    public Command rest = new SetPosition(launchKickServo, KICKER_REST).requires(launchKickServo);
}
