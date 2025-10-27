package org.firstinspires.ftc.teamcode.SubSystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import com.acmerobotics.dashboard.config.Config;

@Config
public class LaunchGate implements Subsystem {
    public static double LAUNCH_GATE_OPEN = 1;
    public static double LAUNCH_GATE_CLOSE = 0.6;

    public static final LaunchGate INSTANCE = new LaunchGate();
    private LaunchGate() {}
    private final ServoEx launchGateServo = new ServoEx("LaunchGateServo");

    public Command open = new SetPosition(launchGateServo, LAUNCH_GATE_OPEN).requires(launchGateServo);
    public Command close = new SetPosition(launchGateServo, LAUNCH_GATE_CLOSE).requires(launchGateServo);
}
