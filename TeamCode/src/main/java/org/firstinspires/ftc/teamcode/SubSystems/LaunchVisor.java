package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.config.Config;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

@Config
public class LaunchVisor implements Subsystem {
    public static double VISOR_LOW = 0.0;
    public static double VISOR_HIGH = 0.1;
    public static final LaunchVisor INSTANCE = new LaunchVisor();
    private LaunchVisor() {}

    private final ServoEx visorServo = new ServoEx("VisorServo");

    public Command lower = new SetPosition(visorServo, VISOR_LOW).requires(visorServo);
    public Command raise = new SetPosition(visorServo, VISOR_HIGH).requires(visorServo);

}
