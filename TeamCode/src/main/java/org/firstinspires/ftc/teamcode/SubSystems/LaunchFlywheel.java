package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.config.Config;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.hardware.powerable.SetPower;


@Config
public class LaunchFlywheel implements Subsystem {
    public static double FLYWHEEL_VELOCITY = 1900;

    private boolean isOn;
    public static final LaunchFlywheel INSTANCE = new LaunchFlywheel();

    private LaunchFlywheel() {
        isOn = false;
    }

    private final MotorEx launchMotor = new MotorEx("LaunchMotor");

    private final ControlSystem controller = ControlSystem.builder()
            .velPid(0.005, 0, 0)
            .basicFF(0.01, 0.02, 0.03)
            .build();

    //dont need to wait for stopping the flywheel - setPower is an instant command
    public final Command turnOff = new SetPower(launchMotor, 0.0)
            .and(new InstantCommand(() -> isOn = false))
            .requires(launchMotor)
            .named("FlywheelOff");

    //RunToVelocity by default will wait to get done until the tolerance is reached.
    public final Command turnOn = new RunToVelocity(controller, FLYWHEEL_VELOCITY)
            .and(new InstantCommand(() -> isOn = true))
            .requires(launchMotor)
            .named("FlywheelOn");

    @Override
    public void periodic() {
        launchMotor.setPower(controller.calculate(launchMotor.getState()));
    }
}
