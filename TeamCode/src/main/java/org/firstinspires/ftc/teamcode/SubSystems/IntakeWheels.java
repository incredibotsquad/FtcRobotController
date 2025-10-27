package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.config.Config;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

@Config
public class IntakeWheels implements Subsystem {
    public static double INTAKE_POWER = 1;
    public static final IntakeWheels INSTANCE = new IntakeWheels();
    public boolean isON;

    private IntakeWheels() {
        isON = false;
    }

    private final MotorEx intakeMotor = new MotorEx("IntakeMotor");

    //instant command on purpose - no need to wait for it to finish
    public final Command turnOff = new SetPower(intakeMotor, 0.0)
                                    .and(new InstantCommand(() -> isON = false))
                                    .requires(intakeMotor)
                                    .named("Intake Wheels Off");

    public final Command turnOn = new SetPower(intakeMotor, INTAKE_POWER)
                                    .and(new InstantCommand(() -> isON = true))
                                    .requires(intakeMotor)
                                    .named("Intake Wheels On");

//    public final Command turnOn = new NullCommand();
}
