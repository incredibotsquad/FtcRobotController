package org.firstinspires.ftc.teamcode.OpModes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.IntakeColorSensors;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeWheels;
import org.firstinspires.ftc.teamcode.SubSystems.LaunchFlywheel;
import org.firstinspires.ftc.teamcode.SubSystems.LaunchGate;
import org.firstinspires.ftc.teamcode.SubSystems.LaunchKicker;
import org.firstinspires.ftc.teamcode.SubSystems.LaunchSystem;
import org.firstinspires.ftc.teamcode.SubSystems.LaunchVisor;
import org.firstinspires.ftc.teamcode.SubSystems.Spindexer;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "IncredibotsMecanumDrive")
public class IncredibotsMecanumDrive extends NextFTCOpMode {
    public IncredibotsMecanumDrive() {
        addComponents(
                new SubsystemComponent(Spindexer.INSTANCE),
                new SubsystemComponent(IntakeSystem.INSTANCE),
                new SubsystemComponent(LaunchSystem.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE);
    }

    // change the names and directions to suit your robot
    private final MotorEx frontLeftMotor = new MotorEx("FLMotor");
    private final MotorEx frontRightMotor = new MotorEx("FRMotor").reversed();
    private final MotorEx backLeftMotor = new MotorEx("BLMotor");
    private final MotorEx backRightMotor = new MotorEx("BRMotor").reversed();

    @Override
    public void onUpdate() {
        BindingManager.update();
    }

    @Override
    public void onStop() {
        BindingManager.reset();
    }

    @Override
    public void onStartButtonPressed() {
        Command driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX()
        );
        driverControlled.schedule();

        //enable intake automation only after start button pressed
        //NextFTC calls the periodic function straight up on init.
        IntakeSystem.INSTANCE.enableIntakeAutomation = true;

        //A is green
        Gamepads.gamepad2().a().toggleOnBecomesTrue().whenBecomesTrue(
            LaunchSystem.INSTANCE.launchGreen
        );

        //X is purple
        Gamepads.gamepad2().x().toggleOnBecomesTrue().whenBecomesTrue(
            LaunchSystem.INSTANCE.launchPurple
        );

        //B is launch one ball
        Gamepads.gamepad2().b().toggleOnBecomesTrue().whenBecomesTrue(
                LaunchSystem.INSTANCE.launchOne
        );

        //Right Trigger is launch 3
        Gamepads.gamepad2().rightTrigger().asButton(value -> value > 0.5).toggleOnBecomesTrue().whenBecomesTrue(
            LaunchSystem.INSTANCE.launchThree
        );

        Gamepads.gamepad2().start().toggleOnBecomesTrue()
                .whenBecomesTrue(
                    new InstantCommand(() -> IntakeSystem.INSTANCE.enableIntakeAutomation = true))
                .whenBecomesFalse(
                    new InstantCommand(() -> IntakeSystem.INSTANCE.enableIntakeAutomation = false));
    }
}