package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.config.Config;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

@Config
public class IntakeLight implements Subsystem {
    public static double INTAKE_LIGHT_SUCCESSFUL_INTAKE = 1;
    public static double INTAKE_LIGHT_ONE_BALL = 1;
    public static double INTAKE_LIGHT_TWO_BALLS = 1;
    public static double INTAKE_LIGHT_THREE_BALLS = 1;

    public static final IntakeLight INSTANCE = new IntakeLight();

    private IntakeLight() {
    }

    private final ServoEx intakeLightAsServo = new ServoEx("IntakeLight");

    private final Command intakeLight = new SetPosition(intakeLightAsServo, INTAKE_LIGHT_SUCCESSFUL_INTAKE).requires(intakeLightAsServo);
    public final Command indicateEmpty = new SetPosition(intakeLightAsServo, INTAKE_LIGHT_ONE_BALL).requires(intakeLightAsServo);
    public final Command indicateOneBall = new SetPosition(intakeLightAsServo, INTAKE_LIGHT_ONE_BALL).requires(intakeLightAsServo);
    public final Command indicateTwoBalls = new SetPosition(intakeLightAsServo, INTAKE_LIGHT_TWO_BALLS).requires(intakeLightAsServo);
    public final Command indicateThreeBalls = new SetPosition(intakeLightAsServo, INTAKE_LIGHT_THREE_BALLS).requires(intakeLightAsServo);
    public final Command turnOffLight = new SetPosition(intakeLightAsServo, 0);
    public final Command indiateSuccessfulIntake = intakeLight
            .thenWait(0.1)
            .then(intakeLight)
            .thenWait(0.1)
            .then(intakeLight)
            .thenWait(0.1)
            .then(turnOffLight);
}
