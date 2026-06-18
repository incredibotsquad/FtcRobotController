package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class Incredibot extends Robot {
    // enum to specify opmode type
    public enum OpModeType {
        TELEOP,
        AUTO
    }

    // Subsystems
    public final DriveSubsystem driveSubsystem;
    public final IntakeSubsystem intakeSubsystem;

    // Hardware
    private final HardwareMap hwMap;

    public Incredibot(HardwareMap hardwareMap, OpModeType opModeType) {
        this.hwMap = hardwareMap;

        // Initialize subsystems - they initialize their own hardware
        driveSubsystem = new DriveSubsystem(hwMap);
        intakeSubsystem = new IntakeSubsystem(hwMap);

        if (opModeType == OpModeType.TELEOP) {
            initTeleop();
        } else if (opModeType == OpModeType.AUTO) {
            initAuto();
        }
    }

    public void initTeleop() {
        //initialize teleop specific scheduler
    }

    public void initAuto() {
        //initialize auto specific scheduler
    }
}