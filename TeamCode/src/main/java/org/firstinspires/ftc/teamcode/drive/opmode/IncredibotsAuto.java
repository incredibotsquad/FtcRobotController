package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;

public abstract class IncredibotsAuto extends LinearOpMode {
    public RobotHardware myHardware;
    public IncredibotsArmControl armControl;
    public MecanumDrive drive;

    protected Action GetArmControlAction(int position, int velocity, boolean waitForAction) {
        return new ArmMotionAsRRAction(myHardware, position, velocity, waitForAction);
    }

    protected Action GetSlideControlAction(int position, boolean waitForAction) {
        return new SlideMotionAsRRAction(myHardware, position, waitForAction);
    }

    protected Action GetClawControlAction(boolean open, double openPosition, double closePosition, boolean waitForAction, boolean shortWait) {
        return new ClawMotionAsRRAction(myHardware, open, openPosition, closePosition, waitForAction, shortWait);
    }
}
