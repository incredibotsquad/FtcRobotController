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

    protected Action GetArmControlAction(int position, int velocity, boolean waitForAction, boolean shortWait) {
        return new ArmMotionAsRRAction(myHardware, position, velocity, waitForAction, shortWait);
    }

    protected Action GetSlideControlAction(int position, boolean waitForAction) {
        return new SlideMotionAsRRAction(myHardware, position, waitForAction);
    }

    protected Action GetSideControlAction(int position, int velocity, boolean waitForAction, boolean shortWait) {
        return new SlideMotionAsRRAction(myHardware, position, velocity, waitForAction, shortWait);
    }

    protected Action GetSlideControlAction(int position, boolean waitForAction, boolean shortWait) {
        return new SlideMotionAsRRAction(myHardware, position, waitForAction, shortWait);
    }

    protected Action GetClawControlAction(boolean open, boolean waitForAction, boolean shortWait) {
        return new ClawMotionAsRRAction(myHardware, open, waitForAction, shortWait);
    }

    protected Action GetWristControlAction(double position, boolean waitForAction, boolean shortWait) {
        return new WristMotionAsRRAction(myHardware, position, waitForAction, shortWait);
    }
}
