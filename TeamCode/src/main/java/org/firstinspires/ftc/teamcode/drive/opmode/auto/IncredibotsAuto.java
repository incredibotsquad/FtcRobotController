package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.opmode.auto.roadrunner.ArmMotionAsRRAction;
import org.firstinspires.ftc.teamcode.drive.opmode.auto.roadrunner.ClawMotionAsRRAction;
import org.firstinspires.ftc.teamcode.drive.opmode.IncredibotsArmControl;
import org.firstinspires.ftc.teamcode.drive.opmode.auto.roadrunner.HighBasketSampleDropIntakeMotionAsRRAction;
import org.firstinspires.ftc.teamcode.drive.opmode.auto.roadrunner.IntakeMotionAsRRAction;
import org.firstinspires.ftc.teamcode.drive.opmode.auto.roadrunner.SlideMotionAsRRAction;
import org.firstinspires.ftc.teamcode.drive.opmode.auto.roadrunner.WristMotionAsRRAction;

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

    protected Action GetIntakeControlAction(boolean intake, boolean stop, boolean waitForAction) {
        return new IntakeMotionAsRRAction(myHardware, intake, stop, waitForAction);
    }

    protected Action GetHighBasketSampleDropIntakeMotionAsRRAction() {
        return new HighBasketSampleDropIntakeMotionAsRRAction(myHardware);
    }
}
