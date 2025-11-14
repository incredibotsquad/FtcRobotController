package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.Actions.IntakeLightAction;
import org.firstinspires.ftc.teamcode.common.RobotHardware;

public class LightSystem {
    private RobotHardware robotHardware;

    public static double INTAKE_ON_LIGHT = 0.277;    //RED
    public static double ROBOT_ALIGNED_TO_SHOOT_LIGHT = 0.5;    //GREEN

    public LightSystem(RobotHardware robotHardware) {
        this.robotHardware = robotHardware;
    }

    public Action getLightOffAction() {
        return new IntakeLightAction(robotHardware, 0);
    }

    public Action getIntakeOnLightAction() {
        return new IntakeLightAction(robotHardware, INTAKE_ON_LIGHT);
    }

    public Action getIntakeOffLightAction() {
        return getLightOffAction();
    }

    public Action getRobotAlignedToShootAction() {
        return new IntakeLightAction(robotHardware, ROBOT_ALIGNED_TO_SHOOT_LIGHT);
    }

    public Action getRobotNOTAlignedToShootAction() {
        return getLightOffAction();
    }
}
