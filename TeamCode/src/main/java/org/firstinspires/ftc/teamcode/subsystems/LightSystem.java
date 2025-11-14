package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.Actions.LightAction;
import org.firstinspires.ftc.teamcode.common.RobotHardware;

public class LightSystem {
    private RobotHardware robotHardware;

    public static double ROBOT_NOT_ALIGNED_TO_SHOOT_LIGHT = 0.277;    //RED
    public static double ROBOT_ALIGNED_TO_SHOOT_LIGHT = 0.5;    //GREEN

    public LightSystem(RobotHardware robotHardware) {
        this.robotHardware = robotHardware;
    }

    public Action getLightOffAction() {
        return new LightAction(robotHardware, 0);
    }

    public Action getRobotAlignedToShootAction() {
        return new LightAction(robotHardware, ROBOT_ALIGNED_TO_SHOOT_LIGHT);
    }

    public Action getRobotNOTAlignedToShootAction() {
        return new LightAction(robotHardware, ROBOT_NOT_ALIGNED_TO_SHOOT_LIGHT);
    }
}
