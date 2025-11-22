package org.firstinspires.ftc.teamcode.common;

public class RobotLaunchParameters {
    public double flywheelVelocity;
    public double visorPosition;

    public RobotLaunchParameters(double flywheelVelocity, double visorPosition){
        this.flywheelVelocity = flywheelVelocity;
        this.visorPosition = visorPosition;
    }

    //make default constructor private
    private RobotLaunchParameters() {}
}
