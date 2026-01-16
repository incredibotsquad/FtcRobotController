package org.firstinspires.ftc.teamcode.common;

import java.util.List;

public class BallLaunchParameters {
    public double distance;
    public double flywheelVelocity;
    public List<Double> visorPositions;


    public BallLaunchParameters(double distance, double flywheelVelocity, double visorPosition1, double visorPosition2, double visorPosition3){
        this.distance = distance;
        this.flywheelVelocity = flywheelVelocity;
        this.visorPositions = List.of(visorPosition1, visorPosition2, visorPosition3);
    }

    //make default constructor private
    private BallLaunchParameters() {}
}
