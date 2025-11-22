package org.firstinspires.ftc.teamcode.common;

public class LimelightLaunchParameters {
    public double yaw;
    public double distance;
    public double tolerance;

    public LimelightLaunchParameters(double yaw, double distance, double tolerance){
        this.yaw = yaw;
        this.distance = distance;
        this.tolerance = tolerance;
    }

    //make default constructor private
    private LimelightLaunchParameters() {}
}
