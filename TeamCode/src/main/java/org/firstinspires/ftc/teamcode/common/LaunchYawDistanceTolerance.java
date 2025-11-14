package org.firstinspires.ftc.teamcode.common;

public class LaunchYawDistanceTolerance {
    public double yaw;
    public double distance;
    public double tolerance;

    public LaunchYawDistanceTolerance(double yaw, double distance, double tolerance){
        this.yaw = yaw;
        this.distance = distance;
        this.tolerance = tolerance;
    }

    //make default constructor private
    private LaunchYawDistanceTolerance() {}
}
