package org.firstinspires.ftc.teamcode.common;

public class LimelightYDT {
    public double yaw;
    public double distance;
    public double tolerance;

    public LimelightYDT(double yaw, double distance, double tolerance){
        this.yaw = yaw;
        this.distance = distance;
        this.tolerance = tolerance;
    }

    //make default constructor private
    private LimelightYDT() {}
}
