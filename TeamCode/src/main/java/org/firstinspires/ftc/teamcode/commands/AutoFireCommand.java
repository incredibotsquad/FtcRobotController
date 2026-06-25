package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.subsystems.LaunchSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchGateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;

public class AutoFireCommand extends CommandBase {
    private final LaunchSubsystem launcher;
    private final LaunchGateSubsystem gate;
    private final OdometrySubsystem odometry;

    // Define the vertices of your Triangle Zones (Example coordinates in inches)
    // TODO: UPDATE THESE NUMBERS
    // Area 1: Big Triangle
    private static final double T1X1 = 12.0, T1Y1 = 12.0;
    private static final double T1X2 = 48.0, T1Y2 = 12.0;
    private static final double T1X3 = 30.0, T1Y3 = 48.0;

    // Area 2: Small Triangle
    private static final double T2X1 = 12.0, T2Y1 = 12.0;
    private static final double T2X2 = 48.0, T2Y2 = 12.0;
    private static final double T2X3 = 30.0, T2Y3 = 48.0;

    public AutoFireCommand(LaunchSubsystem launcher, LaunchGateSubsystem gate, OdometrySubsystem odometry) {
        this.launcher = launcher;
        this.gate = gate;
        this.odometry = odometry;
        addRequirements(gate); 
    }

    @Override
    public void execute() {
        Pose2d currentPose = odometry.getPose();
        
        // 1. Check if we are in the zone
        boolean inZone = isPointInTriangle(
                currentPose.getX(),
                currentPose.getY(),
                T1X1, T1Y1, T1X2, T1Y2, T1X3, T1Y3) ||
                isPointInTriangle(
                    currentPose.getX(),
                    currentPose.getY(),
                    T2X1, T2Y1, T2X2, T2Y2, T2X3, T2Y3);
        
        // 2. Check if systems are aimed and flywheels are at RPM
        // This uses the target variables updated by LaunchReadinessCommand
        boolean systemReady = launcher.isReadyToLaunch();

        // 3. Automatic Trigger
        if (inZone && systemReady) {
            gate.openGate();
        } else {
            gate.closeGate();
        }
    }

    @Override
    public void end(boolean interrupted) {
        gate.closeGate();
    }

    private boolean isPointInTriangle(double px, double py, double x1, double y1, double x2, double y2, double x3, double y3) {
        double d1 = (px - x2) * (y1 - y2) - (x1 - x2) * (py - y2);
        double d2 = (px - x3) * (y2 - y3) - (x2 - x3) * (py - y3);
        double d3 = (px - x1) * (y3 - y1) - (x3 - x1) * (py - y1);
        boolean has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
        boolean has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);
        return !(has_neg && has_pos);
    }
}