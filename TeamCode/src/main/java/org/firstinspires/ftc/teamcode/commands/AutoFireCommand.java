package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.common.AllianceColors;
import org.firstinspires.ftc.teamcode.common.CrossOpModeStorage;
import org.firstinspires.ftc.teamcode.subsystems.LaunchSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchGateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;

import java.util.ArrayList;
import java.util.List;

public class AutoFireCommand extends CommandBase {
    private final LaunchSubsystem launchSubsystem;
    private final LaunchGateSubsystem launchGateSubsystem;
    private final OdometrySubsystem odometry;

    // Robot dimensions
    private static final double HALF_SIZE = 5.0; // We track a 10 inch box around the center of the robot

    // Define the vertices of your Triangle Zones (Example coordinates in inches)
    // Area 1: Big Triangle
    private static final double T1X1 = 72.0, T1Y1 = 72.0;
    private static final double T1X2 = 0.0, T1Y2 = 144.0;
    private static final double T1X3 = 144.0, T1Y3 = 144.0;

    // Area 2: Small Triangle
    private static final double T2X1 = 72.0, T2Y1 = 24.0;
    private static final double T2X2 = 48.0, T2Y2 = 0.0;
    private static final double T2X3 = 96.0, T2Y3 = 0.0;
    private static double TARGET_X, TARGET_Y;

    public static double MIN_DISTANCE_FOR_AUTOFIRE = 85;
    public static double CLOSE_ZONE_MAX_MOVING_SHOT_SPEED_IPS = 18.0;
    public static double FAR_ZONE_MAX_MOVING_SHOT_SPEED_IPS = 6.0;

    public AutoFireCommand(LaunchSubsystem launchSubsystem, LaunchGateSubsystem launchGateSubsystem, OdometrySubsystem odometry) {
        this.launchSubsystem = launchSubsystem;
        this.launchGateSubsystem = launchGateSubsystem;
        this.odometry = odometry;
        addRequirements(launchGateSubsystem);
    }

    @Override
    public void initialize() {
        if (CrossOpModeStorage.allianceColor == AllianceColors.BLUE) {
            TARGET_X = CrossOpModeStorage.BLUE_TARGET_X;
            TARGET_Y = CrossOpModeStorage.BLUE_TARGET_Y;
        }
        else {
            TARGET_X = CrossOpModeStorage.RED_TARGET_X;
            TARGET_Y = CrossOpModeStorage.RED_TARGET_Y;
        }
    }

    @Override
    public void execute() {
        Pose2d currentPose = odometry.getPose();

        // Define your target coordinate point (X, Y)
        Translation2d targetLocation = new Translation2d(TARGET_X, TARGET_Y);

        // FTCLib calculates the straight-line distance automatically!
        double distanceToTarget = currentPose.getTranslation().getDistance(targetLocation);

        //dont autofire if we are too close to the goal
        if (distanceToTarget < MIN_DISTANCE_FOR_AUTOFIRE) {
            launchGateSubsystem.closeGate();
            return;
        }

        // Check if the center OR any of the 4 corners are in the zone
        boolean inCloseZone = false;
        boolean inFarZone = false;
        List<Translation2d> pointsToCheck = getRobotPoints(currentPose);

        for (Translation2d p : pointsToCheck) {
            if (isPointInTriangle(p.getX(), p.getY(), T1X1, T1Y1, T1X2, T1Y2, T1X3, T1Y3)) {
                inCloseZone = true;
            }
            if (isPointInTriangle(p.getX(), p.getY(), T2X1, T2Y1, T2X2, T2Y2, T2X3, T2Y3)) {
                inFarZone = true;
            }
        }

        boolean inZone = inCloseZone || inFarZone;

        if (LaunchReadinessCommand.ENABLE_MOVING_SHOT_COMPENSATION && !isMovingShotSpeedAllowed(inCloseZone, inFarZone)) {
            launchGateSubsystem.closeGate();
            return;
        }
        
        // 2. Check if systems are aimed and flywheels are at RPM
        // This uses the target variables updated by LaunchReadinessCommand
        boolean systemReady = launchSubsystem.isReadyToLaunch();

        // 3. Automatic Trigger
        if (inZone && systemReady) {
            launchGateSubsystem.openGate();
            Log.i("AutoFireCommand", " opening gate to launch ");
        } else {
            Log.i("AutoFireCommand", " closing gate ");
            launchGateSubsystem.closeGate();
        }
    }

    @Override
    public void end(boolean interrupted) {
        launchGateSubsystem.closeGate();
    }

    private boolean isMovingShotSpeedAllowed(boolean inCloseZone, boolean inFarZone) {
        double speedIps = odometry.getFieldSpeedInchesPerSecond();

        if (inCloseZone) {
            return speedIps <= CLOSE_ZONE_MAX_MOVING_SHOT_SPEED_IPS;
        }

        if (inFarZone) {
            return speedIps <= FAR_ZONE_MAX_MOVING_SHOT_SPEED_IPS;
        }

        return false;
    }

    /**
     * Calculates the world-space coordinates of the 4 corners of the robot
     * plus the center point based on current heading.
     */
    private List<Translation2d> getRobotPoints(Pose2d pose) {
        List<Translation2d> points = new ArrayList<>();
        double x = pose.getX();
        double y = pose.getY();
        double heading = pose.getRotation().getRadians();

        // Add the center point
        points.add(new Translation2d(x, y));

        // Corner offsets in local robot space
        double[] localX = {HALF_SIZE, HALF_SIZE, -HALF_SIZE, -HALF_SIZE};
        double[] localY = {HALF_SIZE, -HALF_SIZE, HALF_SIZE, -HALF_SIZE};

        // Rotate local offsets into world coordinates
        for (int i = 0; i < 4; i++) {
            double worldX = x + (localX[i] * Math.cos(heading) - localY[i] * Math.sin(heading));
            double worldY = y + (localX[i] * Math.sin(heading) + localY[i] * Math.cos(heading));
            points.add(new Translation2d(worldX, worldY));
        }

        return points;
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
