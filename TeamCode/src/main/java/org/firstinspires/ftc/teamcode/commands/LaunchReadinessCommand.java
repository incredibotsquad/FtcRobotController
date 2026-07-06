package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.common.AllianceColors;
import org.firstinspires.ftc.teamcode.common.CrossOpModeStorage;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchSubsystem;

import java.util.NavigableMap;
import java.util.TreeMap;

public class LaunchReadinessCommand extends CommandBase {
    private final LaunchSubsystem launchSubsystem;
    private final OdometrySubsystem odometry;

    // Define the fixed field coordinate you want to point at (e.g., center of the backdrop or goal)

    private static final double BLUE_TARGET_X = 0.0;
    private static final double BLUE_TARGET_Y = 144.0;

    private static final double RED_TARGET_X = 144.0;
    private static final double RED_TARGET_Y = 144.0;

    private double TARGET_X;
    private double TARGET_Y;

    private TelemetryManager telemetry;

    public static class FlywheelConstants {
        public double P, I, D, kS, kV, targetRPM;

        public FlywheelConstants(double p, double i, double d, double ks, double kv, double rpm) {
            this.P = p; this.I = i; this.D = d;
            this.kS = ks; this.kV = kv; this.targetRPM = rpm;
        }
    }

    // 2. The Lookup Table (Distance in Inches -> Constants)
    private static final NavigableMap<Double, FlywheelConstants> LOOKUP_TABLE = new TreeMap<>();
    static {
        // Distance (inches), P, I, D, kS, kV, targetRPM
        // These numbers are examples; populate with your tuned values
        LOOKUP_TABLE.put(85.0,  new FlywheelConstants(0.015, 0, 0, 0.1, 0.00061, 1650));
        LOOKUP_TABLE.put(105.0,  new FlywheelConstants(0.015, 0, 0, 0.1, 0.000615, 1700));
        LOOKUP_TABLE.put(125.0,  new FlywheelConstants(0.0125, 0, 0, 0.1, 0.000625, 1800));
        LOOKUP_TABLE.put(140.0, new FlywheelConstants(0.02, 0, 0, 0.1, 0.00066, 2250));
    }

    public LaunchReadinessCommand(LaunchSubsystem launchSubsystem, OdometrySubsystem odometry, TelemetryManager telemetry) {
        this.launchSubsystem = launchSubsystem;
        this.odometry = odometry;
        this.telemetry = telemetry;

        if (CrossOpModeStorage.allianceColor == AllianceColors.BLUE) {
            TARGET_X = BLUE_TARGET_X;
            TARGET_Y = BLUE_TARGET_Y;
        }
        else {
            TARGET_X = RED_TARGET_X;
            TARGET_Y = RED_TARGET_Y;
        }
        
        // This command strictly controls the launch Subsystem
        addRequirements(launchSubsystem);
    }

    @Override
    public void execute() {
        // 1. Get current robot posture from odometry
        Pose2d currentPose = odometry.getPose();

        updateTurretAlignmentFromCurrentPose(currentPose);

        updateFlywheelRPMFromCurrentPose(currentPose);

        updateVisorPositionFromCurrentPose(currentPose);
    }

    @Override
    public boolean isFinished() {
        return false; // Returns false so it runs continuously in the background
    }

    @Override
    public void end(boolean interrupted) {
        launchSubsystem.stop();
    }

    private void updateFlywheelRPMFromCurrentPose(Pose2d currentPose) {
        // Define your target coordinate point (X, Y)
        Translation2d targetLocation = new Translation2d(TARGET_X, TARGET_Y);

        // FTCLib calculates the straight-line distance automatically!
        double distanceToTarget = currentPose.getTranslation().getDistance(targetLocation);

        // Get the interpolated constants
        FlywheelConstants current = getFlywheelConstantsBasedOnDistance(distanceToTarget);

        telemetry.addData("Distnace to target", distanceToTarget);
        telemetry.addData("Flywheel RPM", current.targetRPM);
        telemetry.addData("Flywheel P", current.P);
        telemetry.addData("Flywheel I", current.I);
        telemetry.addData("Flywheel D", current.D);
        telemetry.addData("Flywheel kS", current.kS);
        telemetry.addData("Flywheel kV", current.kV);

        // Apply them to the subsystem
        launchSubsystem.setFlywheelPID(current.P, current.I, current.D);
        launchSubsystem.updateFeedforward(current.kS, current.kV);
        launchSubsystem.updateFlywheel(current.targetRPM);
    }

    private void updateTurretAlignmentFromCurrentPose(Pose2d currentPose) {
        // 2. Calculate trigonometry delta relative to the target
        double deltaX = TARGET_X - currentPose.getX();
        double deltaY = TARGET_Y - currentPose.getY();

        // Math.atan2 returns radians; convert it to degrees for your turret
        double absoluteTargetAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));

        // 3. Compensate for the robot's own chassis heading
        double robotHeading = currentPose.getRotation().getDegrees();
        double localizedTargetAngle = absoluteTargetAngle - robotHeading;

        // 4. Update the turret PID controller
//        launchSubsystem.alignTurretToAngle(launchSubsystem.getTurretAngle(), localizedTargetAngle);

        //TODO: UPDATE THIS FUNCTION TO ALIGN TURRET TO GOAL
        launchSubsystem.setTurretPosition(LaunchSubsystem.TURRET_MID);
    }

    private void updateVisorPositionFromCurrentPose(Pose2d currentPose) {
        //TODO: UPDATE THIS FUNCTION
        launchSubsystem.setVisorPosition(LaunchSubsystem.LAUNCH_VISOR_LOW);
    }

    public FlywheelConstants getFlywheelConstantsBasedOnDistance(double distanceFromTarget) {
        // Find the keys immediately below and above our current distance
        Double lowKey = LOOKUP_TABLE.floorKey(distanceFromTarget);
        Double highKey = LOOKUP_TABLE.ceilingKey(distanceFromTarget);

        // Edge case: Distance is smaller than our lowest tuned point
        if (lowKey == null) return LOOKUP_TABLE.get(highKey);
        // Edge case: Distance is larger than our highest tuned point
        if (highKey == null) return LOOKUP_TABLE.get(lowKey);
        // Exact match
        if (lowKey.equals(highKey)) return LOOKUP_TABLE.get(lowKey);

        // Perform Linear Interpolation between the two points
        FlywheelConstants low = LOOKUP_TABLE.get(lowKey);
        FlywheelConstants high = LOOKUP_TABLE.get(highKey);

        double t = (distanceFromTarget - lowKey) / (highKey - lowKey); // 0.0 to 1.0 factor

        return new FlywheelConstants(
                interpolate(low.P, high.P, t),
                interpolate(low.I, high.I, t),
                interpolate(low.D, high.D, t),
                interpolate(low.kS, high.kS, t),
                interpolate(low.kV, high.kV, t),
                interpolate(low.targetRPM, high.targetRPM, t)
        );
    }

    private double interpolate(double start, double end, double t) {
        return start + (end - start) * t;
    }

}