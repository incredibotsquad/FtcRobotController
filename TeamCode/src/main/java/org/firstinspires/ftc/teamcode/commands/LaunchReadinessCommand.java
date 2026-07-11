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
        //subtracting 10 inches
        LOOKUP_TABLE.put(75.0,  new FlywheelConstants(0.015, 0, 0, 0.1, 0.00061, 1650));
        LOOKUP_TABLE.put(95.0,  new FlywheelConstants(0.015, 0, 0, 0.1, 0.000615, 1700));
        LOOKUP_TABLE.put(115.0,  new FlywheelConstants(0.0125, 0, 0, 0.1, 0.000625, 1800));
        LOOKUP_TABLE.put(130.0, new FlywheelConstants(0.02, 0, 0, 0.1, 0.00066, 2250));
    }

    public LaunchReadinessCommand(LaunchSubsystem launchSubsystem, OdometrySubsystem odometry, TelemetryManager telemetry) {
        this.launchSubsystem = launchSubsystem;
        this.odometry = odometry;
        this.telemetry = telemetry;

        if (CrossOpModeStorage.allianceColor == AllianceColors.BLUE) {
            TARGET_X = CrossOpModeStorage.BLUE_TARGET_X;
            TARGET_Y = CrossOpModeStorage.BLUE_TARGET_Y;
        }
        else {
            TARGET_X = CrossOpModeStorage.RED_TARGET_X;
            TARGET_Y = CrossOpModeStorage.RED_TARGET_Y;
        }
        
        // This command strictly controls the launch Subsystem
        addRequirements(launchSubsystem);
    }

    @Override
    public void execute() {
        // 1. Get current robot posture from odometry
        Pose2d currentPose = odometry.getPose();

        if(launchSubsystem.isTurretLocked())
            launchSubsystem.setTurretPosition(LaunchSubsystem.TURRET_MID);
        else
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

        telemetry.addData("Distance to target", distanceToTarget);
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
        // 1. Calculate the absolute field angle to the target
        double deltaX = TARGET_X - currentPose.getX();
        double deltaY = TARGET_Y - currentPose.getY();

        // Math.atan2(y, x) returns the angle in radians
        double absoluteTargetAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));

        // 2. Account for the robot's current chassis heading
        double robotHeading = currentPose.getRotation().getDegrees();

        // This is the angle the turret needs to be at relative to the front of the robot
        double relativeTargetAngle = absoluteTargetAngle - robotHeading;

        // 3. Normalize the angle to be within [-180, 180]
        // This ensures the turret takes the shortest path to the goal
        while (relativeTargetAngle > 180) relativeTargetAngle -= 360;
        while (relativeTargetAngle < -180) relativeTargetAngle += 360;

        /*
         * MECHANICAL CALCULATION:
         * Ratio: 112 teeth (Turret) / 29 teeth (Servo) = 3.862
         * Servo: GoBILDA 5-Turn = 1800 degrees of total travel (0.0 to 1.0)
         *
         * Formula:
         * (TargetDegrees * Ratio) / TotalServoRange
         */
        double GEAR_RATIO = 112.0 / 29.0;
        double TOTAL_SERVO_RANGE = 1620.0; // servo range is 1800 but we are only going up to 0.9

        // Calculate how many degrees the servo needs to rotate away from center
        double servoOffsetDegrees = relativeTargetAngle * GEAR_RATIO;

        // Convert that degree offset into a 0.0 - 1.0 servo position
        double servoPosAdjustment = servoOffsetDegrees / TOTAL_SERVO_RANGE;

        // 4. Combine with the Midpoint
        double finalServoPosition = LaunchSubsystem.TURRET_MID - servoPosAdjustment;

        // Safety Clamp: Don't let the code command the servo beyond its hardware limits
        finalServoPosition = Math.max(LaunchSubsystem.TURRET_MIN, Math.min(LaunchSubsystem.TURRET_MAX, finalServoPosition));

        // 5. Apply to hardware and Telemetry
        launchSubsystem.setTurretPosition(finalServoPosition);

        telemetry.addData("Turret Target Angle", relativeTargetAngle);
        telemetry.addData("Turret Servo Position", finalServoPosition);
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