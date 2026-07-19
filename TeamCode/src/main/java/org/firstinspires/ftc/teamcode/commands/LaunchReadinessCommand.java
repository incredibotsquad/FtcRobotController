package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.common.AllianceColors;
import org.firstinspires.ftc.teamcode.common.CrossOpModeStorage;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchSubsystem;

import java.util.ArrayList;
import java.util.List;
import java.util.NavigableMap;
import java.util.TreeMap;

@Configurable
public class LaunchReadinessCommand extends CommandBase {
    private final LaunchSubsystem launchSubsystem;
    private final OdometrySubsystem odometry;
    private final LimelightSubsystem limelightSubsystem;

    // Define the fixed field coordinate you want to point at (e.g., center of the backdrop or goal)
    public static boolean ENABLE_MOVING_SHOT_COMPENSATION = false;
    public static double SHOT_FLIGHT_BASE_SECONDS = 0.25;
    public static double SHOT_FLIGHT_SECONDS_PER_INCH = 0.0035;
    public static double BURST_MIDPOINT_SECONDS = 0.175;
    public static double MAX_COMPENSATED_SPEED_IPS = 24.0;
    public static double MAX_TARGET_LEAD_INCHES = 18.0;

    // Feature Flags and Tuning
    public static boolean ENABLE_TURRET_VISION_CORRECTION = false;
    public static double VISION_STABILITY_THRESHOLD_IPS = 3.0; // Max speed allowed for vision lock
    public static double VISION_CORRECTION_GAIN = 0.05; // Sensitivity of the vision fine-tuning

    // ... existing fields ...
    public static int VISION_SAMPLE_SIZE = 5;
    private final List<Pose> visionPoseSamples = new ArrayList<>();

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

    private static class ShotSolution {
        double targetX;
        double targetY;
        double distanceToTarget;
        double relativeTargetAngle;
        double turretServoPosition;
        double visorPosition;
        double robotSpeedIps;
        double compensationTimeSeconds;
        double leadDistanceInches;
        boolean turretTargetClamped;
        boolean shotSolutionReady;
        FlywheelConstants flywheelConstants;
    }

    private static double DISTANCE_FROM_APEX = 87.0;

    // 2. The Lookup Table (Distance in Inches -> Constants)
    private static final NavigableMap<Double, FlywheelConstants> NEAR_LOOKUP_TABLE = new TreeMap<>();
    static {
        // Distance (inches), P, I, D, kS, kV, targetRPM
        // These numbers are examples; populate with your tuned values
        NEAR_LOOKUP_TABLE.put(70.0,  new FlywheelConstants(0.0015, 0, 0, 0.05, 0.000345, 3500));
        NEAR_LOOKUP_TABLE.put(DISTANCE_FROM_APEX,  new FlywheelConstants(0.0035, 0, 0, 0.05, 0.00034, 3600));
        NEAR_LOOKUP_TABLE.put(94.0,  new FlywheelConstants(0.0075, 0, 0, 0.05, 0.000345, 3700));
        NEAR_LOOKUP_TABLE.put(110.0,  new FlywheelConstants(0.0045, 0, 0, 0.05, 0.000345, 3950));
    }

    private static final NavigableMap<Double, FlywheelConstants> FAR_LOOKUP_TABLE = new TreeMap<>();
    static {
        // Distance (inches), P, I, D, kS, kV, targetRPM
        // These numbers are examples; populate with your tuned values
        FAR_LOOKUP_TABLE.put(126.0, new FlywheelConstants(0.005, 0, 0, 0.05, 0.00034, 4200));
        FAR_LOOKUP_TABLE.put(131.0, new FlywheelConstants(0.01, 0, 0, 0.05, 0.0003395, 4300));
        FAR_LOOKUP_TABLE.put(136.0, new FlywheelConstants(0.01, 0, 0, 0.05, 0.0003465, 4325));
        FAR_LOOKUP_TABLE.put(144.0, new FlywheelConstants(0.01, 0, 0, 0.05, 0.00034, 4400));
    }

    public LaunchReadinessCommand(LaunchSubsystem launchSubsystem, OdometrySubsystem odometry, LimelightSubsystem limelight, TelemetryManager telemetry) {
        this.launchSubsystem = launchSubsystem;
        this.odometry = odometry;
        this.limelightSubsystem = limelight;
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

        ShotSolution staticSolution = new ShotSolution();
        ShotSolution movingSolution = new ShotSolution();
        ShotSolution activeSolution = new ShotSolution();

        if (launchSubsystem.isLaunchReadinessLocked()){
            activeSolution.turretServoPosition = LaunchSubsystem.TURRET_MID;
            activeSolution.visorPosition = LaunchSubsystem.LAUNCH_VISOR_LOW;
            activeSolution.distanceToTarget = DISTANCE_FROM_APEX;
            activeSolution.flywheelConstants = getFlywheelConstantsBasedOnDistance(activeSolution.distanceToTarget);

            activeSolution.shotSolutionReady = true;

        } else {
            staticSolution = calculateStaticShot(currentPose);
            movingSolution = calculateVelocityCompensatedShot(currentPose, staticSolution.distanceToTarget);
            activeSolution = ENABLE_MOVING_SHOT_COMPENSATION ? movingSolution : staticSolution;
        }

        applyShotSolution(activeSolution);
        addShotTelemetry(staticSolution, movingSolution, activeSolution);
    }

    @Override
    public boolean isFinished() {
        return false; // Returns false so it runs continuously in the background
    }

    @Override
    public void end(boolean interrupted) {
        launchSubsystem.stop();
    }

    private ShotSolution calculateStaticShot(Pose2d currentPose) {
        ShotSolution solution = calculateShotForTarget(currentPose, new Translation2d(TARGET_X, TARGET_Y));
        solution.shotSolutionReady = true;
        return solution;
    }

    private ShotSolution calculateVelocityCompensatedShot(Pose2d currentPose, double staticDistanceToTarget) {
        Translation2d velocity = odometry.getFieldVelocity();
        double compensationTimeSeconds = estimateFlightTimeSeconds(staticDistanceToTarget) + BURST_MIDPOINT_SECONDS;

        Translation2d compensatedTarget = new Translation2d(
                TARGET_X - velocity.getX() * compensationTimeSeconds,
                TARGET_Y - velocity.getY() * compensationTimeSeconds
        );

        ShotSolution solution = calculateShotForTarget(currentPose, compensatedTarget);
        solution.compensationTimeSeconds = compensationTimeSeconds;
        solution.robotSpeedIps = odometry.getFieldSpeedInchesPerSecond();
        solution.leadDistanceInches = new Translation2d(TARGET_X, TARGET_Y).getDistance(compensatedTarget);
        solution.shotSolutionReady =
                solution.robotSpeedIps <= MAX_COMPENSATED_SPEED_IPS &&
                solution.leadDistanceInches <= MAX_TARGET_LEAD_INCHES &&
                !solution.turretTargetClamped;

        return solution;
    }

    private ShotSolution calculateShotForTarget(Pose2d currentPose, Translation2d targetLocation) {
        ShotSolution solution = new ShotSolution();
        solution.targetX = targetLocation.getX();
        solution.targetY = targetLocation.getY();
        solution.distanceToTarget = currentPose.getTranslation().getDistance(targetLocation);

        // --- STANDARD ODOMETRY CALCULATION ---
        double deltaX = targetLocation.getX() - currentPose.getX();
        double deltaY = targetLocation.getY() - currentPose.getY();
        double absoluteTargetAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));
        double robotHeading = currentPose.getRotation().getDegrees();
        double relativeTargetAngle = absoluteTargetAngle - robotHeading;

        while (relativeTargetAngle > 180) relativeTargetAngle -= 360;
        while (relativeTargetAngle < -180) relativeTargetAngle += 360;

        double servoOffsetDegrees = relativeTargetAngle * LaunchSubsystem.GEAR_RATIO;
        double servoPosAdjustment = servoOffsetDegrees / LaunchSubsystem.TOTAL_SERVO_RANGE;
        double odometryServoPosition = LaunchSubsystem.TURRET_MID - servoPosAdjustment;

        // --- VISION FINE-TUNING LOGIC ---
        double finalServoPosition = odometryServoPosition;

        if (ENABLE_TURRET_VISION_CORRECTION) {
            double currentSpeed = odometry.getFieldSpeedInchesPerSecond();

            // Check if robot is stable enough to use vision
            if (currentSpeed < VISION_STABILITY_THRESHOLD_IPS) {
                // Get the horizontal offset of the target from Limelight (tx)
                // Assuming your LimelightSubsystem has getHorizontalOffset() which returns degrees
                Pose limelightRobotPose = limelightSubsystem.getLatestFieldPose();

                if (limelightRobotPose != null){

                    visionPoseSamples.add(limelightRobotPose);

                    // 2. Statistical Filtering of the Heading component
                    if (visionPoseSamples.size() >= VISION_SAMPLE_SIZE) {
                        // Calculate Mean Heading
                        double sumHeading = 0;
                        for (Pose p : visionPoseSamples) {
                            sumHeading += Math.toDegrees(p.getHeading());
                        }
                        double meanHeading = sumHeading / visionPoseSamples.size();

                        // Calculate Standard Deviation
                        double variance = 0;
                        for (Pose p : visionPoseSamples) {
                            variance += Math.pow(Math.toDegrees(p.getHeading()) - meanHeading, 2);
                        }
                        double stdDev = Math.sqrt(variance / visionPoseSamples.size());

                        // Filter samples: Keep those within 1 StdDev (or keep all if noise is very low)
                        double filteredHeadingSum = 0;
                        int validCount = 0;
                        for (Pose p : visionPoseSamples) {
                            double heading = Math.toDegrees(p.getHeading());
                            if (Math.abs(heading - meanHeading) <= stdDev || stdDev < 0.5) {
                                filteredHeadingSum += heading;
                                validCount++;
                            }
                        }

                        if (validCount > 0) {
                            // This is our stable heading error relative to the target
                            double stableTargetOffsetDegrees = filteredHeadingSum / validCount;

                            // 3. Apply the correction to the turret servo
                            // Convert degrees of error into a servo position adjustment
                            double visionAdjustment = (stableTargetOffsetDegrees * LaunchSubsystem.GEAR_RATIO)
                                    / LaunchSubsystem.TOTAL_SERVO_RANGE;

                            Log.i("LaunchReadinessCommand", "Vision adjustment added");

                            // Subtract visionAdjustment to center the turret on the target
                            finalServoPosition = odometryServoPosition - visionAdjustment;

                            visionPoseSamples.clear();
                        }
                    }
                } else {
                    // Clear samples when moving fast to avoid using stale data when we stop
                    visionPoseSamples.clear();
                }
            }
        }

        // Safety Clamp
        double clampedServoPosition = Math.max(LaunchSubsystem.TURRET_MIN,
                Math.min(LaunchSubsystem.TURRET_MAX, finalServoPosition));

        solution.relativeTargetAngle = relativeTargetAngle;
        solution.turretServoPosition = clampedServoPosition;
        solution.turretTargetClamped = Math.abs(clampedServoPosition - finalServoPosition) > 0.0001;
        solution.visorPosition = LaunchSubsystem.LAUNCH_VISOR_LOW;
        solution.flywheelConstants = getFlywheelConstantsBasedOnDistance(solution.distanceToTarget);

        return solution;
    }

//    private ShotSolution calculateShotForTarget(Pose2d currentPose, Translation2d targetLocation) {
//        ShotSolution solution = new ShotSolution();
//        solution.targetX = targetLocation.getX();
//        solution.targetY = targetLocation.getY();
//        solution.distanceToTarget = currentPose.getTranslation().getDistance(targetLocation);
//
//        // 1. Calculate the absolute field angle to the target
//        double deltaX = targetLocation.getX() - currentPose.getX();
//        double deltaY = targetLocation.getY() - currentPose.getY();
//
//        // Math.atan2(y, x) returns the angle in radians
//        double absoluteTargetAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));
//
//        // 2. Account for the robot's current chassis heading
//        double robotHeading = currentPose.getRotation().getDegrees();
//
//        // This is the angle the turret needs to be at relative to the front of the robot
//        double relativeTargetAngle = absoluteTargetAngle - robotHeading;
//
//        // 3. Normalize the angle to be within [-180, 180]
//        // This ensures the turret takes the shortest path to the goal
//        while (relativeTargetAngle > 180) relativeTargetAngle -= 360;
//        while (relativeTargetAngle < -180) relativeTargetAngle += 360;
//
//        // Calculate how many degrees the servo needs to rotate away from center
//        double servoOffsetDegrees = relativeTargetAngle * LaunchSubsystem.GEAR_RATIO;
//
//        // Convert that degree offset into a 0.0 - 1.0 servo position
//        double servoPosAdjustment = servoOffsetDegrees / LaunchSubsystem.TOTAL_SERVO_RANGE;
//
//        // 4. Combine with the Midpoint
//        double unclampedServoPosition = LaunchSubsystem.TURRET_MID - servoPosAdjustment;
//
//        // Safety Clamp: Don't let the code command the servo beyond its hardware limits
//        double finalServoPosition = Math.max(LaunchSubsystem.TURRET_MIN, Math.min(LaunchSubsystem.TURRET_MAX, unclampedServoPosition));
//
//        solution.relativeTargetAngle = relativeTargetAngle;
//        solution.turretServoPosition = finalServoPosition;
//        solution.turretTargetClamped = Math.abs(finalServoPosition - unclampedServoPosition) > 0.0001;
//        solution.visorPosition = LaunchSubsystem.LAUNCH_VISOR_LOW;
//        solution.flywheelConstants = getFlywheelConstantsBasedOnDistance(solution.distanceToTarget);
//
//        return solution;
//    }

    private void applyShotSolution(ShotSolution solution) {
        FlywheelConstants current = solution.flywheelConstants;

        launchSubsystem.setTurretPosition(solution.turretServoPosition);
        launchSubsystem.setFlywheelPID(current.P, current.I, current.D);
        launchSubsystem.updateFeedforward(current.kS, current.kV);
        launchSubsystem.updateFlywheel(current.targetRPM);
        launchSubsystem.setVisorPosition(solution.visorPosition);
        launchSubsystem.setShotSolutionReady(solution.shotSolutionReady);
    }

    private void addShotTelemetry(ShotSolution staticSolution, ShotSolution movingSolution, ShotSolution activeSolution) {
        FlywheelConstants current = activeSolution.flywheelConstants;

        telemetry.addData("Shot Mode", ENABLE_MOVING_SHOT_COMPENSATION ? "MOVING" : "STATIC");
        telemetry.addData("Shot Ready", activeSolution.shotSolutionReady);
        telemetry.addData("Distance to target", activeSolution.distanceToTarget);
        telemetry.addData("Flywheel RPM", current.targetRPM);
        telemetry.addData("Flywheel P", current.P);
        telemetry.addData("Flywheel I", current.I);
        telemetry.addData("Flywheel D", current.D);
        telemetry.addData("Flywheel kS", current.kS);
        telemetry.addData("Flywheel kV", current.kV);
        telemetry.addData("Turret Target Angle", activeSolution.relativeTargetAngle);
        telemetry.addData("Turret Servo Position", activeSolution.turretServoPosition);
        telemetry.addData("Robot Speed IPS", movingSolution.robotSpeedIps);
        telemetry.addData("Moving Shot Time", movingSolution.compensationTimeSeconds);
        telemetry.addData("Moving Shot Lead Inches", movingSolution.leadDistanceInches);
        telemetry.addData("Moving Shot Angle Delta", movingSolution.relativeTargetAngle - staticSolution.relativeTargetAngle);
        telemetry.addData("Moving Shot RPM Delta", movingSolution.flywheelConstants.targetRPM - staticSolution.flywheelConstants.targetRPM);
        telemetry.addData("Moving Shot Turret Clamped", movingSolution.turretTargetClamped);
    }

    private double estimateFlightTimeSeconds(double distanceInches) {
        return SHOT_FLIGHT_BASE_SECONDS + SHOT_FLIGHT_SECONDS_PER_INCH * distanceInches;
    }

    public FlywheelConstants getFlywheelConstantsBasedOnDistance(double distanceFromTarget) {
        // 1. Determine which table to use.
        // If distance is greater than the max near distance (115.0), use the Far table.
        NavigableMap<Double, FlywheelConstants> activeTable = NEAR_LOOKUP_TABLE;

        if (distanceFromTarget > NEAR_LOOKUP_TABLE.lastKey()) {
            activeTable = FAR_LOOKUP_TABLE;
        }

        // 2. Find the keys immediately below and above our current distance in the active table
        Double lowKey = activeTable.floorKey(distanceFromTarget);
        Double highKey = activeTable.ceilingKey(distanceFromTarget);

        // Edge case: Distance is smaller than the active table's lowest tuned point
        // (e.g., if we are in the FAR table but distance is 120, this returns the 130.0 entry)
        if (lowKey == null) return activeTable.get(highKey);

        // Edge case: Distance is larger than the active table's highest tuned point
        if (highKey == null) return activeTable.get(lowKey);

        // Exact match
        if (lowKey.equals(highKey)) return activeTable.get(lowKey);

        // 3. Perform Linear Interpolation between the two points in the active table
        FlywheelConstants low = activeTable.get(lowKey);
        FlywheelConstants high = activeTable.get(highKey);

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
