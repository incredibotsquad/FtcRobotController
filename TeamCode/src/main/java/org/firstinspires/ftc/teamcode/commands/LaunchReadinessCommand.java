package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.common.AllianceColors;
import org.firstinspires.ftc.teamcode.common.CrossOpModeStorage;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchSubsystem;

import java.util.NavigableMap;
import java.util.TreeMap;

@Configurable
public class LaunchReadinessCommand extends CommandBase {
    private final LaunchSubsystem launchSubsystem;
    private final OdometrySubsystem odometry;

    // Define the fixed field coordinate you want to point at (e.g., center of the backdrop or goal)
    public static boolean ENABLE_MOVING_SHOT_COMPENSATION = false;
    public static double SHOT_FLIGHT_BASE_SECONDS = 0.25;
    public static double SHOT_FLIGHT_SECONDS_PER_INCH = 0.0035;
    public static double BURST_MIDPOINT_SECONDS = 0.175;
    public static double MAX_COMPENSATED_SPEED_IPS = 24.0;
    public static double MAX_TARGET_LEAD_INCHES = 18.0;

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

        ShotSolution staticSolution = calculateStaticShot(currentPose);
        ShotSolution movingSolution = calculateVelocityCompensatedShot(currentPose, staticSolution.distanceToTarget);
        ShotSolution activeSolution = ENABLE_MOVING_SHOT_COMPENSATION ? movingSolution : staticSolution;

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

        // 1. Calculate the absolute field angle to the target
        double deltaX = targetLocation.getX() - currentPose.getX();
        double deltaY = targetLocation.getY() - currentPose.getY();

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
        double unclampedServoPosition = LaunchSubsystem.TURRET_MID - servoPosAdjustment;

        // Safety Clamp: Don't let the code command the servo beyond its hardware limits
        double finalServoPosition = Math.max(LaunchSubsystem.TURRET_MIN, Math.min(LaunchSubsystem.TURRET_MAX, unclampedServoPosition));

        solution.relativeTargetAngle = relativeTargetAngle;
        solution.turretServoPosition = finalServoPosition;
        solution.turretTargetClamped = Math.abs(finalServoPosition - unclampedServoPosition) > 0.0001;
        solution.visorPosition = LaunchSubsystem.LAUNCH_VISOR_LOW;
        solution.flywheelConstants = getFlywheelConstantsBasedOnDistance(solution.distanceToTarget);

        return solution;
    }

    private void applyShotSolution(ShotSolution solution) {
        FlywheelConstants current = solution.flywheelConstants;

        if(launchSubsystem.isTurretLocked())
            launchSubsystem.setTurretPosition(LaunchSubsystem.TURRET_MID);
        else
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
