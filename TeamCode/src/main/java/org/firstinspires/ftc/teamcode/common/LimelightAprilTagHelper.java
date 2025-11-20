package org.firstinspires.ftc.teamcode.common;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;
import java.util.stream.Collectors;

@Config
public class LimelightAprilTagHelper  {

    public static double BASE_YAW_TOLERANCE = 5.0; // Base tolerance in degrees
    public static double TARGET_AREA_INCHES = 16.0; // 8 inches on either side of center
    public static double MIN_TOLERANCE = 2.0; // Minimum tolerance at far distances
    public static double MAX_TOLERANCE = 10.0; // Maximum tolerance at close distances
    public static double TOLERANCE_SCALING_DISTANCE = 40.0; // Distance in inches for scaling

    private RobotHardware robotHardware;
    private AllianceColors allianceColor;

    public LimelightAprilTagHelper(RobotHardware robotHardware) {
        this.robotHardware = robotHardware;
    }

    public void setAllianceColor(AllianceColors allianceColor) {
        this.allianceColor = allianceColor;
        Log.i("LimelightAprilTagHelper", "Alliance Color: " + allianceColor);
    }

    public GamePattern getGamePatternFromObelisk() {
        LLResult result = robotHardware.GetLatestLimelightResults();

        if (result.isValid()) { // Tag is visible

            Log.i("LimelightAprilTagHelper", "getGamePatternFromObelisk: valid results found");

            // Get fiducial (AprilTag) results
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

            if (!fiducialResults.isEmpty()) {
                List<LLResultTypes.FiducialResult> gameTags = fiducialResults.stream().filter(fr -> fr.getFiducialId() == 21 || fr.getFiducialId() == 22 || fr.getFiducialId() == 23).collect(Collectors.toList());

                Log.i("LimelightAprilTagHelper", "getGamePatternFromObelisk: April tag results found: " + gameTags.size());

                if (!gameTags.isEmpty()) {
                    int idToSearch = gameTags.get(0).getFiducialId();

                    Log.i("LimelightAprilTagHelper", "getGamePatternFromObelisk: id to search: " + idToSearch);

                    List<GamePattern> foundPattern = AprilTagConstants.patterns.stream().filter(pattern -> pattern.tagId == idToSearch).collect(Collectors.toList());

                    if (!foundPattern.isEmpty()) {
                        return foundPattern.get(0);
                    }
                }
            }
        }

        return null;
    }

    public LaunchYawDistanceTolerance getGoalYawDistanceToleranceFromCurrentPosition() {

        LLResult result = robotHardware.GetLatestLimelightResults();

        if (result.isValid()) { // Tag is visible

            // Get fiducial (AprilTag) results
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

            if (!fiducialResults.isEmpty()) {
                // Get the first detected AprilTag

                LLResultTypes.FiducialResult primaryTarget = null;

                if(allianceColor == null) return null;

                switch (allianceColor) {
                    case RED:
                        List<LLResultTypes.FiducialResult> redResults = fiducialResults.stream().filter(fr -> fr.getFiducialId() == AprilTagConstants.RED_ALLIANCE_TAG_ID).collect(Collectors.toList());
                        if (!redResults.isEmpty())
                        {
                            Log.i("LimelightAprilTagHelper", allianceColor + " april tag found");
                            primaryTarget = redResults.get(0);
                        }
                        else
                        {
                            Log.i("LimelightAprilTagHelper", allianceColor + " april tag not found");
                            return null;
                        }
                        break;

                    case BLUE:
                        List<LLResultTypes.FiducialResult> blueResults = fiducialResults.stream().filter(fr -> fr.getFiducialId() == AprilTagConstants.BLUE_ALLIANCE_TAG_ID).collect(Collectors.toList());
                        if (!blueResults.isEmpty())
                        {
                            Log.i("LimelightAprilTagHelper", allianceColor + " april tag found");

                            primaryTarget = blueResults.get(0);
                        }
                        else
                        {
                            Log.i("LimelightAprilTagHelper", allianceColor + " april tag not found");
                            return null;
                        }
                        break;
                }

                // Get TARGET POSE IN CAMERA SPACE
                // This is the position of the AprilTag as seen from the camera
                Pose3D targetPoseInCameraSpace = primaryTarget.getCameraPoseTargetSpace();

                // Get RAW position values (target position relative to camera)
                double rawX = targetPoseInCameraSpace.getPosition().x;
                double rawY = targetPoseInCameraSpace.getPosition().y;
                double rawZ = targetPoseInCameraSpace.getPosition().z;

                double yaw = targetPoseInCameraSpace.getOrientation().getYaw();

                // For TARGET POSE IN CAMERA SPACE, Limelight returns METERS
                // Convert to inches for our calculations
                double conversionFactor = 39.3701; // meters to inches

                // Convert to inches
                // Note: In camera space, +X is right, +Y is down, +Z is forward (away from camera)
                double x = rawX * conversionFactor;
                double y = rawY * conversionFactor;
                double z = rawZ * conversionFactor;

                // Calculate horizontal distance to the AprilTag
                double horizontalDistance = calculateHorizontalDistance(x, z);

                // Calculate dynamic tolerance based on distance
                double dynamicTolerance = calculateDistanceBasedTolerance(horizontalDistance);

                Log.i("LimelightAprilTagHelper", "YAW: " + yaw);
                Log.i("LimelightAprilTagHelper", "DISTANCE: " + horizontalDistance);
                Log.i("LimelightAprilTagHelper", "TOLERANCE: " + dynamicTolerance);

                return new LaunchYawDistanceTolerance(yaw, horizontalDistance, dynamicTolerance);
                // Use dynamic tolerance for alignment check
//                if (Math.abs(yaw) < dynamicTolerance) {
//                    Log.i("LimelightAprilTagHelper", " Shooter Ready");
//                    return true;
//                }
            }
        }
        Log.i("LimelightAprilTagHelper", "No April tags found");

        return null;
    }

    /**
     * Calculate horizontal distance from the robot to the AprilTag
     * @param x X distance from robot to tag in inches (left/right)
     * @param y Y distance from robot to tag in inches (forward/backward)
     * @return Horizontal distance in inches (straight-line distance in XY plane)
     */
    private double calculateHorizontalDistance(double x, double y) {
        // Calculate the horizontal distance using Pythagorean theorem
        // This gives us the distance in the XY plane (ignoring Z/height)
        return Math.sqrt(x * x + y * y);
    }

    /**
     * Calculate tolerance based on distance to the AprilTag.
     * The closer the bot is to the tag, the more tolerance is allowed.
     * This accounts for the 8-inch range on either side of the AprilTag center.
     *
     * @param distance Horizontal distance to the AprilTag in inches
     * @return Dynamic yaw tolerance in degrees
     */
    private double calculateDistanceBasedTolerance(double distance) {
        // As the robot gets closer, the angular tolerance should increase
        // because the same physical offset (8 inches) represents a larger angle

        // Calculate the angle subtended by the target area (8 inches on either side)
        // at the current distance
        double targetAngle = Math.toDegrees(Math.atan((TARGET_AREA_INCHES / 2) / distance));

        // Use inverse relationship: closer distance = higher tolerance
        // Formula: tolerance increases as distance decreases
        double scaledTolerance = MIN_TOLERANCE +
                (MAX_TOLERANCE - MIN_TOLERANCE) * (TOLERANCE_SCALING_DISTANCE / (distance + TOLERANCE_SCALING_DISTANCE));

        // Ensure tolerance is within bounds
        scaledTolerance = Math.max(MIN_TOLERANCE, Math.min(MAX_TOLERANCE, scaledTolerance));

        // You can choose to use either the scaled tolerance or the target angle
        // The target angle directly represents the 8-inch tolerance zone
        // For this implementation, we'll use the larger of the two to be more forgiving
        return Math.max(scaledTolerance, targetAngle);
    }
}