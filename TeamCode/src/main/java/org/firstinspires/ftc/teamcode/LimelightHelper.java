package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConstants.PICKUP_ARM_LENGTH;
import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

@Config
public class LimelightHelper {

    RobotHardware robotHardware;
    Telemetry telemetry;

    public static double DETECTION_ASPECT_RATIO = 1.1;

    public LimelightHelper(RobotHardware robotHardware, Telemetry telemetry) {
        this.robotHardware = robotHardware;
        this.telemetry = telemetry;
    }

    public List<LimelightLocation> getDetectionsForMode(GameConstants.GAME_COLORS mode) throws InterruptedException {
        List<LimelightLocation> allLocations = new ArrayList<>();

        switch (mode) {
            case RED:
                robotHardware.setLimelightPipeline(LimelightConstants.RED_PIPELINE);
                sleep(200); // Wait for pipeline switch
                List<LimelightLocation> redDetections = getDetections();
                allLocations.addAll(redDetections);
                break;

            case BLUE:
                robotHardware.setLimelightPipeline(LimelightConstants.BLUE_PIPELINE);
                sleep(200);
                List<LimelightLocation> blueDetections = getDetections();
                allLocations.addAll(blueDetections);
                break;

            case YELLOW:
                robotHardware.setLimelightPipeline(LimelightConstants.YELLOW_PIPELINE);
                sleep(200);
                List<LimelightLocation> yellowDetections = getDetections();
                allLocations.addAll(yellowDetections);
                break;

            case RED_AND_YELLOW:
                // Get detections from RED pipeline
                robotHardware.setLimelightPipeline(LimelightConstants.RED_PIPELINE);
                sleep(200);
                List<LimelightLocation> redDetectionsA = getDetections();
                allLocations.addAll(redDetectionsA);

                // Get detections from YELLOW pipeline
                robotHardware.setLimelightPipeline(LimelightConstants.YELLOW_PIPELINE);
                sleep(200);
                List<LimelightLocation> yellowDetections2 = getDetections();
                allLocations.addAll(yellowDetections2);
                break;

            case BLUE_AND_YELLOW:
                // Get detections from BLUE pipeline
                robotHardware.setLimelightPipeline(LimelightConstants.BLUE_PIPELINE);
                sleep(200);
                List<LimelightLocation> blueDetectionsA = getDetections();
                allLocations.addAll(blueDetectionsA);

                // Get detections from YELLOW pipeline
                robotHardware.setLimelightPipeline(LimelightConstants.YELLOW_PIPELINE);
                sleep(200);
                List<LimelightLocation> yellowDetectionsB = getDetections();
                allLocations.addAll(yellowDetectionsB);
                break;
        }

        return allLocations;
    }

    private List<LimelightLocation> getDetections() {
        List<LimelightLocation> locations = new ArrayList<>();

        // Get the latest result from Limelight
        LLResult result = robotHardware.GetLatestLimelightResults();

        if (result != null && result.isValid()) {
            // Get detector results
            List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();

            if (detectorResults != null) {
                for (LLResultTypes.DetectorResult detection : detectorResults) {
                    // Convert class ID to color (using numeric classID like the original laptop code)
                    GameConstants.GAME_COLORS color;
                    int classID = detection.getClassId();
                    switch (classID) {
                        case 0: color = GameConstants.GAME_COLORS.BLUE; break;
                        case 1: color = GameConstants.GAME_COLORS.RED; break;
                        default: color = GameConstants.GAME_COLORS.YELLOW; break;
                    }

                    // Debug: Store detection info for telemetry (moved to avoid loop spam)

                    // Get corner coordinates
                    List<List<Double>> corners = detection.getTargetCorners();
                    if (corners == null || corners.size() < 4) {
                        continue;
                    }

                    // Calculate width and height using different methods
                    double width1 = calculateDistance(corners.get(0), corners.get(1)); // Edge 0→1
                    double height1 = calculateDistance(corners.get(1), corners.get(2)); // Edge 1→2
                    double width2 = calculateDistance(corners.get(2), corners.get(3)); // Edge 2→3
                    double height2 = calculateDistance(corners.get(3), corners.get(0)); // Edge 3→0

                    // Use average of opposite edges
                    double width = (width1 + width2) / 2;
                    double height = (height1 + height2) / 2;

                    // Calculate orientation using the simplified method
                    double orientationAngle = calculateOrientationFromCornerPattern(corners, width, height);

                    // Calculate aspect ratio and rotation score
                    double aspectRatio = width / height;
                    double rotationScore = Math.abs(aspectRatio - LimelightConstants.IDEAL_ASPECT_RATIO);

                    // Get tx and ty values from the detection
                    double tx = detection.getTargetXDegrees();
                    double ty = detection.getTargetYDegrees();

                    // Calculate distance (approximation based on angles)
                    double actualYAngle = LimelightConstants.LIME_LIGHT_MOUNT_ANGLE - ty;
                    double yDistance = (LimelightConstants.LIME_LIGHT_LENS_HEIGHT_INCHES - LimelightConstants.SAMPLE_HEIGHT_INCHES)
                            / Math.tan(Math.toRadians(actualYAngle)) + LimelightConstants.TELESCOPE_OFFSET;

                    // Calculate X distance using proper trigonometry
                    // The Limelight gives us tx in degrees from center
                    // Use basic trigonometry: X = distance * tan(angle)
                    double xDistance = yDistance * Math.tan(Math.toRadians(tx)) - LimelightConstants.LIME_LIGHT_OFFSET;

                    // Apply correction equations to improve accuracy
                    // Store the observed values before correction
                    double observedX = xDistance;
                    double observedY = yDistance;

                    // Apply the correction equations:
                    // real_y = 0.9935*observed_y - 0.0687*observed_x - 0.0810
                    // real_x = 0.3458*observed_y + 1.1758*observed_x - 3.7199
                    double correctedY = 1.0567 * observedY + 0.0046 * observedX - 1.9551;
                    double correctedX = 0.0120 * observedY + 1.0569 * observedX - 0.0186;

                    // Create location with corrected position values
                    LimelightLocation loc = new LimelightLocation(correctedX, correctedY, orientationAngle, rotationScore, aspectRatio, color, result.getPipelineIndex());
                    // Store the raw (uncorrected) values for debugging
                    loc.rawTranslation = observedX;
                    loc.rawExtension = observedY;
                    locations.add(loc);
                }
            }
        }

        return locations;
    }

    private double calculateOrientationFromCornerPattern(List<List<Double>> corners, double width, double height) {
        double aspectRatio = width / height;

        // Simple mapping:
        // High aspect ratio (wide) = 0° (horizontal)
        // Low aspect ratio (tall) = 90° (vertical)

        if (aspectRatio > DETECTION_ASPECT_RATIO) {
            return 0;    // Horizontal
        }
//        else if (aspectRatio > 1.0) {
//            return 45;   // Diagonal
//        }
        else {
            return 90;   // Vertical
        }
    }

    private double calculateDistance(List<Double> point1, List<Double> point2) {
        double dx = point1.get(0) - point2.get(0);
        double dy = point1.get(1) - point2.get(1);
        return Math.sqrt(dx * dx + dy * dy);
    }

    private boolean isValidColor(GameConstants.GAME_COLORS color, GameConstants.GAME_COLORS mode) {
        switch (mode) {
            case RED:
                return color == GameConstants.GAME_COLORS.RED;
            case BLUE:
                return color == GameConstants.GAME_COLORS.BLUE;
            case YELLOW:
                return color == GameConstants.GAME_COLORS.YELLOW;
            case RED_AND_YELLOW:
                return color == GameConstants.GAME_COLORS.RED || color == GameConstants.GAME_COLORS.YELLOW;
            case BLUE_AND_YELLOW:
                return color == GameConstants.GAME_COLORS.BLUE || color == GameConstants.GAME_COLORS.YELLOW;
            default:
                return false;
        }
    }

    public LimelightLocation getBest(List<LimelightLocation> locations, GameConstants.GAME_COLORS mode) {
        if (locations.isEmpty()) {
            return null; // Return null instead of dummy location
        }

        // Filter locations based on detection mode and acceptable ranges
        List<LimelightLocation> validLocations = new ArrayList<>();
        for (LimelightLocation current : locations) {
            if (!isValidColor(current.color, mode)) {
                continue;
            }

            // Filter out detections outside acceptable ranges
            if (current.translation > PICKUP_ARM_LENGTH || current.translation < -1 * PICKUP_ARM_LENGTH) {
                continue;
            }
            if (current.extension > 20 || current.extension < PICKUP_ARM_LENGTH) {
                continue;
            }

            // Additional filtering based on robot constraints (from sample code)
            // Note: These constraints may be too restrictive for initial testing
            /*
            if (current.rotation > 7 || current.rotation < -7) { // REPLACE 7 With ARM Length
                current.score = Integer.MIN_VALUE;
                continue;
                // Within reach of Turret
            }

            if (current.extension < 8 || current.extension > 20) { // REPLACE Actual Extension.
                current.score = Integer.MIN_VALUE;
                continue;
            } // within reach of Slide
            */

            validLocations.add(current);
        }

        if (validLocations.isEmpty()) {
            return null; // Return null instead of dummy location
        }

        // Sort by score and return a random element from valid locations
//        validLocations.sort((a, b) -> Double.compare(b.score, a.score));
//        java.util.Random random = new java.util.Random();
        validLocations.sort((a, b) -> Double.compare(a.extension, b.extension));
//        return validLocations.get(random.nextInt(validLocations.size()));

        return validLocations.get(validLocations.size() / 2);
    }

}
