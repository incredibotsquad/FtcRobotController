package org.firstinspires.ftc.teamcode.drive.opmode;

import java.util.ArrayList;
import java.util.List;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "Limelight Vision OpMode")
public class LimelightVisionOpMode extends LinearOpMode {

    private Limelight3A limelight;
    public DetectionMode detectionMode = DetectionMode.RED_ONLY; // Default mode
    private List<Location> locations = new ArrayList<>();

    // Detection mode enum
    public enum DetectionMode {
        RED_ONLY,
        BLUE_ONLY,
        YELLOW_ONLY,
        RED_AND_YELLOW,
        BLUE_AND_YELLOW
    }

    // Color enum
    public enum Color {
        RED, BLUE, YELLOW
    }

    // Location class
    public static class Location {
        public double translation;
        public double extension;
        public double rotation;
        public double score;
        public Color color;
        public double rotScore;
        public double distScore;
        public double yScore;
        public double orientationAngle; // Actual orientation angle in degrees
        public double rawTranslation; // Original X position before correction
        public double rawExtension;   // Original Y position before correction

        public Location(double trans, double extension, double rotation, Color color) {
            this.translation = trans;
            this.extension = extension;
            this.rotation = rotation;
            this.color = color;
            this.orientationAngle = 0; // Initialize to 0
            this.rawTranslation = trans; // Default to same as corrected
            this.rawExtension = extension; // Default to same as corrected
        }
    }

    // Constants class
    public static class LimelightConstants {
        public static double IDEAL_ASPECT_RATIO = 3.5 / 1.5; // Expected width:height ratio for a properly aligned sample
        public static double IDEAL_Y = 0; // Set to 0 for camera-only testing

        // Pipeline mappings
        public static int RED_PIPELINE = 2;
        public static int YELLOW_PIPELINE = 3;
        public static int BLUE_PIPELINE = 4;

        public static double LIME_LIGHT_MOUNT_ANGLE = 30; // a1
        public static double LIME_LIGHT_LENS_HEIGHT_INCHES = 7.5; // h1
        public static double LIME_LIGHT_OFFSET = -3; // h1
        public static double SAMPLE_HEIGHT_INCHES = 0; // h2
        public static double TELESCOPE_OFFSET = 0; // Set to 0 for camera-only testing
        public static double X_WEIGHT = 2;
        public static double Y_WEIGHT = 3;
        public static double ROT_WEIGHT = 20;

        // Camera intrinsic parameters for Limelight 3A
        // Based on OV5647 sensor specifications and Limelight 3A documentation
        public static double SENSOR_WIDTH_PIXELS = 640.0; // Limelight 3A resolution: 640x480 @ 90FPS
        public static double SENSOR_HEIGHT_PIXELS = 480.0; // Limelight 3A resolution: 640x480 @ 90FPS
        public static double HORIZONTAL_FOV_DEGREES = 54.5; // Limelight 3A horizontal FOV
        public static double VERTICAL_FOV_DEGREES = 42.0; // Limelight 3A vertical FOV
        public static double SENSOR_WIDTH_MM = 3.76; // OV5647 sensor image area: 3.76 × 2.74 mm
        public static double SENSOR_HEIGHT_MM = 2.74; // OV5647 sensor image area: 3.76 × 2.74 mm
        public static double FOCAL_LENGTH_PIXELS = (SENSOR_WIDTH_PIXELS / 2.0) / Math.tan(Math.toRadians(HORIZONTAL_FOV_DEGREES / 2.0));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.addData("Status", "Initializing Limelight...");
        telemetry.update();

        // Start the Limelight
        limelight.start();

        telemetry.addData("Status", "Limelight initialized");
        telemetry.addData("Detection Mode", detectionMode.toString());
        telemetry.addLine("Controls:");
        telemetry.addLine("A - Red Only");
        telemetry.addLine("B - Blue Only");
        telemetry.addLine("Y - Yellow Only");
        telemetry.addLine("X - Red and Yellow");
        telemetry.addLine("Right Bumper - Blue and Yellow");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Handle gamepad input for detection mode switching
            handleGamepadInput();

            try {
                // Get detections from Limelight using appropriate pipelines
                List<Location> allLocations = getDetectionsForMode(detectionMode);
                locations = allLocations;

                if (!allLocations.isEmpty()) {
                    Location best = getBest(allLocations, detectionMode);
                    displayTelemetry(best, allLocations.size());
                } else {
                    displayTelemetry(null, 0);
                }

            } catch (Exception e) {
                telemetry.addData("Error", e.getMessage());
                telemetry.update();
            }

            sleep(100); // 10 Hz update rate
        }

        // Stop the Limelight when OpMode ends
        limelight.stop();
    }

    private void handleGamepadInput() {
        if (gamepad1.a && detectionMode != DetectionMode.RED_ONLY) {
            detectionMode = DetectionMode.RED_ONLY;
            sleep(200); // Debounce
        } else if (gamepad1.b && detectionMode != DetectionMode.BLUE_ONLY) {
            detectionMode = DetectionMode.BLUE_ONLY;
            sleep(200);
        } else if (gamepad1.y && detectionMode != DetectionMode.YELLOW_ONLY) {
            detectionMode = DetectionMode.YELLOW_ONLY;
            sleep(200);
        } else if (gamepad1.x && detectionMode != DetectionMode.RED_AND_YELLOW) {
            detectionMode = DetectionMode.RED_AND_YELLOW;
            sleep(200);
        } else if (gamepad1.right_bumper && detectionMode != DetectionMode.BLUE_AND_YELLOW) {
            detectionMode = DetectionMode.BLUE_AND_YELLOW;
            sleep(200);
        }
    }

    private List<Location> getDetectionsForMode(DetectionMode mode) throws InterruptedException {
        List<Location> allLocations = new ArrayList<>();

        switch (mode) {
            case RED_ONLY:
                setPipeline(LimelightConstants.RED_PIPELINE);
                sleep(200); // Wait for pipeline switch
                List<Location> redDetections = getDetections();
                allLocations.addAll(redDetections);
                break;

            case BLUE_ONLY:
                setPipeline(LimelightConstants.BLUE_PIPELINE);
                sleep(200);
                List<Location> blueDetections = getDetections();
                allLocations.addAll(blueDetections);
                break;

            case YELLOW_ONLY:
                setPipeline(LimelightConstants.YELLOW_PIPELINE);
                sleep(200);
                List<Location> yellowDetections = getDetections();
                allLocations.addAll(yellowDetections);
                break;

            case RED_AND_YELLOW:
                // Get detections from RED pipeline
                setPipeline(LimelightConstants.RED_PIPELINE);
                sleep(200);
                List<Location> redDetectionsA = getDetections();
                allLocations.addAll(redDetectionsA);

                // Get detections from YELLOW pipeline
                setPipeline(LimelightConstants.YELLOW_PIPELINE);
                sleep(200);
                List<Location> yellowDetections2 = getDetections();
                allLocations.addAll(yellowDetections2);
                break;

            case BLUE_AND_YELLOW:
                // Get detections from BLUE pipeline
                setPipeline(LimelightConstants.BLUE_PIPELINE);
                sleep(200);
                List<Location> blueDetectionsA = getDetections();
                allLocations.addAll(blueDetectionsA);

                // Get detections from YELLOW pipeline
                setPipeline(LimelightConstants.YELLOW_PIPELINE);
                sleep(200);
                List<Location> yellowDetectionsB = getDetections();
                allLocations.addAll(yellowDetectionsB);
                break;
        }

        return allLocations;
    }

    private List<Location> getDetections() {
        List<Location> locations = new ArrayList<>();

        // Get the latest result from Limelight
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            // Get detector results
            List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();

            if (detectorResults != null) {
                for (LLResultTypes.DetectorResult detection : detectorResults) {
                    // Convert class ID to color (using numeric classID like the original laptop code)
                    Color color;
                    int classID = detection.getClassId();
                    switch (classID) {
                        case 0: color = Color.BLUE; break;
                        case 1: color = Color.RED; break;
                        case 2: color = Color.YELLOW; break;
                        default: color = Color.YELLOW; break;
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
                    double correctedY = 0.9935 * observedY - 0.0687 * observedX - 0.0810;
                    double correctedX = 0.3458 * observedY + 1.1758 * observedX - 3.7199;

                    // Create location with corrected position values
                    Location loc = new Location(correctedX, correctedY, rotationScore, color);
                    // Store the raw (uncorrected) values for debugging
                    loc.rawTranslation = observedX;
                    loc.rawExtension = observedY;
                    loc.orientationAngle = orientationAngle;
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

        if (aspectRatio > 2.0) {
            return 0;    // Horizontal
        } else if (aspectRatio > 1.0) {
            return 45;   // Diagonal
        } else {
            return 90;   // Vertical
        }
    }

    private double calculateDistance(List<Double> point1, List<Double> point2) {
        double dx = point1.get(0) - point2.get(0);
        double dy = point1.get(1) - point2.get(1);
        return Math.sqrt(dx * dx + dy * dy);
    }

    private boolean isValidColor(Color color, DetectionMode mode) {
        switch (mode) {
            case RED_ONLY:
                return color == Color.RED;
            case BLUE_ONLY:
                return color == Color.BLUE;
            case YELLOW_ONLY:
                return color == Color.YELLOW;
            case RED_AND_YELLOW:
                return color == Color.RED || color == Color.YELLOW;
            case BLUE_AND_YELLOW:
                return color == Color.BLUE || color == Color.YELLOW;
            default:
                return false;
        }
    }

    private Location getBest(List<Location> locations, DetectionMode mode) {
        if (locations.isEmpty()) {
            return null; // Return null instead of dummy location
        }

        // Filter locations based on detection mode and acceptable ranges
        List<Location> validLocations = new ArrayList<>();
        for (Location current : locations) {
            if (!isValidColor(current.color, mode)) {
                continue;
            }

            // Filter out detections outside acceptable ranges
            if (current.translation > 10 || current.translation < -10) {
                continue;
            }
            if (current.extension > 25 || current.extension < 5) {
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
        validLocations.sort((a, b) -> Double.compare(b.score, a.score));
        java.util.Random random = new java.util.Random();
        return validLocations.get(random.nextInt(validLocations.size()));
    }

    public void setPipeline(int pipeline) {
        if (limelight != null) {
            limelight.pipelineSwitch(pipeline);
        }
    }

    private void displayTelemetry(Location best, int totalDetections) {
        telemetry.addData("Status", "Running");
        telemetry.addData("Detection Mode", detectionMode.toString());
        telemetry.addData("Total Detections", totalDetections);

        // Show controls
        telemetry.addLine("Controls:");
        telemetry.addLine("A=Red, B=Blue, Y=Yellow, X=Red+Yellow, RB=Blue+Yellow");

        // Debug: Show all detected colors
        if (locations != null && !locations.isEmpty()) {
            telemetry.addLine("=== ALL DETECTIONS ===");
            for (int i = 0; i < Math.min(locations.size(), 3); i++) {
                Location loc = locations.get(i);
                telemetry.addData("Detection " + (i+1),
                        String.format("%s: X=%.1f Y=%.1f", loc.color, loc.translation, loc.extension));
            }
        }

        if (best != null && totalDetections > 0 &&
                !(best.translation == 0 && best.extension == 0 && best.rotation == 0)) {
            telemetry.addLine("=== BEST TARGET ===");
            telemetry.addData("X Position", String.format("%.2f", best.translation));
            telemetry.addData("Y Position", String.format("%.2f", best.extension));
            telemetry.addData("Orientation", String.format("%.1f°", best.orientationAngle));
            telemetry.addData("Color", best.color.toString());
            telemetry.addData("Raw X", String.format("%.2f", best.rawTranslation));
            telemetry.addData("Raw Y", String.format("%.2f", best.rawExtension));
            telemetry.addData("Rotation Score", String.format("%.2f", best.rotation));
        } else {
            telemetry.addLine("No valid targets found for " + detectionMode);
        }

        telemetry.update();
    }
}