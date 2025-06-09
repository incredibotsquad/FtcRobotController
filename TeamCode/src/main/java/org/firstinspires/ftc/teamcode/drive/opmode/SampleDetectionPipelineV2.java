package org.firstinspires.ftc.teamcode.drive.opmode;

import org.firstinspires.ftc.teamcode.GameConstants;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.photo.Photo;
import org.openftc.easyopencv.OpenCvPipeline;
//import org.w3c.dom.css.Rect;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.opencv.android.Utils;
import android.graphics.Bitmap;
//import apple.laf.JRSUIConstants.Size;
//import javafx.scene.effect.Light.Point;

// TensorFlow Lite imports
//import org.tensorflow.lite.Interpreter;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;

@Config
public class SampleDetectionPipelineV2 extends OpenCvPipeline {
    private GameConstants.GAME_COLORS colorMode = GameConstants.GAME_COLORS.RED;
    public static GameConstants.GAME_COLORS dashboardColorMode = GameConstants.GAME_COLORS.RED; // Dashboard-controlled
                                                                                                // color mode
    public RotatedRect[] latestRects;
    public double[][] latestDistances;

    // Reusable Mat objects to avoid repeated allocation/deallocation
    private Mat hsv = new Mat();
    private Mat mask = new Mat();
    private Mat hierarchy = new Mat();
    private Mat resizedMat = new Mat(); // For bitmap conversion
    public double REAL_SAMPLE_LENGTH = 3.5;
    public double REAL_SAMPLE_WIDTH = 1.5;
    public double REAL_SAMPLE_HEIGHT = 1.5;
    public static double FOCAL_LEGNTH_X = 400;
    public static double FOCAL_LEGNTH_Y = 600;
    public static double HEIGHT_OF_CAMERA = 10; // Inches
    public static double CENTER_OF_IMAGE = 12; // Inches
    public static double minContourArea = 3000;
    public static double maxContourArea = 8000;
    public static double minAspectRatio = 0.3;
    public static double maxAspectRatio = 0.9;
    public static boolean enableHSVEqualization = false;
    public static boolean useCLAHE = true; // Use CLAHE instead of standard histogram equalization
    public static double claheClipLimit = 4.0; // Default clip limit for CLAHE
    public static int claheTileSize = 8; // Default tile size for CLAHE (8x8)
    public static double BLOB_ASPECT_RATIO = 0.25;
    public static double BLOB_AREA_THRESHOLD = 8000;

    // Blob fragmentation algorithm controls
    public static boolean enableWatershedFragmentation = false;
    public static boolean enableDistanceTransformFragmentation = false;
    public static boolean enableAdaptiveThresholdFragmentation = false;
    public static boolean enableMorphologicalGradientFragmentation = false;
    public static boolean enableContourSplittingFragmentation = false;

    // Camera control settings
    public static boolean enableManualExposure = false;
    public static boolean enableManualWhiteBalance = false;
    public static int manualExposureValue = 50; // Default exposure value (0-100)
    public static int manualWhiteBalanceValue = 50; // Default white balance value (0-100)

    // OpenCV camera instance
    private org.openftc.easyopencv.OpenCvCamera camera;

    private static final int MODEL_INPUT_SIZE = 3; // x, y, orientation
    private static final int MODEL_OUTPUT_SIZE = 3; // adjusted_x, adjusted_y, adjusted_orientation

    // HSV Thresholds for color detection
    // RED (2 ranges)
    public static Scalar RED_LOW_1 = new Scalar(0, 100, 100);
    public static Scalar RED_HIGH_1 = new Scalar(10, 255, 255);

    public static Scalar RED_LOW_2 = new Scalar(160, 100, 100);
    public static Scalar RED_HIGH_2 = new Scalar(180, 255, 255);

    // BLUE
    public static Scalar BLUE_LOW = new Scalar(100, 100, 100);
    public static Scalar BLUE_HIGH = new Scalar(130, 255, 255);

    // YELLOW
    public static Scalar YELLOW_LOW = new Scalar(20, 100, 100);
    public static Scalar YELLOW_HIGH = new Scalar(40, 255, 255);

    public static final double CAMERA_ANGLE = 40;

    private static final double CAMERA_FOV_HORIZONTAL = 78.0;
    private static final double CAMERA_FOV_VERTICAL = 43.0;
    private static final int CAMERA_RESOLUTION_WIDTH = 640;
    private static final int CAMERA_RESOLUTION_HEIGHT = 480;
    private static final double SAMPLE_WIDTH_INCHES = 1.5;

    // Tuning parameters — make these dashboard-tunable if you like
    public static double DISTANCE_THRESHOLD_RATIO = 0.6; // Percentage of max distance to use as threshold
    public static int MORPH_KERNEL_SIZE = 7; // must be odd (3,5,7,...)

    // Helper method to get morphology kernel
    private Mat getMorphKernel() {
        return Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE));
    }

    public void setColorMode(GameConstants.GAME_COLORS mode) {
        this.colorMode = mode;
        dashboardColorMode = mode; // Sync dashboard color mode with instance color mode
    }

    /**
     * Update the color mode from the dashboard value
     * This should be called at the beginning of processFrame
     */
    private void updateColorModeFromDashboard() {
        if (this.colorMode != dashboardColorMode) {
            this.colorMode = dashboardColorMode;
        }
    }

    /**
     * Initialize the camera with manual exposure and white balance settings if
     * enabled
     * 
     * @param camera The OpenCV camera instance
     */
    public void initializeCamera(org.openftc.easyopencv.OpenCvCamera camera) {
        this.camera = camera;
        updateCameraSettings();
    }

    /**
     * Update camera settings based on dashboard values
     * This can be called when dashboard values change
     */
    public void updateCameraSettings() {
        if (camera == null)
            return;
        //
        // try {
        // // For EasyOpenCV, we need to use the camera's gain and exposure controls
        // directly
        // // These methods may vary depending on the camera and EasyOpenCV version
        // if (enableManualExposure) {
        // // Calculate exposure time in milliseconds
        // int exposureMs;
        // if (manualExposureValue < 50) {
        // // For 0-49: Linear scale from 1ms to 20ms (finer control in normal lighting)
        // exposureMs = 1 + (int)(manualExposureValue * 0.4); // 0->1ms, 49->20ms
        // } else {
        // // For 50-100: Exponential scale from 20ms to 100ms (for low light)
        // exposureMs = 20 + (int)(Math.pow((manualExposureValue - 50) / 50.0, 2) * 80);
        // }
        //
        // // Try to set exposure using the available methods
        // try {
        // // Method 1: Try using the camera's exposure property directly
        // camera.setPipeline(null); // Temporarily remove pipeline
        // camera.setExposure(exposureMs); // Set exposure in milliseconds
        // camera.setPipeline(this); // Re-add pipeline
        // } catch (Exception e1) {
        // System.out.println("Error setting exposure directly: " + e1.getMessage());
        //
        // // If direct method fails, try alternative approaches
        // try {
        // // Method 2: Try using camera controls if available
        // org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
        // exposureControl =
        // camera.getCameraControl(org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl.class);
        //
        // if (exposureControl != null) {
        // exposureControl.setMode(org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl.Mode.Manual);
        // exposureControl.setExposure(exposureMs,
        // java.util.concurrent.TimeUnit.MILLISECONDS);
        // }
        // } catch (Exception e2) {
        // System.out.println("Error setting exposure via camera controls: " +
        // e2.getMessage());
        // }
        // }
        // }
        //
        // if (enableManualWhiteBalance) {
        // // Calculate white balance value (typically in Kelvin)
        // int whiteBalanceValue = 2500 + (int)(manualWhiteBalanceValue * 65);
        //
        // // Try to set white balance using available methods
        // try {
        // // Method 1: Try using the camera's white balance property directly
        // camera.setPipeline(null); // Temporarily remove pipeline
        // camera.setWhiteBalance(whiteBalanceValue); // Set white balance
        // camera.setPipeline(this); // Re-add pipeline
        // } catch (Exception e1) {
        // System.out.println("Error setting white balance directly: " +
        // e1.getMessage());
        //
        // // If direct method fails, try alternative approaches
        // try {
        // // Method 2: Try using camera controls if available
        // org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl
        // whiteBalanceControl =
        // camera.getCameraControl(org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl.class);
        //
        // if (whiteBalanceControl != null) {
        // whiteBalanceControl.setMode(org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl.Mode.MANUAL);
        // whiteBalanceControl.setWhiteBalance(whiteBalanceValue);
        // }
        // } catch (Exception e2) {
        // System.out.println("Error setting white balance via camera controls: " +
        // e2.getMessage());
        // }
        // }
        // }
        // } catch (Exception e) {
        // // Log error or handle exception
        // System.out.println("Error setting camera parameters: " + e.getMessage());
        // }
    }

    /**
     * Get the horizontal distance (X) from the robot to the sample in inches
     * 
     * @param index Index of the detected sample
     * @return X distance in inches, or -1 if index is invalid
     */
    public double GetRealXinches(int index) {
        if (latestDistances != null && index >= 0 && index < latestDistances.length) {
            return latestDistances[index][0];
        }
        return -1;
    }

    /**
     * Get the vertical/ground distance (Y) from the robot to the sample in inches
     * 
     * @param index Index of the detected sample
     * @return Y distance in inches, or -1 if index is invalid
     */
    public double GetRealYinches(int index) {
        if (latestDistances != null && index >= 0 && index < latestDistances.length) {
            return latestDistances[index][2];
        }
        return -1;
    }

    /**
     * Get the orientation of the sample in degrees
     * 
     * @param index Index of the detected sample
     * @return Orientation angle in degrees, or -1 if index is invalid
     */
    public double GetRealSampleOrientation(int index) {
        if (latestDistances != null && index >= 0 && index < latestDistances.length) {
            return latestDistances[index][1];
        }
        return -1;
    }

    /**
     * Convert a Mat to a Bitmap for visualization at original resolution
     * 
     * @param input Input Mat image
     * @return Bitmap representation of the input Mat at original resolution
     */
    private Bitmap matToBitmap(Mat input) {
        // Convert directly to Bitmap without resizing
        Bitmap bmp = Bitmap.createBitmap(input.cols(), input.rows(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(input, bmp);
        return bmp;
    }

    /**
     * Combine two Mat objects side by side into a single Mat
     * 
     * @param mat1 First Mat (left side)
     * @param mat2 Second Mat (right side)
     * @return Combined Mat
     */
    private Mat combineMats(Mat mat1, Mat mat2) {
        // Ensure both mats have the same height
        int height = Math.max(mat1.rows(), mat2.rows());
        Mat resizedMat1 = new Mat();
        Mat resizedMat2 = new Mat();

        // Resize mats to have the same height
        if (mat1.rows() != height) {
            double scale = (double) height / mat1.rows();
            Imgproc.resize(mat1, resizedMat1, new Size(mat1.cols() * scale, height));
        } else {
            resizedMat1 = mat1.clone();
        }

        if (mat2.rows() != height) {
            double scale = (double) height / mat2.rows();
            Imgproc.resize(mat2, resizedMat2, new Size(mat2.cols() * scale, height));
        } else {
            resizedMat2 = mat2.clone();
        }

        // Create a new mat to hold the combined image
        Mat combined = new Mat(height, resizedMat1.cols() + resizedMat2.cols(), resizedMat1.type());

        // Copy the two mats side by side
        resizedMat1.copyTo(combined.submat(0, height, 0, resizedMat1.cols()));

        // Convert mat2 to the same type as mat1 if needed
        if (resizedMat1.type() != resizedMat2.type()) {
            Mat convertedMat2 = new Mat();
            if (resizedMat1.channels() == 3 && resizedMat2.channels() == 1) {
                Imgproc.cvtColor(resizedMat2, convertedMat2, Imgproc.COLOR_GRAY2BGR);
            } else if (resizedMat1.channels() == 1 && resizedMat2.channels() == 3) {
                Imgproc.cvtColor(resizedMat2, convertedMat2, Imgproc.COLOR_BGR2GRAY);
            } else {
                convertedMat2 = resizedMat2.clone();
            }
            convertedMat2.copyTo(combined.submat(0, height, resizedMat1.cols(), combined.cols()));
            convertedMat2.release();
        } else {
            resizedMat2.copyTo(combined.submat(0, height, resizedMat1.cols(), combined.cols()));
        }

        // Release temporary mats
        resizedMat1.release();
        resizedMat2.release();

        return combined;
    }

    /**
     * Get the angle from the robot to the sample in degrees
     * 
     * @param index Index of the detected sample
     * @return Angle from robot in degrees, or -1 if index is invalid
     */
    public double GetSampleAngleFromRobot(int index) {
        if (latestDistances != null && index >= 0 && index < latestDistances.length) {
            return latestDistances[index][3];
        }
        return -1;
    }

    /**
     * Get the confidence value of the detected sample
     * 
     * @param index Index of the detected sample
     * @return Confidence value from 0 to 1, or -1 if index is invalid
     */
    public double GetSampleConfidence(int index) {
        if (latestDistances != null && index >= 0 && index < latestDistances.length) {
            return latestDistances[index][4];
        }
        return -1;
    }

    public static Scalar[] computeAdaptiveHSVThresholds(Mat inputFrame, Scalar baseMin, Scalar baseMax) {
        // // Convert to grayscale
        // Mat gray = new Mat();
        // Imgproc.cvtColor(inputFrame, gray, Imgproc.COLOR_BGR2GRAY);
        //
        // // Compute average brightness
        // Scalar avgScalar = Core.mean(gray);
        // double avgBrightness = avgScalar.val[0]; // since grayscale has one channel
        //
        // // Reference brightness
        // double refBrightness = 128.0; //128 is a mid-point for 0-255 range
        // double brightnessRatio = avgBrightness / refBrightness;
        //
        // // Clamp scale factor between 0.5 and 1.5
        // double scaleFactor = 1.0 / Math.max(0.5, Math.min(1.5, brightnessRatio));
        //
        // // Create adaptiveMin based on baseMin
        // double hMin = baseMin.val[0];
        // double sMin = baseMin.val[1] * scaleFactor;
        // double vMin = baseMin.val[2] * scaleFactor;
        //
        // // Clamp S and V to 0–255
        // sMin = Math.max(0, Math.min(255, sMin));
        // vMin = Math.max(0, Math.min(255, vMin));
        //
        // Scalar adaptiveMin = new Scalar(hMin, sMin, vMin);
        // Scalar adaptiveMax = baseMax.clone(); // No change to max in this version
        // return new Scalar[]{adaptiveMin, adaptiveMax};
        return new Scalar[] { baseMin, baseMax };
    }

    private void processDetectedRect(
            RotatedRect rect,
            Mat input,
            ArrayList<RotatedRect> detectedRects,
            ArrayList<double[]> distances,
            double contourArea,
            double confidence) {
        detectedRects.add(rect);

        // Draw rectangle
        org.opencv.core.Point[] vertices = new org.opencv.core.Point[4];

        rect.points(vertices);
        for (int j = 0; j < 4; j++) {
            Imgproc.line(input, vertices[j], vertices[(j + 1) % 4], new Scalar(0, 255, 0), 2);
            Imgproc.putText(
                    input,
                    String.format("A:%.0f", contourArea),
                    vertices[j],
                    Imgproc.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    new Scalar(255, 0, 0),
                    2);
        }

        // Distance and angle calculation
        double distance3D = (REAL_SAMPLE_LENGTH * FOCAL_LEGNTH_X) / Math.max(rect.size.width, rect.size.height);
        double img_center_x = CAMERA_RESOLUTION_WIDTH / 2.0;
        double verticalFovRad = Math.toRadians(CAMERA_FOV_VERTICAL);
        double pixelsPerRad = CAMERA_RESOLUTION_HEIGHT / verticalFovRad;
        double img_center_y = CAMERA_RESOLUTION_HEIGHT / 2.0;

        double pixel_from_center_x = rect.center.x - img_center_x;
        double pixel_from_center_y = rect.center.y - img_center_y;
        double angle_horizontal = Math.atan2(pixel_from_center_x, FOCAL_LEGNTH_X);
        double x_inches_x = distance3D * Math.sin(angle_horizontal);
        double y_inches_x = distance3D * Math.cos(angle_horizontal);

        double verticalAngle = Math.atan2(pixel_from_center_y, FOCAL_LEGNTH_Y);
        double y_inches_y = HEIGHT_OF_CAMERA * Math.cos(verticalAngle);
        double angle_to_object = Math.toRadians(CAMERA_ANGLE) + (pixel_from_center_y / pixelsPerRad);
        double ground_distance = HEIGHT_OF_CAMERA / Math.tan(angle_to_object);

        double angle = rect.angle;
        if (rect.size.width < rect.size.height) {
            angle += 90;
        }

        double turretAngle = Math.toDegrees(Math.atan(x_inches_x / y_inches_x));

        // Use the adjusted values with the provided confidence
        distances.add(new double[] { x_inches_x, angle, ground_distance, turretAngle, confidence });
    }

    /**
     * Convert input image to HSV color space
     * 
     * @param input RGB input image
     * @param output HSV output image
     */
    private void convertToHSV(Mat input, Mat output) {
        Imgproc.cvtColor(input, output, Imgproc.COLOR_RGB2HSV);
    }

    /**
     * Equalize the V channel to normalize brightness
     * 
     * @param hsvImage HSV image to process
     */
    private void equalizeVChannel(Mat hsvImage) {
        if (enableHSVEqualization) {
            List<Mat> hsvChannels = new ArrayList<>();
            Core.split(hsvImage, hsvChannels);

            if (useCLAHE) {
                // Use CLAHE for better local contrast enhancement
                Mat vChannel = hsvChannels.get(2);
                Mat enhancedV = new Mat();

                // Create CLAHE object with specified clip limit and tile grid size
                Photo.createCLAHE(claheClipLimit, new Size(claheTileSize, claheTileSize))
                        .apply(vChannel, enhancedV);

                // Replace V channel with enhanced version
                enhancedV.copyTo(hsvChannels.get(2));
                enhancedV.release();
            } else {
                // Use standard histogram equalization
                Imgproc.equalizeHist(hsvChannels.get(2), hsvChannels.get(2));
            }

            Core.merge(hsvChannels, hsvImage);
        }
    }

    /**
     * Apply Gaussian blur to the image
     * 
     * @param input Input image
     * @param output Output blurred image
     */
    private void applyGaussianBlur(Mat input, Mat output) {
        // Currently not applying blur, just returning the input
        // Uncomment the following line to apply Gaussian blur
        // Imgproc.GaussianBlur(input, output, new Size(5, 5), 0);
    }

    /**
     * Apply color thresholding based on the current color mode
     * 
     * @param input Input HSV image
     * @param output Output binary mask
     */
    private void applyColorThreshold(Mat input, Mat output) {
        switch (colorMode) {
            case RED:
                Scalar[] redThresh1 = computeAdaptiveHSVThresholds(input, RED_LOW_1, RED_HIGH_1);
                Scalar[] redThresh2 = computeAdaptiveHSVThresholds(input, RED_LOW_2, RED_HIGH_2);

                Mat lowerRed = new Mat();
                Mat upperRed = new Mat();
                Core.inRange(input, redThresh1[0], redThresh1[1], lowerRed);
                Core.inRange(input, redThresh2[0], redThresh2[1], upperRed);
                Core.bitwise_or(lowerRed, upperRed, output);

                // release temporary Mats
                lowerRed.release();
                upperRed.release();
                break;

            case BLUE:
                Scalar[] blueThresh = computeAdaptiveHSVThresholds(input, BLUE_LOW, BLUE_HIGH);
                Core.inRange(input, blueThresh[0], blueThresh[1], output);
                break;

            case YELLOW:
                Scalar[] yellowThresh = computeAdaptiveHSVThresholds(input, YELLOW_LOW, YELLOW_HIGH);
                Core.inRange(input, yellowThresh[0], yellowThresh[1], output);
                break;
        }
    }

    /**
     * Apply morphological operations to the binary mask
     * 
     * @param input Input binary mask
     * @param output Output processed mask
     */
    private void applyMorphologicalOperations(Mat input, Mat output) {
        Imgproc.morphologyEx(input, output, Imgproc.MORPH_OPEN, getMorphKernel());
    }

    /**
     * Find contours in the binary mask
     * 
     * @param input Input binary mask
     * @param contours Output list to store found contours
     * @param hierarchyMat Output hierarchy information
     */
    private void findImageContours(Mat input, ArrayList<MatOfPoint> contours, Mat hierarchyMat) {
        Mat tempMat = input.clone();
        Imgproc.findContours(tempMat, contours, hierarchyMat, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        tempMat.release(); // Release the cloned Mat
    }

    /**
     * Calculate the confidence score of a blob being a sample
     * 
     * @param contour The contour to evaluate
     * @param mask Binary mask for ROI extraction
     * @param input Original input image
     * @return Confidence score between 0 and 1
     */
    private double isBlobSampleConfidence(MatOfPoint contour, Mat mask, Mat input) {
        // Weights for different metrics (should sum to 1.0)
        final double ASPECT_RATIO_WEIGHT = 0.2;
        final double AREA_WEIGHT = 0.15;
        final double SOLIDITY_WEIGHT = 0.15;
        final double CIRCULARITY_WEIGHT = 0.1;
        final double COLOR_CONSISTENCY_WEIGHT = 0.1;
        final double RECTANGULARITY_WEIGHT = 0.15;
        final double EDGE_STRAIGHTNESS_WEIGHT = 0.15;
        
        // Calculate metrics
        double confidence = 0.0;
        
        // 1. Aspect Ratio Score
        RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
        double aspectRatio = Math.min(rect.size.width, rect.size.height) / Math.max(rect.size.width, rect.size.height);
        
        // Score is highest when aspectRatio is close to expected value (e.g., 0.5 for a typical sample)
        // Using a Gaussian-like function centered at the ideal aspect ratio
        double idealAspectRatio = 0.43; // Adjust based on your samples
        double aspectRatioScore = Math.exp(-Math.pow(aspectRatio - idealAspectRatio, 2) / 0.1);
        
        // 2. Area Score
        double area = Imgproc.contourArea(contour);
        double idealArea = (minContourArea + maxContourArea) / 2.0;
        double areaTolerance = (maxContourArea - minContourArea) / 2.0;
        
        // Score is highest when area is close to ideal area
        double areaScore = 1.0 - Math.min(1.0, Math.abs(area - idealArea) / areaTolerance);
        
        // 3. Solidity Score (Area / Convex Hull Area)
        MatOfInt hullIndices = new MatOfInt();
        Imgproc.convexHull(contour, hullIndices);
        
        // Convert hull indices to points
        Point[] contourPoints = contour.toArray();
        List<Point> hullPoints = new ArrayList<>();
        for (int index : hullIndices.toArray()) {
            hullPoints.add(contourPoints[index]);
        }
        
        MatOfPoint hull = new MatOfPoint();
        hull.fromList(hullPoints);
        
        double hullArea = Imgproc.contourArea(hull);
        double solidity = (hullArea > 0) ? area / hullArea : 0;
        
        // Score is highest when solidity is close to expected value (e.g., 0.85 for a typical sample)
        double idealSolidity = 0.85; // Adjust based on your samples
        double solidityScore = Math.exp(-Math.pow(solidity - idealSolidity, 2) / 0.1);
        
        // 4. Circularity Score (4π × area / perimeter²)
        double perimeter = Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true);
        double circularity = (perimeter > 0) ? 4 * Math.PI * area / (perimeter * perimeter) : 0;
        
        // Score is highest when circularity is close to expected value
        double idealCircularity = 0.7; // Adjust based on your samples
        double circularityScore = Math.exp(-Math.pow(circularity - idealCircularity, 2) / 0.1);
        
        // 5. Color Consistency Score
        // Create a mask for this contour
        Rect boundingBox = Imgproc.boundingRect(contour);
        Mat contourMask = Mat.zeros(mask.size(), CvType.CV_8UC1);
        Imgproc.drawContours(contourMask, Arrays.asList(contour), 0, new Scalar(255), -1);
        
        // Extract the region from the original image
        Mat roi = new Mat();
        input.copyTo(roi, contourMask);
        
        // Calculate color standard deviation as a measure of consistency
        Mat mean = new Mat();
        Mat stdDev = new Mat();
        Core.meanStdDev(roi, mean, stdDev, contourMask);
        
        // Average the standard deviations of all channels
        double avgStdDev = 0;
        for (int i = 0; i < stdDev.rows(); i++) {
            avgStdDev += stdDev.get(i, 0)[0];
        }
        avgStdDev /= stdDev.rows();
        
        // Score is highest when standard deviation is low (consistent color)
        double colorConsistencyScore = Math.exp(-avgStdDev / 50.0); // Scale factor may need adjustment
        
        // Clean up
        contourMask.release();
        roi.release();
        mean.release();
        stdDev.release();
        hull.release();
        hullIndices.release();
        
        // 6. Rectangularity Score (contourArea / boxArea)
        Rect boundingRect = Imgproc.boundingRect(contour);
        double boxArea = boundingRect.width * boundingRect.height;
        double rectangularity = (boxArea > 0) ? area / boxArea : 0;
        
        // Score is highest when rectangularity is close to 1 (more rectangular)
        // L-shapes and U-shapes will have lower rectangularity
        double rectangularityScore = Math.min(1.0, rectangularity);
        
        // 7. Edge Straightness Score
        double edgeStraightnessScore = 0.0;
        try {
            // Convert contour to MatOfPoint2f for fitLine
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            
            // Fit a line to the contour
            Mat lineParams = new Mat();
            Imgproc.fitLine(contour2f, lineParams, Imgproc.DIST_L2, 0, 0.01, 0.01);
            
            // Extract line parameters (vx, vy, x0, y0)
            float vx = (float) lineParams.get(0, 0)[0];
            float vy = (float) lineParams.get(1, 0)[0];
            float x0 = (float) lineParams.get(2, 0)[0];
            float y0 = (float) lineParams.get(3, 0)[0];
            
            // Calculate RMSE (Root Mean Square Error) of points to the fitted line
            double sumSquaredError = 0.0;
            Point[] points = contour.toArray();
            for (Point point : points) {
                // Calculate distance from point to line
                // Line equation: (x-x0)*vy - (y-y0)*vx = 0
                double distance = Math.abs((point.x - x0) * vy - (point.y - y0) * vx) / 
                                  Math.sqrt(vx * vx + vy * vy);
                sumSquaredError += distance * distance;
            }
            
            double rmse = Math.sqrt(sumSquaredError / points.length);
            
            // Normalize RMSE to a score between 0 and 1
            // Lower RMSE means straighter edges, so we want a higher score
            double maxRmse = 10.0; // Adjust based on your expected range
            edgeStraightnessScore = 1.0 - Math.min(1.0, rmse / maxRmse);
            
            // Release resources
            contour2f.release();
            lineParams.release();
        } catch (Exception e) {
            System.out.println("Error calculating edge straightness: " + e.getMessage());
            edgeStraightnessScore = 0.5; // Default value if calculation fails
        }
        
        // Calculate weighted average
        confidence = aspectRatioScore * ASPECT_RATIO_WEIGHT +
                     areaScore * AREA_WEIGHT +
                     solidityScore * SOLIDITY_WEIGHT +
                     circularityScore * CIRCULARITY_WEIGHT +
                     colorConsistencyScore * COLOR_CONSISTENCY_WEIGHT +
                     rectangularityScore * RECTANGULARITY_WEIGHT +
                     edgeStraightnessScore * EDGE_STRAIGHTNESS_WEIGHT;
        
        // Ensure confidence is between 0 and 1
        confidence = Math.max(0.0, Math.min(1.0, confidence));
        
        return confidence;
    }

    /**
     * Process contours to detect rectangles and calculate distances
     * 
     * @param contours List of contours to process
     * @param mask Binary mask for ROI extraction
     * @param input Original input image for visualization
     * @param detectedRects Output list to store detected rectangles
     * @param distances Output list to store calculated distances
     */
    private void processContours(ArrayList<MatOfPoint> contours, Mat mask, Mat input, 
                                ArrayList<RotatedRect> detectedRects, ArrayList<double[]> distances) {
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);

            if (area < minContourArea)
                continue; // Ignore small contours

            RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
            double aspectRatio = Math.min(rect.size.width, rect.size.height)
                    / Math.max(rect.size.width, rect.size.height);

            if (area >= minContourArea && area <= maxContourArea) {
                // Calculate confidence using the new function
                double confidence = isBlobSampleConfidence(contour, mask, input);
                // Process with calculated confidence
                processDetectedRect(rect, input, detectedRects, distances, area, confidence);
            } else if (aspectRatio > BLOB_ASPECT_RATIO && area > BLOB_AREA_THRESHOLD) { // check if they are worth trying
                processLargeBlob(contour, mask, input, detectedRects, distances);
            }
        }
    }

    /**
     * Process a large blob that might contain multiple objects
     * 
     * @param contour The contour of the large blob
     * @param mask Binary mask for ROI extraction
     * @param input Original input image for visualization
     * @param detectedRects Output list to store detected rectangles
     * @param distances Output list to store calculated distances
     */
    private void processLargeBlob(MatOfPoint contour, Mat mask, Mat input, 
                                ArrayList<RotatedRect> detectedRects, ArrayList<double[]> distances) {
        Rect boundingBox = Imgproc.boundingRect(contour);
        Mat roi = new Mat(mask, boundingBox);

        // Call the blob fragmentation function
        List<MatOfPoint> innerContours = fragmentBlob(contour, roi, mask, boundingBox, input);

        if (innerContours.isEmpty()) {
            // If no fragmentation was successful or enabled, use the original approach
            innerContours = new ArrayList<>();
            Mat hierarchyInner = new Mat();
            Mat tempRoi = roi.clone();
            Imgproc.findContours(tempRoi, innerContours, hierarchyInner, Imgproc.RETR_EXTERNAL,
                    Imgproc.CHAIN_APPROX_SIMPLE);
            hierarchyInner.release();
            tempRoi.release();
        }
        
        // Process inner contours before releasing roi
        List<MatOfPoint> processedContours = new ArrayList<>();
        
        // Filter inner contours based on area and aspect ratio
        for (MatOfPoint innerContour : innerContours) {
            double subArea = Imgproc.contourArea(innerContour);
            if (subArea < minContourArea || subArea > maxContourArea) {
                Imgproc.putText(
                        input,
                        String.format("skipped - Area"),
                        new Point(boundingBox.x, boundingBox.y),
                        Imgproc.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        new Scalar(255, 0, 0),
                        2);
                continue;
            }

            RotatedRect subRect = Imgproc.minAreaRect(new MatOfPoint2f(innerContour.toArray()));
            double subAspectRatio = Math.min(subRect.size.width, subRect.size.height)
                    / Math.max(subRect.size.width, subRect.size.height);
            if (subAspectRatio < minAspectRatio || subAspectRatio > maxAspectRatio) {
                Imgproc.putText(
                        input,
                        "skipped - Aspect Ratio",
                        new Point(boundingBox.x, boundingBox.y),
                        Imgproc.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        new Scalar(0, 0, 255),
                        2);
                continue; // Ignore contours with aspect ratio outside the range
            }

            // Debug visualization to check if the rectangle is correctly positioned
            Imgproc.rectangle(
                    input,
                    new Point(boundingBox.x, boundingBox.y),
                    new Point(boundingBox.x + boundingBox.width, boundingBox.y + boundingBox.height),
                    new Scalar(255, 255, 255),
                    1);

            // The inner contours are found in the ROI coordinate system, which is relative
            // to the bounding box
            // We need to adjust the center coordinates to the original image coordinate system
            subRect.center.x += boundingBox.x;
            subRect.center.y += boundingBox.y;

            // Now process the adjusted rectangle with lower confidence
            processDetectedRect(subRect, input, detectedRects, distances, subArea, 0.8);
            
            // Add to processed contours
            processedContours.add(innerContour);
        }
        
        // Release resources
        roi.release();
        
        // Release any contours that weren't processed
        for (MatOfPoint innerContour : innerContours) {
            if (!processedContours.contains(innerContour)) {
                innerContour.release();
            }
        }
        
    }

    /**
     * Fragment a large blob into smaller contours using multiple algorithms
     * 
     * @param contour The original contour to fragment
     * @param roi Region of interest containing the blob
     * @param mask Full binary mask
     * @param boundingBox Bounding box of the contour
     * @param input Original input image for visualization
     * @return List of fragmented contours
     */
    private List<MatOfPoint> fragmentBlob(MatOfPoint contour, Mat roi, Mat mask, Rect boundingBox, Mat input) {
        List<MatOfPoint> resultContours = new ArrayList<>();
        
        // Apply each fragmentation algorithm if enabled
        if (enableWatershedFragmentation) {
            List<MatOfPoint> watershedContours = applyWatershedFragmentation(contour, roi, mask, boundingBox, input, false);
            resultContours.addAll(watershedContours);
        }
        
        if (enableDistanceTransformFragmentation) {
            List<MatOfPoint> distanceTransformContours = applyDistanceTransformFragmentation(contour, roi, mask, boundingBox, input, false);
            resultContours.addAll(distanceTransformContours);
        }
        
        if (enableAdaptiveThresholdFragmentation) {
            List<MatOfPoint> adaptiveThresholdContours = applyAdaptiveThresholdFragmentation(contour, roi, mask, boundingBox, input, false);
            resultContours.addAll(adaptiveThresholdContours);
        }
        
        if (enableMorphologicalGradientFragmentation) {
            List<MatOfPoint> morphologicalGradientContours = applyMorphologicalGradientFragmentation(contour, roi, mask, boundingBox, input, false);
            resultContours.addAll(morphologicalGradientContours);
        }
        
        if (enableContourSplittingFragmentation) {
            List<MatOfPoint> contourSplittingContours = applyContourSplittingFragmentation(contour, roi, mask, boundingBox, input, false);
            resultContours.addAll(contourSplittingContours);
        }
        
        return resultContours;
    }

    /**
     * Apply watershed algorithm to fragment a blob
     * 
     * @param contour The original contour to fragment
     * @param roi Region of interest containing the blob
     * @param mask Full binary mask
     * @param boundingBox Bounding box of the contour
     * @param input Original input image for visualization
     * @param keepOriginal Whether to keep the original contour in the output list
     * @return List of fragmented contours
     */
    private List<MatOfPoint> applyWatershedFragmentation(MatOfPoint contour, Mat roi, Mat mask, Rect boundingBox, Mat input, boolean keepOriginal) {
        List<MatOfPoint> resultContours = new ArrayList<>();
        
        // Add the original contour if requested
        if (keepOriginal) {
            resultContours.add(contour);
        }
        
        try {
            // Create a markers image
            Mat markers = new Mat(roi.size(), CvType.CV_32SC1, new Scalar(0));
            
            // Background markers
            Rect innerRect = new Rect(
                new Point(roi.cols() * 0.05, roi.rows() * 0.05),
                new Point(roi.cols() * 0.95, roi.rows() * 0.95)
            );
            
            // Draw background (outer) marker
            Imgproc.rectangle(
                markers,
                new Point(0, 0),
                new Point(roi.cols(), roi.rows()),
                new Scalar(1),
                -1
            );
            
            // Draw foreground markers
            Mat distanceTransform = new Mat();
            Imgproc.distanceTransform(roi, distanceTransform, Imgproc.DIST_L2, 3);
            
            // Threshold to find peaks (potential object centers)
            Mat peaks = new Mat();
            Core.normalize(distanceTransform, distanceTransform, 0, 1.0, Core.NORM_MINMAX);
            Imgproc.threshold(distanceTransform, peaks, 0.7, 1.0, Imgproc.THRESH_BINARY);
            
            // Convert to 8-bit for findContours
            Mat peaksU8 = new Mat();
            peaks.convertTo(peaksU8, CvType.CV_8U, 255);
            
            // Find contours of peaks
            List<MatOfPoint> peakContours = new ArrayList<>();
            Mat peakHierarchy = new Mat();
            Imgproc.findContours(peaksU8, peakContours, peakHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            
            // Mark each peak with a unique label
            for (int i = 0; i < peakContours.size(); i++) {
                Moments moments = Imgproc.moments(peakContours.get(i));
                int cx = (int) (moments.get_m10() / moments.get_m00());
                int cy = (int) (moments.get_m01() / moments.get_m00());
                
                // Mark with a unique label (i+2) - 0 is background, 1 is boundary
                Imgproc.circle(markers, new Point(cx, cy), 2, new Scalar(i + 2), -1);
            }
            
            // Apply watershed
            Mat roiRGB = new Mat();
            Imgproc.cvtColor(roi, roiRGB, Imgproc.COLOR_GRAY2BGR);
            Imgproc.watershed(roiRGB, markers);
            
            // Extract each segment
            for (int i = 2; i < peakContours.size() + 2; i++) {
                Mat segment = new Mat(roi.size(), CvType.CV_8UC1, new Scalar(0));
                Core.compare(markers, new Scalar(i), segment, Core.CMP_EQ);
                
                // Convert to 8-bit for findContours
                segment.convertTo(segment, CvType.CV_8U, 255);
                
                // Find contours of the segment
                List<MatOfPoint> segmentContours = new ArrayList<>();
                Mat segmentHierarchy = new Mat();
                Imgproc.findContours(segment, segmentContours, segmentHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
                
                // Add to result
                resultContours.addAll(segmentContours);
                
                // Release resources
                segment.release();
            }
            
            // Release resources
            markers.release();
            distanceTransform.release();
            peaks.release();
            peaksU8.release();
            peakHierarchy.release();
            roiRGB.release();
            
        } catch (Exception e) {
            System.out.println("Error in watershed fragmentation: " + e.getMessage());
        }
        
        return resultContours;
    }

    /**
     * Apply distance transform algorithm to fragment a blob
     * 
     * @param contour The original contour to fragment
     * @param roi Region of interest containing the blob
     * @param mask Full binary mask
     * @param boundingBox Bounding box of the contour
     * @param input Original input image for visualization
     * @param keepOriginal Whether to keep the original contour in the output list
     * @return List of fragmented contours
     */
    private List<MatOfPoint> applyDistanceTransformFragmentation(MatOfPoint contour, Mat roi, Mat mask, Rect boundingBox, Mat input, boolean keepOriginal) {
        List<MatOfPoint> resultContours = new ArrayList<>();
        
        // Add the original contour if requested
        if (keepOriginal) {
            resultContours.add(contour);
        }
        
        try {
            // Apply distance transform
            Mat distanceTransform = new Mat();
            Imgproc.distanceTransform(roi, distanceTransform, Imgproc.DIST_L2, 5);
            
            // Normalize and threshold to find peaks
            Core.normalize(distanceTransform, distanceTransform, 0, 1.0, Core.NORM_MINMAX);
            
            // Apply adaptive threshold to find peaks
            Mat thresholded = new Mat();
            Core.MinMaxLocResult mm = Core.minMaxLoc(distanceTransform);
            double thresh = mm.maxVal * DISTANCE_THRESHOLD_RATIO;
            Imgproc.threshold(distanceTransform, thresholded, thresh, 1.0, Imgproc.THRESH_BINARY);
            
            // Convert to 8-bit for findContours
            Mat thresholdedU8 = new Mat();
            thresholded.convertTo(thresholdedU8, CvType.CV_8U, 255);
            
            // Find contours of the thresholded image
            List<MatOfPoint> dtContours = new ArrayList<>();
            Mat dtHierarchy = new Mat();
            Imgproc.findContours(thresholdedU8, dtContours, dtHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            
            // Add to result
            resultContours.addAll(dtContours);
            
            // Release resources
            distanceTransform.release();
            thresholded.release();
            thresholdedU8.release();
            dtHierarchy.release();
            
        } catch (Exception e) {
            System.out.println("Error in distance transform fragmentation: " + e.getMessage());
        }
        
        return resultContours;
    }

    /**
     * Apply adaptive threshold algorithm to fragment a blob
     * 
     * @param contour The original contour to fragment
     * @param roi Region of interest containing the blob
     * @param mask Full binary mask
     * @param boundingBox Bounding box of the contour
     * @param input Original input image for visualization
     * @param keepOriginal Whether to keep the original contour in the output list
     * @return List of fragmented contours
     */
    private List<MatOfPoint> applyAdaptiveThresholdFragmentation(MatOfPoint contour, Mat roi, Mat mask, Rect boundingBox, Mat input, boolean keepOriginal) {
        List<MatOfPoint> resultContours = new ArrayList<>();
        
        // Add the original contour if requested
        if (keepOriginal) {
            resultContours.add(contour);
        }
        
        try {
            // Apply Gaussian blur to smooth the image
            Mat blurred = new Mat();
            Imgproc.GaussianBlur(roi, blurred, new Size(5, 5), 0);
            
            // Apply adaptive threshold
            Mat adaptiveThresh = new Mat();
            Imgproc.adaptiveThreshold(
                blurred,
                adaptiveThresh,
                255,
                Imgproc.ADAPTIVE_THRESH_GAUSSIAN_C,
                Imgproc.THRESH_BINARY,
                11,
                2
            );
            
            // Find contours of the adaptive threshold result
            List<MatOfPoint> atContours = new ArrayList<>();
            Mat atHierarchy = new Mat();
            Imgproc.findContours(adaptiveThresh, atContours, atHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            
            // Add to result
            resultContours.addAll(atContours);
            
            // Release resources
            blurred.release();
            adaptiveThresh.release();
            atHierarchy.release();
            
        } catch (Exception e) {
            System.out.println("Error in adaptive threshold fragmentation: " + e.getMessage());
        }
        
        return resultContours;
    }

    /**
     * Apply morphological gradient algorithm to fragment a blob
     * 
     * @param contour The original contour to fragment
     * @param roi Region of interest containing the blob
     * @param mask Full binary mask
     * @param boundingBox Bounding box of the contour
     * @param input Original input image for visualization
     * @param keepOriginal Whether to keep the original contour in the output list
     * @return List of fragmented contours
     */
    private List<MatOfPoint> applyMorphologicalGradientFragmentation(MatOfPoint contour, Mat roi, Mat mask, Rect boundingBox, Mat input, boolean keepOriginal) {
        List<MatOfPoint> resultContours = new ArrayList<>();
        
        // Add the original contour if requested
        if (keepOriginal) {
            resultContours.add(contour);
        }
        
        try {
            // Apply morphological gradient
            Mat gradient = new Mat();
            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
            Imgproc.morphologyEx(roi, gradient, Imgproc.MORPH_GRADIENT, kernel);
            
            // Threshold the gradient
            Mat thresholded = new Mat();
            Imgproc.threshold(gradient, thresholded, 50, 255, Imgproc.THRESH_BINARY);
            
            // Invert the gradient to get the regions
            Mat inverted = new Mat();
            Core.bitwise_not(thresholded, inverted);
            
            // Find contours of the inverted gradient
            List<MatOfPoint> mgContours = new ArrayList<>();
            Mat mgHierarchy = new Mat();
            Imgproc.findContours(inverted, mgContours, mgHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            
            // Add to result
            resultContours.addAll(mgContours);
            
            // Release resources
            gradient.release();
            kernel.release();
            thresholded.release();
            inverted.release();
            mgHierarchy.release();
            
        } catch (Exception e) {
            System.out.println("Error in morphological gradient fragmentation: " + e.getMessage());
        }
        
        return resultContours;
    }

    /**
     * Apply contour splitting algorithm to fragment a blob
     * 
     * @param contour The original contour to fragment
     * @param roi Region of interest containing the blob
     * @param mask Full binary mask
     * @param boundingBox Bounding box of the contour
     * @param input Original input image for visualization
     * @param keepOriginal Whether to keep the original contour in the output list
     * @return List of fragmented contours
     */
    private List<MatOfPoint> applyContourSplittingFragmentation(MatOfPoint contour, Mat roi, Mat mask, Rect boundingBox, Mat input, boolean keepOriginal) {
        List<MatOfPoint> resultContours = new ArrayList<>();
        
        // Add the original contour if requested
        if (keepOriginal) {
            resultContours.add(contour);
        }
        
        try {
            // Convert contour to MatOfPoint2f for approxPolyDP
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            
            // Approximate the contour to simplify it
            MatOfPoint2f approxCurve = new MatOfPoint2f();
            double epsilon = 0.02 * Imgproc.arcLength(contour2f, true);
            Imgproc.approxPolyDP(contour2f, approxCurve, epsilon, true);
            
            // Convert back to MatOfPoint
            MatOfPoint approxContour = new MatOfPoint(approxCurve.toArray());
            
            // Find convexity defects
            MatOfInt hull = new MatOfInt();
            Imgproc.convexHull(approxContour, hull, false);
            
            // If we have enough points in the hull, try to split the contour
            if (hull.size().height >= 4) {
                // Create a mask for the contour
                Mat contourMask = Mat.zeros(roi.size(), CvType.CV_8UC1);
                Imgproc.drawContours(
                    contourMask,
                    Arrays.asList(approxContour),
                    0,
                    new Scalar(255),
                    -1
                );
                
                // Find potential split points
                List<Point> splitPoints = new ArrayList<>();
                Point[] points = approxContour.toArray();
                
                // Look for concave regions
                for (int i = 0; i < points.length; i++) {
                    int prev = (i - 1 + points.length) % points.length;
                    int next = (i + 1) % points.length;
                    
                    // Calculate vectors
                    Point v1 = new Point(points[prev].x - points[i].x, points[prev].y - points[i].y);
                    Point v2 = new Point(points[next].x - points[i].x, points[next].y - points[i].y);
                    
                    // Calculate cross product to determine concavity
                    double crossProduct = v1.x * v2.y - v1.y * v2.x;
                    
                    // If cross product is positive, the point is concave
                    if (crossProduct > 0) {
                        splitPoints.add(points[i]);
                    }
                }
                
                // If we have at least 2 split points, try to split the contour
                if (splitPoints.size() >= 2) {
                    // Find the two most distant split points
                    double maxDist = 0;
                    Point p1 = null, p2 = null;
                    
                    for (int i = 0; i < splitPoints.size(); i++) {
                        for (int j = i + 1; j < splitPoints.size(); j++) {
                            double dist = Math.sqrt(
                                Math.pow(splitPoints.get(i).x - splitPoints.get(j).x, 2) +
                                Math.pow(splitPoints.get(i).y - splitPoints.get(j).y, 2)
                            );
                            
                            if (dist > maxDist) {
                                maxDist = dist;
                                p1 = splitPoints.get(i);
                                p2 = splitPoints.get(j);
                            }
                        }
                    }
                    
                    // Draw a line to split the contour
                    if (p1 != null && p2 != null) {
                        Imgproc.line(
                            contourMask,
                            p1,
                            p2,
                            new Scalar(0),
                            2
                        );
                        
                        // Find contours of the split mask
                        List<MatOfPoint> splitContours = new ArrayList<>();
                        Mat splitHierarchy = new Mat();
                        Imgproc.findContours(
                            contourMask,
                            splitContours,
                            splitHierarchy,
                            Imgproc.RETR_EXTERNAL,
                            Imgproc.CHAIN_APPROX_SIMPLE
                        );
                        
                        // Add to result
                        resultContours.addAll(splitContours);
                        
                        // Release resources
                        splitHierarchy.release();
                    }
                }
                
                // Release resources
                contourMask.release();
            }
            
            // Release resources
            contour2f.release();
            approxCurve.release();
            hull.release();
            
        } catch (Exception e) {
            System.out.println("Error in contour splitting fragmentation: " + e.getMessage());
        }
        
        return resultContours;
    }
    
    /**
     * Process the input frame to detect samples
     * 
     * @param input Input frame from the camera
     * @return Processed frame with detected samples highlighted
     */
    @Override
    public Mat processFrame(Mat input) {
        // Update color mode from dashboard if needed
        updateColorModeFromDashboard();
        
        // Initialize result containers
        ArrayList<RotatedRect> detectedRects = new ArrayList<>();
        ArrayList<double[]> distances = new ArrayList<>();
        
        // Clear previous data but don't release the Mat objects
        // Add guards to ensure the Mat objects are not empty
        if (!hsv.empty()) hsv.setTo(new Scalar(0, 0, 0));
        if (!mask.empty()) mask.setTo(new Scalar(0));
        if (!hierarchy.empty()) hierarchy.setTo(new Scalar(0));
        
        // Convert to HSV color space
        convertToHSV(input, hsv);
        
        // Equalize V channel if enabled
        equalizeVChannel(hsv);
        
        // Apply color thresholding
        applyColorThreshold(hsv, mask);
        
        // Apply morphological operations
        applyMorphologicalOperations(mask, mask);
        
        // Find contours
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        findImageContours(mask, contours, hierarchy);
        
        // Process contours to detect rectangles and calculate distances
        processContours(contours, mask, input, detectedRects, distances);
        
        // Store results for external access
        if (!detectedRects.isEmpty()) {
            latestRects = detectedRects.toArray(new RotatedRect[0]);
            latestDistances = distances.toArray(new double[0][0]);
        } else {
            latestRects = new RotatedRect[0];
            latestDistances = new double[0][0];
        }
        
        // Release all contours
        for (MatOfPoint contour : contours) {
            contour.release();
        }
        
        return input;
    }
}
