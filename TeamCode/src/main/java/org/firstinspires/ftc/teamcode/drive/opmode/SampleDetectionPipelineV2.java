package org.firstinspires.ftc.teamcode.drive.opmode;

import org.firstinspires.ftc.teamcode.GameConstants;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.photo.Photo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.opencv.android.Utils;
import android.graphics.Bitmap;
import android.util.Log;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;

@Config
public class SampleDetectionPipelineV2 implements VisionProcessor {
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
    public static boolean useRectangleArea = false; // Flag to switch between contour area and rectangle area
    public static boolean enableHSVEqualization = true;
    public static boolean useCLAHE = true; // Use CLAHE instead of standard histogram equalization
    public static double claheClipLimit = 4.0; // Default clip limit for CLAHE
    public static int claheTileSize = 8; // Default tile size for CLAHE (8x8)
    public static double BLOB_ASPECT_RATIO = 0.25;
    public static double BLOB_AREA_THRESHOLD = 8000;
    

    // Blob fragmentation algorithm controls
    public static boolean enableDistanceTransformFragmentation = false;
    public static boolean enableAdaptiveThresholdFragmentation = false;
    public static boolean enableCannyEdgeFragmentation = false; // Added Canny edge detection
    
    // Morphological operation controls
    public static boolean enableDilationAfterOpening = false; // Optional dilation after opening
    public static int dilationKernelSize = 3; // Kernel size for dilation (must be odd)
    public static double MIN_AREA_RATIO = 0.15; // Minimum area ratio for valid sub-contours in distance transform
    public static double MIN_BRIGHTNESS_THRESHOLD = 50.0; // Minimum brightness threshold (0-255)

    // Weights for different metrics (should sum to 1.0 - BRIGHTNESS_WEIGHT)
    public static double ASPECT_RATIO_WEIGHT = 0.35;
    public static double AREA_WEIGHT = 0.25;
    public static double RECTANGULARITY_WEIGHT = 0.2;
    public static double BRIGHTNESS_WEIGHT = 0.2; // Weight for brightness in confidence calculation
    
    // Tuning mode for displaying HSV values of detected blobs
    public static boolean tuningMode = false; // Toggle this in the dashboard to enable tuning mode

    // Camera control settings
    public static boolean enableManualExposure = false;
    public static boolean enableManualWhiteBalance = false;
    public static int manualExposureValue = 50; // Default exposure value in milliseconds
    public static int manualWhiteBalanceValue = 50; // Default white balance value (0-100)
    public static int manualWhiteTemperatureValue = 4500; // Default white temperature value (typically 2800-6500K)
    public static int manualGainValue = 100; // Default gain value

    // VisionPortal instance
    private VisionPortal visionPortal;

    private static final int MODEL_INPUT_SIZE = 3; // x, y, orientation
    private static final int MODEL_OUTPUT_SIZE = 3; // adjusted_x, adjusted_y, adjusted_orientation

    // HSV Thresholds for color detection
    // RED (multiple ranges)
    public static Scalar[] RED_LOW = {
        new Scalar(0, 100, 100),    // RED_LOW_1
        new Scalar(160, 100, 100)   // RED_LOW_2
    };
    public static Scalar[] RED_HIGH = {
        new Scalar(10, 255, 255),   // RED_HIGH_1
        new Scalar(180, 255, 255)   // RED_HIGH_2
    };

    // BLUE (multiple ranges)
    public static Scalar[] BLUE_LOW = {
        new Scalar(100, 50, 100), // BLUE_LOW_1
        new Scalar(100, 100, 150) //Blue_LOW_2
            
    };
    public static Scalar[] BLUE_HIGH = {
        new Scalar(130, 255, 255),
        new Scalar(130,255,255)   // BLUE_HIGH_1
    };

    // YELLOW (multiple ranges)
    public static Scalar[] YELLOW_LOW = {
        new Scalar(20, 155, 175) ,
         new Scalar(20, 190, 180)   // YELLOW_LOW_1
    };
    public static Scalar[] YELLOW_HIGH = {
        new Scalar(40, 255, 255) ,
         new Scalar(40, 255, 255)   // YELLOW_HIGH_1
    };

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
     * Initialize the VisionPortal with this processor
     * 
     * @param webcamName The webcam to use
     */
    public void initializeVision(WebcamName webcamName) {
        // Create a VisionPortal.Builder and set this processor
        VisionPortal.Builder builder = new VisionPortal.Builder();
        
        // Configure the builder
        builder.setCamera(webcamName)
               .addProcessor(this)
               .enableLiveView(true)
               .setAutoStopLiveView(true);
        
        // Build the VisionPortal
        visionPortal = builder.build();
        
        // Update camera settings after initialization
        updateCameraSettings();
    }
    
    /**
     * Update camera settings including exposure, white balance, and white temperature
     * This function disables auto settings and applies manual values
     */
    public void updateCameraSettings() {
        if (visionPortal != null && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            try {
                // Set exposure
                if (enableManualExposure) {
                    // Get exposure control
                    ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
                    
                    // Check if we need to change the mode
                    if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                        exposureControl.setMode(ExposureControl.Mode.Manual);
                        sleep(50); // Small delay to let the setting take effect
                    }
                    
                    // Set manual exposure value
                    exposureControl.setExposure((long)manualExposureValue, TimeUnit.MILLISECONDS);
                    sleep(20); // Small delay to let the setting take effect
                }
                
                // Set gain
                GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
                
                // Check if we need to change the mode
                if (gainControl.getMode() != GainControl.Mode.Manual) {
                    gainControl.setMode(GainControl.Mode.Manual);
                    sleep(50); // Small delay to let the setting take effect
                }
                
                gainControl.setGain(manualGainValue);
                sleep(20); // Small delay to let the setting take effect
                
                // Set white balance
                if (enableManualWhiteBalance) {
                    // Get white balance control
                    WhiteBalanceControl whiteBalanceControl = visionPortal.getCameraControl(WhiteBalanceControl.class);
                    
                    // Check if we need to change the mode
                    if (whiteBalanceControl.getMode() != WhiteBalanceControl.Mode.Manual) {
                        whiteBalanceControl.setMode(WhiteBalanceControl.Mode.Manual);
                        sleep(50); // Small delay to let the setting take effect
                    }
                    
                    // Set manual white balance temperature
                    whiteBalanceControl.setWhiteBalanceTemperature(manualWhiteTemperatureValue);
                    sleep(20); // Small delay to let the setting take effect
                }
            } catch (Exception e) {
                // Log any errors that occur during camera settings update
                System.out.println("Error updating camera settings: " + e.getMessage());
            }
        }
    }
    
    /**
     * Sleep for the specified number of milliseconds
     * 
     * @param milliseconds Time to sleep in milliseconds
     */
    private void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
    
    /**
     * Stop and close the VisionPortal
     */
    public void stopVision() {
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
    }
    
    /**
     * Pause or resume vision processing
     * 
     * @param pause True to pause, false to resume
     */
    public void pauseVision(boolean pause) {
        if (visionPortal != null) {
            if (pause) {
                visionPortal.stopStreaming();
            } else {
                visionPortal.resumeStreaming();
            }
        }
    }
    
    /**
     * Get the current VisionPortal instance
     * 
     * @return The VisionPortal instance
     */
    public VisionPortal getVisionPortal() {
        return visionPortal;
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
     * Send a Mat image to the FTC Dashboard for visualization.
     * Converts the Mat to a Bitmap and sends it using FtcDashboard.
     *
     * @param toShow The Mat image to display on the dashboard.
     */
    public void showOnDashboard(Mat toShow) {
        if (toShow == null || toShow.empty()) return;
        Bitmap bmp = matToBitmap(toShow);
        FtcDashboard.getInstance().sendImage(bmp);
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
        //adjustment logic can be added here if needed
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
                    String.format("pxA:%.0f", contourArea),
                    vertices[j],
                    Imgproc.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    new Scalar(255, 0, 0),
                    2);
        }

         Imgproc.putText(
                    input,
                    String.format("A:%.2f", confidence),
                    rect.center,
                    Imgproc.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    new Scalar(0, 255, 0),
                    2);

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
                Imgproc.createCLAHE(claheClipLimit, new Size(claheTileSize, claheTileSize))
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
     * @param kernelSize Size of the blur kernel
     */
    private void applyGaussianBlur(Mat input, Mat output, Size kernelSize) {
        Imgproc.GaussianBlur(input, output, kernelSize, 0);
    }
    
    /**
     * Apply Gaussian blur to the image with default kernel size (3x3)
     * 
     * @param input Input image
     * @param output Output blurred image
     */
    private void applyGaussianBlur(Mat input, Mat output) {
        applyGaussianBlur(input, output, new Size(3, 3));
    }

    /**
     * Apply color thresholding based on the current color mode
     * 
     * @param input Input HSV image
     * @param output Output binary mask
     */
    private void applyColorThreshold(Mat input, Mat output) {
        // Initialize output to zeros
        output.setTo(new Scalar(0));
        
        // Temporary mask for each range
        Mat tempMask = new Mat();
        
        switch (colorMode) {
            case RED:
                applyMultipleThresholds(input, output, RED_LOW, RED_HIGH);
                break;

            case BLUE:
                applyMultipleThresholds(input, output, BLUE_LOW, BLUE_HIGH);
                break;

            case YELLOW:
                applyMultipleThresholds(input, output, YELLOW_LOW, YELLOW_HIGH);
                break;
        }
        
        // Release temporary mask
        tempMask.release();
    }
    
    /**
     * Apply multiple HSV thresholds and combine results with bitwise OR
     * 
     * @param input Input HSV image
     * @param output Output binary mask (will be modified)
     * @param lowThresholds Array of low threshold Scalars
     * @param highThresholds Array of high threshold Scalars
     */
    private void applyMultipleThresholds(Mat input, Mat output, Scalar[] lowThresholds, Scalar[] highThresholds) {
        // Ensure arrays have the same length
        if (lowThresholds.length != highThresholds.length) {
            Log.e("SampleDetectionPipeline", "Low and high threshold arrays must have the same length");
            return;
        }
        
        // Apply each threshold range and combine with bitwise OR
        for (int i = 0; i < lowThresholds.length; i++) {
            // Compute adaptive thresholds if needed
            Scalar[] adaptiveThresh = computeAdaptiveHSVThresholds(input, lowThresholds[i], highThresholds[i]);
            
            // Create temporary mask for this range
            Mat rangeMask = new Mat();
            Core.inRange(input, adaptiveThresh[0], adaptiveThresh[1], rangeMask);
            
            // If this is the first range, copy to output, otherwise OR with output
            if (i == 0) {
                rangeMask.copyTo(output);
            } else {
                Core.bitwise_or(output, rangeMask, output);
            }
            
            // Release temporary mask
            rangeMask.release();
            
            // Log the threshold values if in tuning mode
            if (tuningMode) {
                String colorName = "";
                switch (colorMode) {
                    case RED: colorName = "RED"; break;
                    case BLUE: colorName = "BLUE"; break;
                    case YELLOW: colorName = "YELLOW"; break;
                }
                
                Log.d("SampleDetectionPipeline", String.format(
                    "%s Range %d: Low(%.0f,%.0f,%.0f) High(%.0f,%.0f,%.0f)",
                    colorName, i+1,
                    adaptiveThresh[0].val[0], adaptiveThresh[0].val[1], adaptiveThresh[0].val[2],
                    adaptiveThresh[1].val[0], adaptiveThresh[1].val[1], adaptiveThresh[1].val[2]
                ));
            }
        }
    }

    /**
     * Apply morphological operations to the binary mask
     * 
     * @param input Input binary mask
     * @param output Output processed mask
     */
    private void applyMorphologicalOperations(Mat input, Mat output) {
        // Apply opening operation (erosion followed by dilation)
        Imgproc.morphologyEx(input, output, Imgproc.MORPH_OPEN, getMorphKernel());
        
        // Apply optional dilation after opening if enabled
        if (enableDilationAfterOpening) {
            // Create dilation kernel with the specified size
            Mat dilationKernel = Imgproc.getStructuringElement(
                Imgproc.MORPH_RECT, 
                new Size(dilationKernelSize, dilationKernelSize)
            );
            
            // Apply dilation to the output of the opening operation
            Imgproc.dilate(output, output, dilationKernel);
            
            // Release the kernel
            dilationKernel.release();
        }
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
     
        
        // Calculate metrics
        double confidence = 0.0;
        
        // Get basic properties - these are cheap to calculate
        RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
        
        // Calculate area based on the flag
        double area;
        if (useRectangleArea) {
            // Use rectangle area (width * height)
            area = rect.size.width * rect.size.height;
        } else {
            // Use contour area
            area = Imgproc.contourArea(contour);
        }
        
        Rect boundingRect = Imgproc.boundingRect(contour);
        double boxArea = boundingRect.width * boundingRect.height;
        
        // 1. Aspect Ratio Score
        double aspectRatio = Math.min(rect.size.width, rect.size.height) / Math.max(rect.size.width, rect.size.height);
        double idealAspectRatio = 0.43; // Adjust based on your samples
        double aspectRatioScore = Math.exp(-Math.pow(aspectRatio - idealAspectRatio, 2) / 0.1);
        
        // 2. Area Score
        // Calculate distance based on the rectangle size
        double zInches = (REAL_SAMPLE_LENGTH * FOCAL_LEGNTH_X) /
                       Math.max(rect.size.width, rect.size.height);
        
        // Get expected area range for this distance
        double[] expectedAreaRange = calculateExpectedAreaRange(zInches);
        double minAreaForDistance = expectedAreaRange[0];
        double maxAreaForDistance = expectedAreaRange[1];
        
        // Calculate ideal area and tolerance for this distance
        double idealArea = (minAreaForDistance + maxAreaForDistance) / 2.0;
        double areaTolerance = (maxAreaForDistance - minAreaForDistance) / 2.0;
        
        // Score is highest when area is close to ideal area for this distance
        double areaScore = 1.0 - Math.min(1.0, Math.abs(area - idealArea) / areaTolerance);
        
        // 3. Rectangularity Score
        double rectangularity = (boxArea > 0) ? area / boxArea : 0;
        double rectangularityScore = Math.min(1.0, rectangularity);
        
        // 4. Brightness Score - NEW
        double brightnessScore = 1.0; // Default to full score
        
        try {
            // Create a mask for just this contour
            Mat contourMask = Mat.zeros(mask.size(), CvType.CV_8UC1);
            List<MatOfPoint> contourList = new ArrayList<>();
            contourList.add(contour);
            Imgproc.drawContours(contourMask, contourList, 0, new Scalar(255), -1);
            
            // Extract the region from the input image
            Mat gray = new Mat();
            Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);
            
            // Calculate mean brightness of the contour region
            Scalar meanScalar = Core.mean(gray, contourMask);
            double meanBrightness = meanScalar.val[0]; // Grayscale has one channel
            
            // Calculate brightness score
            if (meanBrightness < MIN_BRIGHTNESS_THRESHOLD) {
                // Linear scaling from 0 to 1 based on brightness
                brightnessScore = meanBrightness / MIN_BRIGHTNESS_THRESHOLD;
            }
            
            // Release temporary Mats
            contourMask.release();
            gray.release();
            
        } catch (Exception e) {
            System.out.println("Error calculating brightness: " + e.getMessage());
            // Keep default brightness score on error
        }
        
        // Calculate weighted average of all metrics
        confidence = aspectRatioScore * ASPECT_RATIO_WEIGHT +
                     areaScore * AREA_WEIGHT +
                     rectangularityScore * RECTANGULARITY_WEIGHT +
                     brightnessScore * BRIGHTNESS_WEIGHT;
        
        // Ensure confidence is between 0 and 1
        confidence = Math.max(0.0, Math.min(1.0, confidence));
        
        return confidence;
    }

    /**
     * Calculate the expected min and max contour area for a specific distance
     * 
     * @param zInches Distance in inches
     * @return Array with [minArea, maxArea] in pixels
     */
    private double[] calculateExpectedAreaRange(double zInches) {
        // Use minContourArea and maxContourArea instead of normalized area thresholds
        // We still adjust based on distance to maintain the z-axis scaling
        double distanceFactor = 1.0 / (zInches * zInches);
        double minAreaForDistance = minContourArea * distanceFactor;
        double maxAreaForDistance = maxContourArea * distanceFactor;
        //TODO: Tune these values based on the actual sample size and distance
        return new double[] { minAreaForDistance, maxAreaForDistance };
    }

    /**
     * Calculate the mean HSV values for a contour
     * 
     * @param contour The contour to analyze
     * @param hsvImage The HSV image
     * @return Scalar containing mean H, S, V values
     */
    private Scalar calculateMeanHSV(MatOfPoint contour, Mat hsvImage) {
        try {
            // Create a mask for just this contour
            Mat contourMask = Mat.zeros(hsvImage.size(), CvType.CV_8UC1);
            List<MatOfPoint> contourList = new ArrayList<>();
            contourList.add(contour);
            Imgproc.drawContours(contourMask, contourList, 0, new Scalar(255), -1);
            
            // Calculate mean HSV values
            Scalar meanHSV = Core.mean(hsvImage, contourMask);
            
            // Release the mask
            contourMask.release();
            
            return meanHSV;
        } catch (Exception e) {
            System.out.println("Error calculating mean HSV: " + e.getMessage());
            return new Scalar(0, 0, 0);
        }
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
            // Get the rotated rectangle for this contour
            RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
            
            // Calculate area based on the flag
            double areaPx;
            if (useRectangleArea) {
                // Use rectangle area (width * height)
                areaPx = rect.size.width * rect.size.height;
            } else {
                // Use contour area
                areaPx = Imgproc.contourArea(contour);
            }
            
            // Calculate distance based on the rectangle size
            double zInches = (REAL_SAMPLE_LENGTH * FOCAL_LEGNTH_X) /
                           Math.max(rect.size.width, rect.size.height);
            
            // Calculate expected area range for this distance
            double[] expectedAreaRange = calculateExpectedAreaRange(zInches);
            double minAreaForDistance = expectedAreaRange[0];
            double maxAreaForDistance = expectedAreaRange[1];

            FtcDashboard.getInstance().getTelemetry().addData("contourArea: ", areaPx);
            FtcDashboard.getInstance().getTelemetry().addData("minAreaForDistance: ", minAreaForDistance);
            FtcDashboard.getInstance().getTelemetry().addData("maxAreaForDistance: ", maxAreaForDistance);
            FtcDashboard.getInstance().getTelemetry().update();

            minAreaForDistance = Math.max(minAreaForDistance, minContourArea);

            // Filter based on area for this specific distance
            if (areaPx < minAreaForDistance) {
                // Reject contour – too small or too big *for its distance*
                continue;
            }
            
            // Calculate aspect ratio
            double aspectRatio = Math.min(rect.size.width, rect.size.height)
                    / Math.max(rect.size.width, rect.size.height);

            // For backward compatibility, still check the raw area
            if (areaPx <= maxAreaForDistance) {
                // Calculate confidence using the new function
                double confidence = isBlobSampleConfidence(contour, mask, input);
                
                // If tuning mode is enabled, calculate and display HSV values, Aspect Ratio, and Area
                if (tuningMode) {
                    // Calculate mean HSV values for this contour
                    Scalar meanHSV = calculateMeanHSV(contour, hsv);
                    
                                      
                    // Display HSV values on the image
                    String hsvText = String.format("H:%.0f S:%.0f V:%.0f", 
                                                  meanHSV.val[0], meanHSV.val[1], meanHSV.val[2]);
                    
                    // Position the text above the contour
                    Point textPosition = new Point(rect.center.x, rect.center.y - 20);
                    
                    // Draw the HSV values on the image
                    Imgproc.putText(
                        input,
                        hsvText,
                        textPosition,
                        Imgproc.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        new Scalar(255, 255, 0), // Yellow color
                        1
                    );
                    
                    // Position the text for Aspect Ratio and Area
                    Point arTextPosition = new Point(rect.center.x, rect.center.y - 40);
                    
                    // Draw the Aspect Ratio and Area values on the image
                    String arAreaText = String.format("AR:%.2f Area:%.0f", aspectRatio, areaPx);
                    Imgproc.putText(
                        input,
                        arAreaText,
                        arTextPosition,
                        Imgproc.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        new Scalar(0, 255, 255), // Cyan color
                        1
                    );
                    
                    // Log the values to the console as well
                    System.out.println("Blob HSV values: " + hsvText);
                    System.out.println("Blob Aspect Ratio: " + aspectRatio + ", Area: " + areaPx);
                    FtcDashboard.getInstance().getTelemetry().addData("Blob HSV values: ", hsvText);
                    FtcDashboard.getInstance().getTelemetry().addData("Blob Aspect Ratio: ", aspectRatio);
                    FtcDashboard.getInstance().getTelemetry().addData(" Area: " , areaPx);
                    FtcDashboard.getInstance().getTelemetry().update();
                }
                
                // Process with calculated confidence
                processDetectedRect(rect, input, detectedRects, distances, areaPx, confidence);
            } else if (aspectRatio > BLOB_ASPECT_RATIO && areaPx > BLOB_AREA_THRESHOLD) { // check if they are worth trying
//                processLargeBlob(contour, mask, input, detectedRects, distances);
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
            RotatedRect subRect = Imgproc.minAreaRect(new MatOfPoint2f(innerContour.toArray()));
            
            // Calculate area based on the flag
            double subArea;
            if (useRectangleArea) {
                // Use rectangle area (width * height)
                subArea = subRect.size.width * subRect.size.height;
            } else {
                // Use contour area
                subArea = Imgproc.contourArea(innerContour);
            }

            // Calculate distance based on the rectangle size
            double zInches = (REAL_SAMPLE_LENGTH * FOCAL_LEGNTH_X) /
                    Math.max(subRect.size.width, subRect.size.height);

            // Calculate expected area range for this distance
            double[] expectedAreaRange = calculateExpectedAreaRange(zInches);
            double minAreaForDistance = expectedAreaRange[0];
            double maxAreaForDistance = expectedAreaRange[1];

            //TODO: Add distance normalization logic here if needed
            
            if (subArea < minAreaForDistance || subArea > maxAreaForDistance) {
                Imgproc.putText(
                        input,
                        String.format("skipped - Area: :%.0f", subArea),
                        new Point(boundingBox.x, boundingBox.y),
                        Imgproc.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        new Scalar(255, 0, 0),
                        2);
                continue;// Ignore contours with area outside the range
            }

            // Calculate aspect ratio
            double subAspectRatio = Math.min(subRect.size.width, subRect.size.height)
                    / Math.max(subRect.size.width, subRect.size.height);
            if (subAspectRatio < minAspectRatio || subAspectRatio > maxAspectRatio) {
                Imgproc.putText(
                        input,
                         String.format("skipped - Aspect Ratio: :%.2f", subAspectRatio),
                        new Point(boundingBox.x, boundingBox.y),
                        Imgproc.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        new Scalar(0, 0, 255),
                        2);
                continue; // Ignore contours with aspect ratio outside the range
            }

            // The inner contours are found in the ROI coordinate system, which is relative
            // to the bounding box
            // We need to adjust the center coordinates to the original image coordinate system
            subRect.center.x += boundingBox.x;
            subRect.center.y += boundingBox.y;

            // Now process the adjusted rectangle with lower confidence

            double confidence = Math.min(0.98, isBlobSampleConfidence(innerContour, mask, input)); // cap at .98

            processDetectedRect(subRect, input, detectedRects, distances, subArea, confidence);
            
            // Add to processed contours
            processedContours.add(innerContour);
        }//For Loop End
        
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
     * Count the number of contours with confidence above the threshold
     * 
     * @param contours List of contours to evaluate
     * @param mask Binary mask for ROI extraction
     * @param input Original input image
     * @param confidenceThreshold Minimum confidence threshold
     * @return Number of contours with confidence above threshold
     */
    private int countHighConfidenceContours(List<MatOfPoint> contours, Mat mask, Mat input, double confidenceThreshold) {
        int count = 0;
        for (MatOfPoint contour : contours) {
            double confidence = isBlobSampleConfidence(contour, mask, input);
            if (confidence >= confidenceThreshold) {
                count++;
            }
        }
        return count;
    }
    
    /**
     * Apply a sequence of fragmentation algorithms in the specified order
     * 
     * @param contour The original contour to fragment
     * @param roi Region of interest containing the blob
     * @param mask Full binary mask
     * @param boundingBox Bounding box of the contour
     * @param input Original input image for visualization
     * @param order Array of integers representing the order of algorithms (0=Canny, 1=Distance, 2=Adaptive)
     * @return List of fragmented contours
     */
    private List<MatOfPoint> applyFragmentationSequence(MatOfPoint contour, Mat roi, Mat mask, Rect boundingBox, Mat input, int[] order) {
        List<MatOfPoint> resultContours = new ArrayList<>();
        Mat currentRoi = roi.clone();
        
        for (int i = 0; i < order.length; i++) {
            List<MatOfPoint> stepContours = new ArrayList<>();
            
            switch (order[i]) {
                case 0: // Canny Edge
                    stepContours = applyCannyEdgeFragmentation(contour, currentRoi, mask, boundingBox, input, false);
                    break;
                case 1: // Distance Transform
                    stepContours = applyDistanceTransformFragmentation(contour, currentRoi, mask, boundingBox, input, false);
                    break;
                case 2: // Adaptive Threshold
                    stepContours = applyAdaptiveThresholdFragmentation(contour, currentRoi, mask, boundingBox, input, false);
                    break;
            }
            
            // If we got contours, update the ROI for the next step
            if (!stepContours.isEmpty()) {
                // Create a new mask from these contours
                Mat newRoi = Mat.zeros(currentRoi.size(), CvType.CV_8UC1);
                Imgproc.drawContours(newRoi, stepContours, -1, new Scalar(255), -1);
                
                // Release the previous ROI and use the new one
                currentRoi.release();
                currentRoi = newRoi;
                
                // Add these contours to our result
                resultContours.addAll(stepContours);
            }
        }
        
        // Release the final ROI
        currentRoi.release();
        
        return resultContours;
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
        // Define the confidence threshold
//        double confidenceThreshold = 0.7;
//
//        // Define all 6 possible orderings of the 3 algorithms
//        // 0 = Canny Edge, 1 = Distance Transform, 2 = Adaptive Threshold
//        int[][] allOrders = {
//            {0, 1, 2}, // CDA
//            {0, 2, 1}, // CAD
//            {1, 0, 2}, // DCA
//            {1, 2, 0}, // DAC
//            {2, 0, 1}, // ACD
//            {2, 1, 0}  // ADC
//        };
//
//        // Try each ordering and count high confidence contours
        List<MatOfPoint> bestContours = new ArrayList<>();
//        int maxHighConfCount = -1;
//        int bestOrderIndex = -1;
//
//        for (int i = 0; i < allOrders.length; i++) {
//            // Apply this ordering of algorithms
//            List<MatOfPoint> currentContours = applyFragmentationSequence(contour, roi, mask, boundingBox, input, allOrders[i]);
//
//            // Count high confidence contours
//            int highConfCount = countHighConfidenceContours(currentContours, mask, input, confidenceThreshold);
//
//            // If this is better than our previous best, update
//            if (highConfCount > maxHighConfCount) {
//                // Release previous best contours
//                for (MatOfPoint c : bestContours) {
//                    c.release();
//                }
//                bestContours.clear();
//
//                // Save this as our new best
//                bestContours.addAll(currentContours);
//                maxHighConfCount = highConfCount;
//                bestOrderIndex = i;
//            } else {
//                // Release these contours since we're not using them
//                for (MatOfPoint c : currentContours) {
//                    c.release();
//                }
//            }
//        }
//
        // If we didn't find any good contours with any method, fall back to the default approach
//        if (bestContours.isEmpty()) {
            // Try each algorithm individually
            if (enableCannyEdgeFragmentation) {
                List<MatOfPoint> cannyEdgeContours = applyCannyEdgeFragmentation(contour, roi, mask, boundingBox, input, true);
                bestContours.addAll(cannyEdgeContours);
            }

            if (enableDistanceTransformFragmentation) {
                List<MatOfPoint> distanceTransformContours = applyDistanceTransformFragmentation(contour, roi, mask, boundingBox, input, true);
                bestContours.addAll(distanceTransformContours);
            }
            
            if (enableAdaptiveThresholdFragmentation) {
                List<MatOfPoint> adaptiveThresholdContours = applyAdaptiveThresholdFragmentation(contour, roi, mask, boundingBox, input, false);
                bestContours.addAll(adaptiveThresholdContours);
            }
//        } else {
//            // Log which ordering was best
//            String[] algoNames = {"Canny", "Distance", "Adaptive"};
//            StringBuilder orderStr = new StringBuilder();
//            for (int algo : allOrders[bestOrderIndex]) {
//                orderStr.append(algoNames[algo]).append(" → ");
//            }
//            if (orderStr.length() > 0) {
//                orderStr.setLength(orderStr.length() - 3); // Remove the last arrow
//            }
//
//            System.out.println("Best fragmentation order: " + orderStr.toString() +
//                              " with " + maxHighConfCount + " high confidence contours");
//            FtcDashboard.getInstance().getTelemetry().addData("Best fragmentation order", orderStr.toString() + " with " + maxHighConfCount + " high confidence contours");
//            FtcDashboard.getInstance().getTelemetry().update();
//
            // Optionally, draw this information on the image
//            if (tuningMode) {
//                Imgproc.putText(
//                    input,
//                    "Best order: " + orderStr.toString(),
//                    new Point(10, 30),
//                    Imgproc.FONT_HERSHEY_SIMPLEX,
//                    0.5,
//                    new Scalar(0, 255, 255),
//                    1
//                );
//            }
//        }
        
        return bestContours;
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
            
            // Get the maximum value for thresholding
            Core.MinMaxLocResult mm = Core.minMaxLoc(distanceTransform);
            double maxVal = mm.maxVal;
            
            // Try multiple threshold values and keep the best result
            List<MatOfPoint> bestContours = new ArrayList<>();
            int maxValidContours = 0;
            
            // Original contour area for comparison
            double originalArea = Imgproc.contourArea(contour);
            
            // Try different threshold values from 0.2 to 0.8 of max value
            for (double t = 0.1; t <= 0.9; t += 0.1) {
                // Apply threshold at this level
                Mat thresholded = new Mat();
                Imgproc.threshold(distanceTransform, thresholded, t * maxVal, 1.0, Imgproc.THRESH_BINARY);
                
                // Convert to 8-bit for findContours
                Mat thresholdedU8 = new Mat();
                thresholded.convertTo(thresholdedU8, CvType.CV_8U, 255);
                
                // Find contours at this threshold
                List<MatOfPoint> currentContours = new ArrayList<>();
                Mat hierarchy = new Mat();
                Imgproc.findContours(thresholdedU8, currentContours, hierarchy, 
                                    Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
                
                // Count valid contours (area ratio >= MIN_AREA_RATIO)
                int validContours = 0;
                for (MatOfPoint c : currentContours) {
                    double area = Imgproc.contourArea(c);
                    double areaRatio = area / originalArea;
                    
                    if (areaRatio >= MIN_AREA_RATIO) {
                        validContours++;
                    }
                }
                
                // If this threshold produced more valid contours, keep it as the best
                if (validContours > maxValidContours) {
                    maxValidContours = validContours;
                    
                    // Clear previous best contours and release them
                    for (MatOfPoint c : bestContours) {
                        c.release();
                    }
                    bestContours.clear();
                    
                    // Add all contours from this threshold to best contours
                    bestContours.addAll(currentContours);
                } else {
                    // Release contours that aren't the best
                    for (MatOfPoint c : currentContours) {
                        c.release();
                    }
                }
                
                // Release resources for this iteration
                thresholded.release();
                thresholdedU8.release();
                hierarchy.release();
            }
            
            // Add the best contours to the result
            resultContours.addAll(bestContours);
            
            // Release the distance transform
            distanceTransform.release();
            
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
            applyGaussianBlur(roi, blurred, new Size(5, 5));
            
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
    /**
     * Apply Canny edge detection algorithm to fragment a blob
     * 
     * @param contour The original contour to fragment
     * @param roi Region of interest containing the blob
     * @param mask Full binary mask
     * @param boundingBox Bounding box of the contour
     * @param input Original input image for visualization
     * @param keepOriginal Whether to keep the original contour in the output list
     * @return List of fragmented contours
     */
    private List<MatOfPoint> applyCannyEdgeFragmentation(MatOfPoint contour, Mat roi, Mat mask, Rect boundingBox, Mat input, boolean keepOriginal) {
        List<MatOfPoint> resultContours = new ArrayList<>();
        
        // Add the original contour if requested
        if (keepOriginal) {
            resultContours.add(contour);
        }
        
        try {
            Mat edges = new Mat();
            Mat edgesDil = new Mat();
            Mat maskSplit = new Mat();

            // 1. Slight blur suppresses salt-and-pepper noise
            applyGaussianBlur(roi, edges);

            // 2. Canny on the binary image (roi is 0/255 already)
            Imgproc.Canny(edges, edges, 50, 150);

            // 3. Thicken the edge just a bit so the cut is visible
            Imgproc.dilate(edges, edgesDil,
                           Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3)));

            // 4. Subtract edge pixels from the mask → splits touching blobs
            Mat edgesInv = new Mat();
            Core.bitwise_not(edgesDil, edgesInv);
            Core.bitwise_and(roi, edgesInv, maskSplit);

            // 5. Clean tiny leftovers (optional but helps)
            Imgproc.morphologyEx(maskSplit, maskSplit, Imgproc.MORPH_OPEN,
                    Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3)));

            // 6. Find contours in the split mask
            List<MatOfPoint> cannyContours = new ArrayList<>();
            Mat cannyHierarchy = new Mat();
            Imgproc.findContours(maskSplit, cannyContours, cannyHierarchy,
                                 Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            
            // Add to result
            resultContours.addAll(cannyContours);
            
            // Release resources
            edges.release();
            edgesDil.release();
            edgesInv.release();
            maskSplit.release();
            cannyHierarchy.release();
            
        } catch (Exception e) {
            System.out.println("Error in Canny edge fragmentation: " + e.getMessage());
        }
        
        return resultContours;
    }
    
    
    /**
     * Initialize method required by VisionProcessor interface
     */
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Initialize Mat objects with the correct size
        hsv = new Mat(height, width, CvType.CV_8UC3);
        mask = new Mat(height, width, CvType.CV_8UC1);
        hierarchy = new Mat();
        resizedMat = new Mat();
    }
    
    /**
     * Process the input frame to detect samples
     * Required by VisionProcessor interface
     * 
     * @param input Input frame from the camera
     * @param timestamp Timestamp of the frame
     * @return Processed frame with detected samples highlighted
     */
    @Override
    public Object processFrame(Mat input, long timestamp) {
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

        //Apply gaussian blur to the mask
        applyGaussianBlur(mask, mask, new Size(5, 5));
        
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
        
        // Simply convert the input image to bitmap and send to dashboard
        showOnDashboard(input);
     
        return input;
    }
    
    /**
     * Called when the VisionPortal is stopped
     * Required by VisionProcessor interface
     */
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // This method is called when the VisionPortal wants to draw on the screen
        // We don't need to implement this for our use case
    }
}
