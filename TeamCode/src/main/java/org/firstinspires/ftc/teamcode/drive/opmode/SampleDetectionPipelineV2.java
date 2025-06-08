package org.firstinspires.ftc.teamcode.drive.opmode;

import org.firstinspires.ftc.teamcode.GameConstants;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.opencv.android.Utils;
import android.graphics.Bitmap;

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
    public RotatedRect[] latestRects;
    public double[][] latestDistances;
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
    public static double DISTANCE_THRESHOLD = 0.3; // 0.2–0.4 is a good range
    public static int MORPH_KERNEL_SIZE = 7; // must be odd (3,5,7,...)

    // Helper method to get morphology kernel
    private Mat getMorphKernel() {
        return Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE));
    }

    public void setColorMode(GameConstants.GAME_COLORS mode) {
        this.colorMode = mode;
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
     * Convert a Mat to a Bitmap for visualization
     * 
     * @param input Input Mat image
     * @return Bitmap representation of the input Mat
     */
    private Bitmap matToBitmap(Mat input) {
        Bitmap bmp = Bitmap.createBitmap(input.cols(), input.rows(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(input, bmp);
        return bmp;
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
            double contourArea

    ) {
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

        // Use the adjusted values
        distances.add(new double[] { x_inches_x, angle, ground_distance, turretAngle });
    }

    @Override
    public Mat processFrame(Mat input) {
        // Update camera settings if they've changed
        if (camera != null) {
            updateCameraSettings();
        }

        Mat hsv = new Mat();

        // Step 1: Convert to HSV
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // step 1.5 Equalize the v channel to normalize brightness
        // List<Mat> hsvChannels = new ArrayList<>();
        // Core.split(hsv, hsvChannels);
        // Imgproc.equalizeHist(hsvChannels.get(2), hsvChannels.get(2)); // equalize the
        // v channel
        // Core.merge(hsvChannels, hsv);

        // Step 1.5: Optionally equalize the V channel to normalize brightness
        if (enableHSVEqualization) {
            List<Mat> hsvChannels = new ArrayList<>();
            Core.split(hsv, hsvChannels);
            Imgproc.equalizeHist(hsvChannels.get(2), hsvChannels.get(2)); // Equalize V channel
            Core.merge(hsvChannels, hsv);
        }

        // Step 2: Apply Gaussian blur
        Mat blurred = hsv;
        // Imgproc.GaussianBlur(hsv, blurred, new Size(5, 5), 0);

        // Step 3: Adaptive Thresholding
        Mat mask = new Mat();
        switch (colorMode) {
            case RED:
                Scalar[] redThresh1 = computeAdaptiveHSVThresholds(blurred, RED_LOW_1, RED_HIGH_1);
                Scalar[] redThresh2 = computeAdaptiveHSVThresholds(blurred, RED_LOW_2, RED_HIGH_2);

                Mat lowerRed = new Mat();
                Mat upperRed = new Mat();
                Core.inRange(blurred, redThresh1[0], redThresh1[1], lowerRed);
                Core.inRange(blurred, redThresh2[0], redThresh2[1], upperRed);
                Core.bitwise_or(lowerRed, upperRed, mask);

                // release temporary Mats
                lowerRed.release();
                upperRed.release();
                break;

            case BLUE:
                Scalar[] blueThresh = computeAdaptiveHSVThresholds(blurred, BLUE_LOW, BLUE_HIGH);
                Core.inRange(blurred, blueThresh[0], blueThresh[1], mask);
                break;

            case YELLOW:
                Scalar[] yellowThresh = computeAdaptiveHSVThresholds(blurred, YELLOW_LOW, YELLOW_HIGH);
                Core.inRange(blurred, yellowThresh[0], yellowThresh[1], mask);
                break;
        }

        // Step 4: Morphological operations
        // Imgproc.erode(mask, mask, Imgproc.getStructuringElement(Imgproc.MORPH_RECT,
        // new Size(3, 3)));
        // Imgproc.dilate(mask, mask, Imgproc.getStructuringElement(Imgproc.MORPH_RECT,
        // new Size(5, 5)));
        // Step 4: Morphological Open
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, getMorphKernel());

        // Step 5: Find contours
        ArrayList<RotatedRect> detectedRects = new ArrayList<>();
        ArrayList<double[]> distances = new ArrayList<>();
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        hierarchy.release();

        // Step 6: Process each contour
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);

            if (area < minContourArea)
                continue; // Ignore small contours

            RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
            double aspectRatio = Math.min(rect.size.width, rect.size.height)
                    / Math.max(rect.size.width, rect.size.height);

            if (area >= minContourArea && area <= maxContourArea) {
                processDetectedRect(rect, input, detectedRects, distances, area);
            } else if (aspectRatio > BLOB_ASPECT_RATIO && area > BLOB_AREA_THRESHOLD) { // check if they are worth
                                                                                        // trying
                Rect boundingBox = Imgproc.boundingRect(contour);
                Mat roi = new Mat(mask, boundingBox);

                // Call the blob fragmentation function
                List<MatOfPoint> innerContours = fragmentBlob(contour, roi, mask, boundingBox, input);

                if (innerContours.isEmpty()) {
                    // If no fragmentation was successful or enabled, use the original approach
                    innerContours = new ArrayList<>();
                    Mat hierarchyInner = new Mat();
                    Imgproc.findContours(roi.clone(), innerContours, hierarchyInner, Imgproc.RETR_EXTERNAL,
                            Imgproc.CHAIN_APPROX_SIMPLE);
                    hierarchyInner.release();
                }

                roi.release();
                // Filter inner contours based on area and aspect ratio
                for (MatOfPoint innerContour : innerContours) {
                    double subArea = Imgproc.contourArea(innerContour);
                    // if (subArea < minContourArea || subArea > maxContourArea) continue;

                    RotatedRect subRect = Imgproc.minAreaRect(new MatOfPoint2f(innerContour.toArray()));
                    double subAspectRatio = Math.min(subRect.size.width, subRect.size.height)
                            / Math.max(subRect.size.width, subRect.size.height);
                    // if (subAspectRatio < minAspectRatio || subAspectRatio > maxAspectRatio)
                    // continue;

                    subRect.center.x += boundingBox.x;
                    subRect.center.y += boundingBox.y;

                    processDetectedRect(subRect, input, detectedRects, distances, subArea);
                }
            }
        }

        latestRects = detectedRects.toArray(new RotatedRect[0]);
        latestDistances = distances.toArray(new double[0][]);

        // Push to dashboard for debugging
        // Send image to dashboard
        Bitmap annotatedBitmap = matToBitmap(input);
        FtcDashboard.getInstance().sendImage(annotatedBitmap);
        // release Mats
        hsv.release();
        mask.release();
        blurred.release();

        return input;
    }

    /**
     * Fragment a large blob using multiple algorithms controlled by dashboard
     * settings
     * 
     * @param contour     Original contour of the blob
     * @param roi         Region of interest containing the blob
     * @param mask        Original binary mask
     * @param boundingBox Bounding box of the blob
     * @param input       Original input image for visualization
     * @return List of contours representing the fragmented blob
     */
    private List<MatOfPoint> fragmentBlob(MatOfPoint contour, Mat roi, Mat mask, Rect boundingBox, Mat input) {
        List<MatOfPoint> resultContours = new ArrayList<>();
        StringBuilder appliedAlgorithms = new StringBuilder();

        // If no fragmentation algorithms are enabled, return empty list
        if (!enableWatershedFragmentation &&
                !enableDistanceTransformFragmentation &&
                !enableAdaptiveThresholdFragmentation &&
                !enableMorphologicalGradientFragmentation &&
                !enableContourSplittingFragmentation) {
            return resultContours;
        }

        // Start with the original contour
        resultContours.add(new MatOfPoint(contour.toArray())); // Add a copy of the original contour
        // resultContours.add(new MatOfPoint(contour.toArray())); // Add a copy of the
        // original contour

        // 1. Watershed Fragmentation
        if (enableWatershedFragmentation) {
            try {
                // Create a copy of the ROI for watershed
                Mat watershedMask = roi.clone();

                // Convert to BGR for watershed
                Mat watershedInput = new Mat();
                Imgproc.cvtColor(watershedMask, watershedInput, Imgproc.COLOR_GRAY2BGR);

                // Create markers (background = 1, foreground = 2)
                Mat markers = Mat.zeros(watershedMask.size(), CvType.CV_32SC1);

                // Create foreground and background markers
                Mat sure_fg = new Mat();
                Mat sure_bg = new Mat();

                // Erode to get sure foreground
                Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(3, 3));
                Imgproc.erode(watershedMask, sure_fg, kernel, new Point(-1, -1), 2);

                // Dilate to get sure background
                Imgproc.dilate(watershedMask, sure_bg, kernel, new Point(-1, -1), 3);

                // Set markers: 1 for background, 2 for foreground
                for (int i = 0; i < markers.rows(); i++) {
                    for (int j = 0; j < markers.cols(); j++) {
                        if (sure_fg.get(i, j)[0] > 0) {
                            markers.put(i, j, 2); // Foreground
                        } else if (sure_bg.get(i, j)[0] > 0) {
                            markers.put(i, j, 1); // Background
                        } else {
                            markers.put(i, j, 0); // Unknown
                        }
                    }
                }

                // Apply watershed
                Imgproc.watershed(watershedInput, markers);

                // Extract contours from watershed result
                Mat watershedResult = Mat.zeros(markers.size(), CvType.CV_8UC1);
                for (int i = 0; i < markers.rows(); i++) {
                    for (int j = 0; j < markers.cols(); j++) {
                        if (markers.get(i, j)[0] == 2) { // Foreground
                            watershedResult.put(i, j, 255);
                        }
                    }
                }

                // Find contours in the watershed result
                List<MatOfPoint> watershedContours = new ArrayList<>();
                Mat hierarchyWatershed = new Mat();
                Imgproc.findContours(watershedResult, watershedContours, hierarchyWatershed,
                        Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

                // Add watershed contours to result
                for (MatOfPoint watershedContour : watershedContours) {
                    if (Imgproc.contourArea(watershedContour) > minContourArea) {
                        resultContours.add(watershedContour);
                    }
                }

                // Release temporary Mats
                watershedMask.release();
                watershedInput.release();
                markers.release();
                sure_fg.release();
                sure_bg.release();
                watershedResult.release();
                hierarchyWatershed.release();
                kernel.release();

                // Draw a label on the input image to show which algorithm was used
                if (!resultContours.isEmpty()) {
                    Imgproc.putText(
                            input,
                            "Watershed",
                            new Point(boundingBox.x, boundingBox.y - 5),
                            Imgproc.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            new Scalar(255, 0, 255),
                            1);
                }
            } catch (Exception e) {
                System.out.println("Error in watershed fragmentation: " + e.getMessage());
            }
        }

        // 2. Distance Transform Fragmentation
        if (enableDistanceTransformFragmentation) {
            try {
                // Create a mask from current contours
                Mat contourMask = Mat.zeros(roi.size(), CvType.CV_8UC1);
                Imgproc.drawContours(contourMask, resultContours, -1, new Scalar(255), -1);

                // Apply distance transform
                Mat dist = new Mat();
                Imgproc.distanceTransform(contourMask, dist, Imgproc.DIST_L2, 5);

                // Normalize the distance image
                Core.normalize(dist, dist, 0, 1.0, Core.NORM_MINMAX);

                // Threshold to get peaks
                Mat distPeaks = new Mat();
                Imgproc.threshold(dist, distPeaks, 0.5, 1.0, Imgproc.THRESH_BINARY);

                // Convert to 8-bit for findContours
                distPeaks.convertTo(distPeaks, CvType.CV_8U, 255);

                // Find contours in the distance transform result
                List<MatOfPoint> distContours = new ArrayList<>();
                Mat hierarchyDist = new Mat();
                Imgproc.findContours(distPeaks, distContours, hierarchyDist,
                        Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

                // Add distance transform contours to result
                for (MatOfPoint distContour : distContours) {
                    if (Imgproc.contourArea(distContour) > minContourArea) {
                        resultContours.add(distContour);
                    }
                }

                // Release temporary Mats
                contourMask.release();
                dist.release();
                distPeaks.release();
                hierarchyDist.release();

                // Draw a label on the input image to show which algorithm was used
                if (!resultContours.isEmpty()) {
                    Imgproc.putText(
                            input,
                            "Distance Transform",
                            new Point(boundingBox.x, boundingBox.y - 5),
                            Imgproc.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            new Scalar(0, 255, 255),
                            1);
                }
            } catch (Exception e) {
                System.out.println("Error in distance transform fragmentation: " + e.getMessage());
            }
        }

        // 3. Adaptive Threshold Fragmentation
        if (enableAdaptiveThresholdFragmentation) {
            try {
                // Create a mask from current contours
                Mat adaptiveMask = Mat.zeros(roi.size(), CvType.CV_8UC1);
                Imgproc.drawContours(adaptiveMask, resultContours, -1, new Scalar(255), -1);

                // Convert to grayscale if needed
                Mat gray = new Mat();
                if (adaptiveMask.channels() > 1) {
                    Imgproc.cvtColor(adaptiveMask, gray, Imgproc.COLOR_BGR2GRAY);
                } else {
                    gray = adaptiveMask.clone();
                }

                // Apply adaptive threshold
                Mat adaptiveThresh = new Mat();
                Imgproc.adaptiveThreshold(gray, adaptiveThresh, 255,
                        Imgproc.ADAPTIVE_THRESH_GAUSSIAN_C,
                        Imgproc.THRESH_BINARY, 11, 2);

                // Find contours in the adaptive threshold result
                List<MatOfPoint> adaptiveContours = new ArrayList<>();
                Mat hierarchyAdaptive = new Mat();
                Imgproc.findContours(adaptiveThresh, adaptiveContours, hierarchyAdaptive,
                        Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

                // Add adaptive threshold contours to result
                for (MatOfPoint adaptiveContour : adaptiveContours) {
                    if (Imgproc.contourArea(adaptiveContour) > minContourArea) {
                        resultContours.add(adaptiveContour);
                    }
                }

                // Release temporary Mats
                adaptiveMask.release();
                gray.release();
                adaptiveThresh.release();
                hierarchyAdaptive.release();

                // Draw a label on the input image to show which algorithm was used
                if (!resultContours.isEmpty()) {
                    Imgproc.putText(
                            input,
                            "Adaptive Threshold",
                            new Point(boundingBox.x, boundingBox.y - 5),
                            Imgproc.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            new Scalar(0, 255, 0),
                            1);
                }
            } catch (Exception e) {
                System.out.println("Error in adaptive threshold fragmentation: " + e.getMessage());
            }
        }

        // 4. Morphological Gradient Fragmentation
        if (enableMorphologicalGradientFragmentation) {
            try {
                // Create a mask from current contours
                Mat morphMask = Mat.zeros(roi.size(), CvType.CV_8UC1);
                Imgproc.drawContours(morphMask, resultContours, -1, new Scalar(255), -1);

                // Apply morphological gradient
                Mat gradient = new Mat();
                Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
                Imgproc.morphologyEx(morphMask, gradient, Imgproc.MORPH_GRADIENT, kernel);

                // Threshold the gradient
                Mat gradThresh = new Mat();
                Imgproc.threshold(gradient, gradThresh, 50, 255, Imgproc.THRESH_BINARY);

                // Dilate to connect edges
                Mat dilated = new Mat();
                Imgproc.dilate(gradThresh, dilated, kernel);

                // Invert to get regions
                Mat inverted = new Mat();
                Core.bitwise_not(dilated, inverted);

                // Find contours in the morphological result
                List<MatOfPoint> morphContours = new ArrayList<>();
                Mat hierarchyMorph = new Mat();
                Imgproc.findContours(inverted, morphContours, hierarchyMorph,
                        Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

                // Add morphological contours to result
                for (MatOfPoint morphContour : morphContours) {
                    if (Imgproc.contourArea(morphContour) > minContourArea) {
                        resultContours.add(morphContour);
                    }
                }

                // Release temporary Mats
                morphMask.release();
                gradient.release();
                gradThresh.release();
                dilated.release();
                inverted.release();
                hierarchyMorph.release();
                kernel.release();

                // Draw a label on the input image to show which algorithm was used
                if (!resultContours.isEmpty()) {
                    Imgproc.putText(
                            input,
                            "Morphological Gradient",
                            new Point(boundingBox.x, boundingBox.y - 5),
                            Imgproc.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            new Scalar(255, 0, 0),
                            1);
                }
            } catch (Exception e) {
                System.out.println("Error in morphological gradient fragmentation: " + e.getMessage());
            }
        }

        // 5. Contour Splitting Fragmentation
        if (enableContourSplittingFragmentation) {
            try {
                // Process each contour in the current result set
                List<MatOfPoint> newContours = new ArrayList<>();

                for (MatOfPoint currentContour : resultContours) {
                    // Create a copy of the contour for splitting
                    MatOfPoint2f contour2f = new MatOfPoint2f(currentContour.toArray());

                    // Find convexity defects
                    MatOfInt hull = new MatOfInt();
                    Imgproc.convexHull(currentContour, hull, false);

                    // If we have enough points for convexity defects
                    if (hull.size().height >= 4) {
                        MatOfInt4 convexityDefects = new MatOfInt4();
                        Imgproc.convexityDefects(currentContour, hull, convexityDefects);

                        // Get defect points
                        List<Point> defectPoints = new ArrayList<>();

                        for (int i = 0; i < convexityDefects.rows(); i++) {
                            double[] defect = convexityDefects.get(i, 0);
                            Point startPoint = new Point(currentContour.get((int) defect[0], 0));
                            Point endPoint = new Point(currentContour.get((int) defect[1], 0));
                            Point depthPoint = new Point(currentContour.get((int) defect[2], 0));
                            double depth = defect[3] / 256.0; // Convert to pixels

                            // Only consider deep defects
                            if (depth > 10) {
                                defectPoints.add(depthPoint);
                            }
                        }

                        // If we have defect points, use them to split the contour
                        if (defectPoints.size() >= 2) {
                            // Create a mask for the split contours
                            Mat splitMask = Mat.zeros(roi.size(), CvType.CV_8UC1);

                            // Draw the original contour
                            List<MatOfPoint> contourList = new ArrayList<>();
                            contourList.add(currentContour);
                            Imgproc.drawContours(splitMask, contourList, 0, new Scalar(255), -1);

                            // Draw lines between defect points to split the contour
                            for (int i = 0; i < defectPoints.size() - 1; i++) {
                                Point p1 = new Point(defectPoints.get(i).x - boundingBox.x,
                                        defectPoints.get(i).y - boundingBox.y);
                                Point p2 = new Point(defectPoints.get(i + 1).x - boundingBox.x,
                                        defectPoints.get(i + 1).y - boundingBox.y);
                                Imgproc.line(splitMask, p1, p2, new Scalar(0), 2);
                            }

                            // Find contours in the split mask
                            List<MatOfPoint> splitContours = new ArrayList<>();
                            Mat hierarchySplit = new Mat();
                            Imgproc.findContours(splitMask, splitContours, hierarchySplit,
                                    Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

                            // Add split contours to result
                            for (MatOfPoint splitContour : splitContours) {
                                if (Imgproc.contourArea(splitContour) > minContourArea) {
                                    resultContours.add(splitContour);
                                }
                            }

                            // Release temporary Mats
                            splitMask.release();
                            hierarchySplit.release();
                        }
                    }

                    // Release temporary Mats
                    contour2f.release();
                    hull.release();
                }

                // Draw a label on the input image to show which algorithm was used
                if (!resultContours.isEmpty()) {
                    Imgproc.putText(
                            input,
                            "Contour Splitting",
                            new Point(boundingBox.x, boundingBox.y - 5),
                            Imgproc.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            new Scalar(255, 255, 0),
                            1);
                }
            } catch (Exception e) {
                System.out.println("Error in contour splitting fragmentation: " + e.getMessage());
            }
        }

        return resultContours;
    }
}
