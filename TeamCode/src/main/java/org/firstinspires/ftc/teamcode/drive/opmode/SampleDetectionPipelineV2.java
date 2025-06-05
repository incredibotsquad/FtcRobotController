package org.firstinspires.ftc.teamcode.drive.opmode;

import org.firstinspires.ftc.teamcode.GameConstants;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.List;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.android.Utils;
import android.graphics.Bitmap;

// TensorFlow Lite imports
import org.tensorflow.lite.Interpreter;
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
    public static double HEIGHT_OF_CAMERA = 10; //Inches
    public static double CENTER_OF_IMAGE = 12; //Inches
    public static double minContourArea = 300;
    public static double maxContourArea = 3000;
    public static double minAspectRatio = 0.3;
    public static double maxAspectRatio = 0.9;

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

    public void setColorMode(GameConstants.GAME_COLORS mode) {
        this.colorMode = mode;
    }

    /**
     * Get the horizontal distance (X) from the robot to the sample in inches
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
     * @param index Index of the detected sample
     * @return Orientation angle in degrees, or -1 if index is invalid
     */
    public double GetSampleOrientation(int index) {
        if (latestDistances != null && index >= 0 && index < latestDistances.length) {
            return latestDistances[index][1];
        }
        return -1;
    }

    /**
     * Convert a Mat to a Bitmap for visualization
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
//        // Convert to grayscale
//        Mat gray = new Mat();
//        Imgproc.cvtColor(inputFrame, gray, Imgproc.COLOR_BGR2GRAY);
//
//        // Compute average brightness
//        Scalar avgScalar = Core.mean(gray);
//        double avgBrightness = avgScalar.val[0];  // since grayscale has one channel
//
//        // Reference brightness
//        double refBrightness = 128.0; //128 is a mid-point for 0-255 range
//        double brightnessRatio = avgBrightness / refBrightness;
//
//        // Clamp scale factor between 0.5 and 1.5
//        double scaleFactor = 1.0 / Math.max(0.5, Math.min(1.5, brightnessRatio));
//
//        // Create adaptiveMin based on baseMin
//        double hMin = baseMin.val[0];
//        double sMin = baseMin.val[1] * scaleFactor;
//        double vMin = baseMin.val[2] * scaleFactor;
//
//        // Clamp S and V to 0–255
//        sMin = Math.max(0, Math.min(255, sMin));
//        vMin = Math.max(0, Math.min(255, vMin));
//
//        Scalar adaptiveMin = new Scalar(hMin, sMin, vMin);
//        Scalar adaptiveMax = baseMax.clone();  // No change to max in this version
//        return new Scalar[]{adaptiveMin, adaptiveMax};
        return new Scalar[]{baseMin, baseMax};
    }


    private void processDetectedRect(
            RotatedRect rect,
            Mat input,
            ArrayList<RotatedRect> detectedRects,
            ArrayList<double[]> distances
    ) {
        detectedRects.add(rect);

        // Draw rectangle
        org.opencv.core.Point[] vertices = new org.opencv.core.Point[4];

        rect.points(vertices);
        for (int j = 0; j < 4; j++) {
            Imgproc.line(input, vertices[j], vertices[(j + 1) % 4], new Scalar(0, 255, 0), 2);
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
        distances.add(new double[]{x_inches_x, angle, ground_distance, turretAngle});
    }


    @Override
    public Mat processFrame(Mat input) {
        Mat hsv = new Mat();

        // Step 1: Convert to HSV
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Step 2: Apply Gaussian blur
        Mat blurred = hsv;
//        Imgproc.GaussianBlur(hsv, blurred, new Size(5, 5), 0);

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
        Imgproc.erode(mask, mask, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3)));
        Imgproc.dilate(mask, mask, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));

        // Step 5: Find contours
        ArrayList<RotatedRect> detectedRects = new ArrayList<>();
        ArrayList<double[]> distances = new ArrayList<>();
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Step 6: Process each contour
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area < minContourArea) continue;

            RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
            double aspectRatio = Math.min(rect.size.width, rect.size.height) / Math.max(rect.size.width, rect.size.height);

            if (aspectRatio < minAspectRatio || aspectRatio > maxAspectRatio) {
                // Possibly merged blob
                if (aspectRatio > 0.8 && area > 2 * minContourArea) {
                    Rect boundingBox = Imgproc.boundingRect(contour);
                    Mat roi = new Mat(mask, boundingBox);

                    List<MatOfPoint> innerContours = new ArrayList<>();
                    Imgproc.findContours(roi.clone(), innerContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

                    for (MatOfPoint innerContour : innerContours) {
                        double subArea = Imgproc.contourArea(innerContour);
                        if (subArea < minContourArea || subArea > maxContourArea) continue;

                        RotatedRect subRect = Imgproc.minAreaRect(new MatOfPoint2f(innerContour.toArray()));
                        double subAspectRatio = Math.min(subRect.size.width, subRect.size.height) / Math.max(subRect.size.width, subRect.size.height);
                        if (subAspectRatio < minAspectRatio || subAspectRatio > maxAspectRatio) continue;

                        subRect.center.x += boundingBox.x;
                        subRect.center.y += boundingBox.y;

                        processDetectedRect(subRect, input, detectedRects, distances);
                    }
                    continue; // Skip the original blob
                }

                continue; // Not worth splitting either
            }

            // ✅ Only reach here if valid rect
            processDetectedRect(rect, input, detectedRects, distances);
        }

        latestRects = detectedRects.toArray(new RotatedRect[0]);
        latestDistances = distances.toArray(new double[0][]);

        // Push to dashboard for debugging
        // Send image to dashboard
        Bitmap annotatedBitmap = matToBitmap(input);
        FtcDashboard.getInstance().sendImage(annotatedBitmap);

        return input;
    }
}