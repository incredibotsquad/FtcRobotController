/*
 * Copyright (c) 2024 Phil Malone
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.drive.opmode;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.Console;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.CompletableFuture;

/*
 * This OpMode illustrates how to use a video source (camera) to locate specifically colored regions
 *
 * Unlike a "color sensor" which determines the color of an object in the field of view, this "color locator"
 * will search the Region Of Interest (ROI) in a camera image, and find any "blobs" of color that match the requested color range.
 * These blobs can be further filtered and sorted to find the one most likely to be the item the user is looking for.
 *
 * To perform this function, a VisionPortal runs a ColorBlobLocatorProcessor process.
 *   The ColorBlobLocatorProcessor process is created first, and then the VisionPortal is built to use this process.
 *   The ColorBlobLocatorProcessor analyses the ROI and locates pixels that match the ColorRange to form a "mask".
 *   The matching pixels are then collected into contiguous "blobs" of pixels.  The outer boundaries of these blobs are called its "contour".
 *   For each blob, the process then creates the smallest possible rectangle "boxFit" that will fully encase the contour.
 *   The user can then call getBlobs() to retrieve the list of Blobs, where each Blob contains the contour and the boxFit data.
 *   Note: The default sort order for Blobs is ContourArea, in descending order, so the biggest contours are listed first.
 *
 * To aid the user, a colored boxFit rectangle is drawn on the camera preview to show the location of each Blob
 * The original Blob contour can also be added to the preview.  This is helpful when configuring the ColorBlobLocatorProcessor parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */


@Config
@TeleOp(name = "Concept: Vision Color-Locator", group = "Concept")
public class ConceptVisionColorLocatorREALWith2Servos extends LinearOpMode
{

    public double angleRad;
    Point startPoint = null;
    Point endPoint = null;
    private final double sampleLength = 3.5;
    public static double sampleHeight = 0;
    public static double cameraHeight = 13;
    public static double cameraMountAngle = 45; //175
    public static double imageCenterY = 360;
    public static double verticalFOV = 43; //37.8
    public static double image_height = 720;
    public static double angleDeg;
    public static double sigma = 0;




    public static double getDepth(double sampleY){

        double degreesPerPixel = verticalFOV / image_height;

        // Reverse the sign here: angle increases as sampleY increases
        double pixelOffsetAngle = (sampleY - (image_height / 2.0)) * degreesPerPixel;

        // Now we SUBTRACT because image Y increases downward
        double totalAngle = cameraMountAngle - pixelOffsetAngle;

        // Clamp to avoid tan(near 0) or tan(>89)
        totalAngle = Math.max(1.0, Math.min(89.0, totalAngle));

        return (cameraHeight / Math.tan(Math.toRadians(totalAngle))) + 8;

//        double pixelOffsetAngle = (sampleY - (image_height / 2.0)) * (verticalFOV / image_height);
//        double totalAngle = cameraMountAngle - pixelOffsetAngle;
//        double distance = cameraHeight / Math.tan(Math.toRadians(totalAngle));
//        return distance * 2;

        //return cameraHeight / Math.tan(Math.toRadians(cameraMountAngle + ((sampleY - (image_height/2)) * (verticalFOV/image_height))));
    }
    public static double armLength = 10;
    public double getHorizontalServoPosition(double xpos) {
        double depthFromClaw = Math.sqrt((armLength*armLength) - (xpos * xpos));
        return (1 - Math.toDegrees(Math.atan(depthFromClaw/ xpos))/180);
    };

    /**
     * @param y1 the y value in pixels of one of the corners of the sample
     * @param y2 another y value in pixels of the other corner
     */
    public double getAngle(double y1, double y2){
        double y1Depth = getDepth(y1);
        double y2Depth = getDepth(y2);
        double VerticalDistance = Math.abs(y1 - y2);
        return Math.toDegrees(Math.atan(VerticalDistance/sampleLength));
    };
    public double servo_position_from_claw;

    public static double add = 0.3;
    private static final Logger log = LoggerFactory.getLogger(ConceptVisionColorLocatorREALWith2Servos.class);
    public static int erode = 0;

    public static int dilate = 0;

    private static double addGreaterThan = 0.5;
    private static double addLessThan = 0;

    public static int minArea = 5000;

    public static int maxArea = 1000000000;

    public static int minAspectRatio = 0;

    public static int maxAspectRatio = 100000000;

    private Servo servo;
    public double depth;
    // Fill out the variable widthInches and lengthInches, with how many inches that the camera sees.
    private static double widthInches=9;

    private static double lengthInches=6.75;

    private double SampleY;

    private double SampleX;
    private Point samplePoint;
    private double Degrees;
    private int length;
    private double Radians;
    private List SeenObjects;
    private double samplePixelY;
    private double samplePixelX;
    private static String Ydirection;
    private static String Xdirection;
    public static Object First;
    private static Point intPoint;

    private double Point1x;

    private double Point1y;

    private double Point2x;

    private double Point2y;

    private double slope;

    private VisionPortal visionPortal;

    private static double position;
    private Servo AngleServo;

    List<Point> blobData = new ArrayList<>();
    List<Point> pixelList = new ArrayList<>();
    List<Double> SampleDepth = new ArrayList<>();
    double tolerance = 5;
    double angleRange = 1;
    double angle;

    @Override
    public void runOpMode()
    {
       AngleServo = hardwareMap.get(Servo.class, "angle");
       servo = hardwareMap.get(Servo.class, "servo");

        /* Build a "Color Locator" vision processor based on the ColorBlobLocatorProcessor class.
         * - Specify the color range you are looking for.  You can use a predefined color, or create you own color range
         *     .setTargetColorRange(ColorRange.BLUE)                      // use a predefined color match
         *       Available predefined colors are: RED, BLUE YELLOW GREEN
         *     .setTargetColorRange(new ColorRange(ColorSpace.YCrCb,      // or define your own color match
         *                                           new Scalar( 32, 176,  0),
         *                                           new Scalar(255, 255, 132)))
         *
         * - Focus the color locator by defining a RegionOfInterest (ROI) which you want to search.
         *     This can be the entire frame, or a sub-region defined using:
         *     1) standard image coordinates or 2) a normalized +/- 1.0 coordinate system.
         *     Use one form of the ImageRegion class to define the ROI.
         *         ImageRegion.entireFrame()
         *         ImageRegion.asImageCoordinates(50, 50,  150, 150)  100x100 pixel square near the upper left corner
         *         ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5)  50% width/height square centered on screen
         *
         * - Define which contours are included.
         *     You can get ALL the contours, or you can skip any contours that are completely inside another contour.
         *        .setContourMode(ColorBlobLocatorProcessor.ContourMode.ALL_FLATTENED_HIERARCHY)  // return all contours
         *        .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)            // exclude contours inside other contours
         *        note: EXTERNAL_ONLY helps to avoid bright reflection spots from breaking up areas of solid color.
         *
         * - turn the display of contours ON or OFF.  Turning this on helps debugging but takes up valuable CPU time.
         *        .setDrawContours(true)
         *
         * - include any pre-processing of the image or mask before looking for Blobs.
         *     There are some extra processing you can include to improve the formation of blobs.  Using these features requires
         *     an understanding of how they may effect the final blobs.  The "pixels" argument sets the NxN kernel size.
         *        .setBlurSize(int pixels)    Blurring an image helps to provide a smooth color transition between objects, and smoother contours.
         *                                    The higher the number of pixels, the more blurred the image becomes.
         *                                    Note:  Even "pixels" values will be incremented to satisfy the "odd number" requirement.
         *                                    Blurring too much may hide smaller features.  A "pixels" size of 5 is good for a 320x240 image.
         *        .setErodeSize(int pixels)   Erosion removes floating pixels and thin lines so that only substantive objects remain.
         *                                    Erosion can grow holes inside regions, and also shrink objects.
         *                                    "pixels" in the range of 2-4 are suitable for low res images.
         *        .setDilateSize(int pixels)  Dilation makes objects more visible by filling in small holes, making lines appear thicker,
         *                                    and making filled shapes appear larger. Dilation is useful for joining broken parts of an
         *                                    object, such as when removing noise from an image.
         *                                    "pixels" in the range of 2-4 are suitable for low res images.
         */
        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))  // search central 1/4 of camera view all 0.5
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .setErodeSize(erode)
                .setDilateSize(dilate)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)            // exclude contours inside other contours
                .build();

        /*
         * Build a vision portal to run the Color Locator process.
         *
         *  - Add the colorLocator process created above.
         *  - Set the desired video resolution.
         *      Since a high resolution will not improve this process, choose a lower resolution that is
         *      supported by your camera.  This will improve overall performance and reduce latency.
         *  - Choose your video source.  This may be
         *      .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))  .....   for a webcam
         *  or
         *      .setCamera(BuiltinCameraDirection.BACK)    ... for a Phone Camera
         */
        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(1280, 720))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        // WARNING:  To be able to view the stream preview on the Driver Station, this code runs in INIT mode.
        while (opModeIsActive() || opModeInInit())
        {
           // telemetry.addData("preview on/off", "... Camera Stream\n");

            // Read the current list
            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();





            /*
             * The list of Blobs can be filtered to remove unwanted Blobs.
             *   Note:  All contours will be still displayed on the Stream Preview, but only those that satisfy the filter
             *          conditions will remain in the current list of "blobs".  Multiple filters may be used.
             *
             * Use any of the following filters.
             *
             * ColorBlobLocatorProcessor.Util.filterByArea(minArea, maxArea, blobs);
             *   A Blob's area is the number of pixels contained within the Contour.  Filter out any that are too big or small.
             *   Start with a large range and then refine the range based on the likely size of the desired object in the viewfinder.
             *
             * ColorBlobLocatorProcessor.Util.filterByDensity(minDensity, maxDensity, blobs);
             *   A blob's density is an indication of how "full" the contour is.
             *   If you put a rubber band around the contour you would get the "Convex Hull" of the contour.
             *   The density is the ratio of Contour-area to Convex Hull-area.
             *
             * ColorBlobLocatorProcessor.Util.filterByAspectRatio(minAspect, maxAspect, blobs);
             *   A blob's Aspect ratio is the ratio of boxFit long side to short side.
             *   A perfect Square has an aspect ratio of 1.  All others are > 1
             */

            // Get the latest image from the vision pipeline
//            Mat inputMat = new Mat();  // This should be where the camera frame is stored
//            Mat grayMat = new Mat();   // Placeholder for grayscale image

            // Convert input image to grayscale
            //Imgproc.cvtColor(inputMat, grayMat, Imgproc.COLOR_RGB2GRAY);


           // ColorBlobLocatorProcessor.Util.filterByArea(minArea, maxArea, blobs);  // filter out very small blobs.

//            ColorBlobLocatorProcessor.Util.filterByAspectRatio(minAspectRatio, maxAspectRatio, blobs);
            /*
             * The list of Blobs can be sorted using the same Blob attributes as listed above.
             * No more than one sort call should be made.  Sorting can use ascending or descending order.
             *     ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);      // Default
             *     ColorBlobLocatorProcessor.Util.sortByDensity(SortOrder.DESCENDING, blobs);
             *     ColorBlobLocatorProcessor.Util.sortByAspectRatio(SortOrder.DESCENDING, blobs);
             */

           // telemetry.addLine(" Area Density Aspect  Center");

            /*Imgproc.cvtColor(inputMat, grayMat, Imgproc.COLOR_RGB2GRAY);

// Apply thresholding (optional, to enhance contrast)
            Imgproc.threshold(grayMat, grayMat, 100, 255, Imgproc.THRESH_BINARY);

// Define the kernel for erosion/dilation
            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new org.opencv.core.Size(3, 3));

// Apply erosion (removes noise, shrinks objects)
            Imgproc.erode(grayMat, grayMat, kernel, new Point(-1, -1), 1);

// Apply dilation (fills gaps, enlarges objects)
            Imgproc.dilate(grayMat, grayMat, kernel, new Point(-1, -1), 1);*/
//            telemetry.addData("Found List of Objects: ", blocks.length);
            // Display the size (area) and center location for each Blob.
            for(ColorBlobLocatorProcessor.Blob b : blobs) {
                RotatedRect boxFit = b.getBoxFit();
                if (!blobs.isEmpty()) {

                    // Determine facing direction based on angle
//                    String direction;
//                    if (angleDeg >= 315 || angleDeg < 45) {
//                        direction = "Right";
//                    } else if (angleDeg >= 45 && angleDeg < 135) {
//                        direction = "Down";
//                    } else if (angleDeg >= 135 && angleDeg < 225) {
//                        direction = "Left";
//                    } else {
//                        direction = "Up";
//                    }
//                    telemetry.addData("Moving Servo to position: ", angleDeg/180);


//
//                if (boxFit != null) {
//
//                    Point1x = boxFit.boundingRect().x + boxFit.boundingRect().width; //top Right
//
//                    Point1y = boxFit.boundingRect().y;
//
//                    Point2x = boxFit.boundingRect().x; //Bottom Left
//
//                    Point2y = boxFit.boundingRect().y + boxFit.boundingRect().height;
//
//                    if (Point2x > Point1x) { //angled to the right
//                        slope = 1;
//                    } else if (Point1x > Point2x) {
//                        slope = -1;
//                    }
//
//                    telemetry.addData("Boxfit status", "boxfit found");
//
//                }
//                else{
//                    telemetry.addData("Boxfit status", "no boxfit found");
//                    telemetry.update();
//                }
//                    if (boxFit.angle > 180){
//                        telemetry.addData("changing angle of servo.", "Removing it by 180");
//                        position = (boxFit.angle /300);
//                        servo.setPosition(position);
//                    }else{
//                        position = (boxFit.angle /300);
//                        servo.setPosition(position);
//                    }

                    // Ensure angle is within valid range
//                    double angle = Math.max(0, Math.min(angleDeg, 300));

                    //double angle = angleDeg;
                    // Convert angle to servo position (0.0 to 1.0)
//                    if(position > 180){
//                        position = ((angle - 90)/300);
//                    }
//                    else{
//                        position = ((angle) / 300.0);
//                    }
//
//                    if (position > 1){
//                        position = 1;
//                    }
//                    servo.setPosition(position);
//
//                    telemetry.addData("Servo Target Angle:", angle);


                    //math to turn sample positions into inches on a xy plane
//                    //y coordinate of sample
//                    SampleY = (boxFit.center.y/lengthPixels) * lengthInches;
//                    SampleX = (boxFit.center.x/widthPixels) * widthInches;
                    //amount to move
                    SampleDepth.add(getDepth(360 - boxFit.center.y));
                    CompletableFuture.supplyAsync(() -> {
                        try {
                            SampleDepth.sort(Comparator.comparingDouble(Depth -> Depth));
                        } catch (Error e) {
                            return e;
                        }
                        return null;
                    });
                    SampleX = (boxFit.center.x) / 1280 * widthInches;
                    SampleY = (720 - boxFit.center.y) / 720 * lengthInches;
                    samplePixelX = (boxFit.center.x - 640) / 640;
                    samplePixelY = boxFit.center.y;
                    pixelList.add(new Point(samplePixelX, samplePixelY));
                    CompletableFuture.supplyAsync(() -> {
                        try {
                            pixelList.sort(Comparator.comparingDouble(point -> point.x * point.x + point.y * point.y));
                        } catch (Error error) {
                            return error;
                        }
                        return null;
                    });
                    if (boxFit.center.x - 640 < 0) {
                        Xdirection = "Right";
                    } else if (boxFit.center.x - 640 > 0) {
                        Xdirection = "Left";
                    } else {


                        Xdirection = "On Sample";
                    }
                    if (boxFit.center.y - 360 < 0) {
                        Ydirection = "down";
                    } else if (boxFit.center.y - 360 > 0) {
                        Ydirection = "up";
                    } else {
                        Ydirection = "On Sample";
                    }

                    samplePoint = new Point(SampleX, SampleY);
                    intPoint = new Point((int) SampleX, (int) SampleY);
                    blobData.add(samplePoint);
                    CompletableFuture.supplyAsync(() -> {
                        try {
                            blobData.sort(Comparator.comparingDouble(point -> point.x * point.x + point.y * point.y));
                        } catch (Error error) {
                            return error;
                        }
                        return null;
                    });
                    length = blobData.toArray().length;
                    //get the degrees to move the servo to the closest sample
                    Radians = Math.atan(blobData.get(0).y / blobData.get(0).x);
                    Degrees = (-Math.toDegrees(Radians)) + 90;
                    //servo.setPosition(Math.abs(Degrees/180));
                   // if (Math.abs(pixelList.get(0).x - boxFit.center.x) < tolerance) {
                        Point[] vertices = new Point[4];
                        boxFit.points(vertices); // Get the 4 corner points of the rectangle

                        // Calculate edge lengths
                        double edge1 = Math.hypot(vertices[0].x - vertices[1].x, vertices[0].y - vertices[1].y);
                        double edge2 = Math.hypot(vertices[1].x - vertices[2].x, vertices[1].y - vertices[2].y);


                        telemetry.addData("Got defualt angle: ", boxFit.angle);
                        if(edge1 > edge2){
                            telemetry.addData("Got edge: ", "1");
                            startPoint = vertices[0];
                            endPoint = vertices[1];
//                            telemetry.addData("Got angle: ", angle);
                        }
                        else{
                            telemetry.addData("Got edge: ", "2");
                            startPoint = vertices[1];
                            endPoint = vertices[2];
//                            telemetry.addData("Got angle: ", angle);
                        }
//                        telemetry.addData("Point 1: ", vertices[0]);
//                        telemetry.addData("Point 2: ", vertices[1]);
//                        telemetry.addData("Point 3: ", vertices[2]);
//                        telemetry.addData("Point 4: ", vertices[3]);
//                        telemetry.addData("Got length of 1 edge: ", edge1);
//                        telemetry.addData("Got length of another edge: ", edge2);
                        angleRad = Math.atan((endPoint.y - startPoint.y) / (endPoint.x - startPoint.x));
//                        telemetry.addData()
                        double angleDeg = Math.toDegrees(angleRad);
//                        if(edge1 > edge2){
//                            angleDeg += 90;
//                        }
                        telemetry.addData("Before angle Deg: ", angleDeg);

                        angleDeg = (angleDeg + 360) % 180;
                        telemetry.addData("After angle deg: ", angleDeg);
                        // Normalize angle to [0, 360)
//                        if (angleDeg >= 270) {
//                            angleDeg -= 180;
//                        }
                        //angle = angleDeg;

//                        if (boxFit.center.y > 360) {
//                            angleDeg += sigma;
//                            telemetry.addData("Got Real Angle: ", angleDeg);
//                        }

                        if (!Double.isNaN(angleDeg)) {

                            if (angleDeg < 10 || angleDeg > 170) {
                                AngleServo.setPosition(0.125);
                            } else {
                                AngleServo.setPosition(1 - (angleDeg / 180));
                            }
                        }
                        telemetry.addData("Got angle: ", angle);

//
//                        if (angle >= 90 - 22.5 && angle < 90 + 22.5) {
//                            AngleServo.setPosition(0.5);
//                            telemetry.addData("Orientation:", "LEFT");
//
//                        } else if (angle >= 90 + 22.5 && angle <= 180) {
//                            AngleServo.setPosition(0.75);
//                            telemetry.addData("Orientation:", "STRAIGHT");
//
//                        } else if (angle >= 0.1 && angle < 90 - 22.5) {
//                            AngleServo.setPosition(0.25);//0.75
//                            telemetry.addData("Orientation:", "RIGHT");
//
//                        } else {
//                            AngleServo.setPosition(1);
//                        }
//                        else if (angle >= 135 && angle <= 180) {
//                            AngleServo.setPosition(1);
//                            telemetry.addData("Orientation:", "HORIZONTAL");
//                        }
                    //}
                    // kumbaya
//                    if ((startPoint != null) && (endPoint != null)) {
//                        angleDeg = getAngle(startPoint.y, endPoint.y);
//                    }
                    depth = getDepth(360 - boxFit.center.y);
                    telemetry.addData("got angle: ", angleDeg);
                    servo_position_from_claw = getHorizontalServoPosition(SampleX);
                    if (!Double.isNaN(servo_position_from_claw)) {
                        servo.setPosition(servo_position_from_claw);
                    }
                    if (Double.isNaN(servo_position_from_claw)) {
                        telemetry.addData("Returned Servo Position is: ", "Not a number");
                    }
                    telemetry.addData("Angle Rad: ", angleRad);
                    telemetry.addData("Moving claw servo: ", servo_position_from_claw);
                    telemetry.addData("Old servo pos: ", Math.abs(Degrees / 180));

                    telemetry.addData("got depth: ", depth);

                    telemetry.addData("BlobData: ", blobData);

//                    telemetry.addData("Distance: ", intPoint);

                    telemetry.addData("got length: ", length);

                    telemetry.addData("Got radians: ", Radians);

                    telemetry.addData("Got closest Point: ", blobData.get(0));

                    telemetry.addData("Need to move servo: ", Degrees);

                    telemetry.addData("Need to move servo position: ", Degrees / 180);

                    telemetry.addData("Got blob x: ", boxFit.center.x);

                    telemetry.addData("Got closest sample X pixels: ", pixelList.get(0).x);

                    // telemetry.addData("Got First Blob: ", First);
//                    telemetry.addData("Length Inches: ", lengthInches);
//                    telemetry.addData("Width Inches: ", widthInches);
//                    telemetry.addData("AMOUNT TO MOVE X: ", SampleX);
//                    telemetry.addData("AMOUNT TO MOVE Y: ", SampleY);
//                    telemetry.addData("X direction to move: ", Xdirection);
//                    telemetry.addData("Y direction to move: ", Ydirection);
//                    telemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d)",
//                            b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y));
//                    telemetry.addData("INCREDIBOTS", boxFit.angle);
//                    telemetry.addData("origin x: ", Point2x);
//                    telemetry.addData("origin y: ", Point2y);
//                    telemetry.addData("Corner x: ", Point1x);
//                    telemetry.addData("Corner y: ", Point1y);
//                    telemetry.addData("SLOPE:", slope);
//                    telemetry.addData("Angle (deg):", angleDeg);
//                    telemetry.addData("Facing Direction:", direction);
//                    telemetry.addData("Servo Position:", position);
                }
                SampleDepth.clear();
                pixelList.clear();
                blobData.clear();
                telemetry.update();
                b.getContourPoints();
            }
            sleep(50);
        }
    }
}
