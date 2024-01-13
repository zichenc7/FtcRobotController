package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;

public class PropProcessor implements VisionProcessor {
    public Scalar blueMin = new Scalar(90, 50, 50);
    public Scalar blueMax = new Scalar(128, 255, 255);
    Mat hsvMat = new Mat();
    Mat mask = new Mat();
    public int blur = 1;
    double cX = 0;
    double cY = 0;
    double width = 0;
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels

    List<MatOfPoint> contours = new ArrayList<>();
    MatOfPoint largestContour;


    @Override
    public void init(int width, int height, CameraCalibration calibration) {}
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // masking the frame
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

        Mat mask = new Mat();
        Core.inRange(hsvFrame, blueMin, blueMax, mask);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);


        //determining contours
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // finding largest blob
        largestContour = findLargestContour(contours);


        //GOAL: create bounding boxes around the spike marks
        // then determine which one is more blue or red than the others (mask maybe?)
        // relay that back to opmodebase
        // https://raw.githubusercontent.com/alan412/LearnJavaForFTC/master/LearnJavaForFTC.pdf
        // https://deltacv.gitbook.io/eocv-sim/vision-portal/introduction-to-visionportal/using-visionportal-within-opmodes
        // https://github.com/artemis18715/New-Programming-Tutorial-23-24/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OpModes/Angle_PID_Tutorial/opencv.java

        if (largestContour != null) {
            // Draw a red outline around the largest detected object
            Imgproc.drawContours(frame, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
            // Calculate the width of the bounding box
            width = calculateWidth(largestContour);

            // Display the width next to the label
            String widthLabel = "Width: " + (int) width + " pixels";
            Imgproc.putText(frame, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
            //Display the Distance
            String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
            Imgproc.putText(frame, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
            // Calculate the centroid of the largest contour
            Moments moments = Imgproc.moments(largestContour);
            cX = moments.get_m10() / moments.get_m00();
            cY = moments.get_m01() / moments.get_m00();

            // Draw a dot at the centroid
            String label = "(" + (int) cX + ", " + (int) cY + ")";
            Imgproc.putText(frame, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
            Imgproc.circle(frame, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);
        }
        return null;
    }

    private double calculateWidth(MatOfPoint contour) {
        Rect boundingRect = Imgproc.boundingRect(contour);
        return boundingRect.width;
    }

    private static double getDistance(double width){
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
        return distance;
    }
    private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
        double maxArea = 0;
        MatOfPoint largestContour = null;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                largestContour = contour;
            }
        }

        return largestContour;
    }
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
    }


}
