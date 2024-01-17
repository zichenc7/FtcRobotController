package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;

public class PropProcessor implements VisionProcessor {
    public Scalar blueMin = new Scalar(90, 50, 50);
    public Scalar blueMax = new Scalar(128, 255, 255);
    public Scalar red1Min = new Scalar(0, 50, 50);
    public Scalar red1Max = new Scalar(15, 255, 255);
    public Scalar red2Min = new Scalar(170, 50, 50);
    public Scalar red2Max = new Scalar(180, 255, 255);
    public float xOffset = 25.5F;
    public float xFactor = 65.2F;
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
    TeamColour teamColour = TeamColour.RED;


    public PropProcessor(TeamColour teamColour) {
        this.teamColour = teamColour;
    }
    /*
    Telemetry telemetry;
    ElapsedTime runtime = new ElapsedTime();
    public PropProcessor(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

     */


    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // to prevent contours from ballooning in size and slowing down everything.
        contours = new ArrayList<>();
        // masking the frame
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_RGB2HSV);

        Mat mask = new Mat();
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));

        if (teamColour == TeamColour.RED) {
            Mat mask2 = new Mat();
            Mat combinedMask = new Mat();
            Core.inRange(hsvFrame, red1Min, red1Max, mask);
            Core.inRange(hsvFrame, red2Min, red2Max, mask2);
            Core.add(mask, mask2, combinedMask);

            Imgproc.morphologyEx(combinedMask, combinedMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(combinedMask, mask, Imgproc.MORPH_CLOSE, kernel);
            mask = combinedMask;

        } else if (teamColour == TeamColour.BLUE) {
            Core.inRange(hsvFrame, blueMin, blueMax, mask);
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);
        }

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
            Imgproc.drawContours(frame, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 1);

            // Calculate the centroid of the largest contour
            Moments moments = Imgproc.moments(largestContour);
            cX = moments.get_m10() / moments.get_m00();
            cY = moments.get_m01() / moments.get_m00();

            // Draw a dot at the centroid
            //String label = "(" + (int) cX + ", " + (int) cY + ")";
            //Imgproc.putText(frame, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
            Imgproc.circle(frame, new Point(cX, cY), 2, new Scalar(0, 255, 0), -1);
        }
        return null;
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
        float scale = scaleBmpPxToCanvasPx;
        Paint paint = new Paint();
        paint.setColor(Color.WHITE);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(scaleCanvasDensity * 4);
        for (int i = 1; i <= 3; i++) {
            if (i == 2) continue;
            float x = (xFactor * i + xOffset) * scale;
            canvas.drawLine(x, 0, x, 240 * scale, paint);
        }
    }

    public PropPosition getPropPosition() {
        PropPosition position = null;
        if (largestContour == null) {
            return PropPosition.CENTER;
        }
        Moments moments = Imgproc.moments(largestContour);
        double x = moments.get_m10() / moments.get_m00();

        if (x <= xOffset + xFactor) {
            position = PropPosition.LEFT;
        } else if (x >= xOffset + 3 * xFactor) {
            position = PropPosition.RIGHT;
        } else {
            position = PropPosition.CENTER;
        }
        return position;
    }


}
