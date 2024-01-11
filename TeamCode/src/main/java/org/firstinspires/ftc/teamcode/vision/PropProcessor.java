package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

public class PropProcessor implements VisionProcessor {
    public Scalar blueMin = new Scalar(0, 0, 0);
    public Scalar blueMax = new Scalar(255, 255, 255);

    public int blur = 1;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        /*
        Mat mask = new Mat(frame.size(), frame.type(), Scalar.all(0));
        //Imgproc.blur(frame, frame, new Size(blur, blur));
        //Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2HSV);
        Core.inRange(frame, blueMin, blueMax, mask);
        Core.bitwise_and(frame, mask, frame);

         */

        //GOAL: create bounding boxes around the spike marks
        // then determine which one is more blue or red than the others (mask maybe?)
        // relay that back to opmodebase
        // https://raw.githubusercontent.com/alan412/LearnJavaForFTC/master/LearnJavaForFTC.pdf
        // https://deltacv.gitbook.io/eocv-sim/vision-portal/introduction-to-visionportal/using-visionportal-within-opmodes

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
