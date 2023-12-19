package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.DriveConstants.*;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

public abstract class OpModeBase extends LinearOpMode {
    public MecanumDriveBase drive;
    private double clawPos = CLAW_MAX;
    private double armServoPos = ARM_SERVO_MIN;
    public int armTargetPos = ARM_MIN;

    // auto attributes
    private AprilTagProcessor aprilTag;
    private TfodProcessor tfod;
    public VisionPortal visionPortal;

    private static final String[] LABELS = {
            "Pixel",
    };
    // https://ftc-docs.firstinspires.org/en/latest/ftc_ml/index.html

    public void launchDrone() throws InterruptedException {
        // the position ranges from [0,1]
        // 0 is min, 0.5 is midpoint, 1 is max
        // double check to see if this is true for the servo we're using
        // modify position when needed
        drive.droneLaunchServo.setPosition(DRONE_LAUNCH_POS);
        sleep(500);
        drive.droneLaunchServo.setPosition(DRONE_REST_POS);
        sleep(500);
    }
    public double clawOp() {
        clawPos = Range.clip(clawPos, CLAW_MIN, CLAW_MAX);
        drive.clawServo.setPosition(clawPos);
        return clawPos;
    }
    public void clawModify() throws InterruptedException {
        if (clawPos == CLAW_MIN){
            clawPos = CLAW_MAX;
        } else if (clawPos == CLAW_MAX){
            clawPos = CLAW_MIN;
        }
        sleep(200);
    }

    public double armServoOp() {
        armServoPos = Range.clip(armServoPos, ARM_SERVO_MIN, ARM_SERVO_MAX);
        drive.armServo.setPosition(armServoPos);
        return armServoPos;
    }
    public void armServoModify(double increment){
        armServoPos += increment;
    }

    public int armOp(double armUp, double armDown) {
        double armPower = armUp + armDown;
        int curPos = drive.armMotor.getCurrentPosition();

        if (curPos <= ARM_MIN && armPower < 0) {
            armTargetPos = ARM_MIN;
        } else if (curPos >= ARM_MAX && armPower > 0) {
            armTargetPos = ARM_MAX;
        } else if(armPower != 0) {
            drive.armMotor.setPower(armPower);
            armTargetPos = curPos;
            return curPos;
        } else if(!(percentDifference(armTargetPos, curPos) > ARM_READJUSTMENT_TOLERANCE)){
            return curPos;
        }

        drive.armMotor.setTargetPosition(armTargetPos);
        armModeSwitch();
        return curPos;
    }


    /*
    public void armModify(double armUp, double armDown) {
        armMotorPos += (int) ((armUp + armDown) * ARM_SCALE);
        telemetry.addData("aaa", armMotorPos + " " + " " + armUp + " " + " " + armDown);
    }

    */

    private void armModeSwitch(){
        drive.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.armMotor.setPower(ARM_POWER);
        while (drive.armMotor.isBusy() && opModeIsActive()) {}
        drive.armMotor.setPower(0);
        drive.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void armOutputMacro() {
        armTargetPos = ARM_POS_OUTPUT;
        armServoPos = ARM_SERVO_OUTPUT;
        drive.armMotor.setTargetPosition(armTargetPos);
        drive.armServo.setPosition(armServoPos);
        armModeSwitch();
    }
    public void armIntakeMacro() {
        armTargetPos = ARM_POS_INTAKE;
        armServoPos = ARM_SERVO_INTAKE;
        drive.armMotor.setTargetPosition(armTargetPos);
        drive.armServo.setPosition(armServoPos);
        armModeSwitch();
    }

    public double[] motorOp(double y, double x, double rx) {
        double botHeading = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        drive.leftFront.setPower(frontLeftPower);
        drive.leftRear.setPower(backLeftPower);
        drive.rightFront.setPower(frontRightPower);
        drive.rightRear.setPower(backRightPower);

        return new double[]{frontLeftPower, backLeftPower, frontRightPower, backRightPower};
    }

    public static class CameraStreamProcessor implements VisionProcessor, CameraStreamSource {
        private final AtomicReference<Bitmap> lastFrame =
                new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(frame, b);
            lastFrame.set(b);
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                                Object userContext) {
            // do nothing
        }

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }
    }
    public void initWebcam(HardwareMap hardwareMap){
        final CameraStreamProcessor dashboard = new CameraStreamProcessor();
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        tfod = new TfodProcessor.Builder()
                // use ASSET_NAME if it is an asset?
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        builder.addProcessors(dashboard, aprilTag, tfod);
        visionPortal = builder.build();

        FtcDashboard.getInstance().startCameraStream(dashboard, 0);

        // Exposure Settings:

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            sleep(50);
        }
        exposureControl.setExposure((long)EXPOSURE_MS, TimeUnit.MILLISECONDS);
        sleep(20);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(GAIN);
        sleep(20);
    }

    public Pose2d driveToTargetTag(int desiredTagId) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        boolean targetFound = false;
        AprilTagDetection targetTag = null;
        Pose2d tagPose = new Pose2d();

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (detection.id == desiredTagId) {
                    targetFound = true;
                    targetTag = detection;
                    break;
                }
            }
        }

        if(targetFound){
            // these values are completely unrelated to creating a pose
            // math might be needed to be done.
            // look at this https://ftc-docs.firstinspires.org/en/latest/apriltag/understanding_apriltag_detection_values/understanding-apriltag-detection-values.html

            double  rangeError      = (targetTag.ftcPose.range - DESIRED_DISTANCE);
            double percentRange = rangeError / targetTag.ftcPose.range;

            // this needs a lot of testing

            double headingError    = targetTag.ftcPose.bearing;
            double horizontalError = targetTag.ftcPose.x * percentRange;
            double verticalError = targetTag.ftcPose.y * percentRange;


            tagPose = new Pose2d(horizontalError, verticalError, Math.toRadians(headingError));
        }
        return tagPose;
    }
    public int getPropPos(){
        return 0;
    }


}

