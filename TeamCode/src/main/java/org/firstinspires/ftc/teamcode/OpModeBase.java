package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.DriveConstants.ARM_ADJUST_POWER;
import static org.firstinspires.ftc.teamcode.DriveConstants.ARM_MACRO_POWER;
import static org.firstinspires.ftc.teamcode.DriveConstants.ARM_MAX;
import static org.firstinspires.ftc.teamcode.DriveConstants.ARM_MIN;
import static org.firstinspires.ftc.teamcode.DriveConstants.ARM_POS_INTAKE;
import static org.firstinspires.ftc.teamcode.DriveConstants.ARM_POS_OUTPUT;
import static org.firstinspires.ftc.teamcode.DriveConstants.ARM_READJUSTMENT_TOLERANCE;
import static org.firstinspires.ftc.teamcode.DriveConstants.CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.DriveConstants.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.DriveConstants.DESIRED_DISTANCE;
import static org.firstinspires.ftc.teamcode.DriveConstants.DRONE_LAUNCH_POS;
import static org.firstinspires.ftc.teamcode.DriveConstants.DRONE_REST_POS;
import static org.firstinspires.ftc.teamcode.DriveConstants.EXPOSURE_MS;
import static org.firstinspires.ftc.teamcode.DriveConstants.GAIN;
import static org.firstinspires.ftc.teamcode.DriveConstants.USE_WEBCAM;
import static org.firstinspires.ftc.teamcode.DriveConstants.WRIST_INTAKE;
import static org.firstinspires.ftc.teamcode.DriveConstants.WRIST_DOWN;
import static org.firstinspires.ftc.teamcode.DriveConstants.WRIST_UP;
import static org.firstinspires.ftc.teamcode.DriveConstants.WRIST_OUTPUT;
import static org.firstinspires.ftc.teamcode.DriveConstants.percentDifference;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.vision.PropPosition;
import org.firstinspires.ftc.teamcode.vision.PropProcessor;
import org.firstinspires.ftc.teamcode.vision.TeamColour;
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
    public static Pose2d poseStorage = new Pose2d();
    private double clawPos = CLAW_CLOSE;
    private double wristPos = WRIST_UP;
    public int armTargetPos = ARM_MIN;

    // auto attributes
    public AprilTagProcessor aprilTag;
    public TfodProcessor tfod;
    public PropProcessor prop;
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
        clawPos = Range.clip(clawPos, CLAW_OPEN, CLAW_CLOSE);
        drive.clawServo.setPosition(clawPos);
        return clawPos;
    }
    public void clawModify() throws InterruptedException {
        if (clawPos == CLAW_OPEN){
            clawPos = CLAW_CLOSE;
        } else if (clawPos == CLAW_CLOSE){
            clawPos = CLAW_OPEN;
        }
        sleep(200);
    }

    public double wristOp() {
        wristPos = Range.clip(wristPos, WRIST_UP, WRIST_DOWN);
        drive.wrist.setPosition(wristPos);
        return wristPos;
    }
    public void wristModify(double increment){
        wristPos += increment;
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
            drive.armMotor.setPower(0);
            return curPos;
        }

        drive.armMotor.setTargetPosition(armTargetPos);
        armModeSwitch(ARM_ADJUST_POWER);
        return curPos;
    }

    // with the elastics / springs / counterweights, we shouldn't need any software correction. So bellow is a replacement armOp
    /*
    public int armOp(double armUp, double armDown) {
        double armPower = armUp + armDown;
        int curPos = drive.armMotor.getCurrentPosition();

        if (curPos <= ARM_MIN && armPower < 0) {
            drive.armMotor.setPower(0);
        } else if (curPos >= ARM_MAX && armPower > 0) {
            drive.armMotor.setPower(0);
        } else {
            drive.armMotor.setPower(armPower);
        }
        armTargetPos = curPos;
        return curPos;
    }
     */

    // implement something to keep the claw always parallel with the board, maybe use ratios after the arm is past the mid point?


    private void armModeSwitch(double power){
        drive.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.armMotor.setPower(power);
        while (drive.armMotor.isBusy() && opModeIsActive()) {}
        drive.armMotor.setPower(0);
        drive.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void armOutputMacro() {
        armTargetPos = ARM_POS_OUTPUT;
        wristPos = WRIST_OUTPUT;
        drive.armMotor.setTargetPosition(armTargetPos);
        drive.wrist.setPosition(wristPos);
        armModeSwitch(ARM_MACRO_POWER);
    }
    public void armIntakeMacro() {
        armTargetPos = ARM_POS_INTAKE;
        wristPos = WRIST_INTAKE;
        drive.armMotor.setTargetPosition(armTargetPos);
        drive.wrist.setPosition(wristPos);
        armModeSwitch(ARM_MACRO_POWER);
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

    public void initWebcam(HardwareMap hardwareMap, TeamColour teamColour) {
        final CameraStreamProcessor dashboard = new CameraStreamProcessor();
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        prop = new PropProcessor(teamColour);
        /*
        tfod = new TfodProcessor.Builder()
                // use ASSET_NAME if it is an asset?
                .setModelAssetName(modelName)
                .setModelLabels(LABELS)
                .build();

         */
        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        visionPortal = builder.setCameraResolution(new Size(320, 176))
                .addProcessors(dashboard, prop)
                .build();

        while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
            sleep(20);
        }

        // Exposure Settings:

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            sleep(50);
        }
        exposureControl.setExposure((long) EXPOSURE_MS, TimeUnit.MILLISECONDS);
        sleep(20);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(GAIN);
        sleep(20);

        FtcDashboard.getInstance().startCameraStream(dashboard, 30);
    }

    public Pose2d targetTagPose(int desiredTagId) {
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

            double headingError = targetTag.ftcPose.bearing;
            double horizontalError = targetTag.ftcPose.x;
            double verticalError = targetTag.ftcPose.y * percentRange;

            // y and x potentially need to be swapped, and or reversed

            tagPose = new Pose2d(horizontalError, verticalError, Math.toRadians(headingError));
        }
        return tagPose;
    }

    public PropPosition getPropPosition() {
        return prop.getPropPosition();
    }
}

