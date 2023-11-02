package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public abstract class OpModeBase extends LinearOpMode {
    // constants
    public static final double v = 1;
    private static final double launchPos = 0;
    private static final double restPos = 0.5;
    private static final double clawMax = 2;
    private static final double clawMin = 0;
    //0.48
    private static final double clawSpeed = 0.1;
    private static final double deadband = 0.05;
    // fields
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    // private DcMotor armMotor;
    private Servo droneLaunchServo;
    private Servo clawServo;
    private double clawValue = 0;


    public double[] motorOp(IMU imu, double y, double x, double rx) {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

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

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        return new double[]{frontLeftPower, backLeftPower, frontRightPower, backRightPower};
    }

    public IMU setIMU() {
        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        boolean imuStatus = imu.initialize(parameters);
        telemetry.addData("IMU", "Initialized: ", imuStatus);
        return imu;
    }

    public void launchDrone() {
        // the position ranges from [0,1]
        // 0 is 0 degrees, 0.5 is 90, 1 is 180
        // double check to see if this is true for the servo we're using
        // modify position when needed
        droneLaunchServo.setPosition(OpModeBase.launchPos);
        sleep(500);
        droneLaunchServo.setPosition(OpModeBase.restPos);
    }

    public void clawOp() {
        clawServo.setPosition(clawValue);
    }
    public void clawModify(double x){
        clawValue += x;
        if (clawValue >= clawMax){
            clawValue = clawMax;
        } else if (clawValue <= clawMin) {
            clawValue = clawMin;
        }
    }

    public void setBackLeftMotor(DcMotor backLeftMotor) {
        this.backLeftMotor = backLeftMotor;
    }
    public DcMotor getBackLeftMotor() {
        return backLeftMotor;
    }
    public void setBackRightMotor(DcMotor backRightMotor) {
        this.backRightMotor = backRightMotor;
    }
    public DcMotor getBackRightMotor() {
        return backRightMotor;
    }
    public void setFrontLeftMotor(DcMotor frontLeftMotor) {
        this.frontLeftMotor = frontLeftMotor;
    }
    public DcMotor getFrontLeftMotor() {
        return frontLeftMotor;
    }
    public void setFrontRightMotor(DcMotor frontRightMotor) {
        this.frontRightMotor = frontRightMotor;
    }
    public DcMotor getFrontRightMotor() {
        return frontRightMotor;
    }
    public void setDroneLaunchServo(Servo droneLaunchServo) {
        this.droneLaunchServo = droneLaunchServo;
    }
    public Servo getDroneLaunchServo() {
        return droneLaunchServo;
    }
    public void setClawServo(Servo clawServo) {
        this.clawServo = clawServo;
    }
    public Servo getClawServo() {
        return clawServo;
    }
    public double getClawSpeed() {
        return clawSpeed;
    }

    public double deadband(double x) {
        return Math.abs(x) <= OpModeBase.deadband ? 0 : x;
    }

    public double getClawPos() {
        return clawValue;
    }

    public double v() {
        return v;
    }
}
