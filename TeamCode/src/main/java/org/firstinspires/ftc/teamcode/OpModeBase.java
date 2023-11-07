package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import static org.firstinspires.ftc.teamcode.Constants.*;

public abstract class OpModeBase extends LinearOpMode {
    // fields
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotor armMotor;
    private Servo droneLaunchServo;
    private Servo clawServo;
    private Servo armServo;
    private double clawPos = CLAW_MIN;
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
        // 0 is min, 0.5 is midpoint, 1 is max
        // double check to see if this is true for the servo we're using
        // modify position when needed
        droneLaunchServo.setPosition(DRONE_LAUNCH_POS);
        sleep(500);
        droneLaunchServo.setPosition(DRONE_REST_POS);
        sleep(500);
    }
    public double clawOp() {
        clawPos = Range.clip(clawPos, CLAW_MIN, CLAW_MAX);
        clawServo.setPosition(clawPos);
        return clawPos;
    }
    public void clawModify(double increment){
        clawPos += increment;
    }

    public double armServoOp(double power){
        armServo.setPosition(power);
        return power;
    }
    // We need to set this back to like the claw servo control method
    // this is because we need to constantly readjust the claw angle due to gravity pulling it down
    public double deadband(double x) {
        return Math.abs(x) <= DEAD_BAND ? 0 : x;
    }

    public double armOp(double armUp, double armDown) {
        // armPower for future tuning
        // this has not been tested direction may be messed up.
        double armPower = armUp - armDown;
        armMotor.setPower(armPower);
        return armPower;
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
    public double getClawPos() {
        return clawPos;
    }
    public void setArmServo(Servo armServo) {
        this.armServo = armServo;
    }
    public Servo getArmServo() {
        return armServo;
    }

    public void setArmMotor(DcMotor armMotor) {
        this.armMotor = armMotor;
    }
    public DcMotor getArmMotor() {
        return armMotor;
    }

}
