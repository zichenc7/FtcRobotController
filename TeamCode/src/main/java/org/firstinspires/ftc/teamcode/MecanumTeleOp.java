package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class MecanumTeleOp extends OpModeBase {
    public ElapsedTime runtime = new ElapsedTime();
    /*
    public void armOp(double armUp, double armDown) {
        // armPower for future tuning
        // this has not been tested
        double armPower = armUp - armDown;
        armMotor.setPower(armPower);
        telemetry.addData("Arm", "power: (%.2f", armPower);
    }
*/
    @Override
    public void runOpMode() throws InterruptedException {
        // telemetry.setAutoClear(false);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // motor initialization
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        // armMotor = hardwareMap.dcMotor.get("armMotor");
        Servo droneLaunchServo = hardwareMap.servo.get("droneLaunchServo");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        setFrontLeftMotor(frontLeftMotor);
        setBackLeftMotor(backLeftMotor);
        setFrontRightMotor(frontRightMotor);
        setBackRightMotor(backRightMotor);
        setDroneLaunchServo(droneLaunchServo);

        // Retrieve the IMU from the hardware map
        IMU imu = setIMU();

        waitForStart();
        runtime.reset();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            // double armUp = gamepad1.left_trigger;

            // double armDown = -gamepad1.right_trigger;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.y) {
                imu.resetYaw();
                telemetry.addData("Status", "IMU reset");
                telemetry.update();
            }


            if (gamepad1.x) {
               launchDrone();
               telemetry.addData("Status", "Drone launched");
               telemetry.update();
            }


            double[] motors = motorOp(imu, y, x, rx);
            telemetry.addData("Motors", "frontLeft (%.2f)\n backLeft (%.2f)\n frontRight (%.2f)\n backRight (%.2f)", motors[0], motors[1], motors[2], motors[3]);
            // armOp(armUp, armDown);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
