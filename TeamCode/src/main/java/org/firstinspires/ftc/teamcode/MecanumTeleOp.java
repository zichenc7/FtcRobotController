package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.teamcode.Constants.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class MecanumTeleOp extends OpModeBase {
    public ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        // telemetry.setAutoClear(false);
        telemetry.addData("Status", "Initialized");
        telemetry.log().setCapacity(6);
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        telemetry.update();

        // motor initialization
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        //DcMotor armMotor = hardwareMap.dcMotor.get("armMotor");

        Servo droneLaunchServo = hardwareMap.servo.get("droneLaunchServo");
        Servo clawServo = hardwareMap.servo.get("clawServo");
        Servo armServo = hardwareMap.servo.get("armServo");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        setFrontLeftMotor(frontLeftMotor);
        setBackLeftMotor(backLeftMotor);
        setFrontRightMotor(frontRightMotor);
        setBackRightMotor(backRightMotor);

        //setArmMotor(armMotor);

        setDroneLaunchServo(droneLaunchServo);
        setClawServo(clawServo);
        setArmServo(armServo);

        // Retrieve the IMU from the hardware map
        IMU imu = setIMU();

        waitForStart();
        runtime.reset();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -deadband(gamepad1.left_stick_y) * DRIVE_MULTI; // Remember, Y stick value is reversed
            double x = deadband(gamepad1.left_stick_x) * DRIVE_MULTI;
            double rx = deadband(gamepad1.right_stick_x) * DRIVE_MULTI;
            //double armUp = deadband(gamepad1.left_trigger) * ARM_MULTI;
            //double armDown = deadband(-gamepad1.right_trigger) * ARM_MULTI;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.y) {
                imu.resetYaw();
                telemetry.log().add(runtime + " IMU reset");
            }
            if (gamepad1.x) {
               launchDrone();
               telemetry.log().add(runtime + " Drone launched");
               telemetry.speak("Drone Launched");
            }

            double armServoPower;
            if (gamepad1.right_bumper){
                armServoPower = 0.5 + ARM_SERVO_POWER;
            } else if (gamepad1.left_bumper) {
                armServoPower = 0.5 - ARM_SERVO_POWER;
            } else {
                armServoPower = 0.5;
            }

            if (gamepad1.dpad_left){
                clawModify(-CLAW_INCREMENT);
            } else if (gamepad1.dpad_right) {
                clawModify(CLAW_INCREMENT);
            }


            double[] motors = motorOp(imu, y, x, rx);
            double clawPos = clawOp();
            armServoPower = armServoOp(armServoPower);
            //double armPower = armOp(armUp, armDown);


            telemetry.addData("Motors", "frontLeft (%.2f)\n backLeft (%.2f)\n frontRight (%.2f)\n backRight (%.2f)", motors[0], motors[1], motors[2], motors[3]);
            //telemetry.addData("Arm", "Arm Motor power: (%.2f)", armPower);
            telemetry.addData("Claw", "Claw position: (%.5f)", clawPos);
            telemetry.addData("Arm Servo", "POWER: (%.5f)", armServoPower);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
