package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.teamcode.DriveConstants.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;


// 192.168.43.1:8080/dash
@Config
@TeleOp
public class MecanumTeleOp extends LinearOpMode {
    public ElapsedTime runtime = new ElapsedTime();
    public double speedMulti = 1;
    public static boolean opModeIsActive;
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        // telemetry.setAutoClear(false);
        telemetry.addData("Status", "Initialized");
        telemetry.log().setCapacity(6);
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        telemetry.update();

        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap);
        GamepadEx gamepadEx = new GamepadEx(gamepad1);

        waitForStart();
        runtime.reset();

        if (isStopRequested()) {
            opModeIsActive = false;
            return;
        }

        while (opModeIsActive()) {
            double y = -deadband(gamepad1.left_stick_y) * DRIVE_MULTI * speedMulti; // Remember, Y stick value is reversed
            double x = deadband(gamepad1.left_stick_x) * DRIVE_MULTI * speedMulti;
            double rx = deadband(gamepad1.right_stick_x) * TURN_MULTI * speedMulti;
            double armUp = deadband(gamepad1.left_trigger) * ARM_MULTI;
            double armDown = deadband(-gamepad1.right_trigger) * ARM_MULTI;

            if (gamepad1.start && gamepad1.dpad_up) {
                drive.imu.resetYaw();
                sleep(200);
                telemetry.log().add(runtime + " IMU reset");
            }
            if (gamepad1.back) {
               drive.launchDrone();
               telemetry.log().add(runtime + " Drone launched");
            }
            if (gamepad1.a) {
                drive.clawModify();
                telemetry.log().add(runtime + " Claw opened/closed");
            }
            if (gamepad1.y) {
                if (speedMulti == 1){
                    speedMulti = 0.25;
                } else {
                    speedMulti = 1;
                }
                telemetry.log().add(runtime + "Drive multiplier changed to " + speedMulti);
                sleep(200);
            }

            /*
            if (gamepad1.x) {
                drive.armIntakeMacro();
                telemetry.log().add(runtime + "set to pick up position");
                sleep(200);
            }
            if (gamepad1.y) {
                drive.armOuttakeMacro();
                telemetry.log().add(runtime + " set to release position");
                sleep(200);
            }

             */


            if (gamepad1.left_bumper){
                drive.armServoModify(-ARM_SERVO_INCREMENT);
            } else if (gamepad1.right_bumper) {
                drive.armServoModify(ARM_SERVO_INCREMENT);
            }


            drive.motorOp(y, x, rx);
            double clawPos = drive.clawOp();
            double armServoPos = drive.armServoOp();
            double armPos = drive.armOp(armUp, armDown);

            telemetry.addData("Arm", "Arm Motor position: (%.2f)", armPos);
            telemetry.addData("Claw", "Claw position: (%.5f)", clawPos);
            telemetry.addData("Arm Servo", "position: (%.5f)", armServoPos);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
        drive.visionPortal.close();
    }

}
