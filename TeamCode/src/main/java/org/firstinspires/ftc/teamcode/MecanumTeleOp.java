package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.teamcode.DriveConstants.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;


// 192.168.43.1:8080/dash
@Config
@TeleOp
public class MecanumTeleOp extends OpModeBase {
    public ElapsedTime runtime = new ElapsedTime();
    public double speedMulti = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new MecanumDriveBase(hardwareMap);

        // telemetry.setAutoClear(false);
        telemetry.addData("Status", "Initialized");
        telemetry.log().setCapacity(6);
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        telemetry.update();

        waitForStart();
        runtime.reset();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
            double y = -deadband(gamepad1.left_stick_y) * DRIVE_MULTI * speedMulti; // Remember, Y stick value is reversed
            double x = deadband(gamepad1.left_stick_x) * DRIVE_MULTI * speedMulti;
            double rx = deadband(gamepad1.right_stick_x) * TURN_MULTI * speedMulti;
            double armUp = deadband(gamepad1.left_trigger) * ARM_MULTI;
            double armDown = -deadband(gamepad1.right_trigger) * ARM_MULTI;

            if (gamepad1.start) {
                drive.imu.resetYaw();
                sleep(200);
                telemetry.log().add(runtime + " IMU reset");
            }
            if (gamepad1.back) {
               launchDrone();
               telemetry.log().add(runtime + " Drone launched");
            }
            if (gamepad1.a) {
                clawModify();
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
                armServoModify(-ARM_SERVO_INCREMENT);
            } else if (gamepad1.right_bumper) {
                armServoModify(ARM_SERVO_INCREMENT);
            }

            motorOp(y, x, rx);
            double clawPos = clawOp();
            double armServoPos = armServoOp();
            int armPos = armOp(armUp, armDown);

            telemetry.addData("Arm", "Motor target:" + armTargetPos);
            telemetry.addData("Arm", "Motor current Position:" + armPos);
            telemetry.addData("Claw", "Claw position: (%.5f)", clawPos);
            telemetry.addData("Arm Servo", "position: (%.5f)", drive.armServo.getPosition());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
        visionPortal.close();
    }
}
