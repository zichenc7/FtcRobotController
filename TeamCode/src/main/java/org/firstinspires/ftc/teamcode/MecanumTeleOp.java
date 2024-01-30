package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.DriveConstants.ARM_MULTI;
import static org.firstinspires.ftc.teamcode.DriveConstants.DRIVE_MULTI;
import static org.firstinspires.ftc.teamcode.DriveConstants.TURN_MULTI;
import static org.firstinspires.ftc.teamcode.DriveConstants.WRIST_INCREMENT;
import static org.firstinspires.ftc.teamcode.DriveConstants.deadband;
import static org.firstinspires.ftc.teamcode.DriveConstants.percentDifference;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

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
        drive.setPoseEstimate(new Pose2d());


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

        //GamepadEx gamepad1 = new GamepadEx(new Gamepad());
        //GamepadEx gamepad2 = new GamepadEx(new Gamepad());

        while (opModeIsActive()) {
            double y = -deadband(gamepad1.left_stick_y) * DRIVE_MULTI * speedMulti; // Remember, Y stick value is reversed
            double x = -deadband(gamepad1.left_stick_x) * DRIVE_MULTI * speedMulti;
            double rx = -deadband(gamepad1.right_stick_x) * TURN_MULTI * speedMulti;
            double armUp = deadband(gamepad2.left_trigger) * ARM_MULTI;
            double armDown = -deadband(gamepad2.right_trigger) * ARM_MULTI;

            if (gamepad2.start && gamepad2.x) {
                drive.imu.resetYaw();
                poseStorage = drive.getPoseEstimate();
                drive.setPoseEstimate(new Pose2d());
                sleep(200);
                telemetry.log().add(runtime + " IMU reset");
            }
            if (gamepad2.back) {
               launchDrone();
               telemetry.log().add(runtime + " Drone launched");
            }
            if (gamepad2.a) {
                clawModify();
                telemetry.log().add(runtime + " Claw opened/closed");
            }
            if (gamepad1.y) {
                if (!(speedMulti == 0.25)){
                    speedMulti = 0.25;
                } else {
                    speedMulti = 1;
                }
                telemetry.log().add(runtime + "Drive multiplier changed to " + speedMulti);
                sleep(200);
            }


            if (gamepad2.dpad_down) {
                armIntakeMacro();
                telemetry.log().add(runtime + "set to pick up position");
                sleep(200);
            }
            if (gamepad2.dpad_up) {
                armOutputMacro();
                telemetry.log().add(runtime + " set to release position");
                sleep(200);
            }

            if (gamepad2.right_bumper){
                wristModify(-WRIST_INCREMENT);
            } else if (gamepad2.left_bumper) {
                wristModify(WRIST_INCREMENT);
            }

            //double[] mp = motorOp(y, x, rx);
            double clawPos = clawOp();
            double wristPos = wristOp();
            int armPos = armOp(armUp, armDown);

            drive.update(); // updates the dashboard with the robot's position.

            Pose2d poseEstimate = drive.getPoseEstimate();
            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            // The reason x and y are switched is since the coordinate system is rotated 90 degrees to match FTC's system.
            // x, y, and rx may need to be negative idk
            Vector2d input = new Vector2d(y, x).rotated(-poseEstimate.getHeading());

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            rx
                    )
            );






            telemetry.addData("Controller", "LX: (%.5f) LY: (%.5f)", gamepad1.left_stick_x, gamepad1.left_stick_y);
            //telemetry.addData("Drive", "FL: (%.5f) BL: (%.5f) FR: (%.5f) BR: (%.5f)", mp[0], mp[1], mp[2], mp[3]);
            telemetry.addData("Arm", "Motor target:" + armTargetPos);
            telemetry.addData("Arm", "Motor current Position:" + armPos);
            telemetry.addData("Arm", "Motor error:" + Math.abs( armPos - armTargetPos));
            telemetry.addData("Claw", "Claw position: (%.5f)", clawPos);
            telemetry.addData("Arm Servo", "position: (%.5f)", drive.wrist.getPosition());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
        visionPortal.close();
        drive.armMotor.setPower(0);
    }
}
