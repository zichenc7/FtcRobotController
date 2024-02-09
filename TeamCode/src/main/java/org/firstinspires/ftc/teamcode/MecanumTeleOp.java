package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.DriveConstants.ARM_MULTI;
import static org.firstinspires.ftc.teamcode.DriveConstants.DRIVE_MULTI;
import static org.firstinspires.ftc.teamcode.DriveConstants.WRIST_INCREMENT;
import static org.firstinspires.ftc.teamcode.DriveConstants.deadband;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// 192.168.43.1:8080/dash
@Config
@TeleOp
public class MecanumTeleOp extends OpModeBase {

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

        Gamepad g1cur = new Gamepad();
        Gamepad g1prev= new Gamepad();
        Gamepad g2cur= new Gamepad();
        Gamepad g2prev = new Gamepad();

        while (opModeIsActive()) {
            g1prev.copy(g1cur);
            g2prev.copy(g2cur);
            g1cur.copy(gamepad1);
            g2cur.copy(gamepad2);

            double y = -deadband(gamepad1.left_stick_y) * DRIVE_MULTI; // Remember, Y stick value is reversed
            double x = -deadband(gamepad1.left_stick_x) * DRIVE_MULTI;
            double rx = -deadband(gamepad1.right_stick_x) * DRIVE_MULTI;
            double armUp = deadband(gamepad2.left_trigger) * ARM_MULTI;
            double armDown = -deadband(gamepad2.right_trigger) * ARM_MULTI;

            if (gamepad1.start && !g1prev.start) {
                drive.imu = hardwareMap.get(IMU.class, "imu");
                IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                        DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
                drive.imu.initialize(parameters);
                drive.imu.resetYaw();
                poseStorage = drive.getPoseEstimate();
                drive.setPoseEstimate(new Pose2d());
                telemetry.log().add(runtime + " IMU reset");
            }
            if (gamepad2.back && !g2prev.back) {
               launchDrone();
               telemetry.log().add(runtime + " Drone launched");
            }
            if (gamepad2.a && !g2prev.a) {
                clawModify();
                telemetry.log().add(runtime + " Claw opened/closed");
            }
            if (gamepad1.y && !g1prev.y) {
                if (DRIVE_MULTI != 0.5) {
                    DRIVE_MULTI = 0.5;
                    telemetry.log().add(runtime + " Slow Drive Deactivated: " + DRIVE_MULTI*2);
                } else {
                    DRIVE_MULTI = 0.25;
                    telemetry.log().add(runtime + " Slow Drive Activated: " + DRIVE_MULTI*2);
                }
            }
            if (gamepad1.b && !g1prev.b) {
                if (DRIVE_MULTI != 0.5) {
                    DRIVE_MULTI = 0.5;
                    telemetry.log().add(runtime + " Fast Drive Deactivated: " + DRIVE_MULTI*2);
                } else {
                    DRIVE_MULTI = 1;
                    telemetry.log().add(runtime + " Fast Drive Activated: " + DRIVE_MULTI*2);
                }
            }

            if (gamepad2.dpad_down && !g2prev.dpad_down) {
                armIntakeMacro();
                telemetry.log().add(runtime + " Arm Intake");
            }
            if (gamepad2.dpad_up && !g2prev.dpad_up) {
                armOutputMacro();
                telemetry.log().add(runtime + " Arm Output");
            }
            if (gamepad2.x && !g2prev.x) {
                wristUp();
                telemetry.log().add(runtime + " Wrist Up");
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
            telemetry.addData("IMU", drive.imu.getConnectionInfo());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
        visionPortal.close();
        drive.armMotor.setPower(0);
    }
}
