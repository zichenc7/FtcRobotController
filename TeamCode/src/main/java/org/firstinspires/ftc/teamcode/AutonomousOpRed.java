package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.DriveConstants.RED_START_X;
import static org.firstinspires.ftc.teamcode.DriveConstants.RED_START_Y;
import static org.firstinspires.ftc.teamcode.DriveConstants.USE_WEBCAM;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.nio.file.attribute.FileTime;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous
public class AutonomousOpRed extends OpModeBase {
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        if (USE_WEBCAM) {
            initWebcam(hardwareMap);
        }
        drive = new MecanumDriveBase(hardwareMap);


        Pose2d startPose = new Pose2d(RED_START_X, RED_START_Y, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        Trajectory trajectory = drive.trajectoryBuilder(startPose)
                .strafeRight(40)
                .addTemporalMarker(100, () -> {
                    // This marker runs two seconds into the trajectory
                    // Run your action in here!
                })
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectory);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
        visionPortal.close();
    }
}
