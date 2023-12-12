package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.DriveConstants.BLUE_START_X;
import static org.firstinspires.ftc.teamcode.DriveConstants.BLUE_START_Y;
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
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;

/*
main work will be done here

 */
@Config
@Autonomous

public class AutonomousOpBlue extends OpModeBase {

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new MecanumDriveBase(hardwareMap);

        if (USE_WEBCAM) {
            initWebcam(hardwareMap);
        }

        // use tensorflow to identify the position

        Pose2d startPose = new Pose2d(BLUE_START_X, BLUE_START_Y, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        Trajectory trajectory = drive.trajectoryBuilder(startPose)
                .strafeLeft(40)
                .addTemporalMarker(100, () -> {
                    // This marker runs two seconds into the trajectory
                    // Run your action in here!
                })
                .build();

        waitForStart();

        if (isStopRequested()) return;


        drive.followTrajectory(trajectory);

        /*
        int desiredTagId = 1;
        // do tensorflow stuff

        Pose2d aprilTagPose = trajectory.end().plus(driveToTargetTag(desiredTagId));

        Trajectory aprilTag = drive.trajectoryBuilder(trajectory.end())
                .lineToSplineHeading(aprilTagPose)
                .addDisplacementMarker(() -> {
                    // insert macros here
                })
                .build();
        drive.followTrajectory(aprilTag);
*/


        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
        visionPortal.close();
    }
}
