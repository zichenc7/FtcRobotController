package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.DriveConstants.CLAW_MIN;
import static org.firstinspires.ftc.teamcode.DriveConstants.USE_WEBCAM;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.PropPosition;
import org.firstinspires.ftc.teamcode.vision.TeamColour;

/*
main work will be done here

 */
@Config
@Autonomous

public class BlueFrontOp extends AutonomousOpBase {
    // 1 tile is 24' by 24'
    // on the dashboard, y increases to the left, x increases upwards

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        teamColour = TeamColour.BLUE;
        drive = new MecanumDriveBase(hardwareMap);

        if (USE_WEBCAM) {
            initWebcam(hardwareMap, teamColour);
        }

        // use tensorflow to identify the position

        Pose2d startPose = new Pose2d(BLUE_START_X, BLUE_START_Y, Math.toRadians(270));
        drive.setPoseEstimate(startPose);


        /*
        Trajectory trajectory = drive.trajectoryBuilder(startPose)
                .strafeLeft(40)
                .build();

         */


        waitForStart();


        if (isStopRequested()) return;

        PropPosition prop = getPropPosition();

        Trajectory traj = buildSpikePixelTraj(startPose);
        TrajectorySequence traj2 = buildSpikeTraj(traj.end(), prop);
        TrajectorySequence traj3 = buildBackdropTraj(traj2.end(), prop);
        TrajectorySequence traj4 = buildParkTraj(traj3.end());


        //
        drive.followTrajectory(traj);
        drive.followTrajectorySequence(traj2);
        drive.followTrajectorySequence(traj3);
        armOutputMacro();
        drive.clawServo.setPosition(CLAW_MIN); // open claw
        drive.followTrajectorySequence(traj4); // intake macro within this sequence

        while (!gamepad1.a && opModeIsActive()) {
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("curX", poseEstimate.getX());
            telemetry.addData("curY", poseEstimate.getY());
            //telemetryAprilTag();
        }


        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
        // to transfer robot's position to teleOp
        poseStorage = drive.getPoseEstimate();
        visionPortal.close();
    }
    /*
    public void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()

     */
}
