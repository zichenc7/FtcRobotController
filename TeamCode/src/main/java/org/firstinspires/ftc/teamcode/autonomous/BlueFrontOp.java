package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
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
        initialization(TeamColour.BLUE, StartPosition.FRONT);

        Pose2d startPose = new Pose2d(START_X, START_Y, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        waitForStart();
        if (isStopRequested()) return;
        sleep(5000);
        init2();
        visionPortal.close();

        TrajectorySequence spike = buildSpikeTraj(startPose);
        TrajectorySequence drop = buildBackdropTraj(spike.end());
        TrajectorySequence park = buildParkTraj(drop.end());
        Trajectory home = drive.trajectoryBuilder(park.end()).lineToLinearHeading(startPose).build();

        drive.followTrajectorySequence(spike);
        drive.followTrajectorySequence(drop);
        scoreParkMotions();
        drive.followTrajectorySequence(park);
        drive.followTrajectory(home);

        while (!isStopRequested() && opModeIsActive()) {
            drive.update();
        }
        // to transfer robot's position to teleOp
        poseStorage = drive.getPoseEstimate();
    }
}
