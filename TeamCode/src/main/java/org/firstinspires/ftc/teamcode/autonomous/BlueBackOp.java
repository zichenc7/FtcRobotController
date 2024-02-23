package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.TeamColour;

/*
main work will be done here

 */
@Config
@Autonomous
public class BlueBackOp extends AutonomousOpBase {
    // 1 tile is 24' by 24'
    // on the dashboard, y increases to the left, x increases upwards


    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        initialization(TeamColour.BLUE, StartPosition.BACK);

        Pose2d startPose = new Pose2d(START_X + startPosition.offset * 2, START_Y * teamColour.direction, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        waitForStart();
        if (isStopRequested()) return;
        sleep(3000);
        init2();
        visionPortal.close();
        telemetry.addData("HI", propPosition.toString());
        telemetry.update();

        TrajectorySequence spike = buildSpikeTraj(startPose);
        TrajectorySequence toDrop = driveToBoard(spike.end());
        TrajectorySequence drop = buildBackdropTraj(toDrop.end());
        TrajectorySequence park = buildParkTraj(drop.end());

        drive.followTrajectorySequence(spike);
        drive.followTrajectorySequence(toDrop);
        drive.followTrajectorySequence(drop);
        scoreParkMotions();
        drive.followTrajectorySequence(park);

        while (!isStopRequested() && opModeIsActive()) {
            drive.update();
        }

        // to transfer robot's position to teleOp
        // this should be the last thing called before the opmode is turned off.
        poseStorage = drive.getPoseEstimate();
    }
}
