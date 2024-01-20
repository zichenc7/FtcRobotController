package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.PropPosition;
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

        Pose2d startPose = new Pose2d(START_X + startPosition.offset, START_Y * teamColour.direction, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

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
        //drive.followTrajectorySequence(traj3);
        //armOutputMacro();
        //drive.clawServo.setPosition(CLAW_MIN); // open claw
        //drive.followTrajectorySequence(traj4); // intake macro within this sequence

        //drive.followTrajectory(traj);

        while (!gamepad1.a && opModeIsActive()) {
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("curX", poseEstimate.getX());
            telemetry.addData("curY", poseEstimate.getY());
            telemetry.addData("prop", "Detection" + getPropPosition().toString());
            telemetry.update();
        }

        Trajectory ret = drive.trajectoryBuilder(traj2.end())
                .splineTo(startPose.vec(), startPose.getHeading())
                .build();

        drive.followTrajectory(ret);

        while (!isStopRequested() && opModeIsActive()) ;

        // to transfer robot's position to teleOp
        // this should be the last thing called before the opmode is turned off.
        poseStorage = drive.getPoseEstimate();
        visionPortal.close();
    }
}
