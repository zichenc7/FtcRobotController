package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.DriveConstants.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.DriveConstants.WRIST_DOWN;

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

        PropPosition prop = getPropPosition();
        prop = PropPosition.CENTER;
        telemetry.addData("prop", "Detection" + prop.toString());
        telemetry.update();

        Trajectory traj = buildSpikePixelTraj(startPose);
        TrajectorySequence traj2 = buildSpikeTraj(traj.end(), prop);
        //TrajectorySequence traj3 = buildBackdropTraj(traj2.end(), prop);
        //TrajectorySequence traj4 = buildParkTraj(traj3.end());


        /*
        Trajectory park = drive.trajectoryBuilder(startPose)
                        .strafeLeft(40)
                        .build();

         drive.followTrajectory(park);

         */
        drive.clawServo.setPosition(CLAW_OPEN);
        drive.wrist.setPosition(WRIST_DOWN);
        drive.followTrajectory(traj);
        drive.followTrajectorySequence(traj2);

        //drive.followTrajectorySequence(traj3);
        //armOutputMacro();
        //drive.clawServo.setPosition(CLAW_MIN); // open claw
        //drive.followTrajectorySequence(traj4); // intake macro within this sequence

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("curX", poseEstimate.getX());
            telemetry.addData("curY", poseEstimate.getY());
            telemetry.addData("prop", "Detection" + getPropPosition().toString());
            telemetry.update();
        }
        // to transfer robot's position to teleOp
        poseStorage = drive.getPoseEstimate();
        visionPortal.close();
    }
}
