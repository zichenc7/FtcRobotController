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
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous
public class RedBackOp extends AutonomousOpBase {
    // 1 tile is 24' by 24'
    // on the dashboard, y increases to the left, x increases upwards


    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        initialization(TeamColour.RED, StartPosition.BACK);


        Pose2d startPose = new Pose2d(START_X + startPosition.offset * 2, START_Y * teamColour.direction, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        sleep(5000);
        PropPosition prop = getPropPosition();

        Trajectory traj = buildSpikePixelTraj(startPose);
        TrajectorySequence traj2 = buildSpikeTraj(traj.end(), prop);

        //Trajectory park = drive.trajectoryBuilder(startPose)
              //  .strafeRight(40)
              //  .build();

        //drive.followTrajectory(park);


        drive.followTrajectory(traj);
        drive.followTrajectorySequence(traj2);
        //drive.followTrajectorySequence(traj3);
        //armOutputMacro();
        //drive.clawServo.setPosition(CLAW_MIN); // open claw
        //drive.followTrajectorySequence(traj4); // intake macro within this sequence

        //drive.followTrajectory(traj);

        while (!isStopRequested() && opModeIsActive()) {
            drive.update();
        }

        // to transfer robot's position to teleOp
        // this should be the last thing called before the opmode is turned off.
        poseStorage = drive.getPoseEstimate();
        visionPortal.close();
    }
}
