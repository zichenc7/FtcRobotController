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
public class BlueBackOp extends AutonomousOpBase {
    // 1 tile is 24' by 24'
    // on the dashboard, y increases to the left, x increases upwards


    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        initialization(TeamColour.BLUE, StartPosition.BACK);

        Pose2d startPose = new Pose2d(START_X + startPosition.offset * 2 + BONUS_OFFSET, START_Y * teamColour.direction, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;
        sleep(5000);
        init2();
        telemetry.addData("HI", propPosition.toString());
        telemetry.update();

        Trajectory preSpike = buildSpikePixelTraj(startPose);
        TrajectorySequence spike = buildSpikeTraj(preSpike.end(), propPosition);
        TrajectorySequence toDrop = driveToBoard(spike.end());
        TrajectorySequence drop = buildBackdropTraj(toDrop.end(), propPosition);
        TrajectorySequence park = buildParkTraj(drop.end());

        drive.followTrajectory(preSpike);
        drive.followTrajectorySequence(spike);
        drive.followTrajectorySequence(toDrop);
        drive.followTrajectorySequence(drop);
        armOutputMacro();
        armMacroClose();
        drive.followTrajectorySequence(park);

        //Trajectory park = drive.trajectoryBuilder(startPose)
            //    .strafeLeft(40)
             //   .build();

        //drive.followTrajectory(park);
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
