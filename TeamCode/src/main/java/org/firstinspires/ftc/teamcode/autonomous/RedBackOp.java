package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
        teamColour = TeamColour.RED;
        initialization();

        Pose2d startPose = new Pose2d(RED_START_X - 48, RED_START_Y, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;


        while (!isStopRequested() && opModeIsActive()) {
        }
        // to transfer robot's position to teleOp
        // this should be the last thing called before the opmode is turned off.
        poseStorage = drive.getPoseEstimate();
        visionPortal.close();
    }
}
