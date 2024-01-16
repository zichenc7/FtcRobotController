package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.DriveConstants.CLAW_MAX;
import static org.firstinspires.ftc.teamcode.DriveConstants.USE_WEBCAM;
import static org.firstinspires.ftc.teamcode.DriveConstants.WRIST_MIN;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.OpModeBase;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.vision.PropPosition;
import org.firstinspires.ftc.teamcode.vision.TeamColour;

@Config
public abstract class AutonomousOpBase extends OpModeBase {
    public double RED_START_X = 11.375;
    public double RED_START_Y = -63;
    public double BLUE_START_X = 11.375;
    public double BLUE_START_Y = 63;
    public double SPIKE_CENTER_Y = 32.76;
    public double SPIKE_LR_Y = 35;
    public double SPIKE_LR_X = 8;
    public double SPIKE_LR_HEAD = 30;
    private double SPIKE_HEAD = Math.toRadians(SPIKE_LR_HEAD);
    public double BD_START_X = 39;
    public double BD_CENTER = 24;
    // I think it's slightly less than 7
    public double BD_OFFSET = 7;
    public double P_BACKUP = 1;


    TeamColour teamColour;
    // all base cases are blueFront oriented

    public Trajectory buildSpikePixelTraj(Pose2d start) {
        return drive.trajectoryBuilder(start)
                .splineTo(new Vector2d(start.getX(), 48 * teamColour.direction), start.getHeading())
                .build();
    }

    public TrajectorySequence buildSpikeTraj(Pose2d start, PropPosition position) {
        double startX = start.getX();
        double startY = start.getY();
        double startHeading = start.getHeading();
        double dir = teamColour.direction;
        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(start);

        switch (position) {
            case CENTER:
                builder.splineTo(new Vector2d(startX, SPIKE_CENTER_Y * dir), startHeading);
                break;
            case LEFT:
                builder.splineTo(new Vector2d(startX + SPIKE_LR_X * dir, SPIKE_LR_Y * dir), startHeading - SPIKE_HEAD * dir);
                break;
            case RIGHT:
                builder.splineTo(new Vector2d(startX - SPIKE_LR_X * dir, SPIKE_LR_Y * dir), startHeading + SPIKE_HEAD * dir);
                break;
        }
        builder.waitSeconds(1) // might be unnecessary
                .setReversed(true) // might actually reverse all the trajectories :fear:
                .splineTo(start.vec(), start.getHeading() * -1);
        return builder.build();
    }

    // when / if april tags don't work
    public TrajectorySequence buildBackdropTraj(Pose2d start, PropPosition position) {
        double startX = start.getX();
        double startY = start.getY();
        double startHeading = start.getHeading();
        double dir = teamColour.direction;
        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(start);

        builder.lineToLinearHeading(new Pose2d(startX + BD_START_X, startY, Math.toRadians(180)));

        switch (position) {
            case CENTER:
                builder.strafeTo(new Vector2d(startX, startY - BD_CENTER * dir));
                break;
            case LEFT:
                builder.strafeTo(new Vector2d(startX, startY - (BD_CENTER + BD_OFFSET) * dir));
                break;
            case RIGHT:
                builder.strafeTo(new Vector2d(startX, startY - (BD_CENTER - BD_OFFSET) * dir));
                break;
        }
        return builder.build();
    }

    public TrajectorySequence buildParkTraj(Pose2d start) {
        double startX = start.getX();
        double startY = start.getY();
        double startHeading = start.getHeading();
        double dir = teamColour.direction;
        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(start);
        builder.back(1)
                // .addTemporalMarker(() -> armIntakeMacro()) // comment out when only testing drive
                .waitSeconds(2) // potentially unnecessary
                .strafeTo(new Vector2d(startX + P_BACKUP, 60 * dir))
                .strafeTo(new Vector2d(60, 60 * dir));
        return builder.build();
    }

    public void initialization() {
        drive = new MecanumDriveBase(hardwareMap);

        if (USE_WEBCAM) {
            initWebcam(hardwareMap, teamColour);
        }

        drive.wrist.setPosition(WRIST_MIN);
        drive.clawServo.setPosition(CLAW_MAX);

    }

}
