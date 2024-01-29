package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.DriveConstants.CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.DriveConstants.USE_WEBCAM;
import static org.firstinspires.ftc.teamcode.DriveConstants.WRIST_UP;

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
    public static double WIDTH = 17.78;
    public static double LENGTH = 17.5;
    // always starting boardside
    public static double START_X = 24 - WIDTH / 2;
    public static double START_Y = 72 - (LENGTH / 2);
    public static double BONUS_OFFSET = -6.22;
    public static double BASE_X = 30, BASE_Y = 36, BASE_H = 180;
    public static double SC_X = 15.11, SC_Y = 27;
    public static double SR_X = 8, SR_Y = 36, SR_H = -90;
    public static double SL_X = 29, SL_Y = 36, SL_H = 90;
    public static double SPIKE_BACK_OFFSET = -10;
    public static double DROP_X = 49, DROP_Y = 48;
    public static double DROP_CENTER = 35, DROP_OFFSET = 5;
    public static double PARK_X = 60, PARK_Y = 60;

    public static double BOARD_X1 = -48, BOARD_Y1 = 12;
    public static double BOARD_X2 = 38, BOARD_Y2 = 12;


    TeamColour teamColour;
    double Tdir;
    double Sdir;
    StartPosition startPosition;
    // all base cases are blueFront oriented

    public Trajectory buildSpikePixelTraj(Pose2d start) {
        double heading = Math.toRadians(BASE_H);
        if(startPosition.equals(StartPosition.BACK)){
            heading = 0;
        }

        double x = BASE_X * Sdir + startPosition.offset;
        double y = BASE_Y * Tdir;
        Pose2d base = new Pose2d(x, y, heading);
        return drive.trajectoryBuilder(start)
                .lineToLinearHeading(base).build();
    }

    public TrajectorySequence buildSpikeTraj(Pose2d start, PropPosition position) {
        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(start);
        double x = startPosition.offset * 2;
        double y = teamColour.direction;
        // put spike pixels boardside
        if(startPosition.equals(StartPosition.BACK)){
            x += SPIKE_BACK_OFFSET;
        }

        switch (position) {
            case CENTER:
                x += SC_X;
                y *= SC_Y;
                break;
            case RIGHT:
                x += SR_X;
                y *= SR_Y;
                break;
            case LEFT:
                x += SL_X;
                y *= SL_Y;
                break;
        }

        builder.strafeTo(new Vector2d(x, y))
                .addTemporalMarker(() ->{
                    drive.wrist.setPosition(WRIST_UP);})
                .waitSeconds(1)
                .strafeTo(start.vec());
        return builder.build();
    }
    // drive.wrist.setPosition(WRIST_MAX);
    // when / if april tags don't work
    public TrajectorySequence buildBackdropTraj(Pose2d start, PropPosition position) {
        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(start);
        double x = DROP_X;
        double y = teamColour.direction;
        switch (position) {
            case CENTER:
                y *= DROP_CENTER;
                builder.strafeTo(new Vector2d(x, y));
                break;
            case RIGHT:
                y *= DROP_CENTER - DROP_OFFSET * Tdir;
                builder.strafeTo(new Vector2d(x, y));
                break;
            case LEFT:
                y *= DROP_CENTER + DROP_OFFSET * Tdir;
                builder.strafeTo(new Vector2d(x, y));
                break;
        }
        return builder.build();
    }

    public TrajectorySequence driveToBoard(Pose2d start){
        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(start);
        double x = DROP_X;
        double y = teamColour.direction;
        return builder.build();
    }

    public TrajectorySequence buildParkTraj(Pose2d start) {
        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(start);
        builder.back(1)
                // .addTemporalMarker(() -> armIntakeMacro()) // comment out when only testing drive
                .waitSeconds(2) // potentially unnecessary
                .strafeTo(new Vector2d(DROP_X, PARK_Y * Tdir))
                .strafeTo(new Vector2d(PARK_X, PARK_Y * Tdir));
        return builder.build();
    }

    public void initialization(TeamColour teamColour, StartPosition startPosition) {
        drive = new MecanumDriveBase(hardwareMap);

        if (USE_WEBCAM) {
            initWebcam(hardwareMap, teamColour);
        }

        drive.wrist.setPosition(WRIST_UP);
        drive.clawServo.setPosition(CLAW_CLOSE);
        this.teamColour = teamColour;
        this.startPosition = startPosition;
        Tdir = teamColour.direction;
        Sdir = startPosition.direction;
    }

}
