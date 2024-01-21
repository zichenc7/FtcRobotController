package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.DriveConstants.CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.DriveConstants.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.DriveConstants.USE_WEBCAM;
import static org.firstinspires.ftc.teamcode.DriveConstants.WRIST_DOWN;
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
    public static double BASE_X = 15.11, BASE_Y = 36;
    public static double SC_X = 15.11, SC_Y = 30.01;
    public static double SR_X = 14, SR_Y = 36, SR_H = -90;
    public static double SL_X = 21, SL_Y = 32.7, SL_H = 90;
    public static double DROP_X = 49, DROP_Y = 48;
    public static double DROP_CENTER = 35, DROP_OFFSET = 5;
    public static double PARK_X = 60, PARK_Y = 60;


    TeamColour teamColour;
    double dir;
    StartPosition startPosition;
    // all base cases are blueFront oriented

    public Trajectory buildSpikePixelTraj(Pose2d start) {
        double x = BASE_X + startPosition.offset;
        if (startPosition.equals(StartPosition.BACK)) {
            x += BONUS_OFFSET;
        }
        double y = BASE_Y * teamColour.direction;
        return drive.trajectoryBuilder(start)
                .splineTo(new Vector2d(x, y), start.getHeading())
                .build();
    }

    public TrajectorySequence buildSpikeTraj(Pose2d start, PropPosition position) {
        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(start);
        double x = startPosition.offset;
        double y = teamColour.direction;
        double heading = teamColour.direction;
        // put spike pixels boardside
        switch (position) {
            case CENTER:
                x += SC_X;
                y *= SC_Y;
                heading = 0;
                break;
            case RIGHT:
                x += SR_X;
                y *= SR_Y;
                heading *= SR_H;
                break;
            case LEFT:
                x += SL_X;
                y *= SL_Y;
                heading *= SL_H;
                break;
        }
        Vector2d base = new Vector2d(start.getX(), start.getY() + 12 * dir);

        builder.turn(Math.toRadians(heading))
                .strafeTo(new Vector2d(x, y))
                .addTemporalMarker(() ->{
                    drive.wrist.setPosition(WRIST_UP);});
                //.waitSeconds(1)
                //.setReversed(true)
                //.splineTo(base, start.getHeading() * -1);
        return builder.build();
    }
    // drive.wrist.setPosition(WRIST_MAX);
    // when / if april tags don't work
    public TrajectorySequence buildBackdropTraj(Pose2d start, PropPosition position) {
        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(start);
        double x = DROP_X;
        double y = teamColour.direction;
        builder.lineToLinearHeading(new Pose2d(DROP_X, DROP_Y, Math.toRadians(180)));
        switch (position) {
            case CENTER:
                y *= DROP_CENTER;
                builder.strafeTo(new Vector2d(x, y));
                break;
            case RIGHT:
                y *= DROP_CENTER - DROP_OFFSET * dir;
                builder.strafeTo(new Vector2d(x, y));
                break;
            case LEFT:
                y *= DROP_CENTER + DROP_OFFSET * dir;
                builder.strafeTo(new Vector2d(x, y));
                break;
        }
        return builder.build();
    }

    public TrajectorySequence buildParkTraj(Pose2d start) {
        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(start);
        builder.back(1)
                // .addTemporalMarker(() -> armIntakeMacro()) // comment out when only testing drive
                .waitSeconds(2) // potentially unnecessary
                .strafeTo(new Vector2d(DROP_X, PARK_Y * dir))
                .strafeTo(new Vector2d(PARK_X, PARK_Y * dir));
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
        dir = teamColour.direction;
    }

}
