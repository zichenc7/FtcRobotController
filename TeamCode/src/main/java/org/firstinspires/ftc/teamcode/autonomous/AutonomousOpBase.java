package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.DriveConstants.ARM_MACRO_POWER;
import static org.firstinspires.ftc.teamcode.DriveConstants.ARM_MIN;
import static org.firstinspires.ftc.teamcode.DriveConstants.ARM_POS_INTAKE;
import static org.firstinspires.ftc.teamcode.DriveConstants.CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.DriveConstants.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.DriveConstants.USE_WEBCAM;
import static org.firstinspires.ftc.teamcode.DriveConstants.WRIST_MID;
import static org.firstinspires.ftc.teamcode.DriveConstants.WRIST_UP;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;

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
    public static long TIME_OUT = 5000;
    // always starting boardside
    public static double START_X = 12;
    public static double START_Y = 72 - (LENGTH / 2);
    // FIXME determine this offset
    public static double SPIKE_BACK_OFFSET = -15.5;
    public static double BASE_X = 34, BASE_Y = 36, BASE_H = 180;
    public static double SC_X = 13, SC_Y = 33;
    public static double CENTER_BASE_X = 12, CENTER_BASE_Y = 36;
    public static double SR_X = 9, SR_Y = 36, SR_H = -90;
    public static double SL_X = 31, SL_Y = 36, SL_H = 90;
    public static double DROP_X = 39, DROP_Y = 48;
    public static double DROP_CENTER = 34, DROP_OFFSET = 7;
    public static double PARK_X = 60, PARK_Y = 60;
    public static double BOARD_X1 = -55, BOARD_Y1 = 12;
    public static double BOARD_X2 = 38, BOARD_Y2 = 12;
    TeamColour teamColour;
    double Tdir;
    double Sdir;
    StartPosition startPosition;
    PropPosition propPosition;

    int dropArm = 4005;
    double dropWrist = 0.17;


    // all base cases are blueFront oriented


    public TrajectorySequence buildSpikeTraj(Pose2d start) {
        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(start);

        double heading = Math.toRadians(BASE_H);
        double x = BASE_X * Sdir + startPosition.offset;
        double y = BASE_Y * Tdir;
        if (startPosition == StartPosition.BACK) {
            heading = 0;
        }
        Pose2d potentialBase = new Pose2d(x, y, heading);
        Pose2d base = potentialBase;
        if (propPosition == PropPosition.CENTER) {
            base = new Pose2d(startPosition.offset * 2 + CENTER_BASE_X, CENTER_BASE_Y * Tdir, start.getHeading());
        } else {
            builder.lineToLinearHeading(base);
        }

        x = startPosition.offset * 2;
        y = Tdir;
        // put spike pixels boardside
        // TODO: fix autonomous coordinates


        if (startPosition == StartPosition.BACK && propPosition != PropPosition.CENTER) {
            x += SPIKE_BACK_OFFSET;
        }

        switch (propPosition) {
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

        builder.lineTo(new Vector2d(x, y))
                .addTemporalMarker(() -> {
                    drive.wrist.setPosition(WRIST_UP);
                })
                .waitSeconds(1)
                .lineToLinearHeading(base);

        if (propPosition == PropPosition.CENTER) builder.lineToLinearHeading(potentialBase);
        return builder.build();
    }

    // drive.wrist.setPosition(WRIST_MAX);
    // when / if april tags don't work
    public TrajectorySequence buildBackdropTraj(Pose2d start) {
        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(start);
        double x = DROP_X;
        double y = Tdir;
        switch (propPosition) {
            case CENTER:
                y *= DROP_CENTER;
                break;
            case RIGHT:
                y *= DROP_CENTER - DROP_OFFSET;
                break;
            case LEFT:
                y *= DROP_CENTER + DROP_OFFSET;
                break;
        }
        builder.lineToSplineHeading(new Pose2d(new Vector2d(x, y), Math.toRadians(180)));
        return builder.build();
    }

    public TrajectorySequence driveToBoard(Pose2d start) {
        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(start);
        builder.strafeTo(new Vector2d(BOARD_X1, BOARD_Y1 * teamColour.direction))
                .strafeTo(new Vector2d(BOARD_X2, BOARD_Y2 * teamColour.direction));
        return builder.build();
    }

    public TrajectorySequence buildParkTraj(Pose2d start) {
        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(start);
        builder.back(1);
        //.addTemporalMarker(() -> armIntakeMacro()) // comment out when only testing drive
        //.waitSeconds(2); // potentially unnecessary

        double y = Tdir;
        if (startPosition == StartPosition.BACK) {
            y *= PARK_Y - 48;
        } else {
            y *= PARK_Y;
        }

        builder.strafeTo(new Vector2d(DROP_X, y))
                .strafeTo(new Vector2d(PARK_X, y));

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

    public void init2() {
        drive.armMotor.setTargetPosition(ARM_POS_INTAKE);
        drive.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.armMotor.setPower(0.5);
        armMacroClose();
        propPosition = getPropPosition();

        if (teamColour == TeamColour.RED) {
            if (propPosition == PropPosition.LEFT) {
                propPosition = PropPosition.RIGHT;
            } else if (propPosition == PropPosition.RIGHT) {
                propPosition = PropPosition.LEFT;
            }
        }


        drive.wrist.setPosition(WRIST_MID);
        sleep(100);
        drive.armMotor.setTargetPosition(ARM_MIN);
        drive.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.armMotor.setPower(0.5);
        armMacroClose();
    }

    public void armMacroClose() {
        while (drive.armMotor.isBusy() && opModeIsActive()) {
        }
        drive.armMotor.setPower(0);
        drive.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void scoreParkMotions() throws InterruptedException {
        drive.armMotor.setTargetPosition(dropArm);
        drive.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.armMotor.setPower(ARM_MACRO_POWER);
        drive.wrist.setPosition(dropWrist);
        armMacroClose();
        drive.clawServo.setPosition(CLAW_OPEN);
        sleep(500);
        drive.armMotor.setTargetPosition(ARM_MIN);
        drive.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.armMotor.setPower(ARM_MACRO_POWER);
        armMacroClose();
    }

}
