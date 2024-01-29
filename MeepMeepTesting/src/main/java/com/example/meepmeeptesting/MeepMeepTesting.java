
package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;



public class MeepMeepTesting {
    public static double WIDTH = 17.78;
    public static double LENGTH = 17.5;
    // always starting boardside
    public static double START_X = 24 - WIDTH / 2;
    public static double START_Y = 72 - (LENGTH / 2);
    public static double offset = -48;
    public static double bonus = -6.22;
    public static double BONUS_OFFSET = -6.22;
    public static double BASE_X = 34, BASE_Y = 36, BASE_H = 180;
    public static double SC_X = 15.11, SC_Y = 27;
    public static double SR_X = 8, SR_Y = 36, SR_H = -90;
    public static double SL_X = 29, SL_Y = 36, SL_H = 90;
    public static double DROP_X = 49, DROP_Y = 48;
    public static double DROP_CENTER = 35, DROP_OFFSET = 5;
    public static double PARK_X = 60, PARK_Y = 60;

    public static double BOARD_X1 = -55, BOARD_Y1 = 12;
    public static double BOARD_X2 = 38, BOARD_Y2 = 12;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        PropPosition position = PropPosition.RIGHT;
        TeamColour teamColour = TeamColour.BLUE;
        StartPosition startPosition = StartPosition.BACK;
        double Tdir = teamColour.direction;
        double Sdir = startPosition.direction;
        offset = startPosition.offset;
        if(startPosition.equals(StartPosition.FRONT)){ bonus = 0;}
        Pose2d start = new Pose2d(START_X + offset * 2 + bonus, Tdir * START_Y, Math.toRadians(-90 * Tdir));

        double x = BASE_X * Sdir + offset;
        double y = BASE_Y * Tdir;
        double heading = Math.toRadians(BASE_H);
        if(startPosition.equals(StartPosition.BACK)){
            heading = 0;
        }


        //Vector2d base = new Vector2d(x, y);
        Pose2d base = new Pose2d(x, y, heading);
        Vector2d ret = new Vector2d(x, y + 12 * Tdir);

        x = offset * 2;
        y = Tdir;

        if(startPosition.equals(StartPosition.BACK)){
            x -= 10;
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
        }

        Vector2d spike = new Vector2d(x, y);

        Vector2d drive1 = new Vector2d(BOARD_X1, BOARD_Y1 * Tdir);
        Vector2d drive2 = new Vector2d(BOARD_X2, BOARD_Y2 * Tdir);


        x = DROP_X;
        y = teamColour.direction;

        switch (position) {
            case CENTER:
                y *= DROP_CENTER;
                break;
            case RIGHT:
                y *= DROP_CENTER - DROP_OFFSET * Tdir;
                break;
            case LEFT:
                y *= DROP_CENTER + DROP_OFFSET * Tdir;
                break;
        }

        Vector2d board = new Vector2d(x, y);


        double finalHeading = heading;
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 24)
                .setDimensions(WIDTH, LENGTH)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(start)
                                .lineToLinearHeading(base)
                                .strafeTo(spike)
                                .strafeTo(base.vec())
                                .lineToSplineHeading(new Pose2d(BOARD_X1, BOARD_Y1, 0))
                                .strafeTo(new Vector2d(BOARD_X2, BOARD_Y2))
                                .lineToSplineHeading(new Pose2d( board, Math.toRadians(180)))
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}