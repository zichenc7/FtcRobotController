
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
    public static double BASE_X = 15.11, BASE_Y = 36;
    public static double SC_X = 15.11, SC_Y = 30;
    public static double SR_X = 2, SR_Y = 36, SR_H = -90;
    public static double SL_X = 19.11, SL_Y = 36, SL_H = 90;
    public static double DROP_X = 49, DROP_Y = 48;
    public static double DROP_CENTER = 35, DROP_OFFSET = 7;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        double dir = 1;
        offset = 0;
        bonus = 0;
        Pose2d start = new Pose2d(START_X + offset + bonus, dir * START_Y, Math.toRadians(-90 * dir));

        double x = BASE_X + offset + bonus;
        double y = BASE_Y * dir;

        Vector2d base = new Vector2d(x, y);
        Vector2d ret = new Vector2d(x, y + 12 * dir);

        x = offset;
        y = dir;
        double heading = dir;
        PropPosition position = PropPosition.LEFT;

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
        }

        Vector2d spike = new Vector2d(x, y);


        double finalHeading = Math.toRadians(heading);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 24)
                .setDimensions(WIDTH, LENGTH)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(start)
                                .splineTo(base, start.getHeading())
                                .waitSeconds(1)
                                .turn(finalHeading)
                                .strafeTo(spike)
                                .setReversed(true)
                                .splineTo(ret, start.getHeading() * -1)
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}