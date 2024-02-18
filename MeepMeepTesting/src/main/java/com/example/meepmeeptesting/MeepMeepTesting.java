package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        int caz = 0;
        double x_stanga = -24;
        double y_stanga = 29;
        double x_dreapta = 7.3;
        double y_dreapta = 31.8;
        double forward_center = 31;
        double slide_dr = 15;
        double back_dreapta = 5;
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(0), 12.85)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(10, 62, Math.toRadians(270))) // start!!
//                .forward(15)
                                        .strafeLeft(slide_dr)
                                        .lineToLinearHeading(new Pose2d(x_dreapta, y_dreapta, Math.toRadians(180)))
                                        .back(back_dreapta)
                                        // .strafeRight(40)
                                        //.forward(50)
                                        //.lineTo(new Vector2d(47, 34.8))
                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}