package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(120, 200, Math.toRadians(360), Math.toRadians(720), 15)
                .setDimensions(13, 17.5)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(23.5, -61, Math.toRadians(90)))
                        .waitSeconds(0.5)
                        .splineTo(new Vector2d(30, -31), Math.toRadians(56))
                        .forward(20)
                        .splineTo(new Vector2d(47.5, 0), Math.toRadians(90))
                        .waitSeconds(0.5)
                        .back(53)
                        .lineTo(new Vector2d(47.5, -25))
                        .splineTo(new Vector2d(58, 0), Math.toRadians(90))
                        .waitSeconds(0.5)
                        .back(53)
                        .lineTo(new Vector2d(58, -25))
                        .splineTo(new Vector2d(64, 0), Math.toRadians(90))
                        .waitSeconds(0.5)
                        .back(53)
                        .lineTo(new Vector2d(45, -55))
                        .waitSeconds(0.5)
                        .lineTo(new Vector2d(45, -61))
                        .waitSeconds(1)

                        .forward(2)
                        .splineTo(new Vector2d(39.5, -50), Math.toRadians(167))
                        .forward(20)
                        .splineTo(new Vector2d(8, -35), Math.toRadians(90))
                        .forward(2.5)
                        .waitSeconds(2)

                        .back(2.5)
                        .splineTo(new Vector2d(19, -42), Math.toRadians(-19))
                        .back(24)
                        .splineTo(new Vector2d(45, -55), Math.toRadians(90))
                        .waitSeconds(2)


                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}