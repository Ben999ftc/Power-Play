package com.example.meepmeepnew;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class LeftMid {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(31.1, 61.6, Math.toRadians(90)))
                                .lineToConstantHeading(new Vector2d(36, 50))
                                .splineToConstantHeading(new Vector2d(36, 25), Math.toRadians(270))
                                .splineToSplineHeading(new Pose2d(28.6, 6.5, Math.toRadians(65)), Math.toRadians(245))
                                .waitSeconds(0.2)
                                .splineToSplineHeading(new Pose2d(58, 11.8, Math.toRadians(0)), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(63, 11.8), Math.toRadians(0))
                                .waitSeconds(0.2)
                                .lineToConstantHeading(new Vector2d(50, 13))
                                .splineToSplineHeading(new Pose2d(31.1, 20.8, Math.toRadians(346)), Math.toRadians(166))
                                .waitSeconds(0.2)
                                .splineToSplineHeading(new Pose2d(58, 11.8, Math.toRadians(0)), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(63, 11.8), Math.toRadians(0))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
