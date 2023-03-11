package com.example.meepmeepnew;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RightBlue {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-31.5, 63, Math.toRadians(180)))
                                .splineToConstantHeading(new Vector2d(-36, 50), Math.toRadians(630))
                                .splineToConstantHeading(new Vector2d(-36, 25), Math.toRadians(630))
                                .splineToSplineHeading(new Pose2d(-31, 8, Math.toRadians(135)), Math.toRadians(325))
                                .waitSeconds(0.2)
                                .splineToSplineHeading(new Pose2d(-40, 11.5, Math.toRadians(180)), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-63, 11.5), Math.toRadians(180))
                                .waitSeconds(0.2)
                                .lineToConstantHeading(new Vector2d(-62.5, 11.5))
                                .splineToSplineHeading(new Pose2d(-31, 8, Math.toRadians(135)), Math.toRadians(325))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
