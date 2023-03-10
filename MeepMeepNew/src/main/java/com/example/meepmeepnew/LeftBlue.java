package com.example.meepmeepnew;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class LeftBlue {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(31, 63, Math.toRadians(270)))
                                .splineToConstantHeading(new Vector2d(36, 48), Math.toRadians(270))
                                .splineToConstantHeading(new Vector2d(36, 0), Math.toRadians(270))
                                .splineToLinearHeading(new Pose2d(30, 3.4, Math.toRadians(45)), Math.toRadians(225))
                                .splineToSplineHeading(new Pose2d(36, 11.5, Math.toRadians(0)), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(65, 11.5), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(40, 11.5), Math.toRadians(180))
                                .splineTo(new Vector2d(30, 3.4), Math.toRadians(225))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
