package com.example.meepmeepnew;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class LeftRed {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-45, -63, Math.toRadians(180)))
                                .splineToConstantHeading(new Vector2d(-36, -52), Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(-36, -11.5), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-60, -11.5), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-33, -11.5), Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(-33, 0), Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(-36, -36), Math.toRadians(270)) //Middle case, go here anyway
                                .splineToConstantHeading(new Vector2d(-60, -36), Math.toRadians(180)) //Left case
                                .splineToConstantHeading(new Vector2d(-12, -36), Math.toRadians(0)) //Right case
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
