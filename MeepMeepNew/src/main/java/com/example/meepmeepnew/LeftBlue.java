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
                        drive.trajectorySequenceBuilder(new Pose2d(41, 65, Math.toRadians(270)))
                                // Claw grips
                                .splineToConstantHeading(new Vector2d(36, 50), Math.toRadians(270))
                                .splineToConstantHeading(new Vector2d(36, 12), Math.toRadians(270))
                                .turn(Math.toRadians(135))
                                // Raise slides, raise arm, lower V
                                .back(8)
                                // Claw releases
                                .forward(8)
                                // Lower slides, lower arm, raise V
                                .turn(Math.toRadians(-45))
                                // Repeat following trajectory
                                // Raise slides to correct height, put arm out slightly
                                .splineToConstantHeading(new Vector2d(65, 12), Math.toRadians(0))
                                // Claw grips, slides raise
                                .splineToConstantHeading(new Vector2d(36, 12), Math.toRadians(180))
                                .turn(Math.toRadians(45))
                                // Raise slides, raise arm, lower V
                                .back(8)
                                // Claw releases
                                .forward(8)
                                // Lower slides, lower arm, raise V
                                .turn(Math.toRadians(-45))
                                // Move to randomized position
                                .splineToConstantHeading(new Vector2d(60, 12), Math.toRadians(0)) // Left, Position 1
                                .splineToConstantHeading(new Vector2d(12, 12), Math.toRadians(180)) // Right, Position 3
                                // Middle case stays put
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
