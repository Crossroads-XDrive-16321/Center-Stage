package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep mm = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(mm)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15) //TODO: change these values once we finish the constraints
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-34,60, Math.toRadians(270)))
                                .addDisplacementMarker(8, () -> {
                                    //clawController.setClawLevelPos(); //TODO: yep
                                })
                                .splineToConstantHeading(new Vector2d(-36,20),Math.toRadians(270))
                                .splineToLinearHeading(new Pose2d(-48,18,Math.toRadians(270)),Math.toRadians(90))
                                .build()
                );

        mm.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}