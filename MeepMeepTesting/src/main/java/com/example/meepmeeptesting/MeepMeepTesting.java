package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep mm = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(mm)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15) //TODO: change these values once we finish the constraints
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-34,61, Math.toRadians(270)))
                                .splineToLinearHeading(new Pose2d(-36,34,Math.toRadians(0)),Math.toRadians(270))
                                .waitSeconds(2)//drop purple
                                .strafeTo(new Vector2d(-12,36))
                                .strafeTo(new Vector2d(-12,16))
                                .splineToLinearHeading(new Pose2d(32,11,Math.toRadians(0)),Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(46,28, Math.toRadians(0)), Math.toRadians(0))
                                .waitSeconds(2)//drop yellow
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(32,-11,Math.toRadians(0)),Math.toRadians(180))
                                .lineToLinearHeading(new Pose2d(-56,-11,Math.toRadians(0)))
                                .waitSeconds(2)//pickup two white
                                .lineToLinearHeading(new Pose2d(32,-11,Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(46,-36), Math.toRadians(0))
                                .waitSeconds(2)//drop two white
                                .splineToLinearHeading(new Pose2d(32,-11,Math.toRadians(0)),Math.toRadians(180))
                                .lineToLinearHeading(new Pose2d(-56,-11,Math.toRadians(0)))
                                .waitSeconds(2)//pickup two white
                                .lineToLinearHeading(new Pose2d(32,-11,Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(46,-36), Math.toRadians(0))
                                .waitSeconds(2)//drop two white
                                .setReversed(false)
                                .lineToConstantHeading(new Vector2d(46,-60))
                                .forward(14)
                                .build()
                );

        mm.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}