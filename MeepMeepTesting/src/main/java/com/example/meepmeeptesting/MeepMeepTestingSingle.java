package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTestingSingle {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.PI, Math.PI, 18)
                .build();

        //No vert slides
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, -63, Math.toRadians(90)))
                        .strafeTo(new Vector2d(0, -43))
                        //Check for vertical slide extension
                        .strafeTo(new Vector2d(0, -34))
                        //Retract vertical slide, snapping piece onto chamber
                        //Open claw

                        /*
                        .strafeToSplineHeading(new Vector2d(47, -60), Math.toRadians(-90))
                        //Wait
                        //Close claw
                        .strafeToSplineHeading(new Vector2d(0, -43), Math.toRadians(90))
                        //Check for vertical slide extension
                        .strafeTo(new Vector2d(0, -34))
                        //Retract vertical slide, snapping piece onto chamber
                        //Open claw

                         */

                        //All below = push
                        .strafeTo(new Vector2d(36, -34))
                        .strafeTo(new Vector2d(36, -9))
                        .splineToConstantHeading(new Vector2d(46, -8), Math.toRadians(-90))
                        .splineTo(new Vector2d(46, -60), Math.toRadians(-90))
                        .strafeTo(new Vector2d(46, -9))
                        .splineToConstantHeading(new Vector2d(58, -9), Math.toRadians(-90))
                        .splineTo(new Vector2d(58, -60), Math.toRadians(-90))
                        .build());

                /*
s
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(48, 48), Math.toRadians(60))
                .build());

                */

                /*
                .lineToY(-34)
                .turn(Math.toRadians(-90))
                .lineToX(30)
                .turn(Math.toRadians(-45))
                .turn(Math.toRadians(45))
                .turn(Math.toRadians(-45))
                .turn(Math.toRadians(45))
                .turn(Math.toRadians(-45))
                .turn(Math.toRadians(45))
                .build());
                 */

                /*

                .lineToX(30)
                .turn(Math.toRadians(90))
                .lineToY(30)
                .turn(Math.toRadians(90))
                .lineToX(0)
                .turn(Math.toRadians(90))
                .lineToY(0)
                .turn(Math.toRadians(90))
                .build());

                 */

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}