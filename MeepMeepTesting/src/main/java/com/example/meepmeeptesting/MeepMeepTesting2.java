package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(14, -63, Math.toRadians(-90)))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(46, -8), Math.toRadians(-60))
                .strafeTo(new Vector2d(46, -60))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(58, -9), Math.toRadians(-40))
                        
                .strafeTo(new Vector2d(58, -60))
                .strafeTo(new Vector2d(58, -9))
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