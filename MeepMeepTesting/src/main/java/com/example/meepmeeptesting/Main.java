package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Main {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        int wantedAuto=2;
        switch (wantedAuto) {
            case 1:  myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-34, -34, 0))
                .turn(Math.toRadians(-30))
                .lineToX(-8)
                .lineToX(-38)
                .turn(Math.toRadians(15))
                .lineToX(10)
                .lineToX(-38)
                    .build());
                    break;
            case 2:
                myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(52, -52, 0))
                        .lineToX(32)
                        .lineToX(52)
                        .lineToX(10)
                        .lineToX(52).build());
                break;
        }


        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}