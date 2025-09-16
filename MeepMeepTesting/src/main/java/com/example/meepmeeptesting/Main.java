package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Main {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        //remove mybot when turning into actual auto
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        int wantedAuto=2;
        switch (wantedAuto) {
            case 1:  myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-34, -34, 0))
//                BANG
                .turn(Math.toRadians(-30))
                .lineToX(-8)
                .lineToX(-38)
//                BANG
                .turn(Math.toRadians(15))
                .lineToX(10)
                .lineToX(-38)
//                BANG
                    .build());
                    break;
            case 2:
                myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(52, -52, 0))
//                BANG

                        .lineToX(32)
                        .lineToX(52)
//                BANG

                        .lineToX(10)
                        .lineToX(52).build());
                break;}

//                .turn(Math.toRadians(73.5)) //174
//                .turn(Math.toRadians(100.5))
//                .lineToX(-40)
//                .turn(Math.toRadians(181))
//                .lineToX(48)
//                .turn(Math.toRadians(75))
//                .turn(Math.toRadians(90))
//                .lineToX(-37)
//                .turn(Math.toRadians(-95))
//                .lineToY(-7)


        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}