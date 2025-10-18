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
        int wantedAuto=1;
        switch (wantedAuto) {
            case 1:
                //A bit off dead center y, against right of x
                //The offset from the bot is auto applied (I think), so don't worry about it
                Pose2d startPose = new Pose2d(x(0), y(0), heading(180));
                //Quick sample
                Pose2d target1 = new Pose2d(x(30), y(30), heading(90));
                Pose2d target2 = new Pose2d(x(Constants.Sizes.field/2), y(-40), heading(270));
                myBot.runAction(myBot.getDrive().actionBuilder(startPose)
                        .splineToLinearHeading(target1, target1.heading)
                        .splineToLinearHeading(target2, target2.heading)
                        .build());
                break;
        }


        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
    private static double y(double offset){
        if (Constants.TEAM.equals("BLUE")){
            offset *= -1;
        }
        return offset;
    }
    private static double x(double fromWall){
        return Constants.Sizes.field/2 - fromWall;
    }
    private static double heading(double angle) {
        if (Constants.TEAM.equals("RED")) {
            return Math.toRadians(angle);
        } else {
            return Math.toRadians(angle - 2 * (180 - angle));
        }
    }
}