package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Main {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        RoadRunnerBotEntity allianceBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        int wantedAuto=2;
        switch (wantedAuto) {
            case 1:
                //A bit off dead center y, against right of x
                //The offset from the bot is auto applied (I think), so don't worry about it
                Pose2d startPose = new Pose2d(x(0), y(0), heading(180));
                //Quick sample
                Pose2d target1 = new Pose2d(x(30), y(30), heading(290));
                Pose2d target2 = new Pose2d(x(Constants.Sizes.field/2), y(-40), heading(290));
                myBot.runAction(myBot.getDrive().actionBuilder(startPose)
                        .splineToLinearHeading(target1, target1.heading)
                        .splineToLinearHeading(target2, target2.heading)
                        .build());
                break;
            case 2:
                myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(66.30, -8.66, Math.toRadians(180)))
                                .splineToLinearHeading(new Pose2d(12.00, 34.03, Math.toRadians(37.41)), Math.toRadians(37.41))
                                .lineToY(64.13)
                                .splineToLinearHeading(new Pose2d(55.28, 4.33, Math.toRadians(-53.86)), Math.toRadians(-53.86))
                                .splineToLinearHeading(new Pose2d(62.36, 58.23, Math.toRadians(82.51)), Math.toRadians(82.51))
                                .splineToLinearHeading(new Pose2d(62.95, 9.25, Math.toRadians(-89.31)), Math.toRadians(-89.31))
                                .splineToLinearHeading(new Pose2d(65.51, 57.25, Math.toRadians(86.95)), Math.toRadians(86.95))
                                .splineToLinearHeading(new Pose2d(58.43, 7.67, Math.toRadians(261.87)), Math.toRadians(261.87))
                                .build());

                break;
            case 3:
                myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-36.05, 13.38, Math.toRadians(39.50)))
                        .lineToY(66.03)
                        .splineToLinearHeading(new Pose2d(54.05, 2.48, Math.toRadians(-55.32)), Math.toRadians(-55.32))
                        .splineTo(new Vector2d(61.46, -55.68), Math.toRadians(82.07))
                        .splineTo(new Vector2d(64.36, -10.43), Math.toRadians(-86.34))
                        .splineTo(new Vector2d(56.09, -60.64), Math.toRadians(99.35))
                        .splineTo(new Vector2d(68.28, -12.71), Math.toRadians(-75.73))
                        .build());
                break;
        }
        allianceBot.runAction(allianceBot.getDrive().actionBuilder(new Pose2d(66.30, -8.66, Math.toRadians(180)))
                .splineTo(new Vector2d(61.77, 51.34), Math.toRadians(92.57))
                .splineToLinearHeading(new Pose2d(-58.62, 45.84, Math.toRadians(182.62)), Math.toRadians(182.62))
                .splineToLinearHeading(new Pose2d(57.05, 57.44, Math.toRadians(5.73)), Math.toRadians(5.73))
                .splineToLinearHeading(new Pose2d(-60.59, 43.87, Math.toRadians(186.58)), Math.toRadians(186.58))
                .splineToLinearHeading(new Pose2d(60.00, 6.89, Math.toRadians(-17.05)), Math.toRadians(-17.05))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .addEntity(allianceBot)
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
            return Math.toRadians(180 - (angle - 180));
        }
    }
}