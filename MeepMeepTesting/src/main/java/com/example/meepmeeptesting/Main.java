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
                Pose2d startPose = new Pose2d(0, y(0), heading(180));
                //Quick sample
                Pose2d target1 = new Pose2d(30, y(30), heading(290));
                Pose2d target2 = new Pose2d(Constants.Sizes.field/2, y(-40), heading(290));
                myBot.runAction(myBot.getDrive().actionBuilder(startPose)
                        .splineToLinearHeading(target1, target1.heading)
                        .splineToLinearHeading(target2, target2.heading)
                        .build());
                break;
            case 2:
                myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(28.92, y(2.16), heading(90.00)))
                        .splineTo(new Vector2d(-4.72, y(23.21)), heading(147.96))
                        .splineTo(new Vector2d(-7.87, y(-18.69)), heading(265.70))
                        .splineTo(new Vector2d(30.69, y(-20.26)), heading(-2.34))
                        .splineTo(new Vector2d(64.72, y(24.79)), heading(52.93))
                        .splineTo(new Vector2d(15.15, y(52.52)), heading(150.77))
                        .splineTo(new Vector2d(-41.51, y(53.11)), heading(179.40))
                        .splineTo(new Vector2d(-48.00, y(17.70)), heading(259.61))
                        .splineTo(new Vector2d(-37.18, y(-23.41)), heading(-75.26))
                        .splineTo(new Vector2d(-21.64, y(-59.02)), heading(-66.42))
                        .splineTo(new Vector2d(23.21, y(-52.33)), heading(8.48))
                        .splineTo(new Vector2d(60.39, y(-45.25)), heading(10.78))
                        .build());
                break;
            case 3:
                myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-11.02, 0.39, Math.toRadians(90.00)))
                        .splineTo(new Vector2d(29.51, y(-2.16)), heading(-3.61))
                        .splineToSplineHeading(new Pose2d(-0.20, y(-19.48), heading(210.23)), heading(210.23))
                        .splineToLinearHeading(new Pose2d(-36.00, y(-25.18), heading(189.05)), heading(189.05))
                        .splineToConstantHeading(new Vector2d(-58.23, y(6.49)), heading(125.06))
                        /*
                        So like, don't use any of these they're garbage lmaoo
                        .lineToX(-50.36) //the new command is either lineToX or lineToY, x is -50.36, y is 27.34
                        .lineToXSplineHeading(-20.46, heading(14.39)) //the real command replaces the underscore with x and y, x is -20.46 and y is 35.02, choose one to align with
                        .lineToXLinearHeading(24.39,  heading(-6.75)) //the real command replaces the underscore with x and y, x is 24.39 and y is 29.70, choose one to align with
                        .lineToXConstantHeading(61.77) //the real command replaces the underscore with x and y, x is -21.54 and y is 61.77, choose one to align with

                         */
                        .build());
                break;

        }
        allianceBot.runAction(allianceBot.getDrive().actionBuilder(new Pose2d(39.40, y(3.71), heading(90.00)))
                .splineTo(new Vector2d(14.85, y(21.04)), heading(144.78))
                .splineTo(new Vector2d(-28.47, y(-9.90)), heading(215.54))
                .splineTo(new Vector2d(9.90, y(-27.23)), heading(-24.30))
                .splineTo(new Vector2d(34.25, y(37.34)), heading(69.34))
                .splineTo(new Vector2d(29.09, y(-29.71)), heading(265.60))
                .splineTo(new Vector2d(12.79, y(-10.52)), heading(130.35))
                .splineTo(new Vector2d(-48.48, y(27.44)), heading(148.22))
                .splineTo(new Vector2d(-48.28, y(55.91)), heading(89.58))
                .splineTo(new Vector2d(58.38, y(-57.97)), heading(-46.88))
                .splineTo(new Vector2d(20.42, y(-70.14)), heading(197.78))
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
    private static double heading(double angle) {
        double angleR = Math.toRadians(angle);
        if (Constants.TEAM.equals("BLUE")) angleR *= -1;
        return angleR;
    }
}