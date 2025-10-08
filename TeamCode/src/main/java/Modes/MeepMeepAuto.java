package Modes;
import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepAuto {

    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        // Pick autonomous routine
        int wantedAuto = 1;

        switch (wantedAuto) {
            case 1:
                myBot.runAction(
                        myBot.getDrive()
                                .actionBuilder(new Pose2d(-36, -36, 0))
                                .lineToX(-10)
                                .turn(Math.toRadians(90))
                                .lineToX(0)
                                .build()
                );
                break;

            case 2:
                myBot.runAction(
                        myBot.getDrive()
                                .actionBuilder(new Pose2d(52, -52, 0))
                                .lineToX(32)
                                .lineToX(52)
                                .build()
                );
                break;
        }

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
