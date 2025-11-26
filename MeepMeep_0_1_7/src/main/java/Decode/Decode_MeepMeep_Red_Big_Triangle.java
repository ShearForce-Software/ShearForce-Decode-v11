package Decode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class Decode_MeepMeep_Red_Big_Triangle {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        myBot.runAction(
                myBot.getDrive().actionBuilder(
                                new Pose2d(-60,-39,Math.toRadians(270)))
                        .waitSeconds(2)
                        .strafeToConstantHeading(new Vector2d(-11.4, -10))
                        .waitSeconds(2)
                        .turnTo(Math.toRadians(270))
                        .strafeToConstantHeading(new Vector2d(-11.4, -55))
                        .waitSeconds(2)
                        .strafeToConstantHeading(new Vector2d(-11.4, -10))
                        .waitSeconds(2)
                        .strafeToConstantHeading(new Vector2d(-54, -16))

                        .waitSeconds(2)




                        .build()

        );
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

}







