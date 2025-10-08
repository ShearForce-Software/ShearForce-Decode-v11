package Decode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Decode_MeepMeep_ID21_Top_Small {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-49, 49, Math.toRadians(305)))
                .strafeToConstantHeading(new Vector2d(-30, 30))
                .waitSeconds(2)
                /*.strafeToLinearHeading(new Vector2d(-27, 55), Math.toRadians(315))
                .strafeToConstantHeading(new Vector2d(0, 30))
                .strafeToConstantHeading(new Vector2d(0, 50))
                .strafeToConstantHeading(new Vector2d(0, 30))
                .strafeToLinearHeading(new Vector2d(35, 30), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(35, 40))*/
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
