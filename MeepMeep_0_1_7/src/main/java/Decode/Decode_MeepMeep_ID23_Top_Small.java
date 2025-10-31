package Decode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Decode_MeepMeep_ID23_Top_Small {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-55, 55, 90))
                /*.lineToX(-12)
                .turn(Math.toRadians(90))
                .lineToY(48)
                .build());*/
                .strafeToLinearHeading(new Vector2d(-10,10),Math.toRadians(90))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(-12,55),Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-10,10),Math.toRadians(90))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(30,62),Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(38,62),Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(38,50),Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-10,10),Math.toRadians(270))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(-54,16),Math.toRadians(270))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
