package Decode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Decode_MeepMeep_ID22_Bottom_Small {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(60, -12, Math.toRadians(270)))
                .waitSeconds(2)
                .strafeToConstantHeading(new Vector2d(11.5,-30))
                .strafeToConstantHeading(new Vector2d(11.5,-48))
                //.strafeToLinearHeading(new Vector2d(0,-48),Math.toRadians(0))
                //.strafeToConstantHeading(new Vector2d(0,-52))
                .strafeToConstantHeading(new Vector2d(20,-12))
                .strafeToConstantHeading(new Vector2d(60,-12))
                .waitSeconds(2)
                .strafeToConstantHeading(new Vector2d(34.75,-30))
                .strafeToConstantHeading(new Vector2d(34.75,-48))
                .strafeToConstantHeading(new Vector2d(60,-12))
                .waitSeconds(2)
                .strafeToConstantHeading(new Vector2d(20,-12))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
