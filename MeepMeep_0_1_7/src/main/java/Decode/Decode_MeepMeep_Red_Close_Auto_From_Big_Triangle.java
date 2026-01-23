package Decode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class Decode_MeepMeep_Red_Close_Auto_From_Big_Triangle {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        myBot.runAction(
                myBot.getDrive().actionBuilder(
                                new Pose2d(-60,38,Math.toRadians(90)))
                        .waitSeconds(2)
                        // Big triangle -> center-ish
                        .strafeToConstantHeading(new Vector2d(-11.4, 29))
                        .waitSeconds(2)

                        // Up left lane
                        .strafeToConstantHeading(new Vector2d(-11.4, 55))
                        .waitSeconds(2)

                        // Down left lane
                        .strafeToConstantHeading(new Vector2d(-11.4, 10))
                        .waitSeconds(2)

                        // Cross to right lane
                        .strafeToConstantHeading(new Vector2d(11, 29))
                        .waitSeconds(2)

                        // Second mark area (right side)
                        .strafeToConstantHeading(new Vector2d(11.4, 60))
                        .waitSeconds(2)

                        // Loop up to hit/open the gate (U-shape), then come down to lock
                        .strafeToConstantHeading(new Vector2d(11.4, 70))
                        .waitSeconds(2)
                        .strafeToConstantHeading(new Vector2d(-8, 70))
                        .waitSeconds(2)
                        .strafeToConstantHeading(new Vector2d(0, 56))
                        .waitSeconds(2)
                        .strafeToConstantHeading(new Vector2d(0, 40))
                        .waitSeconds(2)

                        // Back to big triangle
                        .strafeToConstantHeading(new Vector2d(-11.5, 13))
                        .waitSeconds(2)

                        // Big triangle -> third mark -> back
                        .strafeToConstantHeading(new Vector2d(0, 40))
                        .waitSeconds(2)
                        .strafeToConstantHeading(new Vector2d(-15, 60))
                        .waitSeconds(2)
                        .strafeToConstantHeading(new Vector2d(-11.5, 13))
                        .waitSeconds(2)

                        // End / park
                        .strafeToConstantHeading(new Vector2d(-11.4, 10))
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
