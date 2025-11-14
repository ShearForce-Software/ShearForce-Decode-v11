package Decode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Decode_MeepMeep_Blue_ID21_Small_Triangle {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(60, -12, Math.toRadians(270)))
                // shoot 3 balls
                .waitSeconds(2)
                // Get line 3
                .strafeToConstantHeading(new Vector2d(36,-35))
                .strafeToConstantHeading(new Vector2d(36,-50))
                // return to launch area
                .strafeToConstantHeading(new Vector2d(60,-12))
                // shoot 3 balls
                .waitSeconds(2)
                // Get Line 2
                .strafeToConstantHeading(new Vector2d(11.5,-30))
                .strafeToConstantHeading(new Vector2d(11.5,-48))
                // return to launch area
                .strafeToConstantHeading(new Vector2d(60,-12))
                // shoot 3 balls
                .waitSeconds(2)
                // park
                .strafeToConstantHeading(new Vector2d(20,-12))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
