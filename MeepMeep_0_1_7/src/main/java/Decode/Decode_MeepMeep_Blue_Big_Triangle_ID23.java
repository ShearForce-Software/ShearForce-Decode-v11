package Decode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Decode_MeepMeep_Blue_Big_Triangle_ID23 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(750);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

       /*myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(60, -12, 0))
                .lineToX(-12)
                .turn(Math.toRadians(90))
                .lineToY(-48)
                .build());*/

       myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-60, -39, Math.toRadians(270)))
               // move away from goal to a launchable position
               .strafeToLinearHeading(new Vector2d(-10,-10),Math.toRadians(270))
               // shoot 3 balls
               .waitSeconds(2)
               // Get line 1
               .strafeToConstantHeading(new Vector2d(-12,-53))
               // return to launch zone
               .strafeToConstantHeading(new Vector2d(-10,-10))
               // shoot 3 balls
               .waitSeconds(2)
               // get line 3 in correct order (messing up line 2 in the process)
               .strafeToLinearHeading(new Vector2d(22,-62),Math.toRadians(90))
               .strafeToLinearHeading(new Vector2d(38,-62),Math.toRadians(90))
               .strafeToLinearHeading(new Vector2d(38,-50),Math.toRadians(90))
               // return to launch zone
               .strafeToLinearHeading(new Vector2d(-10,-10),Math.toRadians(270))
               // shoot 3 balls
               .waitSeconds(2)
               // park
               .strafeToConstantHeading(new Vector2d(-54,-16))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
