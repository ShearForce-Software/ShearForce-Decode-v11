package Decode;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Decode_MeepMeep_Blue_Far_12Balls_Strafe {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(700);

        // Global bot constraints
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(
                        85,                         // maxVel
                        60,                         // maxAccel
                        Math.toRadians(180),        // maxAngVel
                        Math.toRadians(180),        // maxAngAccel
                        15                          // trackWidth
                )
                .build();

        // Constraints (same style as your 12-balls autos)
        VelConstraint fastVel = new TranslationalVelConstraint(85);
        AccelConstraint fastAccel = new ProfileAccelConstraint(-60, 60);

        VelConstraint intakeVel = new TranslationalVelConstraint(60);
        AccelConstraint intakeAccel = new ProfileAccelConstraint(-30, 40);

        // StartPose (Blue)
        Pose2d startPose = new Pose2d(63, -12, Math.toRadians(-90));

        myBot.runAction(
                myBot.getDrive().actionBuilder(startPose)

                        // DriveToShootingPosition
                        .strafeToConstantHeading(new Vector2d(48, -12), fastVel, fastAccel)

                        // Shoot #1-#3 (sim)
                        .waitSeconds(2)

                        // ===== FIRST STRIP =====
                        .strafeToConstantHeading(new Vector2d(34.75, -30), fastVel, fastAccel)
                        .strafeToConstantHeading(new Vector2d(34.75, -63), intakeVel, intakeAccel)

                        // ReturnFromFirstMark
                        .strafeToConstantHeading(new Vector2d(48, -12), fastVel, fastAccel)

                        // Shoot #4-#6 (sim)
                        .waitSeconds(2)

                        // ===== SECOND STRIP =====
                        .strafeToConstantHeading(new Vector2d(11.5, -30), fastVel, fastAccel)
                        .strafeToConstantHeading(new Vector2d(11.5, -63), intakeVel, intakeAccel)

                        // ReturnFromSecondMark

                        .strafeToConstantHeading(new Vector2d(48, -12), fastVel, fastAccel)

                        // Shoot #7-#9 (sim)
                        .waitSeconds(2)

                        // ===== THIRD STRIP =====
                        .strafeToConstantHeading(new Vector2d(-15, -32), fastVel, fastAccel)
                        .strafeToConstantHeading(new Vector2d(-15, -60), intakeVel, intakeAccel)

                        // DriveThirdMarkToBigTriangle
                        .strafeToConstantHeading(new Vector2d(-12.4, -12.7), fastVel, fastAccel)

                        // Shoot #10-#12 (sim)
                        .waitSeconds(2)

                        .strafeToConstantHeading(new Vector2d(0,-46), fastVel, fastAccel)

                        .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}