package Decode;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Decode_MeepMeep_Red_Far_12Balls_Spline {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(700);

        // MeepMeep "global" bot constraints (roughly aligned to your auto constraints)
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(
                        85,                         // maxVel
                        60,                         // maxAccel
                        Math.toRadians(180),        // maxAngVel
                        Math.toRadians(180),        // maxAngAccel
                        15                          // trackWidth (keep whatever you use)
                )
                .build();

        // EXACT constraints from your Red_Far_Auto_12balls.java
        VelConstraint fastVel = new TranslationalVelConstraint(85);
        AccelConstraint fastAccel = new ProfileAccelConstraint(-60, 60);

        VelConstraint intakeVel = new TranslationalVelConstraint(60);
        AccelConstraint intakeAccel = new ProfileAccelConstraint(-30, 40);

        // Path matches your auto:
        // Start (60,12,90) -> Shooting (48,12)
        // First strip: (34.75,30)->(34.75,63) -> back (48,12)
        // Second strip: (11.5,30)->(11.5,63) -> back (20,12)->(48,12)
        // Third strip: (-15,32)->(-15,60) -> Big triangle (-12.4,12.7)

        myBot.runAction(
                myBot.getDrive().actionBuilder(new Pose2d(60, 12, Math.toRadians(90)))

                        // DriveToShootingPosition
                        .strafeToConstantHeading(new Vector2d(48, 12), fastVel, fastAccel)

                        // Shoot #1-#3 (sim)
                        .waitSeconds(2)

                        // ===== FIRST STRIP =====
                        // End tangent set to +Y so it smoothly transitions into the forward run up the strip
                        .splineToConstantHeading(new Vector2d(34.75, 30), Math.toRadians(90), fastVel, fastAccel)
                        .splineToConstantHeading(new Vector2d(34.75, 63), Math.toRadians(90), intakeVel, intakeAccel)

                        // ReturnFromFirstMark
                        .strafeToConstantHeading(new Vector2d(45, 23), fastVel, fastAccel)
                        .strafeToConstantHeading(new Vector2d(48, 12), intakeVel, intakeAccel)

                        // Shoot #4-#6 (sim)
                        .waitSeconds(2)

                        // ===== SECOND STRIP =====
                        // Smooth into the strip (fast -> intake)
                        .splineToConstantHeading(new Vector2d(11.5, 30), Math.toRadians(90), fastVel, fastAccel)
                        .splineToConstantHeading(new Vector2d(11.5, 63), Math.toRadians(90), intakeVel, intakeAccel)

                        // ReturnFromSecondMark
                        .strafeToConstantHeading(new Vector2d(45, 23), fastVel, fastAccel)
                        .strafeToConstantHeading(new Vector2d(48, 12), intakeVel, intakeAccel)

                        // Shoot #7-#9 (sim)
                        .waitSeconds(2)

                        // ===== THIRD STRIP =====
                        // Smooth into the strip (fast -> intake)
                        .splineToConstantHeading(new Vector2d(-15, 27), Math.toRadians(90), fastVel, intakeAccel)
                        .splineToConstantHeading(new Vector2d(-15, 31), Math.toRadians(90), fastVel, intakeAccel)
                        .splineToConstantHeading(new Vector2d(-15, 33), Math.toRadians(90), intakeVel, intakeAccel)
                        .splineToConstantHeading(new Vector2d(-15, 60), Math.toRadians(90), intakeVel, intakeAccel)

                        // DriveThirdMarkToBigTriangle


                        .strafeToConstantHeading(new Vector2d(-11.5, 13), fastVel, fastAccel)

                        // Shoot #10-#12 (sim)
                        .waitSeconds(2)

                        .strafeToConstantHeading(new Vector2d(0,46), fastVel, fastAccel)



                        .build()
        );

        meepMeep.setTheme(new ColorSchemeBlueDark(), new ColorSchemeRedLight());
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}