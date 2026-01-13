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

public class Decode_MeepMeep_Blue_Far_12Balls_Spline {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(700);

        // Global bot constraints (match your fast constraint envelope)
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(
                        85,                  // maxVel
                        60,                  // maxAccel
                        Math.toRadians(180), // maxAngVel
                        Math.toRadians(180), // maxAngAccel
                        15                   // trackWidth (keep yours)
                )
                .build();

        // EXACT RR constraints used in Blue_Far_Auto_12balls
        VelConstraint fastVel = new TranslationalVelConstraint(85);
        AccelConstraint fastAccel = new ProfileAccelConstraint(-60, 60);

        VelConstraint intakeVel = new TranslationalVelConstraint(60);
        AccelConstraint intakeAccel = new ProfileAccelConstraint(-30, 40);

        // StartPose matches Blue_Far_Auto_12balls
        Pose2d startPose = new Pose2d(63, -12, Math.toRadians(-90));

        myBot.runAction(
                myBot.getDrive().actionBuilder(startPose)

                        // ===== DriveToShootingPosition (NOTE: your action had NO explicit constraints here) =====
                        .strafeToConstantHeading(new Vector2d(48, -12))

                        // Shoot #1-#3 (sim)
                        .waitSeconds(2)

                        // ===== DriveToFirstMark =====
                        // Smooth into the strip (fast -> intake). Tangent set to -Y so it flows into the downfield run.
                        .splineToConstantHeading(new Vector2d(34.75, -30), Math.toRadians(-90), fastVel, fastAccel)
                        .splineToConstantHeading(new Vector2d(34.75, -63), Math.toRadians(-90), intakeVel, intakeAccel)

                        // ===== ReturnFromFirstMark =====
                        .strafeToConstantHeading(new Vector2d(48, -12), fastVel, fastAccel)

                        // Shoot #4-#6 (sim)
                        .waitSeconds(2)

                        // ===== DriveToSecondMark =====
                        // Smooth into the strip (fast -> intake)
                        .splineToConstantHeading(new Vector2d(11.5, -30), Math.toRadians(-90), fastVel, fastAccel)
                        .splineToConstantHeading(new Vector2d(11.5, -63), Math.toRadians(-90), intakeVel, intakeAccel)

                        // ===== ReturnFromSecondMark =====
                        .strafeToConstantHeading(new Vector2d(48, -12), fastVel, fastAccel)

                        // Shoot #7-#9 (sim)
                        .waitSeconds(2)

                        // ===== DriveToThirdMark =====
                        // Smooth into the strip (fast -> intake)
                        .splineToConstantHeading(new Vector2d(-15, -31), Math.toRadians(-90), fastVel, fastAccel)
                        .splineToConstantHeading(new Vector2d(-15, -60), Math.toRadians(-90), intakeVel, intakeAccel)

                        // ===== DriveThirdMarkToBigTriangle =====
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