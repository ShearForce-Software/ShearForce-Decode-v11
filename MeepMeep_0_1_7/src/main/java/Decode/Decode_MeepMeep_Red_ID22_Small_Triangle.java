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

public class Decode_MeepMeep_Red_ID22_Small_Triangle {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        VelConstraint fastVel = new TranslationalVelConstraint(85);
        AccelConstraint fastAccel = new ProfileAccelConstraint(-60, 60);

        VelConstraint intakeVel = new TranslationalVelConstraint(45);
        AccelConstraint intakeAccel = new ProfileAccelConstraint(-20, 25);

        // Matches Red_Far_Auto_Obelisk for ID22 => SampleLine.SECOND
        // Start -> ShootingPosition (48,12)
        // Tick 1: DriveToSecondMark (11.5,30)->(11.5,60), ReturnFromSecondMark -> (20,12)->(48,12)
        // Tick 2: DriveToFirstMark (34.75,30)->(34.75,60), ReturnFromFirstMark -> (48,12)
        // Tick 3: DriveToThirdMark (-15,32)->(-15,55), DriveThirdMarkToLargeTriangle -> (-23.7,23.4)
        // Then "park" (you can change these last points to your real park later)

        myBot.runAction(
                myBot.getDrive().actionBuilder(new Pose2d(60, 12, Math.toRadians(90)))

                        // DriveToShootingPosition
                        .strafeToConstantHeading(new Vector2d(48, 12), fastVel, fastAccel)

                        // Shoot #1-#3 (sim)
                        .waitSeconds(2)

                        // ===== TICK MARK 1 (SECOND) =====
                        // DriveToSecondMark
                        .strafeToConstantHeading(new Vector2d(11.5, 30), fastVel, fastAccel)
                        .strafeToConstantHeading(new Vector2d(11.5, 60), intakeVel, intakeAccel)

                        // ReturnFromSecondMark
                        .strafeToConstantHeading(new Vector2d(20, 12), fastVel, fastAccel)
                        .strafeToConstantHeading(new Vector2d(48, 12), fastVel, fastAccel)

                        // Shoot #4-#6 (sim)
                        .waitSeconds(2)

                        // ===== TICK MARK 2 (FIRST) =====
                        // DriveToFirstMark
                        .strafeToConstantHeading(new Vector2d(34.75, 30), fastVel, fastAccel)
                        .strafeToConstantHeading(new Vector2d(34.75, 60), intakeVel, intakeAccel)

                        // ReturnFromFirstMark
                        .strafeToConstantHeading(new Vector2d(48, 12), fastVel, fastAccel)

                        // Shoot #7-#9 (sim)
                        .waitSeconds(2)

                        // ===== TICK MARK 3 (THIRD) =====
                        // DriveToThirdMark
                        .strafeToConstantHeading(new Vector2d(-15, 32), fastVel, fastAccel)
                        .strafeToConstantHeading(new Vector2d(-15, 55), intakeVel, intakeAccel)

                        // DriveThirdMarkToLargeTriangle (this is what your auto uses for ID22)
                        .strafeToConstantHeading(new Vector2d(-23.7, 23.4), fastVel, fastAccel)

                        // Shoot #10-#12 (sim)
                        .waitSeconds(2)

                        // ===== "PARK" (placeholder) =====
                        .strafeToConstantHeading(new Vector2d(0, 12), fastVel, fastAccel)
                        .strafeToConstantHeading(new Vector2d(20, 12), fastVel, fastAccel)

                        .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}