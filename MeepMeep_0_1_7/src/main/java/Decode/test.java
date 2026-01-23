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

public class test {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(500);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        VelConstraint fastVel = new TranslationalVelConstraint(85);
        AccelConstraint fastAccel = new ProfileAccelConstraint(-60, 60);

        VelConstraint intakeVel = new TranslationalVelConstraint(60);
        AccelConstraint intakeAccel = new ProfileAccelConstraint(-30, 40);

        VelConstraint loopVel = new TranslationalVelConstraint(40);
        AccelConstraint loopAccel = new ProfileAccelConstraint(-25, 25);

        // Order you specified:
        // DriveToShootingPosition -> shoot3
        // DriveToSecondMark -> shoot3
        // SecondMarkToLock -> LockToBigTriangle -> shoot3
        // DriveBigTriangleToThirdMark -> ThirdMarkBackToBigTriangle -> shoot3
        // DriveBigTriangleToFirstMark -> DriveFirstMarkToShootingPosition -> shoot3
        // DriveFirstMarkToLock and end there

        myBot.runAction(
                myBot.getDrive().actionBuilder(new Pose2d(60, 12, Math.toRadians(90)))

                        // DriveToShootingPosition
                        .strafeToConstantHeading(new Vector2d(48, 12), fastVel, fastAccel)
                        // shoot3
                        .waitSeconds(2)

                        // DriveToSecondMark
                        .splineToConstantHeading(new Vector2d(11.5, 30), Math.toRadians(90), fastVel, fastAccel)
                        .splineToConstantHeading(new Vector2d(11.5, 60), Math.toRadians(90), intakeVel, intakeAccel)
                        // shoot3 (per your requested order)
                        .waitSeconds(0.5)


                        // SecondMarkToLock (your working 2-point "U" turn)
                        .splineToConstantHeading(new Vector2d(5, 40), Math.toRadians(180), loopVel, loopAccel)
                        .splineToConstantHeading(new Vector2d(0, 56), Math.toRadians(90), loopVel, loopAccel)
                        .waitSeconds(1)

                        // LockToBigTriangle
                        .splineToConstantHeading(new Vector2d(0, 20), Math.toRadians(270), intakeVel, intakeAccel)
                        .splineToConstantHeading(new Vector2d(-11.5, 13), Math.toRadians(90), fastVel, fastAccel)
                        // shoot3
                        .waitSeconds(2)

                        // DriveBigTriangleToThirdMark
                        .splineToConstantHeading(new Vector2d(-15, 31), Math.toRadians(90), fastVel, fastAccel)
                        .strafeToConstantHeading(new Vector2d(-15, 60), intakeVel, intakeAccel)

                        // ThirdMark back to BigTriangle
                        .strafeToConstantHeading(new Vector2d(-11.5, 13), fastVel, fastAccel)
                        // shoot3
                        .waitSeconds(2)

                        // DriveBigTriangleToFirstMark
                        .strafeToConstantHeading(new Vector2d(34.75, 30), fastVel, fastAccel)
                        .strafeToConstantHeading(new Vector2d(34.75, 60), intakeVel, intakeAccel)

                        // DriveFirstMarkToShootingPosition
                        .strafeToConstantHeading(new Vector2d(48, 12), fastVel, fastAccel)
                        // shoot3
                        .waitSeconds(2)

                        // DriveFirstMarkToLock (end here)
                        .strafeToConstantHeading(new Vector2d(0, 40), fastVel, fastAccel)

                        .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}