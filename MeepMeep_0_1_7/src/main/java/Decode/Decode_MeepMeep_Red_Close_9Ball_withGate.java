package Decode;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Decode_MeepMeep_Red_Close_9Ball_withGate {

    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(700);
        double shooterSleepTime = 2.175;
        Pose2d startPose = new Pose2d(-60,39,Math.toRadians(90));

        // MeepMeep "global" bot constraints (roughly aligned to your auto constraints)
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(
                        60,                         // maxVel
                        40,                         // maxAccel
                        Math.PI,        // maxAngVel
                        Math.PI,        // maxAngAccel
                        15                          // trackWidth (keep whatever you use)
                )
                .build();

        // CONSTRAINTS
    VelConstraint fastVel = new TranslationalVelConstraint(90);
    AccelConstraint fastAccel = new ProfileAccelConstraint(-65, 65);

    VelConstraint normalVel = new TranslationalVelConstraint(60);
    AccelConstraint normalAccel = new ProfileAccelConstraint(-30, 40);

    VelConstraint slowVel = new TranslationalVelConstraint(40);
    AccelConstraint slowAccel = new ProfileAccelConstraint(-25, 25);

    VelConstraint superSlowVel = new TranslationalVelConstraint(40);
    AccelConstraint superSlowAccel = new ProfileAccelConstraint(-15, 15);

        // ***************************************************
        // ****  Define Trajectories    **********************
        // ***************************************************
        DriveShim drive = myBot.getDrive();

        Action DriveCloseStartPositiontoBigTriangle = drive.actionBuilder(startPose)
                .strafeToConstantHeading(new Vector2d(-11.5, 21))
                .build();

        Action DriveBigTriangleToThirdMark = drive.actionBuilder(new Pose2d(-11.5, 21, Math.toRadians(90)))
                //.splineToConstantHeading(new Vector2d(-15, -31), Math.toRadians(-90), slowVel, slowAccel)
                .strafeToConstantHeading(new Vector2d(-11.5, 60), slowVel, slowAccel)
                .build();

        Action ThirdMarkToLock = drive.actionBuilder(new Pose2d(-11.5, 60, Math.toRadians(90)))
                //.splineToConstantHeading(new Vector2d(-5, -40),  Math.toRadians(-180), loopVel, loopAccel)
                .strafeToConstantHeading(new Vector2d(-2, 45),slowVel, slowAccel)
                .splineToConstantHeading(new Vector2d(0, 58),  Math.toRadians(90), slowVel, slowAccel)
                //.strafeToConstantHeading(new Vector2d(0, -50),loopVel, loopAccel)
                .build();

        Action LockToBigTriangle = drive.actionBuilder(new Pose2d(0, 58, Math.toRadians(90)))
                //.splineToConstantHeading(new Vector2d(0, -20),  Math.toRadians(-270), normalVel, normalAccel)
                .strafeToConstantHeading(new Vector2d(0, 40),slowVel, slowAccel)
                .splineToConstantHeading(new Vector2d(-11.5, 21),  Math.toRadians(90),slowVel, superSlowAccel)
                .build();

        Action DriveBigTriangletoSecondMark = drive.actionBuilder(new Pose2d(-11.5,21,Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(14.5, 20), Math.toRadians(90), fastVel, fastAccel)
                //.splineToConstantHeading(new Vector2d(11.5, -60), Math.toRadians(-90), normalVel, normalAccel)
                .strafeToConstantHeading(new Vector2d(14.5, 61),slowVel, slowAccel)
                .build();

        Action DriveSecondMarktoBigTriangle = drive.actionBuilder(new Pose2d(14.5,61,Math.toRadians(90)))
                //.splineToConstantHeading(new Vector2d(0, -20),  Math.toRadians(-270), intakeVel, intakeAccel)
                .strafeToConstantHeading(new Vector2d(11.5, 30),fastVel, normalAccel)
                //.strafeToConstantHeading(new Vector2d(-11.5, -21),specialVel, specialAccel)
                .splineToConstantHeading(new Vector2d(-11.5, 21),  Math.toRadians(90),fastVel, slowAccel)
                .build();

        Action DriveShootingPositionToGateLock =  drive.actionBuilder(new Pose2d(-11.5, 21, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(0, 30), normalVel, normalAccel)
                .build();

        // ***************************************************
        // ****  WAIT for START/PLAY to be pushed ************
        // ***************************************************
        myBot.runAction(new SequentialAction(

        // -------------------------
        // START -> SHOOT FROM BIG TRIANGLE
        // -------------------------
        // Drive to the shooting position
                        DriveCloseStartPositiontoBigTriangle,
                        new SleepAction(0.5),
				        // SHOOT-3
				        new SleepAction(shooterSleepTime),
        // -------------------------
        // BIG TRIANGLE -> THIRD STRIP -> WAIT
        // -------------------------
                        DriveBigTriangleToThirdMark,
				        new SleepAction(0.4),
        // -------------------------
        // THIRD STRIP -> GATE -> BIG TRIANGLE SHOOT
        // -------------------------
						
                        ThirdMarkToLock,
						//wait for Frooty Loops
						new SleepAction(5.0),
                        LockToBigTriangle,
				        // SHOOT-3
				        new SleepAction(shooterSleepTime),
        // -------------------------
        // BIG TRIANGLE -> SECOND MARK -> BIG TRIANGLE SHOOT
        // -------------------------
                        DriveBigTriangletoSecondMark,
                        new SleepAction(0.250),
                        DriveSecondMarktoBigTriangle,
				        // SHOOT-3
				        new SleepAction(shooterSleepTime),
        // -------------------------
        // BIG TRIANGLE -> PARK NEXT TO GATE
        // -------------------------
                        DriveShootingPositionToGateLock
        ));


        meepMeep.setTheme(new ColorSchemeBlueDark(), new ColorSchemeRedLight());
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}