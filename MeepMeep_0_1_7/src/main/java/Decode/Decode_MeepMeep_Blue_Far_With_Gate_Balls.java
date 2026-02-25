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

public class Decode_MeepMeep_Blue_Far_With_Gate_Balls {


    public static void main(String[] args) {
        final double startPoseHeadingDegrees = -90;
        Pose2d startPose = new Pose2d(60, -12, Math.toRadians(startPoseHeadingDegrees));

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

        // ***************************************************
        // ****  Define Velocity and Acceleration Constraints
        // ***************************************************

        VelConstraint fastVel = new TranslationalVelConstraint(90);
        AccelConstraint fastAccel = new ProfileAccelConstraint(-65, 65);

        VelConstraint normalVel = new TranslationalVelConstraint(60);
        AccelConstraint normalAccel = new ProfileAccelConstraint(-30, 40);

        VelConstraint slowVel = new TranslationalVelConstraint(40);
        AccelConstraint slowAccel = new ProfileAccelConstraint(-25, 25);

        VelConstraint superSlowVel = new TranslationalVelConstraint(30);
        AccelConstraint superSlowAccel = new ProfileAccelConstraint(-15, 15);


        // ***************************************************
        // ****  Define Trajectories    **********************
        // ***************************************************
        DriveShim drive = myBot.getDrive();
        Action DriveToShootingPosition = drive.actionBuilder(new Pose2d(startPose.position.x, startPose.position.y, Math.toRadians(startPoseHeadingDegrees)))
                .strafeToConstantHeading(new Vector2d(48, -12), fastVel, fastAccel)
                .build();

        Action DriveToSecondMark = drive.actionBuilder(new Pose2d(48, -12, Math.toRadians(-90)))
                .splineToConstantHeading(new Vector2d(11.5, -30), Math.toRadians(-90), fastVel, fastAccel)
                .splineToConstantHeading(new Vector2d(11.5, -60), Math.toRadians(-90), normalVel, normalAccel)
                .build();

        Action SecondMarkToLock = drive.actionBuilder(new Pose2d(11.5, -60, Math.toRadians(-90)))   // FIX
                .splineToConstantHeading(new Vector2d(5, -40),  Math.toRadians(-180), slowVel, slowAccel)  // up-left
                .splineToConstantHeading(new Vector2d(0, -50),  Math.toRadians(-90), slowVel, slowAccel)  // up-left
                .build();

        Action LockToBigTriangle = drive.actionBuilder(new Pose2d(0, -50, Math.toRadians(-90)))   // FIX
                .splineToConstantHeading(new Vector2d(0, -25),  Math.toRadians(-270), normalVel, normalAccel)  // up-left
                .splineToConstantHeading(new Vector2d(-11.5, -25),  Math.toRadians(-90),slowVel, superSlowAccel)  // up-left
                .build();

        Action SmallTriangleToLock = drive.actionBuilder(new Pose2d(48, -12, Math.toRadians(-90)))   // FIX
                .splineToConstantHeading(new Vector2d(0, -25),  Math.toRadians(-90), normalVel, normalAccel)  // up-left
                .splineToConstantHeading(new Vector2d(0, -50),  Math.toRadians(-90),slowVel, superSlowAccel)  // up-left
                .build();

        Action DriveBigTriangleToThirdMark = drive.actionBuilder(new Pose2d(-11.5, -25, Math.toRadians(-90)))   // FIX
                .splineToConstantHeading(new Vector2d(-15, -31), Math.toRadians(-90), slowVel, slowAccel)
                .strafeToConstantHeading(new Vector2d(-15, -60), slowVel, slowAccel)
                .build();

        Action DriveThirdMarkToBigTriangle = drive.actionBuilder(new Pose2d(-15, -60, Math.toRadians(-90)))
                .strafeToConstantHeading(new Vector2d(-11.5, -25), fastVel, fastAccel)
                .build();

        Action DriveLockToFirstMark = drive.actionBuilder(new Pose2d(0,-50, Math.toRadians(-90)))
                .splineToConstantHeading(new Vector2d(0, -25),  Math.toRadians(-270), normalVel, normalAccel)
                .splineToConstantHeading(new Vector2d(34.75, -30),  Math.toRadians(-90),fastVel, fastAccel)
                .strafeToConstantHeading(new Vector2d(34.75, -60), normalVel, normalAccel)
                .build();


        Action DriveBigTriangleToFirstMark =  drive.actionBuilder(new Pose2d(-11.5, -25, Math.toRadians(-90)))
                .strafeToConstantHeading(new Vector2d(34.75, -30), fastVel, fastAccel)
                //.splineToConstantHeading(new Vector2d(34.75, 60), Math.toRadians(90), intakeVel, intakeAccel)
                .strafeToConstantHeading(new Vector2d(34.75, -60), normalVel, normalAccel)
                .build();

        Action DriveFirstMarkToShootingPosition =  drive.actionBuilder(new Pose2d(34.74, -60, Math.toRadians(-90)))
                .strafeToConstantHeading(new Vector2d(48, -12), fastVel, normalAccel)
                .build();

        Action DriveShootingPositionToGateLock =  drive.actionBuilder(new Pose2d(48, -12, Math.toRadians(-90)))
                .strafeToConstantHeading(new Vector2d(0, -30), fastVel, fastAccel)
                .build();

        Action DriveSecondMarkToShootingPosition = drive.actionBuilder(new Pose2d(11.5, -68, Math.toRadians(-90)))
                .strafeToConstantHeading(new Vector2d(24,-12),fastVel,fastAccel)
                .strafeToConstantHeading(new Vector2d(48,-12),normalVel,normalAccel)
                .build();

        Action DriveShootingPositionToCollectGateBalls = drive.actionBuilder(new Pose2d(48,-12, Math.toRadians(-90)))
                .splineToLinearHeading(new Pose2d(25,-56,Math.toRadians(-30)),Math.toRadians(-150))
                .strafeToConstantHeading(new Vector2d(60,-60))
                .build();

        Action DriveCollectGateBallsToShootingPosition = drive.actionBuilder(new Pose2d(60,-60,Math.toRadians(-30)))
                .strafeToLinearHeading(new Vector2d(48,-12), Math.toRadians(-90))
                .build();

        Action DriveOutOfSmallTriangle = drive.actionBuilder(new Pose2d(48,-12,Math.toRadians(-90)))
                .strafeToConstantHeading(new Vector2d(36,-12))
                        .build();

        // ***************************************************
        // ****  WAIT for START/PLAY to be pushed ************
        // ***************************************************
        // -------------------------
        // START -> SHOOT FROM SMALL TRIANGLE
        // -------------------------
        // Drive to the shooting position
        myBot.runAction(new SequentialAction(DriveToShootingPosition,
                        new SleepAction(0.5),
				        // SHOOT-3
				        new SleepAction(2.175),
        // -------------------------
        // -> SECOND STRIP -> SHOOT FROM SMALL TRIANGLE
        // -------------------------
                        DriveToSecondMark,
				        //new SleepAction(0.250),
                        DriveSecondMarkToShootingPosition,
        // -------------------------
        // SMALL TRIANGLE -> GATE -> FIRST STRIP -> SHOOT FROM SMALL TRIANGLE
        // -------------------------
                        SmallTriangleToLock,
                        new SleepAction(0.1),
                        DriveLockToFirstMark,
                        //new SleepAction(0.250),
                        DriveFirstMarkToShootingPosition,
                        new SleepAction(2.175),

        // -------------------------
        // SMALL TRIANGLE -> COLLECT BALLS FROM GATE -> SMALL TRIANGLE -> SHOOT
        // -------------------------
            DriveShootingPositionToCollectGateBalls,
                DriveCollectGateBallsToShootingPosition,
                new SleepAction(2.175),
                DriveOutOfSmallTriangle


        ));


        meepMeep.setTheme(new ColorSchemeBlueDark(), new ColorSchemeRedLight());
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}