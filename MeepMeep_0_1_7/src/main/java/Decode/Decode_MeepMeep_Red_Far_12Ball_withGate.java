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

public class Decode_MeepMeep_Red_Far_12Ball_withGate {

    // Trajectories
    static Action DriveToShootingPosition;
    static Action DriveToSecondMark;
    static Action DriveThirdMarkToBigTriangle;
    static Action SecondMarkToLock;
    static Action LockToBigTriangle;
    static Action DriveBigTriangleToThirdMark;
    static Action DriveBigTriangleToFirstMark;
    static Action DriveFirstMarkToLock;
    static Action DriveFirstMarkToShootingPosition;
    static Action DriveShootingPositionToGateLock;

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
    VelConstraint fastVel = new TranslationalVelConstraint(90);
    AccelConstraint fastAccel = new ProfileAccelConstraint(-65, 65);

    VelConstraint intakeVel = new TranslationalVelConstraint(60);
    AccelConstraint intakeAccel = new ProfileAccelConstraint(-30, 40);

    VelConstraint loopVel = new TranslationalVelConstraint(40);
    AccelConstraint loopAccel = new ProfileAccelConstraint(-25, 25);
        // Path matches your auto:
        // Start (60,12,90) -> Shooting (48,12)
        // First strip: (34.75,30)->(34.75,63) -> back (48,12)
        // Second strip: (11.5,30)->(11.5,63) -> back (20,12)->(48,12)
        // Third strip: (-15,32)->(-15,60) -> Big triangle (-12.4,12.7)

        // ***************************************************
        // ****  Define Trajectories    **********************
        // ***************************************************
        DriveShim drive = myBot.getDrive();
        //Shoot 3 preloaded balls
        DriveToShootingPosition = drive.actionBuilder(new Pose2d(60, 12, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(48, 12), fastVel, fastAccel)
                .build();

        //DriveToSecondMarkToCollectBallsandgetclosertogatelock
        DriveToSecondMark = drive.actionBuilder(new Pose2d(48, 12, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(11.5, 30), Math.toRadians(90), fastVel, fastAccel)
                .splineToConstantHeading(new Vector2d(11.5, 60), Math.toRadians(90), intakeVel, intakeAccel)
                .build();

        SecondMarkToLock = drive.actionBuilder(new Pose2d(11.5, 60, Math.toRadians(90)))   // FIX
                // U-loop to bump/open the gate near the top wall, then come back down to the lock
                // (tweak these waypoints to match your exact gate location)
                .splineToConstantHeading(new Vector2d(5, 40),  Math.toRadians(180), loopVel, loopAccel)  // up-left
                .splineToConstantHeading(new Vector2d(0, 50),  Math.toRadians(90), loopVel, loopAccel)  // up-left
                .build();

        LockToBigTriangle = drive.actionBuilder(new Pose2d(0, 50, Math.toRadians(90)))   // FIX
                .splineToConstantHeading(new Vector2d(0, 20),  Math.toRadians(270), intakeVel, intakeAccel)  // up-left
                .splineToConstantHeading(new Vector2d(-11.5, 21),  Math.toRadians(90),fastVel, fastAccel)  // up-left
                .build();

        DriveBigTriangleToThirdMark = drive.actionBuilder(new Pose2d(-11.5, 21, Math.toRadians(90)))   // FIX
                .splineToConstantHeading(new Vector2d(-15, 31), Math.toRadians(90), loopVel, loopAccel)
                .strafeToConstantHeading(new Vector2d(-15, 60), loopVel, loopAccel)
                .build();

        DriveThirdMarkToBigTriangle = drive.actionBuilder(new Pose2d(-15, 60, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(-11.5, 21), fastVel, fastAccel)
                .build();

        DriveBigTriangleToFirstMark =  drive.actionBuilder(new Pose2d(-11.5, 21, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(34.75, 30), fastVel, fastAccel)
                //.splineToConstantHeading(new Vector2d(34.75, 60), Math.toRadians(90), intakeVel, intakeAccel)
                .strafeToConstantHeading(new Vector2d(34.75, 60), intakeVel, intakeAccel)
                .build();

        DriveFirstMarkToShootingPosition =  drive.actionBuilder(new Pose2d(34.74, 60, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(48, 12), fastVel, fastAccel)
                .build();

        DriveFirstMarkToLock =  drive.actionBuilder(new Pose2d(48, 12, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(0, 40), fastVel, fastAccel)
                .build();

        DriveShootingPositionToGateLock =  drive.actionBuilder(new Pose2d(48, 12, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(0, 40), fastVel, fastAccel)
                .build();

        // ***************************************************
        // ****  WAIT for START/PLAY to be pushed ************
        // ***************************************************
        myBot.runAction(new SequentialAction(
                        // DriveToShootingPosition
                        DriveToShootingPosition,
                        new SleepAction(0.5),
				        // SHOOT-3
				        new SleepAction(2.175),
        // -------------------------
        // FIRST STRIP -> BACK -> SHOOT
        // -------------------------
                        DriveToSecondMark,
				        new SleepAction(0.4),
                        SecondMarkToLock,
                        LockToBigTriangle,
				        // SHOOT-3
				        new SleepAction(2.175),
        // -------------------------
        // SECOND STRIP -> BACK -> SHOOT
        // -------------------------
                        DriveBigTriangleToThirdMark,
                        new SleepAction(0.250),
                        DriveThirdMarkToBigTriangle,
				        // SHOOT-3
				        new SleepAction(2.175),
        // -------------------------
        // THIRD STRIP -> BIG TRIANGLE -> SHOOT
        // -------------------------
                        DriveBigTriangleToFirstMark,
                        new SleepAction(0.250),
                        DriveFirstMarkToShootingPosition,
				        // SHOOT-3
				        new SleepAction(2.175),
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