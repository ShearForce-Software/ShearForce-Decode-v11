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

public class Decode_MeepMeep_Red_Far_9Balls {

    // Trajectories
    static Action DriveToShootingPosition;
    static Action DriveToFirstMark;
    static Action DriveToSecondMark;
    static Action DriveToThirdMark;
    static Action DriveOutofLaunchZone;
    static Action ReturnFromFirstMark;
    static Action ReturnFromSecondMark;
    static Action ReturnFromThirdMark;


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


        // ***************************************************
        // ****  Define Trajectories    **********************
        // ***************************************************
        DriveShim drive = myBot.getDrive();

        DriveToShootingPosition = drive.actionBuilder(new Pose2d(60, 12, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(48, 12))
                .build();

// FIRST  = far-right strip (closest to GOAL side)
// SECOND = center strip
// THIRD  = far-left strip

        DriveToFirstMark = drive.actionBuilder(new Pose2d(48, 12, Math.toRadians(90)))

                .strafeToConstantHeading(new Vector2d(32.25, 30))
                .strafeToConstantHeading(new Vector2d(32.25, 60))
                .build();

        DriveToSecondMark = drive.actionBuilder(new Pose2d(48, 12, Math.toRadians(90)))

                .strafeToConstantHeading(new Vector2d(9, 30))
                .strafeToConstantHeading(new Vector2d(9, 60))
                .build();

        DriveToThirdMark = drive.actionBuilder(new Pose2d(48, 12, Math.toRadians(90)))

                .strafeToConstantHeading(new Vector2d(-17.5, 32))
                .strafeToConstantHeading(new Vector2d(-17.5, 55))
                .build();

        ReturnFromFirstMark = drive.actionBuilder(new Pose2d(32.25, 60, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(48, 12))
                .build();


        ReturnFromSecondMark = drive.actionBuilder(new Pose2d(9, 60, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(20, 12))
                .strafeToConstantHeading(new Vector2d(48, 12))
                .build();

        ReturnFromThirdMark = drive.actionBuilder(new Pose2d(-17.5, 55, Math.toRadians(90)))

                .strafeToConstantHeading(new Vector2d(-12, 32))
                .strafeToConstantHeading(new Vector2d(48, 12))
                .build();




        DriveOutofLaunchZone = drive.actionBuilder(new Pose2d(48, 12, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(20, 12))
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
                DriveToFirstMark,
				        new SleepAction(0.4),
                ReturnFromFirstMark,
				        // SHOOT-3
				        new SleepAction(2.175),
        // -------------------------
        // SECOND STRIP -> BACK -> SHOOT
        // -------------------------
                DriveToSecondMark,
                        new SleepAction(0.250),
                ReturnFromSecondMark,
				        // SHOOT-3
				        new SleepAction(2.175),
        // -------------------------
        // THIRD STRIP -> BIG TRIANGLE -> SHOOT
        // -------------------------
                DriveOutofLaunchZone
        ));


        meepMeep.setTheme(new ColorSchemeBlueDark(), new ColorSchemeRedLight());
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}