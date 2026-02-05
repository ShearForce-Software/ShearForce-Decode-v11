package Decode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class Decode_MeepMeep_Red_Close_6Ball {

    // Trajectories
    static Action DriveStartToMidPosition;
    static Action DriveMidToClosestLine;
    static Action DriveClosestLineBackToLaunchPark;

    static Action DriveClosestLineBackToMid;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        // ***************************************************
        // ****  Define Trajectories    **********************
        // ***************************************************
        DriveShim drive = myBot.getDrive();

        DriveStartToMidPosition = drive.actionBuilder(new Pose2d(-60, 39, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(-11.4, 10))
                .build();

        // Mid (-10, -10, 270) -> closest line (-10, -40)
        DriveMidToClosestLine = drive.actionBuilder(new Pose2d(-11.4, 10, Math.toRadians(90)))

                .strafeToConstantHeading(new Vector2d(-11.4, 55))
                .build();



        DriveClosestLineBackToMid = drive.actionBuilder(new Pose2d(-11.4, 55, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(-11.4, 10))
                .build();

        // Closest line (-10, -40, 270) -> launch park (-54, -16)
        DriveClosestLineBackToLaunchPark = drive.actionBuilder(new Pose2d(-11.4, 10, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(-54, 16))
                .build();


        // ***************************************************
        // ****  WAIT for START/PLAY to be pushed ************
        // ***************************************************
        myBot.runAction(new SequentialAction(
                // Big triangle -> center-ish
                DriveStartToMidPosition,
                // SHOOT-3
                new SleepAction(2.175),
                DriveMidToClosestLine,
                DriveClosestLineBackToMid,
                // SHOOT-3
                new SleepAction(2.175),
                DriveClosestLineBackToLaunchPark

        ));

        meepMeep.setTheme(new ColorSchemeBlueDark(), new ColorSchemeRedLight());
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

}
