package Into_The_Deep;

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
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MM_HighFiveBasketsRoute {
    static Action DeliverStartingSample;
    static Action DriveToSample1;
    static Action DeliverSample1;
    static Action DriveToSample2;
    static Action DeliverSample2;
    static Action DriveToSample3;
    static Action DeliverSample3;
    static Action DriveToSubmersible4;
    static Action DeliverSample4;
    static Action DriveToSubmersible5;
    static Action DeliverSample5;
    static Action DriveToSubmersible6;
    static Action DeliverSample6;

    static VelConstraint speedUpVelocityConstraint;
    static AccelConstraint speedUpAccelerationConstraint;
    static VelConstraint normalVelocityConstraint;
    static AccelConstraint normalAccelerationConstraint;
    static VelConstraint slowDownVelocityConstraint;
    static AccelConstraint slowDownAccelerationConstraint;
    static VelConstraint intakeVelocityConstraint;
    static VelConstraint humanPlayerVelocityConstraint;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 50, Math.PI*.6, Math.PI*.8, 15)
                .build();

        Pose2d StartPose = new Pose2d(-47, -58, Math.toRadians(45));

        // Define some custom constraints to use when wanting to go faster than defaults
        speedUpVelocityConstraint = new TranslationalVelConstraint(60.0);
        speedUpAccelerationConstraint = new ProfileAccelConstraint(-40.0, 60.0);
        normalVelocityConstraint = new TranslationalVelConstraint(50.0);
        normalAccelerationConstraint = new ProfileAccelConstraint(-35.0, 50.0);
        slowDownVelocityConstraint = new TranslationalVelConstraint(30);
        slowDownAccelerationConstraint = new ProfileAccelConstraint(-20, 50);
        intakeVelocityConstraint = new TranslationalVelConstraint(15);
        humanPlayerVelocityConstraint = new TranslationalVelConstraint(7);

        // ***************************************************
        // ****  Define Trajectories    **********************
        // ***************************************************

        DeliverStartingSample = myBot.getDrive().actionBuilder(StartPose)
                //.splineToConstantHeading(new Vector2d(-36, -36),Math.toRadians(90))
                .setReversed(true)
                // move center of front of robot to center of diagonal basket line (6,6) diff from start
                .strafeToConstantHeading(new Vector2d(-53,-52))
                // put front corners of robot exactly in corner at a 45 degree angle (12,0) diff from start
                .strafeToConstantHeading(new Vector2d(-59,-58))
                .build();

        DriveToSample1 = myBot.getDrive().actionBuilder(new Pose2d(-59, -58, Math.toRadians(45)))
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(-48,-47), Math.toRadians(90))
                .build();

        DeliverSample1 = myBot.getDrive().actionBuilder(new Pose2d(-48,-47, Math.toRadians(90)))
                .setReversed(true)
                // move center of front of robot to center of diagonal basket line (6,6) diff from start
                .strafeToLinearHeading(new Vector2d(-53,-52), Math.toRadians(45.0))
                // put front corners of robot exactly in corner at a 45 degree angle (12,0) diff from start
                .strafeToConstantHeading(new Vector2d(-53,-52))
                .build();

        DriveToSample2 = myBot.getDrive().actionBuilder(new Pose2d(-53, -52, Math.toRadians(45)))
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(-57,-47), Math.toRadians(90))
                .build();
        DeliverSample2 = myBot.getDrive().actionBuilder(new Pose2d(-57,-47, Math.toRadians(90)))
                .setReversed(true)
                // move center of front of robot to center of diagonal basket line (6,6) diff from start
                .strafeToLinearHeading(new Vector2d(-53,-52), Math.toRadians(45.0))
                // put front corners of robot exactly in corner at a 45 degree angle (12,0) diff from start
                .strafeToConstantHeading(new Vector2d(-59,-58))
                .build();

        DriveToSample3 = myBot.getDrive().actionBuilder(new Pose2d(-59, -58, Math.toRadians(45)))
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(-55,-40), Math.toRadians(135))
                .build();
        DeliverSample3 = myBot.getDrive().actionBuilder(new Pose2d(-55,-40, Math.toRadians(135)))
                .setReversed(true)
                // move center of front of robot to center of diagonal basket line (6,6) diff from start
                .strafeToLinearHeading(new Vector2d(-53,-52), Math.toRadians(45.0))
                // put front corners of robot exactly in corner at a 45 degree angle (12,0) diff from start
                .strafeToConstantHeading(new Vector2d(-59,-58))
                .build();
        DriveToSubmersible4 = myBot.getDrive().actionBuilder(new Pose2d(-59, -58, Math.toRadians(45)))
                .splineTo(new Vector2d(-36,-12),Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-24, -12), Math.toRadians(0))
                .build();
        DeliverSample4 = myBot.getDrive().actionBuilder(new Pose2d(-24,-12, Math.toRadians(0)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-48,-48),Math.toRadians(45))
                .strafeToLinearHeading(new Vector2d(-59,-59), Math.toRadians(45))
                .build();
        DriveToSubmersible5 = myBot.getDrive().actionBuilder(new Pose2d(-59, -59, Math.toRadians(45)))
                .splineTo(new Vector2d(-36,-12),Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-24, -12), Math.toRadians(0))
                .build();
        DeliverSample5 = myBot.getDrive().actionBuilder(new Pose2d(-24,-12, Math.toRadians(0)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-48,-48),Math.toRadians(45))
                .strafeToLinearHeading(new Vector2d(-59,-59), Math.toRadians(45))
                .build();
        DriveToSubmersible6 = myBot.getDrive().actionBuilder(new Pose2d(-59, -59, Math.toRadians(45)))
                .splineTo(new Vector2d(-36,-12),Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-24, -12), Math.toRadians(0))
                .build();
        // ***************************************************
        // ****  START DRIVING    ****************************
        // ***************************************************
        myBot.runAction(
                new SequentialAction(
                        //Drive to submersible and pick up sample
                        DeliverStartingSample,
                        // Put sample in the basket and safely Lower the slides and arms before moving
                        new SleepAction(1.0),

                        DriveToSample1,
                        // Pick up Sample from floor
                        new SleepAction(1.5),
                        // Deliver Sample to High Bassket
                        DeliverSample1,
                        // Put sample in the basket and safely Lower the slides and arms before moving
                        new SleepAction(1.0),

                        DriveToSample2,
                        // Pick up Sample from floor
                        new SleepAction(1.5),
                        // Deliver Sample to High Basket
                        DeliverSample2,
                        new SleepAction(1.0),
                        DriveToSample3,
                        new SleepAction(1.5),
                        DeliverSample3,
                        DriveToSubmersible4,
                        DeliverSample4,
                        DriveToSubmersible5,
                        DeliverSample5,
                        DriveToSubmersible6
                        // Put sample in the basket and safely Lower the slides and arms before moving
                     //   new SleepAction(6.0),

                        //DriveToSample3,
                        // Pick up Sample from floor
                       // new SleepAction(1.5),
                        // Deliver Sample to High Basket
                        //DeliverSample3,
                        // Put sample in the basket and safely Lower the slides and arms before moving
                      //  new SleepAction(6.0)

                        ));

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
